"""Generic, high-performance QAbstractTableModel over a telemetry_store.MessageStore.

Not hardcoded to any particular app's columns -- `display_columns` (passed
at construction) is a list of dicts choosing which of the store's declared
columns to show, in what order, with what label, and how header-click
filtering should behave for that column:

    {"name": "arrival_dt", "label": "Timestamp", "filter": "range"}
    {"name": "flight_label", "label": "Flight #", "filter": "categorical"}
    {"name": "aircraft_type", "label": "Aircraft Type", "filter": "categorical"}

"filter" is "categorical" (checkbox-list popup, backed by
store.distinct_values(name) -- name must have been passed to
store.track_distinct_values()), "range" (from/to popup, backed by
store.column_bounds(name)), or omitted/None (header click does nothing for
that column). An optional "format" callable(value) -> str overrides the
default display formatting for that column.

Performance design (see ARCHITECTURE_pyqtgraph_earth_demo.md's house style
for the same philosophy applied to the map's own history/ellipse layers):

- rowCount()/data() read straight through the store's numpy columns by ring
  position -- no per-row Python object materialization ahead of time.
  QTableView only ever calls data() for currently-visible rows, so this
  stays cheap regardless of how many rows the store holds.
- New rows / evicted rows are applied via targeted beginInsertRows/
  beginRemoveRows (see on_batch_added), not a full model reset per tick.
- An active filter or pause snapshot is tracked as `_view_seqs`, an ascending
  array of the store's monotonic `seq` values that currently pass the filter
  (all held seqs, if no filter). Extending it for a new batch, or trimming it
  for evicted rows, is O(batch size) / O(log filtered-size), never O(store
  size) -- only an actual filter *criteria* change (set_filter) triggers a
  full O(store size) rescan, and that happens once per explicit user action,
  not once per tick.
- While the tab hosting this model isn't the active/visible one, per-tick
  Qt row-change signals are skipped (set_active(False)) -- bookkeeping
  (_view_seqs, counts) still stays correct, so reactivating just needs one
  full resync (a single reset), not a backlog of missed signal emissions.

Pause/live-tail (see pause()/resume()): the underlying MessageStore is a
fixed-cap ring buffer that keeps evicting oldest rows regardless of what the
view is doing, so a selected/scrolled-to row can otherwise be yanked out
from under the user the moment the store wraps. Pausing freezes the model's
row<->seq mapping (_view_seqs) at whatever it was when pausing started and
stops applying on_batch_added to the view (incoming batches are just
counted) until resume() -- so the displayed rows hold still. This is a
*view*-level freeze, not a data-retention guarantee: the store keeps writing
new rows underneath, so a pause long enough to wrap the store's cap (500k
rows by default) will start reading stale/overwritten data for the oldest
frozen rows. Fine for the realistic "inspect a row for a few seconds/minutes"
use case this exists for; not a substitute for actually persisting history.
"""
from datetime import datetime

import numpy as np
from pyqtgraph.Qt import QtCore

Qt = QtCore.Qt
QAbstractTableModel = QtCore.QAbstractTableModel
QModelIndex = QtCore.QModelIndex

__all__ = ["MessageTableModel"]


def _default_format(value):
    if isinstance(value, datetime):
        return value.strftime("%H:%M:%S.%f")[:-3]
    if isinstance(value, (np.floating, float)):
        return f"{float(value):.2f}"
    return str(value)


def _predicate_mask(values, predicate):
    if isinstance(predicate, (set, frozenset)):
        return np.isin(values, np.asarray(list(predicate), dtype=object))
    lo, hi = predicate
    return (values >= lo) & (values <= hi)


def _validate_display_columns(display_columns):
    """Catch a typo'd/missing "name" or "label" key at construction time,
    rather than deep in Qt's paint path the first time that column's cell
    or header actually gets drawn."""
    for i, spec in enumerate(display_columns):
        missing = [k for k in ("name", "label") if k not in spec]
        if missing:
            raise ValueError(
                f"display_columns[{i}] ({spec!r}) is missing required key(s): {missing}"
            )


class MessageTableModel(QAbstractTableModel):
    # Emitted with the new pending_count() whenever a batch arrives while
    # paused -- lets the panel keep its "N new" indicator live without a
    # rowsInserted signal (which is deliberately suppressed while paused).
    pending_changed = QtCore.pyqtSignal(int)

    def __init__(self, store, display_columns, parent=None):
        super().__init__(parent)
        _validate_display_columns(display_columns)
        self.store = store
        self.display_columns = display_columns
        self._filters = {}              # column name -> predicate
        self._view_seqs = None          # ascending np.int64 array, or None == live/unfiltered
        self._active = True
        self._paused = False
        self._pending_new = 0           # batches arrived while paused, not yet shown

    # -- Qt model interface ----------------------------------------------

    def rowCount(self, parent=QModelIndex()):
        if parent.isValid():
            return 0
        return len(self._view_seqs) if self._view_seqs is not None else len(self.store)

    def columnCount(self, parent=QModelIndex()):
        return len(self.display_columns)

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return self.display_columns[section]["label"]
        return None

    def data(self, index, role=Qt.DisplayRole):
        if role != Qt.DisplayRole or not index.isValid():
            return None
        spec = self.display_columns[index.column()]
        value = self._raw_value(index.row(), spec["name"])
        fmt = spec.get("format")
        return fmt(value) if fmt else _default_format(value)

    # -- row/seq/physical-slot mapping ------------------------------------

    def _physical_index_for_row(self, row):
        """row 0 == oldest, regardless of whether a filter/pause snapshot is
        active."""
        if self._view_seqs is not None:
            seq = self._view_seqs[row]
            return self.store.slot_for_seq(int(seq))
        return self.store.physical_index(len(self.store) - 1 - row)

    def _raw_value(self, row, name):
        idx = self._physical_index_for_row(row)
        return self.store.column(name)[idx]

    def row_dict(self, row):
        """Full dict for a displayed row -- every store column, plus "raw"
        and "seq" -- a superset of what's shown in display_columns. Used by
        the detail panel."""
        idx = self._physical_index_for_row(row)
        out = {name: self.store.column(name)[idx] for name in self.store.column_names}
        out["raw"] = self.store.raw[idx]
        out["seq"] = int(self.store.seq[idx])
        return out

    # -- filtering ---------------------------------------------------------

    def get_filter(self, name):
        """Currently-active predicate for a column, or None if unfiltered
        -- so a filter-menu popup can show its previous selection state."""
        return self._filters.get(name)

    def set_filter(self, name, predicate):
        """predicate: a set of allowed values (categorical column) or a
        (lo, hi) tuple (range column), or None to clear that column's filter."""
        if predicate is None:
            self._filters.pop(name, None)
        else:
            self._filters[name] = predicate
        self.full_reset()

    def _compute_view_seqs(self):
        """Every currently-held seq that passes self._filters (all of them,
        if _filters is empty) -- used both for an actual column filter and
        for a pause snapshot (see pause())."""
        n = len(self.store)
        if n == 0:
            return np.empty(0, dtype=np.int64)
        seqs_held = np.arange(
            self.store.total_written - n, self.store.total_written, dtype=np.int64
        )
        idxs = seqs_held % self.store.cap
        mask = np.ones(len(seqs_held), dtype=bool)
        for name, predicate in self._filters.items():
            mask &= _predicate_mask(self.store.column(name)[idxs], predicate)
        return seqs_held[mask]  # ascending: seqs_held is ascending, boolean mask preserves order

    def full_reset(self):
        """Full O(store size) recompute -- used when filter criteria
        actually change (a user action, not a per-tick event), when
        reactivating after a period with signals suppressed (set_active),
        and when pausing/resuming (see pause()/resume())."""
        self.beginResetModel()
        self._view_seqs = self._compute_view_seqs() if (self._filters or self._paused) else None
        self.endResetModel()

    # -- pause/live-tail ----------------------------------------------------

    def is_paused(self):
        return self._paused

    def pending_count(self):
        """How many new rows have arrived since pause() was called and are
        not yet reflected in the view -- for a "N new -- Resume" indicator."""
        return self._pending_new

    def pause(self):
        """Freeze the displayed rows -- see module docstring. Idempotent.

        Deliberately does NOT go through full_reset()/beginResetModel(): the
        snapshot taken here is, by construction, the exact same rows in the
        exact same order the view was already showing (pausing doesn't
        change what's on screen, only how future updates are handled) -- a
        model reset would needlessly clear the view's current selection."""
        if self._paused:
            return
        self._paused = True
        self._pending_new = 0
        if self._view_seqs is None:
            self._view_seqs = self._compute_view_seqs()

    def resume(self):
        """Drop the freeze and resync to the store's current state (a
        jump, not a smooth catch-up -- see module docstring). Idempotent."""
        if not self._paused:
            return
        self._paused = False
        self._pending_new = 0
        self.full_reset()

    # -- feeding from EarthPlot._sample_messages --------------------------

    def set_active(self, active):
        """Call when the tab hosting this model becomes visible/hidden.
        While inactive, on_batch_added() still updates bookkeeping but
        skips emitting Qt row-change signals (nothing is rendering them
        anyway) -- reactivating does one full resync instead of replaying
        a backlog of missed per-tick signals."""
        was_active = self._active
        self._active = active
        if active and not was_active:
            self.full_reset()

    def on_batch_added(self, n_new, evicted):
        """Called right after MessageStore.add_batch() with its return
        value. NOTE: the store has already been mutated by the time this
        runs (add_batch already happened) -- begin/end signals below are
        therefore an approximation of Qt's usual "signal brackets the
        mutation" contract, accepted here since this model never needs to
        query pre-mutation state beyond simple row counts."""
        if n_new == 0 and evicted == 0:
            return
        if self._paused:
            self._pending_new += n_new
            self.pending_changed.emit(self._pending_new)
            return
        if self._view_seqs is not None:
            self._on_batch_added_filtered(n_new, evicted)
        else:
            self._on_batch_added_unfiltered(n_new, evicted)

    def _on_batch_added_unfiltered(self, n_new, evicted):
        new_count = len(self.store)
        old_count = new_count - n_new + evicted
        if evicted > 0 and self._active:
            self.beginRemoveRows(QModelIndex(), 0, evicted - 1)
            self.endRemoveRows()
        if n_new > 0 and self._active:
            self.beginInsertRows(QModelIndex(), old_count, old_count + n_new - 1)
            self.endInsertRows()

    def _on_batch_added_filtered(self, n_new, evicted):
        # 1. Drop whatever fell below the store's new logical floor from
        # the filtered index -- these are the filtered view's *top* rows
        # (oldest), since _view_seqs is ascending (oldest .. newest).
        floor = self.store.total_written - len(self.store)
        cut = int(np.searchsorted(self._view_seqs, floor, side="left"))
        if cut > 0:
            if self._active:
                self.beginRemoveRows(QModelIndex(), 0, cut - 1)
            self._view_seqs = self._view_seqs[cut:]
            if self._active:
                self.endRemoveRows()

        # 2. Mask just the newly-arrived rows (cheap: proportional to batch
        # size, not store size) and append whatever passes to the tail
        # (newest) of _view_seqs.
        new_seqs = np.arange(
            self.store.total_written - n_new, self.store.total_written, dtype=np.int64
        )
        idxs = new_seqs % self.store.cap
        mask = np.ones(len(new_seqs), dtype=bool)
        for name, predicate in self._filters.items():
            mask &= _predicate_mask(self.store.column(name)[idxs], predicate)
        passing = new_seqs[mask]
        if len(passing) > 0:
            k = len(passing)
            old_len = len(self._view_seqs)
            if self._active:
                self.beginInsertRows(QModelIndex(), old_len, old_len + k - 1)
            self._view_seqs = np.concatenate([self._view_seqs, passing])
            if self._active:
                self.endInsertRows()
