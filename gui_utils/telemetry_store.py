"""Generic, schema-driven ring-buffer store for per-tick record logs.

Modeled as a standalone "database class" (parallel numpy columns, like the
hist_* buffers in pyqtgraph_earth_demo.py) rather than a real SQL engine --
consistent with this project's existing in-memory, performance-tiered
storage pattern, and cheap to query since max_messages bounds it well below
anything that would need indexing.

Not hardcoded to flight telemetry: the set of structured columns is
declared at construction time (`columns`), so the same class can back any
per-tick record log. Two of those declared columns are singled out as
`id_field`/`time_field` -- the keys `latest_at_or_before()` looks up by --
everything else is opaque to this class.

Every row also carries a `raw` object, independent of the declared
columns -- the full original message/record, preserved as-is (e.g. a dict
with more fields than this store's caller chose to break out into their own
column). Structured columns exist so filtering/sorting/rendering a table
stays vectorized and cheap; `raw` exists so nothing is ever lost.

**Ring buffer, not append-and-trim**: at a low-hundred-thousand cap with
per-tick overflow (new rows arriving every tick, oldest rows evicted every
tick once full), a naive `np.concatenate` + slice-trim approach copies the
entire buffer on every single tick -- an O(cap) cost paid every tick forever
once the buffer is full. This store instead preallocates fixed-size arrays
and writes into them at a wrapping cursor, so both append and eviction are
O(batch size), never O(cap).
"""
import numpy as np

MESSAGE_STORE_MAX = 500_000

# Cap on how many distinct values a tracked categorical column will
# remember for filter-menu population -- same rationale as
# FLIGHT_LIST_MAX in pyqtgraph_earth_demo.py (an unbounded checkbox list
# would be unusable and slow to build).
DISTINCT_VALUES_MAX = 500

__all__ = ["MessageStore", "MESSAGE_STORE_MAX", "DISTINCT_VALUES_MAX"]


class MessageStore:
    """Bounded, ring-buffered log of per-tick records.

    add_batch() is called once per tick with one row per source entity
    (mirroring EarthPlot._sample_history's hist_* append/evict pattern).
    latest_at_or_before() answers "what was the last record for this id, as
    of time t" -- the question the map-synced info panel needs.
    """

    def __init__(self, columns, id_field="entity_id", time_field="sim_time_s",
                 max_messages=MESSAGE_STORE_MAX):
        """
        columns: list of (name, dtype) pairs declaring every structured
            column this store holds -- e.g.
            [("entity_id", np.int64), ("sim_time_s", np.float64), ...].
        id_field / time_field: names of the two columns latest_at_or_before()
            keys its lookup on. Must both be present in `columns`.
        """
        names = [name for name, _ in columns]
        if id_field not in names or time_field not in names:
            raise ValueError(
                f"id_field ({id_field!r}) and time_field ({time_field!r}) "
                f"must both be declared in columns"
            )

        self.cap = max_messages
        self.id_field = id_field
        self.time_field = time_field
        self._columns = {name: np.empty(self.cap, dtype=dtype) for name, dtype in columns}
        # Full original message/record, independent of the declared
        # structured columns -- see module docstring.
        self.raw = np.empty(self.cap, dtype=object)
        # Global monotonic id, never reused -- lets filtered row-index sets
        # (see message_table_model.py) be trimmed/extended incrementally
        # instead of rescanned from scratch on every tick.
        self.seq = np.empty(self.cap, dtype=np.int64)

        self._write = 0          # next physical slot to write
        self._count = 0          # valid rows currently held, saturates at cap
        self._total_written = 0  # monotonic; seq of the next message == this value

        # Per-column distinct-value tracking, opt-in (see
        # track_distinct_values) -- insertion-ordered dict-as-ordered-set,
        # capped at DISTINCT_VALUES_MAX, never evicted even as the
        # underlying ring evicts old rows (the filter menu lists everything
        # "seen this session", matching pyqtgraph_earth_demo.py's
        # _refresh_flight_list/FLIGHT_LIST_MAX precedent).
        self._distinct = {}

    def track_distinct_values(self, *names):
        """Declare which columns should maintain a capped, insertion-ordered
        set of distinct values seen -- for categorical filter-menu
        population (see message_filter_menu.py). Opt-in per column since
        tracking distinct values for e.g. a lon/lat column would be
        meaningless and wasteful."""
        for name in names:
            if name not in self._columns:
                raise KeyError(f"unknown column: {name!r}")
            self._distinct.setdefault(name, {})

    def reset(self):
        """Called instead of reassigning `self.message_store = MessageStore(...)`
        at vehicle-(re)spawn time, so a table model holding a reference to
        this object stays valid across the reset. Caller is responsible for
        telling any attached table model to do a full reset after this."""
        self._write = 0
        self._count = 0
        self._total_written = 0
        for name in self._distinct:
            self._distinct[name].clear()

    def add_batch(self, raw, **column_values):
        """Append one batch (one tick's worth of rows) to the ring buffer.

        raw: array-like of length n, one opaque object per row (see module
            docstring) -- e.g. a dict carrying every field of that row's
            source record, including any not broken out into a column.
        column_values: keyword args, one per column declared at
            construction, each an array-like of length n (or a scalar,
            broadcast -- e.g. a single tick's shared timestamp).

        Returns (n_new, n_evicted): n_new is how many rows this call added
        (== n, unless the degenerate single-batch-overflows-cap case below
        trims it), n_evicted is how many previously-held rows this call
        pushed out of the buffer. A table model built on top of this store
        uses these two numbers to emit precise beginInsertRows/
        beginRemoveRows calls instead of a full reset per tick.
        """
        missing = set(self._columns) - set(column_values)
        unknown = set(column_values) - set(self._columns)
        if missing or unknown:
            raise ValueError(
                f"add_batch column mismatch -- missing: {sorted(missing)}, "
                f"unknown: {sorted(unknown)}"
            )

        raw = np.asarray(raw, dtype=object)
        n = len(raw)
        if n == 0:
            return 0, 0

        resolved = {}
        for name, arr in self._columns.items():
            values = column_values[name]
            values = np.broadcast_to(np.asarray(values, dtype=arr.dtype), (n,))
            resolved[name] = values

        if n >= self.cap:
            # Degenerate case: a single batch alone exceeds the buffer's
            # capacity. Ring semantics ("oldest N evicted as newest N
            # arrive") don't really apply when one batch already can't fit
            # -- just keep this batch's last `cap` rows and treat it as a
            # full-buffer replace. Defensive only: unreachable through this
            # app's UI today since VEHICLE_MAX_COUNT (100,000) is well under
            # MESSAGE_STORE_MAX (500,000).
            keep = slice(n - self.cap, n)
            raw = raw[keep]
            resolved = {name: values[keep] for name, values in resolved.items()}
            n = self.cap
            evicted = self._count
            self._write = 0
            self._count = 0
            # slot_for_seq() relies on slot == seq % cap, which only holds
            # when the write cursor is a multiple of cap whenever seq
            # numbering restarts at that cursor -- true by construction in
            # the normal (non-degenerate) path, but this branch just forced
            # _write to 0 out of band, so realign _total_written to the
            # next multiple of cap (leaving a gap in seq values, which is
            # fine -- seq only needs to be monotonic and unique).
            if self._total_written % self.cap != 0:
                self._total_written += self.cap - (self._total_written % self.cap)
        else:
            evicted = max(0, (self._count + n) - self.cap)

        idx = (self._write + np.arange(n)) % self.cap
        for name, values in resolved.items():
            self._columns[name][idx] = values
        self.raw[idx] = raw
        self.seq[idx] = self._total_written + np.arange(n)

        self._write = (self._write + n) % self.cap
        self._count = min(self._count + n, self.cap)
        self._total_written += n

        for name, tracked in self._distinct.items():
            for value in resolved[name]:
                if value not in tracked and len(tracked) >= DISTINCT_VALUES_MAX:
                    continue
                tracked[value] = None

        return n, evicted

    def __len__(self):
        return self._count

    @property
    def total_written(self):
        return self._total_written

    def physical_index(self, row_from_newest):
        """Newest-first logical row -> physical ring slot. row 0 == most
        recently written row."""
        return (self._write - 1 - row_from_newest) % self.cap

    @property
    def column_names(self):
        return list(self._columns.keys())

    def slot_for_seq(self, seq):
        """Physical ring slot a given seq was written to. Valid because
        add_batch's wraparound index for a row is always (write-cursor +
        offset) % cap, and the write cursor always equals total_written %
        cap at the start of any batch -- so slot == seq % cap, unconditionally,
        with no need to track historical cursor positions."""
        return seq % self.cap

    def column(self, name):
        """The raw underlying numpy array for a declared column -- read-only,
        no copy. Only slots [0, count) in ring order are logically valid;
        callers needing a specific logical row should go through row_at()
        or physical_index() rather than indexing this directly."""
        return self._columns[name]

    def row_at(self, row_from_newest):
        """Full dict for one logical row (0 == newest): every declared
        column, plus "raw" and "seq"."""
        i = self.physical_index(row_from_newest)
        row = {name: arr[i] for name, arr in self._columns.items()}
        row["raw"] = self.raw[i]
        row["seq"] = int(self.seq[i])
        return row

    def distinct_values(self, name):
        """Insertion-ordered list of values seen so far for a column passed
        to track_distinct_values(), capped at DISTINCT_VALUES_MAX."""
        return list(self._distinct[name].keys())

    def _valid_mask(self):
        """Boolean array, length cap: True at physical slots currently
        holding a logically-valid row. NOTE: physical slot order is *not*
        chronological order once the buffer has wrapped at least once --
        e.g. after wraparound, low slot numbers can hold the most recently
        written rows and high slot numbers the oldest. Anything that needs
        "most recent" must compare by `seq`, never by raw physical index
        (see latest_at_or_before below, which got this wrong in an earlier
        version)."""
        mask = np.zeros(self.cap, dtype=bool)
        if self._count == 0:
            return mask
        if self._count == self.cap:
            mask[:] = True
        elif self._write >= self._count:
            mask[self._write - self._count:self._write] = True
        else:
            mask[self.cap - (self._count - self._write):] = True
            mask[:self._write] = True
        return mask

    def column_bounds(self, name):
        """(min, max) currently held for a numeric/time column, restricted
        to the logically-valid ring range -- for a range-filter popup's
        default bounds. None if the store is empty."""
        if self._count == 0:
            return None
        values = self._columns[name][self._valid_mask()]
        return values.min(), values.max()

    def latest_at_or_before(self, id_value, t):
        """Most recent row for id_value with time_field <= t, or None."""
        if self._count == 0:
            return None
        id_col = self._columns[self.id_field]
        time_col = self._columns[self.time_field]
        mask = self._valid_mask() & (id_col == id_value) & (time_col <= t)
        idx = np.nonzero(mask)[0]
        if len(idx) == 0:
            return None
        # Among matches, "most recent" means largest seq -- physical slot
        # order is not chronological once the buffer has wrapped (see
        # _valid_mask), so this must compare seq, not just take idx[-1].
        i = idx[np.argmax(self.seq[idx])]

        row = {name: arr[i] for name, arr in self._columns.items()}
        row["raw"] = self.raw[i]
        row["seq"] = int(self.seq[i])
        return row
