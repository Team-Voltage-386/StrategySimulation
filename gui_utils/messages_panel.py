"""Second-tab UI: a table view over a telemetry_store.MessageStore, with
Excel-style header-click filters and a selection-driven detail panel.

Deliberately independent of any map-side selection/time-scrub machinery --
see info_panel.py for that (map-synced) panel; MessageDetailPanel here only
ever reacts to a row being selected in this tab's own table.

Both classes are constructed generically from a `display_columns` spec (see
message_table_model.py), so this module isn't hardcoded to flight/aircraft
data -- it works for any MessageStore built with any column schema.
"""
from pyqtgraph.Qt import QtCore, QtWidgets

from map_utils.message_table_model import MessageTableModel
from map_utils.message_filter_menu import show_filter_menu

Qt = QtCore.Qt

__all__ = ["MessageDetailPanel", "MessagesPanel"]


def _format_field(name, value):
    if isinstance(value, float):
        return f"{value:.2f}"
    return str(value)


class MessageDetailPanel(QtWidgets.QWidget):
    """Selection-driven, table-only -- NOT wired to EarthPlot.entity_selected
    or playhead_time_changed (that's InfoPanel's job)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(260)
        layout = QtWidgets.QVBoxLayout(self)

        self.title_label = QtWidgets.QLabel("No message selected")
        self.title_label.setStyleSheet("font-weight: bold; font-size: 13px;")
        layout.addWidget(self.title_label)

        self.fields_label = QtWidgets.QLabel("")
        self.fields_label.setWordWrap(True)
        self.fields_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.fields_label)

        layout.addWidget(QtWidgets.QLabel("Raw message:"))
        self.raw_label = QtWidgets.QLabel("")
        self.raw_label.setWordWrap(True)
        self.raw_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.raw_label)
        layout.addStretch(1)

    def show_message(self, row_dict):
        if row_dict is None:
            self.title_label.setText("No message selected")
            self.fields_label.setText("")
            self.raw_label.setText("")
            return

        self.title_label.setText(f"Message seq {row_dict['seq']}")
        field_lines = [
            f"{name}: {_format_field(name, value)}"
            for name, value in row_dict.items()
            if name not in ("raw", "seq")
        ]
        self.fields_label.setText("\n".join(field_lines))

        raw = row_dict.get("raw")
        if isinstance(raw, dict):
            raw_lines = [f"{k}: {_format_field(k, v)}" for k, v in raw.items()]
            self.raw_label.setText("\n".join(raw_lines))
        else:
            self.raw_label.setText(str(raw))


_NAV_KEYS = {
    Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right,
    Qt.Key_PageUp, Qt.Key_PageDown, Qt.Key_Home, Qt.Key_End,
}


class MessagesPanel(QtWidgets.QWidget):
    """Live-tail vs. paused/browsing (see message_table_model.py's
    pause()/resume()): while live, new rows keep the view pinned to the
    bottom (_on_rows_inserted). Selecting a row, or manually scrolling away
    from the bottom, pauses the model so the currently-displayed rows hold
    still -- otherwise a selected/scrolled-to row could get evicted or
    re-indexed out from under the user within a few ticks. The status bar's
    Resume button (or scrolling back to the bottom with nothing selected)
    goes back live, resyncing to whatever the store holds at that point.

    Pausing is gated on an actual user gesture -- QAbstractItemView.pressed
    for a row click, QScrollBar.sliderPressed/actionTriggered for a manual
    scroll -- not on *inferring* intent from currentRowChanged/valueChanged.
    Qt can fire both of those on its own (e.g. auto-assigning a current
    index the first time rows populate an empty view, or resetting scroll
    position to the top as part of any model reset), with no reliable way
    to tell those apart from a real click/scroll purely from signal timing.

    Deliberately NOT an eventFilter on the viewport/scrollbar: those receive
    a constant stream of paint/hover/range-update events while live data is
    streaming in, and routing all of that through a Python eventFilter call
    on every single one was measurably enough overhead to stall the app
    after a minute or so of streaming -- the dedicated signals used here
    only fire for genuine user-triggered interaction, so there's no
    per-event tax on the normal live-tail path."""

    # Emitted whenever the selected row changes, in either direction: the
    # selected row's dict (see MessageTableModel.row_dict), or None when the
    # selection is cleared. self.detail already reflects every selection
    # change on its own; this is for an owning app that wants to react too
    # (e.g. syncing selection to some other view) without polling this
    # widget's table/model directly.
    message_selected = QtCore.pyqtSignal(object)

    def __init__(self, message_store, display_columns, parent=None):
        super().__init__(parent)
        self.display_columns = display_columns
        self.model = MessageTableModel(message_store, display_columns)
        self._user_gesture = False

        self.table = QtWidgets.QTableView()
        self.table.setModel(self.model)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().sectionClicked.connect(self._on_header_clicked)
        self.table.selectionModel().currentRowChanged.connect(self._on_row_selected)
        self.table.pressed.connect(self._on_row_pressed)
        self.table.model().rowsInserted.connect(self._on_rows_inserted)
        self.table.verticalScrollBar().valueChanged.connect(self._on_scrolled)
        self.table.verticalScrollBar().sliderPressed.connect(self._mark_user_gesture)
        self.table.verticalScrollBar().actionTriggered.connect(lambda _action: self._mark_user_gesture())
        self.model.pending_changed.connect(lambda _n: self._update_status())
        self.model.modelReset.connect(self._on_model_reset)
        self.table.installEventFilter(self)  # table itself, NOT viewport/scrollbar -- see class docstring

        self.detail = MessageDetailPanel()

        splitter = QtWidgets.QSplitter(Qt.Horizontal)
        splitter.addWidget(self.table)
        splitter.addWidget(self.detail)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        self.status_label = QtWidgets.QLabel("")
        self.resume_button = QtWidgets.QPushButton("Resume")
        self.resume_button.clicked.connect(self._resume)
        self.resume_button.hide()

        status_row = QtWidgets.QHBoxLayout()
        status_row.addWidget(self.status_label)
        status_row.addStretch(1)
        status_row.addWidget(self.resume_button)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(status_row, 0)
        layout.addWidget(splitter, 1)

        self._update_status()

    def _on_header_clicked(self, column_index):
        spec = self.display_columns[column_index]
        current = self.model.get_filter(spec["name"])
        show_filter_menu(
            self.table.horizontalHeader(), column_index, spec, self.model.store, current,
            lambda predicate: self.model.set_filter(spec["name"], predicate),
        )

    def eventFilter(self, obj, event):
        """Only installed on self.table (the frame widget) -- NOT its
        viewport or scrollbar, which is where the per-frame paint/hover/
        range-update traffic lives during live streaming (see class
        docstring). self.table itself only sees rare events (focus,
        resize, keys), so a Python-side filter here is cheap. Used solely
        to catch keyboard row navigation, which has no dedicated Qt signal
        the way a mouse press (pressed) or scrollbar action
        (sliderPressed/actionTriggered) does."""
        if obj is self.table and event.type() == QtCore.QEvent.KeyPress and event.key() in _NAV_KEYS:
            self._mark_user_gesture()
        return super().eventFilter(obj, event)

    def _mark_user_gesture(self):
        self._user_gesture = True
        QtCore.QTimer.singleShot(0, self._clear_user_gesture)

    def _clear_user_gesture(self):
        self._user_gesture = False

    def _on_row_pressed(self, index):
        # QAbstractItemView.pressed -- fires only for a genuine mouse press
        # on a valid row, never programmatically -- so this can pause
        # directly, with no need to go through the gesture-flag dance
        # _on_scrolled/keyboard nav need (see class docstring).
        if index.isValid():
            self.model.pause()
            self._update_status()

    def _on_row_selected(self, current, previous):
        if not current.isValid():
            self.detail.show_message(None)
            self.message_selected.emit(None)
            return
        # Mouse-driven selection is already paused directly by
        # _on_row_pressed; this only needs to catch the keyboard-nav case
        # (eventFilter sets _user_gesture before Qt processes the key,
        # which then triggers this signal synchronously). Anything else
        # landing here (Qt auto-assigning/reassigning a current index on
        # its own) is not a real selection -- see class docstring.
        if self._user_gesture:
            self.model.pause()
            self._update_status()
        row_dict = self.model.row_dict(current.row())
        self.detail.show_message(row_dict)
        self.message_selected.emit(row_dict)

    def _on_rows_inserted(self, parent, first, last):
        # on_batch_added() never emits rowsInserted while paused (batches
        # are just counted -- see MessageTableModel.pending_changed), so
        # this only fires while live.
        self.table.scrollToBottom()

    def _on_model_reset(self):
        # Cosmetic cleanup after resume()/set_filter()/tab-reactivation: no
        # need to guard against this feeding back into a pause -- gating on
        # _user_gesture already means neither the scroll-to-bottom below
        # nor any leftover/reassigned selection Qt produces during the
        # reset can trigger one.
        if not self.model.is_paused():
            self.table.scrollToBottom()
        self.table.clearSelection()

    def _on_scrolled(self, value):
        at_bottom = value >= self.table.verticalScrollBar().maximum()
        if self.model.is_paused():
            has_selection = self.table.selectionModel().currentIndex().isValid()
            if at_bottom and not has_selection:
                self._resume()
        elif not at_bottom and self._user_gesture:
            self.model.pause()
            self._update_status()

    def _resume(self):
        self.table.clearSelection()
        self.model.resume()  # triggers full_reset() -> _on_model_reset() scrolls to bottom
        self._update_status()

    def _update_status(self):
        if self.model.is_paused():
            n = self.model.pending_count()
            suffix = f" -- {n:,} new" if n else ""
            self.status_label.setText(f"Paused{suffix}")
            self.resume_button.show()
        else:
            self.status_label.setText("Live")
            self.resume_button.hide()
