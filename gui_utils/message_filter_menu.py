"""Generic Excel-style header-click filter popups for MessageTableModel.

Two widgets, dispatched on a column's declared "filter" kind (see
message_table_model.py's `display_columns` spec):

- ColumnFilterMenu ("categorical"): a search box + checkable list of
  distinct values seen so far (store.distinct_values(name)), with
  Select all / Clear / Apply. Calls back with a `set` of allowed values, or
  None if everything is checked (== no filter).
- TimeRangeFilterMenu ("range"): a from/to pair (QDateTimeEdit for a
  datetime column, QDoubleSpinBox for a numeric one -- picked from the
  bounds' own type) defaulted to the column's current min/max
  (store.column_bounds(name)). Calls back with a (lo, hi) tuple, or None if
  cleared.

Neither widget is specific to any one app's columns -- both are constructed
from whatever `MessageTableModel`/`MessageStore` hand them (distinct values
or bounds for an arbitrary column name), so the same two classes serve any
table built on this module.

Shown the same way `overlay_panel.ComboControl._show_menu` shows its own
popup: `menu.exec_(widget.mapToGlobal(...))`, positioned by the caller at
the clicked header section's rect.
"""
from datetime import datetime

from pyqtgraph.Qt import QtCore, QtWidgets

Qt = QtCore.Qt
QDateTime = QtCore.QDateTime

__all__ = ["ColumnFilterMenu", "TimeRangeFilterMenu", "show_filter_menu", "header_section_popup_pos"]


class ColumnFilterMenu(QtWidgets.QMenu):
    def __init__(self, values, selected, on_apply, parent=None):
        """
        values: distinct values to offer as checkboxes (already capped --
            see telemetry_store.DISTINCT_VALUES_MAX).
        selected: the currently-active predicate -- a set of allowed
            values, or None meaning "no filter" (== everything checked).
        on_apply(predicate_or_None): called with a set of checked values,
            or None if every value ended up checked (equivalent to no filter).
        """
        super().__init__(parent)
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)

        search = QtWidgets.QLineEdit()
        search.setPlaceholderText("Search…")
        layout.addWidget(search)

        self._list = QtWidgets.QListWidget()
        self._list.setMinimumWidth(180)
        for value in values:
            item = QtWidgets.QListWidgetItem(str(value))
            item.setData(Qt.UserRole, value)
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(
                Qt.Checked if selected is None or value in selected else Qt.Unchecked
            )
            self._list.addItem(item)
        layout.addWidget(self._list)

        search.textChanged.connect(self._apply_search)

        btn_row = QtWidgets.QHBoxLayout()
        select_all_btn = QtWidgets.QPushButton("Select all")
        clear_btn = QtWidgets.QPushButton("Clear")
        apply_btn = QtWidgets.QPushButton("Apply")
        for b in (select_all_btn, clear_btn, apply_btn):
            btn_row.addWidget(b)
        layout.addLayout(btn_row)

        select_all_btn.clicked.connect(lambda: self._set_visible(Qt.Checked))
        clear_btn.clicked.connect(lambda: self._set_visible(Qt.Unchecked))
        apply_btn.clicked.connect(lambda: self._apply(on_apply))

        action = QtWidgets.QWidgetAction(self)
        action.setDefaultWidget(container)
        self.addAction(action)

    def _apply_search(self, text):
        text = text.lower()
        for i in range(self._list.count()):
            item = self._list.item(i)
            item.setHidden(text not in item.text().lower())

    def _set_visible(self, state):
        for i in range(self._list.count()):
            item = self._list.item(i)
            if not item.isHidden():
                item.setCheckState(state)

    def _apply(self, on_apply):
        checked = set()
        for i in range(self._list.count()):
            item = self._list.item(i)
            if item.checkState() == Qt.Checked:
                checked.add(item.data(Qt.UserRole))
        predicate = None if len(checked) == self._list.count() else checked
        on_apply(predicate)
        self.close()


class TimeRangeFilterMenu(QtWidgets.QMenu):
    def __init__(self, bounds, selected, on_apply, parent=None):
        """
        bounds: (lo, hi) currently held for the column, or None if the
            store is empty. Used as the popup's default from/to values.
        selected: currently-active (lo, hi) predicate, or None.
        on_apply((lo, hi)_or_None): called with the chosen range, or None
            if cleared.
        """
        super().__init__(parent)
        container = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(container)

        lo_default, hi_default = selected if selected is not None else (bounds or (0.0, 0.0))
        is_datetime = isinstance(lo_default, datetime)

        layout.addWidget(QtWidgets.QLabel("From:"))
        self._from_field = self._make_field(lo_default, is_datetime)
        layout.addWidget(self._from_field)
        layout.addWidget(QtWidgets.QLabel("To:"))
        self._to_field = self._make_field(hi_default, is_datetime)
        layout.addWidget(self._to_field)

        btn_row = QtWidgets.QHBoxLayout()
        clear_btn = QtWidgets.QPushButton("Clear")
        apply_btn = QtWidgets.QPushButton("Apply")
        btn_row.addWidget(clear_btn)
        btn_row.addWidget(apply_btn)
        layout.addLayout(btn_row)

        clear_btn.clicked.connect(lambda: self._apply_clear(on_apply))
        apply_btn.clicked.connect(lambda: self._apply(on_apply, is_datetime))

        action = QtWidgets.QWidgetAction(self)
        action.setDefaultWidget(container)
        self.addAction(action)

    @staticmethod
    def _make_field(value, is_datetime):
        if is_datetime:
            field = QtWidgets.QDateTimeEdit()
            field.setDisplayFormat("HH:mm:ss.zzz")
            field.setDateTime(QDateTime(value.year, value.month, value.day,
                                         value.hour, value.minute, value.second,
                                         value.microsecond // 1000))
            return field
        field = QtWidgets.QDoubleSpinBox()
        field.setRange(-1e18, 1e18)
        field.setDecimals(3)
        field.setValue(float(value))
        return field

    def _apply(self, on_apply, is_datetime):
        if is_datetime:
            lo = self._from_field.dateTime().toPyDateTime()
            hi = self._to_field.dateTime().toPyDateTime()
        else:
            lo = self._from_field.value()
            hi = self._to_field.value()
        if lo > hi:
            # An inverted range would otherwise match nothing (mask ends up
            # (col >= lo) & (col <= hi) with lo > hi) with no feedback to
            # the user that their filter is a no-op -- swap instead.
            lo, hi = hi, lo
        on_apply((lo, hi))
        self.close()

    def _apply_clear(self, on_apply):
        on_apply(None)
        self.close()


def header_section_popup_pos(header_widget, column_index):
    """Global position just below the given header section -- shared by
    show_filter_menu and entity_table.EntityTable._on_header_clicked (which
    can't call show_filter_menu directly: it builds its menus from raw
    columnar data rather than a store object), so the two don't drift apart
    on how a header-click popup gets positioned."""
    rect = header_widget.sectionViewportPosition(column_index)
    return header_widget.mapToGlobal(QtCore.QPoint(rect, header_widget.height()))


def show_filter_menu(header_widget, column_index, spec, store, current_predicate, on_apply):
    """Build and pop up the right menu kind for `spec` ("categorical" /
    "range" / anything else -> no-op), positioned under the clicked header
    section. Call from a QHeaderView.sectionClicked handler."""
    kind = spec.get("filter")
    if kind is None:
        return
    pos = header_section_popup_pos(header_widget, column_index)
    if kind == "categorical":
        menu = ColumnFilterMenu(store.distinct_values(spec["name"]), current_predicate, on_apply)
    elif kind == "range":
        menu = TimeRangeFilterMenu(store.column_bounds(spec["name"]), current_predicate, on_apply)
    else:
        return
    menu.exec_(pos)
