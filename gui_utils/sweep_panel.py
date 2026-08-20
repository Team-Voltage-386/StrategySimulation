"""
Generic (game-agnostic) SWEEP-tab widgets: the variable table, the run
control panel, the results table, the plots pane, and the QThread
plumbing that drives a sweep without blocking the Qt event loop. Built
from the FieldDescriptors handed in -- same schema-driven style as
gui_utils/strategy_editor.py -- so this file never imports
game_specific and a new RobotCharacteristics field needs zero edits
here (apps/sweep_tab.py supplies the descriptors and the trial
function).
"""
from __future__ import annotations

import random

from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

import matplotlib
matplotlib.use("Qt5Agg")  # after pyqtgraph.Qt import, so matplotlib binds to the already-loaded PyQt5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT
from matplotlib.figure import Figure

from common_sim.analysis.runner import CancelToken, default_worker_count, iter_results
from common_sim.analysis.sweep_spec import NumericSampling, SweepVariable
from common_sim.analysis.variability import VariabilityModel
from gui_utils import theme
from gui_utils.doc_tags import document
from gui_utils.sweep_plots import apply_dark_style, render_sweep

RESULT_METRIC_COLUMNS = (
    "total_score", "score_blue", "score_red", "pieces_scored", "pieces_deposited", "misses", "mean_cycle_time",
)
AMBER_RUN_THRESHOLD = 500
RED_RUN_THRESHOLD = 5000
FLUSH_INTERVAL_MS = 100    # 10 Hz results-model flush
PLOT_REDRAW_INTERVAL_MS = 1000  # 1 Hz plot refresh


# -- variable table --------------------------------------------------------

class VariableRow(QtWidgets.QWidget):
    """[x] enable | TARGET | PROPERTY | MODE (Uniform/List) | MIN | MAX |
    POINTS, or a checkable list of choices for a categorical property
    (e.g. "strategy"). The preview label under the row shows the
    resolved values in display units -- the highest-value usability
    affordance in the whole tab."""

    changed = QtCore.Signal()
    remove_requested = QtCore.Signal(object)  # self

    def __init__(self, targets, descriptors_for_target, parent=None):
        super().__init__(parent)
        self._descriptors_for_target = descriptors_for_target
        self._descriptor = None

        row = QtWidgets.QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)

        def _caption(text: str) -> QtWidgets.QLabel:
            label = QtWidgets.QLabel(text)
            label.setStyleSheet(f"color: {theme.TEXT_DIM};")
            row.addWidget(label)
            return label

        self.enable_check = QtWidgets.QCheckBox()
        self.enable_check.setChecked(True)
        self.enable_check.toggled.connect(self._on_changed)
        row.addWidget(self.enable_check)

        _caption("Target")
        self.target_combo = QtWidgets.QComboBox()
        self.target_combo.addItems(targets)
        self.target_combo.currentTextChanged.connect(self._on_target_changed)
        row.addWidget(self.target_combo)

        _caption("Property")
        self.property_combo = QtWidgets.QComboBox()
        self.property_combo.currentIndexChanged.connect(self._on_property_changed)
        row.addWidget(self.property_combo, stretch=1)

        self.mode_label = _caption("Mode")
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["Uniform", "List"])
        self.mode_combo.currentTextChanged.connect(self._on_mode_changed)
        row.addWidget(self.mode_combo)

        self.min_label = _caption("Min")
        self.min_spin = QtWidgets.QDoubleSpinBox()
        self.min_spin.setRange(-1e6, 1e6)
        self.min_spin.setDecimals(3)
        self.min_spin.valueChanged.connect(self._on_changed)
        row.addWidget(self.min_spin)

        self.max_label = _caption("Max")
        self.max_spin = QtWidgets.QDoubleSpinBox()
        self.max_spin.setRange(-1e6, 1e6)
        self.max_spin.setDecimals(3)
        self.max_spin.valueChanged.connect(self._on_changed)
        row.addWidget(self.max_spin)

        self.points_label = _caption("Points")
        self.points_spin = QtWidgets.QSpinBox()
        self.points_spin.setRange(1, 200)
        self.points_spin.setValue(3)
        self.points_spin.valueChanged.connect(self._on_changed)
        row.addWidget(self.points_spin)

        self.list_label = _caption("Values")
        self.list_edit = QtWidgets.QLineEdit()
        self.list_edit.setPlaceholderText("comma-separated values")
        self.list_edit.textChanged.connect(self._on_changed)
        row.addWidget(self.list_edit, stretch=1)

        self.choices_label = _caption("Choices")
        self.choices_list = QtWidgets.QListWidget()
        self.choices_list.setFixedHeight(64)
        self.choices_list.itemChanged.connect(self._on_changed)
        row.addWidget(self.choices_list, stretch=1)

        remove_button = QtWidgets.QPushButton("✕")
        remove_button.setFixedSize(28, 28)
        remove_button.setToolTip("Remove this variable")
        remove_font = remove_button.font()
        remove_font.setPointSize(remove_font.pointSize() + 3)
        remove_font.setBold(True)
        remove_button.setFont(remove_font)
        remove_button.setStyleSheet(f"color: {theme.ACCENT_RED};")
        remove_button.clicked.connect(lambda: self.remove_requested.emit(self))
        row.addWidget(remove_button)

        self.preview_label = QtWidgets.QLabel()
        self.preview_label.setStyleSheet(f"color: {theme.TEXT_DIM};")
        self.preview_label.setWordWrap(True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(row)
        layout.addWidget(self.preview_label)

        self._refresh_properties()

    # -- target/property wiring ------------------------------------------

    def _on_target_changed(self, *_args) -> None:
        self._refresh_properties()
        self._on_changed()

    def _refresh_properties(self) -> None:
        target = self.target_combo.currentText()
        descriptors = self._descriptors_for_target(target) if target else ()
        self.property_combo.blockSignals(True)
        self.property_combo.clear()
        for descriptor in descriptors:
            self.property_combo.addItem(descriptor.label or descriptor.path, descriptor)
        self.property_combo.blockSignals(False)
        self._on_property_changed()

    def _on_property_changed(self, *_args) -> None:
        self._descriptor = self.property_combo.currentData()
        is_categorical = self._descriptor is not None and self._descriptor.kind == "categorical"
        uniform = not is_categorical and self.mode_combo.currentText() == "Uniform"
        as_list = not is_categorical and self.mode_combo.currentText() == "List"

        self.mode_combo.setVisible(not is_categorical)
        self.mode_label.setVisible(not is_categorical)
        self.min_spin.setVisible(uniform)
        self.min_label.setVisible(uniform)
        self.max_spin.setVisible(uniform)
        self.max_label.setVisible(uniform)
        self.points_spin.setVisible(uniform)
        self.points_label.setVisible(uniform)
        self.list_edit.setVisible(as_list)
        self.list_label.setVisible(as_list)
        self.choices_list.setVisible(is_categorical)
        self.choices_label.setVisible(is_categorical)

        if is_categorical:
            self.choices_list.blockSignals(True)
            self.choices_list.clear()
            for choice in self._descriptor.choices:
                item = QtWidgets.QListWidgetItem(str(choice))
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                item.setCheckState(QtCore.Qt.Checked)
                self.choices_list.addItem(item)
            self.choices_list.blockSignals(False)
        elif self._descriptor is not None and isinstance(self._descriptor.default, (int, float)):
            self.min_spin.setSuffix(self._descriptor.suffix)
            self.max_spin.setSuffix(self._descriptor.suffix)
            display_default = self._descriptor.default * self._descriptor.display_scale
            self.min_spin.setValue(display_default)
            self.max_spin.setValue(display_default)

        self._on_changed()

    def _on_mode_changed(self, *_args) -> None:
        self._on_property_changed()

    def _on_changed(self, *_args) -> None:
        self._update_preview()
        self.changed.emit()

    # -- resolved variable -------------------------------------------------

    def variable(self):
        """The SweepVariable this row describes, or None if disabled,
        invalid (min > max, no points, no categorical choice checked),
        or not yet configured."""
        if not self.enable_check.isChecked() or self._descriptor is None:
            return None
        target = self.target_combo.currentText()
        path = self._descriptor.path

        if self._descriptor.kind == "categorical":
            values = tuple(
                self.choices_list.item(i).text()
                for i in range(self.choices_list.count())
                if self.choices_list.item(i).checkState() == QtCore.Qt.Checked
            )
            return SweepVariable(target=target, path=path, values=values) if values else None

        scale = self._descriptor.display_scale or 1.0
        cast = int if self._descriptor.kind == "int" else float

        if self.mode_combo.currentText() == "List":
            values = self._parse_list_values(scale, cast)
            return SweepVariable(target=target, path=path, values=values) if values else None

        minimum = self.min_spin.value() / scale
        maximum = self.max_spin.value() / scale
        count = self.points_spin.value()
        if minimum > maximum or count < 1:
            return None
        raw_values = NumericSampling(minimum, maximum, count).values()
        values = tuple(int(round(v)) if cast is int else v for v in raw_values)
        return SweepVariable(target=target, path=path, values=values)

    def _parse_list_values(self, scale: float, cast) -> tuple:
        values = []
        for token in self.list_edit.text().split(","):
            token = token.strip()
            if not token:
                continue
            try:
                raw = float(token) / scale
            except ValueError:
                continue
            values.append(int(round(raw)) if cast is int else raw)
        return tuple(values)

    def is_valid(self) -> bool:
        return True if not self.enable_check.isChecked() else self.variable() is not None

    def _update_preview(self) -> None:
        if self._descriptor is None:
            self.preview_label.setText("")
            return
        var = self.variable()
        if var is None:
            self.preview_label.setText("(disabled)" if not self.enable_check.isChecked() else "invalid range")
            return
        if self._descriptor.kind == "categorical":
            self.preview_label.setText(", ".join(str(v) for v in var.values))
            return
        scale = self._descriptor.display_scale or 1.0
        suffix = self._descriptor.suffix
        self.preview_label.setText(", ".join(f"{v * scale:g}{suffix}" for v in var.values))


class VariableTable(QtWidgets.QWidget):
    """Add/remove VariableRows. `targets_provider() -> list[str]` and
    `descriptors_for_target(label) -> tuple[FieldDescriptor, ...]` are
    supplied by apps/sweep_tab.py, which is the only layer that knows
    about REEFSCAPE robots -- this widget stays entirely generic."""

    changed = QtCore.Signal()

    def __init__(self, targets_provider, descriptors_for_target, parent=None):
        super().__init__(parent)
        self._targets_provider = targets_provider
        self._descriptors_for_target = descriptors_for_target
        self._rows: list[VariableRow] = []

        self.rows_layout = QtWidgets.QVBoxLayout()
        add_button = document(
            QtWidgets.QPushButton("+ ADD VARIABLE"), "add_variable", "Add variable",
            "Adds a row for one thing to vary across the sweep -- a robot's characteristic "
            "(like max speed) or its strategy -- and every combination gets simulated.",
            "Add two variables and you get a grid: every value of the first crossed with every "
            "value of the second. That's how a sweep answers \"what does speed vs. strategy "
            "look like\" in one run instead of one match at a time.")
        add_button.clicked.connect(self.add_row)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(self.rows_layout)
        layout.addWidget(add_button)

    def add_row(self) -> None:
        row = VariableRow(self._targets_provider(), self._descriptors_for_target)
        row.changed.connect(self.changed)
        row.remove_requested.connect(self._remove_row)
        self.rows_layout.addWidget(row)
        self._rows.append(row)
        self.changed.emit()

    def _remove_row(self, row: VariableRow) -> None:
        self._rows.remove(row)
        row.setParent(None)
        row.deleteLater()
        self.changed.emit()

    def refresh_targets(self) -> None:
        """Called when the roster changes (a robot added/removed) so
        each row's TARGET combo and, if it was pointed at the changed
        robot, its PROPERTY combo stay current."""
        targets = self._targets_provider()
        for row in self._rows:
            current = row.target_combo.currentText()
            row.target_combo.blockSignals(True)
            row.target_combo.clear()
            row.target_combo.addItems(targets)
            if current in targets:
                row.target_combo.setCurrentText(current)
            row.target_combo.blockSignals(False)
            row._refresh_properties()
        self.changed.emit()

    def variables(self) -> list:
        return [v for row in self._rows for v in (row.variable(),) if v is not None]

    def is_valid(self) -> bool:
        return all(row.is_valid() for row in self._rows)


# -- run control -------------------------------------------------------

class SweepControlPanel(QtWidgets.QWidget):
    """Repetitions, base seed (+ randomize), worker count, a VARIABILITY
    group bound 1:1 to VariabilityModel's fields behind an enable
    checkbox (laid out several-per-line to stay compact), and a single
    run row with EXECUTE/ABORT on the left, the progress bar in the
    middle, and the TOTAL RUNS readout on the right, plus a status
    line."""

    execute_clicked = QtCore.Signal()
    abort_clicked = QtCore.Signal()
    changed = QtCore.Signal()

    _VARIABILITY_FIELDS = (
        ("intake_time_pct", "Intake Time", " σ"),
        ("deposit_time_pct", "Deposit Time", " σ"),
        ("max_speed_pct", "Max Speed", " σ"),
        ("max_accel_pct", "Max Accel", " σ"),
        ("start_pose_xy_in", "Start Pose XY", " in"),
        ("start_pose_heading_deg", "Start Heading", " deg"),
        ("piece_scatter_in", "Piece Scatter", " in"),
    )

    def __init__(self, parent=None):
        super().__init__(parent)
        form = QtWidgets.QFormLayout()

        self.repetitions_spin = document(
            QtWidgets.QSpinBox(), "repetitions", "Repetitions",
            "How many times each point in the sweep grid gets re-run with a different seed.",
            "One repetition tells you what happened once. More repetitions tell you what "
            "usually happens -- and with Enable Variability off, every repetition of the same "
            "point is the identical match, so this setting does nothing until that's checked.")
        self.repetitions_spin.setRange(1, 1000)
        self.repetitions_spin.setValue(1)
        self.repetitions_spin.valueChanged.connect(self.changed)
        form.addRow("Repetitions", self.repetitions_spin)

        seed_row = QtWidgets.QHBoxLayout()
        self.seed_spin = document(
            QtWidgets.QSpinBox(), "base_seed", "Base seed",
            "The starting seed for the sweep's randomness. Repetitions count up from here, so "
            "the same base seed reproduces the exact same set of matches.",
            "Keep this fixed while you compare two designs -- both should see the same seeds, "
            "or a difference in score might just be a difference in luck.")
        self.seed_spin.setRange(0, 2_000_000_000)
        self.seed_spin.valueChanged.connect(self.changed)
        seed_row.addWidget(self.seed_spin)
        randomize_button = document(
            QtWidgets.QPushButton("Randomize"), "randomize_seed", "Randomize",
            "Picks a fresh base seed at random -- for when you specifically want a set of "
            "matches nobody has looked at yet.")
        randomize_button.clicked.connect(self._randomize_seed)
        seed_row.addWidget(randomize_button)
        form.addRow("Base Seed", seed_row)

        self.worker_spin = document(
            QtWidgets.QSpinBox(), "workers", "Workers",
            "How many matches run at the same time, across CPU cores. Defaults to what this "
            "machine has.",
            "Turn it down to keep using the computer for something else while a sweep runs.")
        self.worker_spin.setRange(1, 64)
        self.worker_spin.setValue(default_worker_count())
        self.worker_spin.valueChanged.connect(self.changed)
        form.addRow("Workers", self.worker_spin)

        self.variability_check = document(
            QtWidgets.QCheckBox("Enable variability"), "variability", "Enable variability",
            "Turns on random perturbation of robot characteristics, start positions and piece "
            "scatter, so repeated matches aren't identical copies of each other.",
            "This has to be on for Repetitions to mean anything -- see the amber warning below "
            "if it isn't.")
        self.variability_check.toggled.connect(self.changed)
        form.addRow(self.variability_check)

        # Several label+spin pairs per line (rather than one per QFormLayout
        # row) so the variability block doesn't dominate the panel's height.
        variability_grid = QtWidgets.QGridLayout()
        variability_grid.setHorizontalSpacing(12)
        FIELDS_PER_ROW = 3
        self.variability_spins: dict = {}
        for i, (field_name, label, suffix) in enumerate(self._VARIABILITY_FIELDS):
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(0.0, 100.0)
            spin.setSuffix(suffix)
            spin.valueChanged.connect(self.changed)
            self.variability_spins[field_name] = spin

            row, col = divmod(i, FIELDS_PER_ROW)
            caption = QtWidgets.QLabel(label)
            caption.setStyleSheet(f"color: {theme.TEXT_DIM};")
            variability_grid.addWidget(caption, row, col * 2)
            variability_grid.addWidget(spin, row, col * 2 + 1)
        form.addRow(variability_grid)

        self.variability_hint = QtWidgets.QLabel(
            "Repetitions > 1 with variability disabled always gives identical rows."
        )
        self.variability_hint.setStyleSheet(f"color: {theme.ACCENT_AMBER};")
        self.variability_hint.setWordWrap(True)
        self.variability_hint.setVisible(False)
        form.addRow(self.variability_hint)

        # EXECUTE/ABORT to the left, progress bar in the middle, and the
        # runs readout to the right, all on one line -- this frees the
        # vertical space that stacking them separately used to cost.
        run_row = QtWidgets.QHBoxLayout()
        self.execute_button = document(
            QtWidgets.QPushButton("EXECUTE"), "execute", "Execute",
            "Runs every match in the sweep grid: every combination of variable values, times "
            "Repetitions.",
            "The window stays usable while it runs. Above 500 matches the readout turns amber; "
            "above 5000 it turns red and asks you to confirm before starting.")
        self.execute_button.clicked.connect(self.execute_clicked)
        self.abort_button = document(
            QtWidgets.QPushButton("ABORT"), "abort", "Abort",
            "Stops the sweep. Matches already running when you click it still finish -- they "
            "can't be interrupted mid-flight -- but no new ones start.")
        self.abort_button.setEnabled(False)
        self.abort_button.clicked.connect(self.abort_clicked)
        run_row.addWidget(self.execute_button)
        run_row.addWidget(self.abort_button)

        self.progress_bar = QtWidgets.QProgressBar()
        run_row.addWidget(self.progress_bar, stretch=1)

        self.total_runs_label = document(
            QtWidgets.QLabel("0 runs"), "total_runs", "Total runs",
            "How many matches this sweep will run, and about how long that takes on this "
            "machine -- updates live as you add variables or change repetitions.",
            "Colour is a warning level: white is normal, amber is a lot, red is a great deal. "
            "Check this before EXECUTE, not after.")
        self.total_runs_label.setFont(theme.technical_font(13, bold=True))
        run_row.addWidget(self.total_runs_label)
        form.addRow(run_row)

        self.status_label = QtWidgets.QLabel("")
        self.status_label.setWordWrap(True)
        form.addRow(self.status_label)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(form)

        self._total_valid = True

    def _randomize_seed(self) -> None:
        self.seed_spin.setValue(random.randint(0, 2_000_000_000))

    def repetitions(self) -> int:
        return self.repetitions_spin.value()

    def base_seed(self) -> int:
        return self.seed_spin.value()

    def worker_count(self) -> int:
        return self.worker_spin.value()

    def variability_model(self) -> VariabilityModel:
        kwargs = {name: spin.value() for name, spin in self.variability_spins.items()}
        return VariabilityModel(enabled=self.variability_check.isChecked(), **kwargs)

    def set_total_runs(self, total: int, eta_s: float | None = None) -> None:
        eta_text = f"  (~{eta_s:.0f}s on {self.worker_count()} workers)" if eta_s else ""
        self.total_runs_label.setText(f"{total} runs{eta_text}")
        if total > RED_RUN_THRESHOLD:
            color = theme.ACCENT_RED
        elif total > AMBER_RUN_THRESHOLD:
            color = theme.ACCENT_AMBER
        else:
            color = theme.TEXT_PRIMARY
        self.total_runs_label.setStyleSheet(f"color: {color};")
        self.variability_hint.setVisible(self.repetitions() > 1 and not self.variability_check.isChecked())

    def set_running(self, running: bool) -> None:
        self.execute_button.setEnabled(not running and self._total_valid)
        self.abort_button.setEnabled(running)

    def set_execute_enabled(self, enabled: bool) -> None:
        self._total_valid = enabled
        if not self.abort_button.isEnabled():
            self.execute_button.setEnabled(enabled)

    def set_progress(self, completed: int, total: int) -> None:
        self.progress_bar.setMaximum(max(1, total))
        self.progress_bar.setValue(completed)

    def set_status(self, text: str) -> None:
        self.status_label.setText(text)


# -- results table -----------------------------------------------------

class SweepResultsModel(QtCore.QAbstractTableModel):
    """A widget-per-cell QTableWidget is unusable at 5000 rows --
    QAbstractTableModel + append_batch()'s single begin/endInsertRows
    keeps a big sweep's results table responsive."""

    def __init__(self, variable_columns=(), parent=None):
        super().__init__(parent)
        self._variable_columns: list[str] = []
        self._rows: list = []
        self._headers: list[str] = []
        self.set_columns(variable_columns)

    def set_columns(self, variable_columns) -> None:
        self.beginResetModel()
        self._variable_columns = list(variable_columns)
        self._headers = (
            ["#"] + self._variable_columns + ["seed"] + list(RESULT_METRIC_COLUMNS) + ["status"]
        )
        self._rows = []
        self.endResetModel()

    def rowCount(self, parent=QtCore.QModelIndex()) -> int:
        return 0 if parent.isValid() else len(self._rows)

    def columnCount(self, parent=QtCore.QModelIndex()) -> int:
        return 0 if parent.isValid() else len(self._headers)

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if role != QtCore.Qt.DisplayRole or orientation != QtCore.Qt.Horizontal:
            return None
        return self._headers[section]

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid():
            return None
        outcome = self._rows[index.row()]
        column = self._headers[index.column()]
        if role == QtCore.Qt.DisplayRole:
            return self._cell_text(outcome, column)
        if role == QtCore.Qt.ForegroundRole and outcome.error is not None:
            return QtGui.QColor(theme.ACCENT_RED)
        if role == QtCore.Qt.ToolTipRole and outcome.error is not None:
            return outcome.error
        return None

    def _cell_text(self, outcome, column: str) -> str:
        if column == "#":
            return str(outcome.index)
        if column == "seed":
            return str(outcome.seed)
        if column == "status":
            return "ERROR" if outcome.error is not None else "ok"
        if column in self._variable_columns:
            value = outcome.params.get(column)
            return "" if value is None else str(value)
        if outcome.metrics is None:
            return ""
        m = outcome.metrics
        if column == "total_score":
            return f"{sum(m.final_scores.values()):.1f}"
        if column == "score_blue":
            return f"{m.final_scores.get('blue', 0):.1f}"
        if column == "score_red":
            return f"{m.final_scores.get('red', 0):.1f}"
        if column == "pieces_scored":
            return str(m.pieces_scored)
        if column == "pieces_deposited":
            return str(m.pieces_deposited)
        if column == "misses":
            return str(m.misses)
        if column == "mean_cycle_time":
            return f"{m.mean_cycle_time:.2f}" if m.mean_cycle_time is not None else "--"
        return ""

    def append_batch(self, outcomes: list) -> None:
        if not outcomes:
            return
        start = len(self._rows)
        self.beginInsertRows(QtCore.QModelIndex(), start, start + len(outcomes) - 1)
        self._rows.extend(outcomes)
        self.endInsertRows()

    def clear(self) -> None:
        self.beginResetModel()
        self._rows = []
        self.endResetModel()

    def successful_outcomes(self) -> list:
        return [o for o in self._rows if o.error is None]

    def outcome_at(self, row: int):
        return self._rows[row]


class SweepResultsTable(QtWidgets.QTableView):
    """replay_requested(job_index) fires on double-click AND on an
    explicit "REPLAY IN MATCH TAB" context-menu action -- a non-engineer
    will not guess that double-clicking a row does anything."""

    replay_requested = QtCore.Signal(int)

    def __init__(self, model: SweepResultsModel, parent=None):
        super().__init__(parent)
        self._results_model = model
        self.setModel(model)
        self.setSelectionBehavior(QtWidgets.QTableView.SelectRows)
        self.horizontalHeader().setStretchLastSection(True)
        document(
            self, "results_table", "Results table",
            "One row per match: the values it was run with, its final score and other "
            "metrics, and whether it errored.",
            "Double-click a row -- or right-click and choose REPLAY IN MATCH TAB -- to watch "
            "that exact match again on the MATCH tab. A row in red is a match that crashed; the "
            "tooltip on it has the error.")
        self.doubleClicked.connect(lambda index: self._emit_replay(index.row()))
        self.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self._on_context_menu)

    def _on_context_menu(self, pos) -> None:
        index = self.indexAt(pos)
        if not index.isValid():
            return
        menu = QtWidgets.QMenu(self)
        action = menu.addAction("REPLAY IN MATCH TAB")
        action.triggered.connect(lambda: self._emit_replay(index.row()))
        menu.exec_(self.viewport().mapToGlobal(pos))

    def _emit_replay(self, row: int) -> None:
        if row < 0:
            return
        self.replay_requested.emit(self._results_model.outcome_at(row).index)


# -- plots ---------------------------------------------------------------

class SweepPlotPanel(QtWidgets.QWidget):
    """FigureCanvasQTAgg + toolbar, with METRIC / X / Y / FACET combos
    populated from the swept columns. Redraws on combo change and on
    each results batch, throttled to 1 Hz so live plotting doesn't
    starve the results table."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.figure = Figure()
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.toolbar = NavigationToolbar2QT(self.canvas, self)

        self.metric_combo = QtWidgets.QComboBox()
        self.metric_combo.addItems(list(RESULT_METRIC_COLUMNS))
        self.metric_combo.currentTextChanged.connect(self.request_redraw)

        self.x_combo = QtWidgets.QComboBox()
        self.y_combo = QtWidgets.QComboBox()
        self.facet_combo = QtWidgets.QComboBox()
        for combo in (self.x_combo, self.y_combo, self.facet_combo):
            combo.currentTextChanged.connect(self.request_redraw)

        controls = QtWidgets.QHBoxLayout()
        for label, combo in (
            ("Metric", self.metric_combo), ("X", self.x_combo), ("Y", self.y_combo), ("Facet", self.facet_combo),
        ):
            controls.addWidget(QtWidgets.QLabel(label))
            controls.addWidget(combo)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addLayout(controls)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas, stretch=1)

        document(
            self, "plot_panel", "Plots",
            "Charts the sweep's results: pick a Metric for the vertical axis and which swept "
            "variable goes on X (and optionally Y for a heatmap, or Facet to split into a grid "
            "of small charts).",
            "With zero swept variables this just shows the score's spread as a histogram. Add "
            "one variable and it becomes a line; add two and X/Y becomes a heatmap.")

        self._df_provider = None
        self._columns: list[str] = []

        self._redraw_timer = QtCore.QTimer(self)
        self._redraw_timer.setSingleShot(True)
        self._redraw_timer.setInterval(PLOT_REDRAW_INTERVAL_MS)
        self._redraw_timer.timeout.connect(self._do_redraw)

    def set_data_provider(self, provider) -> None:
        """`provider() -> pandas.DataFrame | None` -- called lazily on
        each (throttled) redraw, never cached here."""
        self._df_provider = provider

    def set_columns(self, columns) -> None:
        self._columns = list(columns)
        for combo in (self.x_combo, self.y_combo, self.facet_combo):
            current = combo.currentText()
            combo.blockSignals(True)
            combo.clear()
            combo.addItem("")
            combo.addItems(self._columns)
            if current in self._columns:
                combo.setCurrentText(current)
            combo.blockSignals(False)
        if self._columns:
            self.x_combo.setCurrentText(self._columns[0])
        if len(self._columns) > 1:
            self.y_combo.setCurrentText(self._columns[1])
        if len(self._columns) > 2:
            self.facet_combo.setCurrentText(self._columns[2])
        self.request_redraw()

    def request_redraw(self, *_args) -> None:
        if not self._redraw_timer.isActive():
            self._redraw_timer.start()

    def _do_redraw(self) -> None:
        self.figure.clear()
        if self._df_provider is not None:
            df = self._df_provider()
            columns = [c for c in (self.x_combo.currentText(), self.y_combo.currentText(), self.facet_combo.currentText()) if c]
            if df is not None and not df.empty and columns:
                try:
                    render_sweep(
                        self.figure, df, columns[:3], metric=self.metric_combo.currentText() or "total_score",
                        facet_column=self.facet_combo.currentText() or None,
                    )
                except (ValueError, KeyError):
                    pass
        apply_dark_style(self.figure)
        self.canvas.draw_idle()


# -- threading -------------------------------------------------------------

class SweepWorker(QtCore.QObject):
    """Lives on a QThread; touches NO Qt widgets. `trial_fn` and `jobs`
    cross into this worker as plain data/callables -- gui_utils never
    learns what a REEFSCAPE trial actually is."""

    progress = QtCore.Signal(int, int)       # completed, total
    outcome_ready = QtCore.Signal(object)    # TrialOutcome
    finished = QtCore.Signal(bool, str)

    def __init__(self, trial_fn, jobs: list, *, parallel: bool, max_workers: int, cancel_token: CancelToken):
        super().__init__()
        self._trial_fn = trial_fn
        self._jobs = jobs
        self._parallel = parallel
        self._max_workers = max_workers
        self._cancel_token = cancel_token

    def run(self) -> None:
        total = len(self._jobs)
        completed = 0
        try:
            for _index, outcome in iter_results(
                self._trial_fn, self._jobs, parallel=self._parallel,
                max_workers=self._max_workers, cancel=self._cancel_token,
            ):
                completed += 1
                self.outcome_ready.emit(outcome)
                self.progress.emit(completed, total)
        except Exception as exc:  # a bad ProcessPoolExecutor setup, not a per-trial failure (run_trial catches those)
            self.finished.emit(False, f"Sweep failed: {exc}")
            return

        if self._cancel_token.cancelled:
            self.finished.emit(True, f"Aborted after {completed}/{total} runs. Already-running trials cannot be interrupted.")
        else:
            self.finished.emit(True, f"Completed {completed}/{total} runs.")

    def request_abort(self) -> None:
        self._cancel_token.cancel()


class SweepRunController(QtCore.QObject):
    """Owns the QThread lifecycle. Buffers outcomes and flushes them
    into the results model on a main-thread QTimer at 10 Hz -- emitting
    5000 individual row-insert signals into a live QTableView is what
    makes a big sweep feel like a hang."""

    batch_ready = QtCore.Signal(list)
    progress = QtCore.Signal(int, int)
    finished = QtCore.Signal(bool, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread: QtCore.QThread | None = None
        self._worker: SweepWorker | None = None
        self._buffer: list = []
        self._flush_timer = QtCore.QTimer(self)
        self._flush_timer.setInterval(FLUSH_INTERVAL_MS)
        self._flush_timer.timeout.connect(self._flush)

    def is_running(self) -> bool:
        return self._thread is not None

    def start(self, trial_fn, jobs: list, *, parallel: bool = True, max_workers: int | None = None) -> None:
        cancel_token = CancelToken()
        self._thread = QtCore.QThread()
        self._worker = SweepWorker(trial_fn, jobs, parallel=parallel, max_workers=max_workers, cancel_token=cancel_token)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.outcome_ready.connect(self._on_outcome)
        self._worker.progress.connect(self.progress)
        self._worker.finished.connect(self._on_finished)
        self._buffer = []
        self._flush_timer.start()
        self._thread.start()

    def _on_outcome(self, outcome) -> None:
        self._buffer.append(outcome)

    def _flush(self) -> None:
        if self._buffer:
            batch, self._buffer = self._buffer, []
            self.batch_ready.emit(batch)

    def _on_finished(self, success: bool, message: str) -> None:
        self._flush_timer.stop()
        self._flush()
        self.finished.emit(success, message)
        if self._thread is not None:
            self._thread.quit()
            self._thread.wait()
        self._thread = None
        self._worker = None

    def abort(self) -> None:
        if self._worker is not None:
            self._worker.request_abort()

    def shutdown(self) -> None:
        """Abort and join the QThread -- called from
        ReefscapeWindow.closeEvent so a live ProcessPoolExecutor doesn't
        keep the process alive after the window closes."""
        self.abort()
        if self._thread is not None:
            self._thread.quit()
            self._thread.wait()
        self._thread = None
        self._worker = None
