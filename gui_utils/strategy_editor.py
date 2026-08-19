"""
Schema-driven STRATEGY tab widget. Every Trigger/Tactic in
common_sim/control declares a PARAM_SCHEMA tuple (see
common_sim.control.param.Param); this module turns that declaration
into a property-inspector form, so adding a new trigger or tactic to
common_sim makes it available here with zero edits to this file.

Layout: a rule list on the left (checkbox = included on Apply/Save,
drag to reorder = priority -- top of the list is the highest priority,
matching strategy_graph's priority bands), a property inspector on the
right (trigger + tactic + rule timing), a persistent fallback-tactic
editor, and Load / Save / Apply controls -- one in-memory Strategy per
robot, selected by the robot combo at the top.

Stays under the ARCHITECTURE.md contract: this module never imports
game_specific. Region/action/piece-type combo choices come from
whatever FieldConfig is handed to set_field(), read by string name only.
"""
from __future__ import annotations

from pyqtgraph.Qt import QtCore, QtWidgets

from common_sim.control import strategy_io, tactics, triggers
from common_sim.control.strategy import Rule, Strategy
from common_sim.control.tactics import Idle, Tactic
from common_sim.control.triggers import Always, Trigger
from gui_utils import theme

Qt = QtCore.Qt

TRIGGER_TYPES: dict[str, type] = {
    name: cls for name, cls in strategy_io.REGISTRY.items() if issubclass(cls, Trigger)
}
# RunScript is deliberately excluded from the buildable tactic list --
# it wraps a plain Sequence of behavior.py primitives (Wait, RunIntake,
# ...), which have no PARAM_SCHEMA and no GUI authoring story yet. A
# RunScript rule loaded from a file is still shown (see TacticEditor's
# RUNSCRIPT_LABEL handling) and round-trips unchanged; it just can't be
# authored fresh from this editor.
TACTIC_TYPES: dict[str, type] = {
    name: cls for name, cls in strategy_io.REGISTRY.items()
    if issubclass(cls, Tactic) and name != "RunScript"
}
RUNSCRIPT_LABEL = "RunScript (scripted)"

RULE_ROLE = Qt.UserRole


def _label_for(name: str) -> str:
    return name.replace("_", " ").title()


# -- Param -> widget -----------------------------------------------------

class _ChoiceField(QtWidgets.QWidget):
    """`region_name` / `action` / `piece_type` -- a combo populated from
    live field data via `choices_provider(kind)`, plus a leading
    "(any)" entry mapping to None when the param is optional."""

    def __init__(self, param, choices_provider, parent=None):
        super().__init__(parent)
        self._param = param
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.combo = QtWidgets.QComboBox()
        if param.optional:
            self.combo.addItem("(any)", None)
        for choice in choices_provider(param.kind):
            self.combo.addItem(choice, choice)
        layout.addWidget(self.combo)

    def get(self):
        return self.combo.currentData()

    def set(self, value) -> None:
        idx = self.combo.findData(value)
        if idx < 0 and value is not None:
            self.combo.addItem(str(value), value)
            idx = self.combo.count() - 1
        self.combo.setCurrentIndex(max(idx, 0))


class _FixedChoiceField(QtWidgets.QWidget):
    """`choice` -- a combo over the Param's own fixed `choices` tuple."""

    def __init__(self, param, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.combo = QtWidgets.QComboBox()
        if param.optional:
            self.combo.addItem("(any)", None)
        for choice in param.choices:
            self.combo.addItem(str(choice), choice)
        layout.addWidget(self.combo)

    def get(self):
        return self.combo.currentData()

    def set(self, value) -> None:
        idx = self.combo.findData(value)
        self.combo.setCurrentIndex(max(idx, 0))


class _NumberField(QtWidgets.QWidget):
    """`float` / `int`. Optional params get a leading checkbox that
    toggles the spinbox and stands in for None when unchecked."""

    def __init__(self, param, is_int: bool, parent=None):
        super().__init__(parent)
        self._param = param
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.enabled_check = None
        if param.optional:
            self.enabled_check = QtWidgets.QCheckBox()
            self.enabled_check.toggled.connect(self._on_toggled)
            layout.addWidget(self.enabled_check)

        self.spin = QtWidgets.QSpinBox() if is_int else QtWidgets.QDoubleSpinBox()
        lo = param.min if param.min is not None else (-1_000_000 if is_int else -1e6)
        hi = param.max if param.max is not None else (1_000_000 if is_int else 1e6)
        self.spin.setRange(lo, hi)
        if not is_int:
            self.spin.setDecimals(3)
            self.spin.setSingleStep(1.0)
        if param.suffix:
            self.spin.setSuffix(param.suffix)
        if param.default is not None:
            self.spin.setValue(param.default)
        layout.addWidget(self.spin, stretch=1)

        if self.enabled_check is not None:
            self.enabled_check.setChecked(param.default is not None)
            self._on_toggled(self.enabled_check.isChecked())

    def _on_toggled(self, checked: bool) -> None:
        self.spin.setEnabled(checked)

    def get(self):
        if self.enabled_check is not None and not self.enabled_check.isChecked():
            return None
        return self.spin.value()

    def set(self, value) -> None:
        if self.enabled_check is not None:
            self.enabled_check.setChecked(value is not None)
        if value is not None:
            self.spin.setValue(value)


class _BoolField(QtWidgets.QWidget):
    def __init__(self, param, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.check = QtWidgets.QCheckBox()
        self.check.setChecked(bool(param.default))
        layout.addWidget(self.check)
        layout.addStretch(1)

    def get(self) -> bool:
        return self.check.isChecked()

    def set(self, value) -> None:
        self.check.setChecked(bool(value))


class _TextField(QtWidgets.QWidget):
    def __init__(self, param, parent=None):
        super().__init__(parent)
        self._param = param
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.edit = QtWidgets.QLineEdit()
        if param.default:
            self.edit.setText(str(param.default))
        layout.addWidget(self.edit)

    def get(self):
        text = self.edit.text().strip()
        if not text:
            return None if self._param.optional else self._param.default
        return text

    def set(self, value) -> None:
        self.edit.setText("" if value is None else str(value))


class _TriggerRefField(QtWidgets.QGroupBox):
    """`trigger` -- a single nested TriggerEditor (e.g. Not.trigger).
    Never returns None: Not.evaluate asserts a real trigger is present,
    so this defaults to Always() rather than modeling "no trigger"."""

    def __init__(self, choices_provider, parent=None):
        super().__init__("", parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.editor = TriggerEditor(choices_provider)
        layout.addWidget(self.editor)

    def get(self) -> Trigger:
        return self.editor.build()

    def set(self, value) -> None:
        self.editor.load(value if value is not None else Always())


class _TriggerListField(QtWidgets.QGroupBox):
    """`trigger_list` -- a dynamic list of nested TriggerEditors, used
    by AllOf/AnyOf. Each row gets its own remove button; "+ Condition"
    appends a new Always()."""

    def __init__(self, choices_provider, parent=None):
        super().__init__("", parent)
        self._choices_provider = choices_provider
        self._rows_layout = QtWidgets.QVBoxLayout()
        self._editors: list[TriggerEditor] = []

        outer = QtWidgets.QVBoxLayout(self)
        outer.addLayout(self._rows_layout)
        add_button = QtWidgets.QPushButton("+ Condition")
        add_button.clicked.connect(lambda: self._add_row(Always()))
        outer.addWidget(add_button)

    def _add_row(self, trigger: Trigger) -> None:
        row = QtWidgets.QWidget()
        row_layout = QtWidgets.QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        editor = TriggerEditor(self._choices_provider)
        editor.load(trigger)
        row_layout.addWidget(editor, stretch=1)
        remove_button = QtWidgets.QPushButton("✕")
        remove_button.setFixedWidth(24)
        remove_button.clicked.connect(lambda: self._remove_row(row, editor))
        row_layout.addWidget(remove_button, alignment=Qt.AlignTop)
        self._rows_layout.addWidget(row)
        self._editors.append(editor)

    def _remove_row(self, row: QtWidgets.QWidget, editor: "TriggerEditor") -> None:
        self._editors.remove(editor)
        self._rows_layout.removeWidget(row)
        row.setParent(None)
        row.deleteLater()

    def get(self) -> tuple:
        return tuple(editor.build() for editor in self._editors)

    def set(self, value) -> None:
        while self._editors:
            editor = self._editors[0]
            row = editor.parentWidget()
            self._remove_row(row, editor)
        for trigger in value or ():
            self._add_row(trigger)


def make_param_field(param, choices_provider) -> QtWidgets.QWidget:
    if param.kind == "trigger":
        return _TriggerRefField(choices_provider)
    if param.kind == "trigger_list":
        return _TriggerListField(choices_provider)
    if param.kind in ("region_name", "action", "piece_type"):
        return _ChoiceField(param, choices_provider)
    if param.kind == "choice":
        return _FixedChoiceField(param)
    if param.kind == "float":
        return _NumberField(param, is_int=False)
    if param.kind == "int":
        return _NumberField(param, is_int=True)
    if param.kind == "bool":
        return _BoolField(param)
    if param.kind == "str":
        return _TextField(param)
    raise ValueError(f"strategy_editor has no widget for Param kind {param.kind!r}")


class TriggerEditor(QtWidgets.QWidget):
    """One Trigger, fully schema-driven: a type combo plus a form built
    from the selected type's PARAM_SCHEMA (+ the `for_duration`
    hysteresis common to every Trigger). Recurses into itself for
    `trigger` / `trigger_list` params (AllOf/AnyOf/Not)."""

    def __init__(self, choices_provider, parent=None):
        super().__init__(parent)
        self._choices_provider = choices_provider
        self._fields: dict[str, QtWidgets.QWidget] = {}
        self._for_duration_field = None

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        header = QtWidgets.QHBoxLayout()
        header.addWidget(QtWidgets.QLabel("Trigger"))
        self.type_combo = QtWidgets.QComboBox()
        self.type_combo.addItems(sorted(TRIGGER_TYPES))
        header.addWidget(self.type_combo, stretch=1)
        layout.addLayout(header)

        self.form_container = QtWidgets.QWidget()
        self.form_layout = QtWidgets.QFormLayout(self.form_container)
        self.form_layout.setContentsMargins(12, 4, 0, 4)
        layout.addWidget(self.form_container)

        self.type_combo.currentTextChanged.connect(self._on_type_changed)
        self._on_type_changed(self.type_combo.currentText())

    def _clear_form(self) -> None:
        while self.form_layout.rowCount():
            self.form_layout.removeRow(0)
        self._fields.clear()
        self._for_duration_field = None

    def _on_type_changed(self, type_name: str, source_instance=None) -> None:
        self._clear_form()
        cls = TRIGGER_TYPES[type_name]
        for param in cls.PARAM_SCHEMA:
            field = make_param_field(param, self._choices_provider)
            self._fields[param.name] = field
            self.form_layout.addRow(_label_for(param.name), field)

        from common_sim.control.param import Param
        self._for_duration_field = make_param_field(
            Param("for_duration", kind="float", default=None, optional=True, min=0, suffix=" s"),
            self._choices_provider,
        )
        self.form_layout.addRow("For Duration", self._for_duration_field)

        if source_instance is not None and type(source_instance).__name__ == type_name:
            for name, field in self._fields.items():
                field.set(getattr(source_instance, name))
            self._for_duration_field.set(source_instance.for_duration)

    def load(self, trigger: Trigger) -> None:
        type_name = type(trigger).__name__
        if type_name not in TRIGGER_TYPES:
            type_name, trigger = "Always", Always()
        blocked = self.type_combo.blockSignals(True)
        self.type_combo.setCurrentText(type_name)
        self.type_combo.blockSignals(blocked)
        self._on_type_changed(type_name, source_instance=trigger)

    def build(self) -> Trigger:
        cls = TRIGGER_TYPES[self.type_combo.currentText()]
        kwargs = {name: field.get() for name, field in self._fields.items()}
        kwargs["for_duration"] = self._for_duration_field.get()
        return cls(**kwargs)


class TacticEditor(QtWidgets.QWidget):
    """One Tactic, schema-driven like TriggerEditor. A RunScript loaded
    from a file is preserved read-only (see module docstring) via a
    synthetic combo entry rather than being editable field-by-field."""

    def __init__(self, choices_provider, parent=None):
        super().__init__(parent)
        self._choices_provider = choices_provider
        self._fields: dict[str, QtWidgets.QWidget] = {}
        self._runscript = None

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        header = QtWidgets.QHBoxLayout()
        header.addWidget(QtWidgets.QLabel("Tactic"))
        self.type_combo = QtWidgets.QComboBox()
        self.type_combo.addItems(sorted(TACTIC_TYPES))
        header.addWidget(self.type_combo, stretch=1)
        layout.addLayout(header)

        self.form_container = QtWidgets.QWidget()
        self.form_layout = QtWidgets.QFormLayout(self.form_container)
        self.form_layout.setContentsMargins(12, 4, 0, 4)
        layout.addWidget(self.form_container)

        self.note_label = QtWidgets.QLabel("Scripted rule -- edit via the strategy JSON file.")
        self.note_label.setStyleSheet(f"color: {theme.TEXT_DIM};")
        self.note_label.setWordWrap(True)
        self.note_label.setVisible(False)
        layout.addWidget(self.note_label)

        self.type_combo.currentTextChanged.connect(self._on_type_changed)
        self._on_type_changed(self.type_combo.currentText())

    def _clear_form(self) -> None:
        while self.form_layout.rowCount():
            self.form_layout.removeRow(0)
        self._fields.clear()

    def _on_type_changed(self, type_name: str, source_instance=None) -> None:
        if type_name != RUNSCRIPT_LABEL and self.type_combo.itemText(0) == RUNSCRIPT_LABEL:
            self.type_combo.removeItem(0)
            self._runscript = None

        if type_name == RUNSCRIPT_LABEL:
            self._clear_form()
            self.form_container.setVisible(False)
            self.note_label.setVisible(True)
            return

        self.form_container.setVisible(True)
        self.note_label.setVisible(False)
        self._clear_form()
        cls = TACTIC_TYPES[type_name]
        for param in cls.PARAM_SCHEMA:
            field = make_param_field(param, self._choices_provider)
            self._fields[param.name] = field
            self.form_layout.addRow(_label_for(param.name), field)

        if source_instance is not None and type(source_instance).__name__ == type_name:
            for name, field in self._fields.items():
                field.set(getattr(source_instance, name))

    def load(self, tactic: Tactic) -> None:
        if isinstance(tactic, tactics.RunScript):
            self._runscript = tactic
            if self.type_combo.itemText(0) != RUNSCRIPT_LABEL:
                self.type_combo.insertItem(0, RUNSCRIPT_LABEL)
            blocked = self.type_combo.blockSignals(True)
            self.type_combo.setCurrentIndex(0)
            self.type_combo.blockSignals(blocked)
            self._on_type_changed(RUNSCRIPT_LABEL)
            return

        type_name = type(tactic).__name__
        if type_name not in TACTIC_TYPES:
            type_name, tactic = "Idle", Idle()
        if self.type_combo.itemText(0) == RUNSCRIPT_LABEL:
            self.type_combo.removeItem(0)
        blocked = self.type_combo.blockSignals(True)
        self.type_combo.setCurrentText(type_name)
        self.type_combo.blockSignals(blocked)
        self._on_type_changed(type_name, source_instance=tactic)

    def build(self) -> Tactic:
        type_name = self.type_combo.currentText()
        if type_name == RUNSCRIPT_LABEL:
            return self._runscript if self._runscript is not None else tactics.RunScript(children=[])
        cls = TACTIC_TYPES[type_name]
        kwargs = {name: field.get() for name, field in self._fields.items()}
        return cls(**kwargs)


class RuleInspector(QtWidgets.QWidget):
    """Everything about one Rule: its name, trigger, tactic, and the
    arbiter timing knobs (min_duration/cooldown/once). Priority isn't
    edited here -- list order in the owning StrategyEditor *is* the
    priority (see module docstring)."""

    def __init__(self, choices_provider, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)

        meta_form = QtWidgets.QFormLayout()
        self.name_edit = QtWidgets.QLineEdit()
        meta_form.addRow("Name", self.name_edit)
        self.min_duration_spin = QtWidgets.QDoubleSpinBox()
        self.min_duration_spin.setRange(0, 3600)
        self.min_duration_spin.setSuffix(" s")
        meta_form.addRow("Min Duration", self.min_duration_spin)
        self.cooldown_spin = QtWidgets.QDoubleSpinBox()
        self.cooldown_spin.setRange(0, 3600)
        self.cooldown_spin.setSuffix(" s")
        meta_form.addRow("Cooldown", self.cooldown_spin)
        self.once_check = QtWidgets.QCheckBox()
        meta_form.addRow("Once", self.once_check)
        layout.addLayout(meta_form)

        self.trigger_editor = TriggerEditor(choices_provider)
        trigger_box = QtWidgets.QGroupBox("TRIGGER")
        trigger_box_layout = QtWidgets.QVBoxLayout(trigger_box)
        trigger_box_layout.addWidget(self.trigger_editor)
        layout.addWidget(trigger_box)

        self.tactic_editor = TacticEditor(choices_provider)
        tactic_box = QtWidgets.QGroupBox("TACTIC")
        tactic_box_layout = QtWidgets.QVBoxLayout(tactic_box)
        tactic_box_layout.addWidget(self.tactic_editor)
        layout.addWidget(tactic_box)
        layout.addStretch(1)

    def load(self, rule: Rule) -> None:
        self.name_edit.setText(rule.name)
        self.min_duration_spin.setValue(rule.min_duration)
        self.cooldown_spin.setValue(rule.cooldown)
        self.once_check.setChecked(rule.once)
        self.trigger_editor.load(rule.trigger)
        self.tactic_editor.load(rule.tactic)

    def build(self, priority: int = 0) -> Rule:
        return Rule(
            name=self.name_edit.text().strip() or "rule",
            trigger=self.trigger_editor.build(),
            tactic=self.tactic_editor.build(),
            priority=priority,
            min_duration=self.min_duration_spin.value(),
            cooldown=self.cooldown_spin.value(),
            once=self.once_check.isChecked(),
        )


def _field_choices(field_config) -> dict[str, tuple]:
    if field_config is None:
        return {"region_name": (), "action": (), "piece_type": ()}
    region_names = tuple(sorted(r.name for r in field_config.scoring_regions))
    actions = tuple(sorted({a for r in field_config.scoring_regions for a in r.actions}))
    piece_types = set()
    for region in field_config.scoring_regions:
        piece_types.update(region.piece_types)
    for spawn in field_config.spawn_regions:
        piece_types.add(spawn.piece_type)
    for location in field_config.intake_locations:
        piece_types.add(location.piece_type)
    return {"region_name": region_names, "action": actions, "piece_type": tuple(sorted(piece_types))}


class StrategyEditor(QtWidgets.QWidget):
    """The STRATEGY tab: one in-memory Strategy per robot (keyed by the
    label the owning window uses for it, e.g. "PRIMARY" / "BLUE 0"),
    edited through a rule list + property inspector and staged for the
    next Reset via `apply_requested`.

    Robots are added/removed by the owning window (`set_robots`); the
    field is supplied once (`set_field`) to populate region/action/
    piece-type combo boxes from whatever game is loaded, keeping this
    module game-agnostic."""

    apply_requested = QtCore.Signal(str)  # robot label
    changed = QtCore.Signal()  # rule list/fallback edited -- for a live graph preview to resync

    def __init__(self, parent=None):
        super().__init__(parent)
        self._choices: dict[str, tuple] = _field_choices(None)
        self._strategies: dict[str, Strategy] = {}
        self._applied: set[str] = set()
        self._current_label: str | None = None
        self._loading = False
        self._prev_row = -1

        root = QtWidgets.QVBoxLayout(self)

        top_bar = QtWidgets.QHBoxLayout()
        top_bar.addWidget(QtWidgets.QLabel("Robot"))
        self.robot_combo = QtWidgets.QComboBox()
        self.robot_combo.currentTextChanged.connect(self._on_robot_changed)
        top_bar.addWidget(self.robot_combo, stretch=1)
        new_button = QtWidgets.QPushButton("New")
        new_button.clicked.connect(self._new_strategy)
        top_bar.addWidget(new_button)
        load_button = QtWidgets.QPushButton("Load...")
        load_button.clicked.connect(self._load_from_file)
        top_bar.addWidget(load_button)
        save_button = QtWidgets.QPushButton("Save As...")
        save_button.clicked.connect(self._save_to_file)
        top_bar.addWidget(save_button)
        root.addLayout(top_bar)

        splitter = QtWidgets.QSplitter(Qt.Horizontal)
        root.addWidget(splitter, stretch=1)

        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.addWidget(QtWidgets.QLabel("RULES (drag to reorder = priority)"))
        self.rule_list = QtWidgets.QListWidget()
        self.rule_list.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)
        self.rule_list.currentRowChanged.connect(self._on_row_changed)
        self.rule_list.model().rowsMoved.connect(lambda *_: self._on_reordered())
        left_layout.addWidget(self.rule_list, stretch=1)
        rule_buttons = QtWidgets.QHBoxLayout()
        add_rule_button = QtWidgets.QPushButton("+ Rule")
        add_rule_button.clicked.connect(self._add_rule)
        rule_buttons.addWidget(add_rule_button)
        remove_rule_button = QtWidgets.QPushButton("- Rule")
        remove_rule_button.clicked.connect(self._remove_rule)
        rule_buttons.addWidget(remove_rule_button)
        left_layout.addLayout(rule_buttons)
        splitter.addWidget(left)

        right = QtWidgets.QScrollArea()
        right.setWidgetResizable(True)
        right_content = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_content)
        self.rule_inspector = RuleInspector(self._choices_provider)
        self.inspector_box = QtWidgets.QGroupBox("SELECTED RULE")
        inspector_box_layout = QtWidgets.QVBoxLayout(self.inspector_box)
        inspector_box_layout.addWidget(self.rule_inspector)
        right_layout.addWidget(self.inspector_box)

        self.fallback_editor = TacticEditor(self._choices_provider)
        fallback_box = QtWidgets.QGroupBox("FALLBACK TACTIC (no rule satisfied)")
        fallback_box_layout = QtWidgets.QVBoxLayout(fallback_box)
        fallback_box_layout.addWidget(self.fallback_editor)
        right_layout.addWidget(fallback_box)
        right_layout.addStretch(1)
        right.setWidget(right_content)
        splitter.addWidget(right)
        splitter.setSizes([260, 480])

        bottom_bar = QtWidgets.QHBoxLayout()
        self.status_label = QtWidgets.QLabel("")
        self.status_label.setStyleSheet(f"color: {theme.TEXT_DIM};")
        bottom_bar.addWidget(self.status_label, stretch=1)
        apply_button = QtWidgets.QPushButton("APPLY TO ROBOT")
        apply_button.setStyleSheet(f"font-weight: bold; color: {theme.ACCENT_AMBER};")
        apply_button.setToolTip("Stage this robot's edited strategy to be used the next time RESET runs.")
        apply_button.clicked.connect(self._apply)
        bottom_bar.addWidget(apply_button)
        root.addLayout(bottom_bar)

        self._set_inspector_enabled(False)

    def _choices_provider(self, kind: str) -> tuple:
        return self._choices.get(kind, ())

    # -- external API -----------------------------------------------------

    def set_field(self, field_config) -> None:
        self._choices = _field_choices(field_config)

    def set_robots(self, labels: list[str]) -> None:
        """Sync the robot combo (and per-robot Strategy storage) with the
        owning window's current roster. Keeps existing edits for labels
        that persist; drops storage for labels that were removed."""
        self._commit_current()
        for label in list(self._strategies):
            if label not in labels:
                self._strategies.pop(label, None)
                self._applied.discard(label)

        block = self.robot_combo.blockSignals(True)
        current = self.robot_combo.currentText()
        self.robot_combo.clear()
        self.robot_combo.addItems(labels)
        self.robot_combo.blockSignals(block)

        if current in labels:
            self.robot_combo.setCurrentText(current)
            self._load_robot(current)
        elif labels:
            self.robot_combo.setCurrentIndex(0)
            self._load_robot(labels[0])
        else:
            self._current_label = None
            self.rule_list.clear()
            self._set_inspector_enabled(False)

    def strategy_for(self, label: str) -> Strategy | None:
        """The staged (Apply-to-Robot'd) Strategy for `label`, or None
        if the user hasn't applied an edited one -- callers should fall
        back to their own default (e.g. a roster-selected strategy file)
        in that case."""
        if label == self._current_label:
            self._commit_current()
        return self._strategies.get(label) if label in self._applied else None

    def current_label(self) -> str | None:
        return self._current_label

    def current_strategy(self) -> Strategy | None:
        """The currently-displayed robot's Strategy, including
        in-progress edits (same as what Save/Apply would write) -- for
        a live strategy_graph.py panel to mirror as the user edits."""
        return self._build_current()

    def select_rule(self, name: str | None) -> bool:
        """Select the rule list row for `name` (e.g. from a graph node
        click). Returns False without changing selection if no visible
        row has that name -- `name` may be a rule that exists in the
        built Strategy but was unchecked (excluded from the list), or
        the FALLBACK sentinel, which has no row of its own."""
        for row in range(self.rule_list.count()):
            item = self.rule_list.item(row)
            if item.data(RULE_ROLE).name == name:
                self.rule_list.setCurrentRow(row)
                return True
        return False

    # -- robot switching ----------------------------------------------------

    def _on_robot_changed(self, label: str) -> None:
        if self._loading or not label:
            return
        self._load_robot(label)

    def _load_robot(self, label: str) -> None:
        """Switch the editor to `label`, first committing whatever robot
        was previously displayed (so its edits aren't lost). Use
        `_display_strategy` instead when the intent is to *replace* the
        currently-displayed robot's strategy (New/Load) -- committing
        first there would read the stale on-screen state and stomp the
        strategy that's about to be shown."""
        self._commit_current(previous_label=self._current_label)
        strategy = self._strategies.get(label) or Strategy(name=label)
        self._display_strategy(label, strategy)

    def _display_strategy(self, label: str, strategy: Strategy) -> None:
        self._loading = True
        try:
            self._current_label = label
            self._strategies[label] = strategy
            self._populate_rule_list(strategy)
            self.fallback_editor.load(strategy.fallback)
            self._update_status()
        finally:
            self._loading = False
        self.changed.emit()

    def _populate_rule_list(self, strategy: Strategy) -> None:
        self.rule_list.clear()
        for rule in strategy.rules:
            self._add_list_item(rule)
        if self.rule_list.count():
            self.rule_list.setCurrentRow(0)
        else:
            self._set_inspector_enabled(False)

    def _add_list_item(self, rule: Rule) -> QtWidgets.QListWidgetItem:
        item = QtWidgets.QListWidgetItem(f"{rule.name}  [{type(rule.tactic).__name__}]")
        item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
        item.setCheckState(Qt.Checked)
        item.setData(RULE_ROLE, rule)
        self.rule_list.addItem(item)
        return item

    # -- rule list editing --------------------------------------------------

    def _on_row_changed(self, row: int) -> None:
        # Only skip the *pre-switch commit* while a bulk load
        # (_display_strategy) is driving selection -- there's nothing
        # valid to commit yet, and committing here is what previously
        # stomped a just-loaded rule with stale inspector state. Always
        # still load the newly-selected row into the inspector, even
        # during a bulk load, or the auto-selected first row (set by
        # _populate_rule_list) would be left showing whatever the
        # inspector last displayed.
        if not self._loading:
            # currentRow() already reflects the *new* selection by the
            # time this signal fires, so committing against it would
            # write the inspector's still-stale (previous-rule) widget
            # state onto the newly-selected row instead of the row that
            # state actually belongs to. Commit against the row we
            # tracked before the selection changed.
            self._commit_selected_rule(self._prev_row)
        self._prev_row = row
        if row < 0:
            self._set_inspector_enabled(False)
            return
        self._set_inspector_enabled(True)
        item = self.rule_list.item(row)
        self.rule_inspector.load(item.data(RULE_ROLE))
        if not self._loading:
            self.changed.emit()

    def _commit_selected_rule(self, row: int | None = None) -> None:
        """Read the inspector's current widget state back into `row`'s
        Rule (defaults to the current row). Also called from
        `_commit_current` (a pure read, e.g. for Save/Apply/the live
        graph preview) -- so this must NOT emit `changed` itself, or
        those reads would re-enter `changed`'s handlers (which may call
        right back into `_commit_current`) and recurse forever. Genuine
        user edits emit `changed` from their own entry point
        (_on_row_changed, _add_rule, _remove_rule, _on_reordered)
        instead."""
        if row is None:
            row = self.rule_list.currentRow()
        if row < 0 or row >= self.rule_list.count():
            return
        item = self.rule_list.item(row)
        rule = self.rule_inspector.build(priority=item.data(RULE_ROLE).priority)
        item.setData(RULE_ROLE, rule)
        item.setText(f"{rule.name}  [{type(rule.tactic).__name__}]")

    def _on_reordered(self) -> None:
        # InternalMove drag already moved the QListWidgetItem (and its
        # RULE_ROLE payload) for us -- nothing to resync here beyond
        # what strategy_for() recomputes from list order on demand.
        self.changed.emit()

    def _add_rule(self) -> None:
        # _loading brackets the selection change so _on_row_changed
        # skips its pre-switch commit (there's nothing stale to save --
        # we already committed the old row above) but still loads the
        # inspector for the new row. Without this, setCurrentItem's
        # currentRowChanged fires with the new (blank) row already
        # current but the inspector still showing the *old* row, and
        # the commit that follows overwrites the new rule with a copy
        # of the old one instead of the fresh Always()/Idle() just built.
        self._commit_selected_rule()
        n = self.rule_list.count() + 1
        rule = Rule(name=f"rule_{n}", trigger=Always(), tactic=Idle())
        self._loading = True
        try:
            item = self._add_list_item(rule)
            self.rule_list.setCurrentItem(item)
        finally:
            self._loading = False
        self.changed.emit()

    def _remove_rule(self) -> None:
        row = self.rule_list.currentRow()
        if row < 0:
            return
        # Same _loading guard as _add_rule: takeItem's auto-reselection
        # of the next row would otherwise commit the inspector's
        # (about-to-be-stale) state for the removed row onto whatever
        # rule ends up selected next -- _on_row_changed still loads the
        # inspector for that new selection regardless of `_loading`.
        self._loading = True
        try:
            self.rule_list.takeItem(row)
        finally:
            self._loading = False
        self.changed.emit()

    def _set_inspector_enabled(self, enabled: bool) -> None:
        self.inspector_box.setEnabled(enabled)
        if not enabled:
            self.inspector_box.setTitle("SELECTED RULE (none -- add or select one)")
        else:
            self.inspector_box.setTitle("SELECTED RULE")

    # -- committing / building -----------------------------------------------

    def _commit_current(self, previous_label: str | None = None) -> None:
        label = previous_label if previous_label is not None else self._current_label
        if label is None:
            return
        self._commit_selected_rule()
        rules = []
        for row in range(self.rule_list.count()):
            item = self.rule_list.item(row)
            if item.checkState() != Qt.Checked:
                continue
            rule = item.data(RULE_ROLE)
            rule.priority = self.rule_list.count() - row
            rules.append(rule)
        fallback = self.fallback_editor.build()
        self._strategies[label] = Strategy(name=label, rules=rules, fallback=fallback)

    def _build_current(self) -> Strategy | None:
        if self._current_label is None:
            return None
        self._commit_current()
        return self._strategies[self._current_label]

    # -- toolbar actions ------------------------------------------------

    def _new_strategy(self) -> None:
        if self._current_label is None:
            return
        self._applied.discard(self._current_label)
        self._display_strategy(self._current_label, Strategy(name=self._current_label))
        self._update_status()

    def _load_from_file(self) -> None:
        if self._current_label is None:
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load Strategy", "", "Strategy JSON (*.json)")
        if not path:
            return
        # A hand-edited strategy file is the expected way to get a load
        # error, and this runs in a Qt slot -- an exception escaping here
        # unwinds into the event loop, where the traceback goes to a
        # console the user may not even have open. strategy_io's message
        # already names the file, the location inside it, and the valid
        # vocabulary, so showing it is the whole fix.
        try:
            strategy = strategy_io.load_strategy(path)
        except strategy_io.StrategyLoadError as exc:
            QtWidgets.QMessageBox.warning(self, "Could not load strategy", str(exc))
            self.status_label.setText(f"Failed to load {path}")
            return
        self._display_strategy(self._current_label, strategy)
        self.status_label.setText(f"Loaded {path}")

    def _save_to_file(self) -> None:
        strategy = self._build_current()
        if strategy is None:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Strategy", "", "Strategy JSON (*.json)")
        if not path:
            return
        strategy_io.save_strategy(strategy, path)
        self.status_label.setText(f"Saved {path}")

    def _apply(self) -> None:
        if self._current_label is None:
            return
        self._build_current()
        self._applied.add(self._current_label)
        self._update_status()
        self.apply_requested.emit(self._current_label)

    def _update_status(self) -> None:
        if self._current_label is None:
            self.status_label.setText("")
            return
        state = "applied -- used on next RESET" if self._current_label in self._applied else "edited, not yet applied"
        self.status_label.setText(f"{self._current_label}: {state}")
