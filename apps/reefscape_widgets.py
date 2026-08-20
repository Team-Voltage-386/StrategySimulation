"""
Shared REEFSCAPE robot-config Qt widgets -- extracted from
apps/run_reefscape.py so the MATCH tab and the SWEEP tab
(apps/sweep_tab.py) can reuse the exact same classes and
spec-building code without importing each other's tab. A QWidget has
exactly one parent, so the two tabs cannot *share widget instances*
(each builds its own RobotRosterConfigPanel) -- what they share is the
class definitions and, critically, `RobotRosterConfigPanel.robot_specs()`,
so a config that scores X in MATCH scores X in a sweep. Pure move from
run_reefscape.py, no behavior change; run_reefscape.py re-imports these
names so existing references there still resolve.
"""
from __future__ import annotations

import math
from pathlib import Path

from pyqtgraph.Qt import QtCore, QtWidgets

from common_sim.analysis.sweep_spec import RobotSpec, characteristics_to_spec
from common_sim.match.match import Match, MatchConfig
from common_sim.robot.characteristics import INTAKE_SOURCES, RobotCharacteristics, SideManipulators, SIDES
from game_specific.reefscape.field import CORAL_MARK_ALGAE_OFFSET, build_field, coral_mark_positions
from game_specific.reefscape.game_pieces import ALGAE_TYPE, CORAL_TYPE, spawn_algae, spawn_coral
from game_specific.reefscape.scoring import REEFSCAPE_SCORING_RULES
from gui_utils import theme
from gui_utils.doc_tags import document

Qt = QtCore.Qt

# Unit conversions for the GUI's display/entry fields -- RobotCharacteristics
# and everything else in common_sim stays in inches/radians/seconds; only
# what the user types/reads in RobotSettingsPanel is metric/degrees.
M_TO_IN = 39.3701
RAD_TO_DEG = 180.0 / math.pi

DEPOSIT_ACTIONS = (("l1", "L1"), ("l2", "L2"), ("l3", "L3"), ("l4", "L4"), ("processor", "PROCESSOR"), ("net", "NET"))
DEFAULT_DEPOSIT_TIMES = {"l1": 0.3, "l2": 0.6, "l3": 1.0, "l4": 1.8, "processor": 0.4, "net": 1.2}
DEFAULT_INTAKE_TIMES = {CORAL_TYPE: 0.4, ALGAE_TYPE: 0.4}
DEFAULT_PIECE_CAPACITY = {CORAL_TYPE: 1, ALGAE_TYPE: 1}
# 100% (deterministic scoring, the legacy behavior) for both piece types.
DEFAULT_SCORING_RELIABILITY = {CORAL_TYPE: 1.0, ALGAE_TYPE: 1.0}

PIECE_TYPES = (CORAL_TYPE, ALGAE_TYPE)
# Default side layout, matching the example from the feature request: a
# back coral intake, front coral scoring, and a right-side algae intake +
# score -- gives a non-empty, visually distinct starting point rather than
# every checkbox unchecked.
DEFAULT_SIDE_CHECKS = {
    ("back", "in", CORAL_TYPE), ("front", "out", CORAL_TYPE),
    ("right", "in", ALGAE_TYPE), ("right", "out", ALGAE_TYPE),
}

STRATEGIES_DIR = Path(__file__).resolve().parent.parent / "game_specific" / "reefscape" / "strategies"
STRATEGY_FILES: dict[str, Path] = {p.stem: p for p in sorted(STRATEGIES_DIR.glob("*.json"))}
STRATEGY_NAMES = list(STRATEGY_FILES.keys())

# Per-path (display_scale, suffix) hints for sweep_spec.sweepable_fields --
# matches the metric/degree units RobotSettingsPanel displays.
UNIT_HINTS = {
    "max_speed": (1 / M_TO_IN, " m/s"),
    "max_accel": (1 / M_TO_IN, " m/s²"),
    "max_angular_speed": (RAD_TO_DEG, " deg/s"),
    "max_angular_accel": (RAD_TO_DEG, " deg/s²"),
    "width": (1 / M_TO_IN, " m"),
    "length": (1 / M_TO_IN, " m"),
}

# Human-readable names for the SWEEP tab's variable picker -- distinguishes
# names that otherwise collide once RobotCharacteristics' dict-field prefix
# is stripped: "intake_time_by_type.coral" (picking a piece up off the
# field) vs "station_intake_time" (picking one up from the human-player
# station) both read as just "coral"/"intake time" without this. See
# sweepable_fields()'s docstring for the "{type}" placeholder convention.
LABEL_HINTS = {
    "intake_time": "Field Intake Time (Default)",
    "intake_time_by_type": "Field Intake Time ({type})",
    "station_intake_time": "Station Intake Time (Coral)",
    "deposit_time": "Deposit Time (Default)",
    "deposit_time_by_action": "Deposit Time ({type})",
    "piece_capacity_by_type": "Piece Capacity ({type})",
    "scoring_reliability_by_type": "Scoring Reliability ({type})",
}


def build_demo_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="demo",
        max_speed=150.0, max_accel=400.0, max_angular_speed=6.0, max_angular_accel=20.0,
        width=28.0, length=28.0,
        piece_capacity_by_type=dict(DEFAULT_PIECE_CAPACITY),
        # Game Manual (V13) 6.3.4.1.B: up to 1 CORAL may be preloaded in
        # each ROBOT by its DRIVE TEAM.
        starting_piece_count=1, preload_piece_type=CORAL_TYPE,
        intake_time_by_type=dict(DEFAULT_INTAKE_TIMES), station_intake_time=0.6, intake_range=6.0,
        deposit_time=0.5,
        deposit_time_by_action=dict(DEFAULT_DEPOSIT_TIMES),
        accepted_piece_types=frozenset({"coral", "algae"}),
        scoring_reliability_by_type=dict(DEFAULT_SCORING_RELIABILITY),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def build_demo_match(disable_friendly_collisions: bool = False, emit_coral_to_field: bool = False) -> Match:
    field = build_field()
    match = Match(
        field,
        REEFSCAPE_SCORING_RULES,
        MatchConfig(
            auto_duration=15, teleop_duration=135,
            disable_friendly_collisions=disable_friendly_collisions, emit_coral_to_field=emit_coral_to_field,
        ),
    )

    # Game Manual (V13) 6.3.4: 1 CORAL staged on each of 3 CORAL MARKs
    # per alliance, with 1 ALGAE placed on top of each -- staged for both
    # alliances, not just whichever one the primary robot belongs to.
    for side in ("blue", "red"):
        for x, y in coral_mark_positions(side):
            spawn_coral(match, (x, y))
            spawn_algae(match, (x, y + CORAL_MARK_ALGAE_OFFSET))

    return match


class RobotSettingsPanel(QtWidgets.QGroupBox):
    """Robot characteristics + alliance selection. Values here only take
    effect once the transport bar's RESET button rebuilds the match --
    tuning a live robot's chassis limits mid-flight isn't a real
    scenario this sim needs to model.

    Displayed/entered in metric (m/s, m/s², deg/s) even though
    RobotCharacteristics itself stays in inches/radians internally --
    characteristics_overrides() does the conversion.

    show_alliance is False for roster (AI) robots -- their alliance is
    already fixed by which roster box (BLUE/RED) they were added to, so
    a second alliance picker in their own tab would be redundant and
    could be set inconsistently with their roster box."""

    def __init__(self, parent=None, show_alliance: bool = True):
        super().__init__("ROBOT SETTINGS", parent)
        form = QtWidgets.QFormLayout(self)

        self.alliance_combo = QtWidgets.QComboBox()
        self.alliance_combo.addItems(["Blue", "Red"])
        if show_alliance:
            form.addRow("Alliance", self.alliance_combo)

        self.max_speed_spin = QtWidgets.QDoubleSpinBox()
        self.max_speed_spin.setRange(0.5, 8.0)
        self.max_speed_spin.setSingleStep(0.1)
        self.max_speed_spin.setSuffix(" m/s")
        self.max_speed_spin.setValue(round(150.0 / M_TO_IN, 2))
        form.addRow("Max Speed", self.max_speed_spin)

        self.max_accel_spin = QtWidgets.QDoubleSpinBox()
        self.max_accel_spin.setRange(1.0, 25.0)
        self.max_accel_spin.setSingleStep(0.5)
        self.max_accel_spin.setSuffix(" m/s²")
        self.max_accel_spin.setValue(round(400.0 / M_TO_IN, 2))
        form.addRow("Max Accel", self.max_accel_spin)

        self.max_angular_speed_spin = QtWidgets.QDoubleSpinBox()
        self.max_angular_speed_spin.setRange(30.0, 1080.0)
        self.max_angular_speed_spin.setSingleStep(10.0)
        self.max_angular_speed_spin.setSuffix(" deg/s")
        self.max_angular_speed_spin.setValue(round(6.0 * RAD_TO_DEG, 1))
        form.addRow("Max Rotation Speed", self.max_angular_speed_spin)

        self.coral_capacity_spin = QtWidgets.QSpinBox()
        self.coral_capacity_spin.setRange(0, 5)
        self.coral_capacity_spin.setValue(DEFAULT_PIECE_CAPACITY[CORAL_TYPE])
        form.addRow("Coral Capacity", self.coral_capacity_spin)

        self.algae_capacity_spin = QtWidgets.QSpinBox()
        self.algae_capacity_spin.setRange(0, 5)
        self.algae_capacity_spin.setValue(DEFAULT_PIECE_CAPACITY[ALGAE_TYPE])
        form.addRow("Algae Capacity", self.algae_capacity_spin)

        self.coral_intake_time_spin = QtWidgets.QDoubleSpinBox()
        self.coral_intake_time_spin.setRange(0.05, 3.0)
        self.coral_intake_time_spin.setSingleStep(0.05)
        self.coral_intake_time_spin.setSuffix(" s")
        self.coral_intake_time_spin.setValue(DEFAULT_INTAKE_TIMES[CORAL_TYPE])
        form.addRow("Coral Intake Time", self.coral_intake_time_spin)

        self.algae_intake_time_spin = QtWidgets.QDoubleSpinBox()
        self.algae_intake_time_spin.setRange(0.05, 3.0)
        self.algae_intake_time_spin.setSingleStep(0.05)
        self.algae_intake_time_spin.setSuffix(" s")
        self.algae_intake_time_spin.setValue(DEFAULT_INTAKE_TIMES[ALGAE_TYPE])
        form.addRow("Algae Intake Time", self.algae_intake_time_spin)

        self.coral_reliability_spin = QtWidgets.QDoubleSpinBox()
        self.coral_reliability_spin.setRange(0.0, 100.0)
        self.coral_reliability_spin.setSingleStep(1.0)
        self.coral_reliability_spin.setSuffix(" %")
        self.coral_reliability_spin.setValue(DEFAULT_SCORING_RELIABILITY[CORAL_TYPE] * 100.0)
        form.addRow("Coral Scoring Reliability", self.coral_reliability_spin)

        self.algae_reliability_spin = QtWidgets.QDoubleSpinBox()
        self.algae_reliability_spin.setRange(0.0, 100.0)
        self.algae_reliability_spin.setSingleStep(1.0)
        self.algae_reliability_spin.setSuffix(" %")
        self.algae_reliability_spin.setValue(DEFAULT_SCORING_RELIABILITY[ALGAE_TYPE] * 100.0)
        form.addRow("Algae Scoring Reliability", self.algae_reliability_spin)

        hint = QtWidgets.QLabel("Changes apply on RESET (below field).")
        hint.setStyleSheet(f"color: {theme.TEXT_DIM};")
        hint.setWordWrap(True)
        form.addRow(hint)

        document(
            self, "robot_settings", "Robot settings",
            "This robot's chassis limits and piece handling: how fast it drives and turns, how "
            "much it can carry, how long picking up or scoring a piece takes, and how often "
            "scoring actually succeeds.",
            "These are what make one design different from another in the simulator -- the same "
            "strategy run by a fast, reliable robot and a slow, fumbling one produces very "
            "different matches. Nothing here takes effect until RESET.")

    def alliance(self) -> str:
        return self.alliance_combo.currentText().lower()

    def characteristics_overrides(self) -> dict:
        return dict(
            max_speed=self.max_speed_spin.value() * M_TO_IN,
            max_accel=self.max_accel_spin.value() * M_TO_IN,
            max_angular_speed=self.max_angular_speed_spin.value() / RAD_TO_DEG,
            piece_capacity_by_type={
                CORAL_TYPE: self.coral_capacity_spin.value(),
                ALGAE_TYPE: self.algae_capacity_spin.value(),
            },
            intake_time_by_type={
                CORAL_TYPE: self.coral_intake_time_spin.value(),
                ALGAE_TYPE: self.algae_intake_time_spin.value(),
            },
            scoring_reliability_by_type={
                CORAL_TYPE: self.coral_reliability_spin.value() / 100.0,
                ALGAE_TYPE: self.algae_reliability_spin.value() / 100.0,
            },
        )


class TimingPanel(QtWidgets.QGroupBox):
    """Per-scoring-action deposit time, one spinbox per REEFSCAPE action
    (REEF L1-L4, PROCESSOR, NET) -- feeds
    RobotCharacteristics.deposit_time_by_action. Like RobotSettingsPanel,
    changes apply on RESET."""

    def __init__(self, parent=None):
        super().__init__("TIMING", parent)
        form = QtWidgets.QFormLayout(self)

        self._deposit_spins: dict[str, QtWidgets.QDoubleSpinBox] = {}
        for action, label in DEPOSIT_ACTIONS:
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(0.05, 5.0)
            spin.setSingleStep(0.05)
            spin.setSuffix(" s")
            spin.setValue(DEFAULT_DEPOSIT_TIMES[action])
            form.addRow(f"{label} Deposit Time", spin)
            self._deposit_spins[action] = spin

        hint = QtWidgets.QLabel("Changes apply on RESET (below field).")
        hint.setStyleSheet(f"color: {theme.TEXT_DIM};")
        hint.setWordWrap(True)
        form.addRow(hint)

        document(
            self, "timing", "Scoring timing",
            "How many seconds this robot spends actually depositing a piece at each scoring "
            "location, from L1 up to the net.",
            "Higher reef levels and the net take longer in real life, and the defaults reflect "
            "that -- a robot that can reach L4 quickly is a real design advantage this number "
            "captures.")

    def deposit_time_by_action(self) -> dict[str, float]:
        return {action: spin.value() for action, spin in self._deposit_spins.items()}


class SideManipulatorPanel(QtWidgets.QGroupBox):
    """Which physical side(s) of the robot have an intake/scoring
    manipulator for each REEFSCAPE piece type -- one checkbox per
    (side, in/out, piece type) combination. Feeds
    RobotCharacteristics.side_manipulators, which gates both the physical
    intake sensor geometry and which edge a completed deposit ejects
    from (see common_sim/robot/robot.py); FieldCanvas draws a badge on
    the robot for each checked box. Like RobotSettingsPanel, changes
    apply on RESET."""

    def __init__(self, parent=None):
        super().__init__("MANIPULATORS", parent)
        grid = QtWidgets.QGridLayout(self)
        grid.setHorizontalSpacing(8)

        columns = [(piece_type, mode) for piece_type in PIECE_TYPES for mode in ("in", "out")]
        for col, (piece_type, mode) in enumerate(columns, start=1):
            label = QtWidgets.QLabel(f"{piece_type[:1].upper()} {mode.upper()}")
            label.setFont(theme.technical_font(8, bold=True))
            label.setAlignment(QtCore.Qt.AlignCenter)
            grid.addWidget(label, 0, col)
        source_col = len(columns) + 1
        source_label = QtWidgets.QLabel("IN SOURCE")
        source_label.setFont(theme.technical_font(8, bold=True))
        source_label.setAlignment(QtCore.Qt.AlignCenter)
        grid.addWidget(source_label, 0, source_col)

        self._checks: dict[tuple[str, str, str], QtWidgets.QCheckBox] = {}
        self._source_combos: dict[str, QtWidgets.QComboBox] = {}
        for row, side in enumerate(SIDES, start=1):
            grid.addWidget(QtWidgets.QLabel(side.upper()), row, 0)
            for col, (piece_type, mode) in enumerate(columns, start=1):
                box = QtWidgets.QCheckBox()
                box.setChecked((side, mode, piece_type) in DEFAULT_SIDE_CHECKS)
                grid.addWidget(box, row, col, alignment=QtCore.Qt.AlignCenter)
                self._checks[(side, mode, piece_type)] = box
            # Where this side's intake (if any) can actually pick a piece
            # up from -- a loose piece on the field, a human-player
            # collection region, or both. One combo per side rather than
            # per piece type since a physical mechanism's placement (floor
            # intake vs. wall-height station scoop) doesn't usually vary
            # by what it's grabbing.
            combo = QtWidgets.QComboBox()
            combo.addItems([s.capitalize() for s in INTAKE_SOURCES])
            combo.setCurrentText("Both")
            grid.addWidget(combo, row, source_col)
            self._source_combos[side] = combo

        hint = QtWidgets.QLabel("Changes apply on RESET (below field).")
        hint.setStyleSheet(f"color: {theme.TEXT_DIM};")
        hint.setWordWrap(True)
        grid.addWidget(hint, len(SIDES) + 1, 0, 1, source_col + 1)

        document(
            self, "manipulators", "Manipulators",
            "Which physical sides of the robot can pick up or score each piece type, and where "
            "an intaking side is allowed to grab a piece from.",
            "A checkbox here is a claim about the real robot's mechanisms -- a front coral "
            "intake, a back algae scorer, whatever your build actually has. FieldCanvas draws a "
            "small badge on the robot for every box checked, so you can sanity-check this "
            "against a screenshot of the field.")

    def side_manipulators(self) -> dict[str, SideManipulators]:
        """Always returns one entry per SIDES member (even if empty),
        never an empty dict -- an empty dict would be indistinguishable
        from "not configured" and fall back to RobotCharacteristics'
        legacy single-front layout instead of the intended "no
        manipulators anywhere" state a user could deliberately choose."""
        result = {}
        for side in SIDES:
            intake_types = frozenset(
                pt for pt in PIECE_TYPES if self._checks[(side, "in", pt)].isChecked()
            )
            score_types = frozenset(
                pt for pt in PIECE_TYPES if self._checks[(side, "out", pt)].isChecked()
            )
            intake_source = self._source_combos[side].currentText().lower()
            result[side] = SideManipulators(
                intake_piece_types=intake_types, score_piece_types=score_types, intake_source=intake_source,
            )
        return result


class RobotConfigTab(QtWidgets.QWidget):
    """One robot's full config -- chassis settings, manipulators, and
    per-action deposit timing -- bundled as a single scrollable tab page.
    One of these exists per robot in the match (the primary robot plus
    every AI roster entry), hosted inside ReefscapeWindow's ROBOT CONFIG
    tab widget."""

    def __init__(self, show_alliance: bool = True, parent=None):
        super().__init__(parent)
        self.settings_panel = RobotSettingsPanel(show_alliance=show_alliance)
        self.manipulator_panel = SideManipulatorPanel()
        self.timing_panel = TimingPanel()

        content = QtWidgets.QWidget()
        content_layout = QtWidgets.QVBoxLayout(content)
        content_layout.addWidget(self.settings_panel)
        content_layout.addWidget(self.manipulator_panel)
        content_layout.addWidget(self.timing_panel)
        content_layout.addStretch(1)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(content)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(scroll)

    def characteristics_overrides(self) -> dict:
        overrides = self.settings_panel.characteristics_overrides()
        overrides["side_manipulators"] = self.manipulator_panel.side_manipulators()
        overrides["deposit_time_by_action"] = self.timing_panel.deposit_time_by_action()
        return overrides


class CollapsibleBox(QtWidgets.QWidget):
    """A titled section that can be collapsed to just its header --
    used to keep ROBOT CONFIG's per-robot tabs from permanently eating
    left-column space once the roster grows."""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.toggle_button = QtWidgets.QToolButton(text=f" {title}")
        self.toggle_button.setCheckable(True)
        self.toggle_button.setChecked(True)
        self.toggle_button.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.toggle_button.setArrowType(Qt.DownArrow)
        self.toggle_button.setStyleSheet("QToolButton { border: none; font-weight: bold; }")
        self.toggle_button.toggled.connect(self._on_toggled)

        self.content = QtWidgets.QWidget()

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.toggle_button)
        layout.addWidget(self.content)

    def setContentLayout(self, content_layout) -> None:
        # QWidget can only own one layout; reparent the placeholder's
        # away first so Qt allows the real one to be installed.
        old_layout = self.content.layout()
        if old_layout is not None:
            QtWidgets.QWidget().setLayout(old_layout)
        self.content.setLayout(content_layout)

    def _on_toggled(self, checked: bool) -> None:
        self.content.setVisible(checked)
        self.toggle_button.setArrowType(Qt.DownArrow if checked else Qt.RightArrow)


class MatchSettingsPanel(QtWidgets.QGroupBox):
    """Match-wide toggles that apply to the whole field regardless of
    roster: whether same-alliance robots collide with each other, and
    whether each alliance's coral emitter is active."""

    def __init__(self, parent=None):
        super().__init__("MATCH SETTINGS", parent)
        layout = QtWidgets.QVBoxLayout(self)

        self.disable_friendly_collisions_check = document(
            QtWidgets.QCheckBox("Disable friendly collisions"), "friendly_collisions",
            "Disable friendly collisions",
            "Robots on the same alliance pass through each other instead of bumping. Collisions "
            "with the opposing alliance are unaffected.",
            "Handy while you are testing a strategy alone and don't want your own alliance "
            "partner's spawn point to shove your robot off course.")
        layout.addWidget(self.disable_friendly_collisions_check)

        self.emit_coral_to_field_check = document(
            QtWidgets.QCheckBox("Emit coral to field"), "emit_coral",
            "Emit coral to field",
            "Each alliance's coral emitter drops one CORAL every 10 seconds during teleop, from "
            "that alliance's top coral station's remaining supply.",
            "Switch this on to test a strategy against a steady trickle of new game pieces "
            "instead of only the ones already on the field at kickoff.")
        layout.addWidget(self.emit_coral_to_field_check)

        hint = QtWidgets.QLabel("Changes apply on RESET (below field).")
        hint.setStyleSheet(f"color: {theme.TEXT_DIM};")
        hint.setWordWrap(True)
        layout.addWidget(hint)

    def disable_friendly_collisions(self) -> bool:
        return self.disable_friendly_collisions_check.isChecked()

    def emit_coral_to_field(self) -> bool:
        return self.emit_coral_to_field_check.isChecked()


class RosterEntryRow(QtWidgets.QWidget):
    """One extra AI robot: a strategy-file combo plus a remove button."""

    removed = QtCore.Signal(object)

    def __init__(self, strategy_names, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.strategy_combo = QtWidgets.QComboBox()
        self.strategy_combo.addItems(strategy_names)
        layout.addWidget(self.strategy_combo, stretch=1)

        remove_button = QtWidgets.QPushButton("✕")
        remove_button.setFixedWidth(24)
        remove_button.clicked.connect(lambda: self.removed.emit(self))
        layout.addWidget(remove_button)

    def strategy_name(self) -> str:
        return self.strategy_combo.currentText()


class AllianceRosterBox(QtWidgets.QGroupBox):
    """Add/remove AI robots for one alliance, each with its own strategy
    selection. Every extra robot gets its own RobotConfigTab (chassis,
    manipulators, timing) so per-bot tuning is possible -- the shared
    default characteristics remain just the starting point for that tab.

    row_added/row_removed let the owning window keep a ROBOT CONFIG tab
    widget in sync with the roster as entries are added/removed."""

    row_added = QtCore.Signal(object, object)  # (RosterEntryRow, RobotConfigTab)
    row_removed = QtCore.Signal(object)  # RosterEntryRow

    def __init__(self, title: str, strategy_names, parent=None):
        super().__init__(title, parent)
        self._strategy_names = strategy_names
        self._rows: list[RosterEntryRow] = []
        self._config_tabs: dict[RosterEntryRow, "RobotConfigTab"] = {}

        layout = QtWidgets.QVBoxLayout(self)
        self.rows_layout = QtWidgets.QVBoxLayout()
        layout.addLayout(self.rows_layout)

        add_button = document(
            QtWidgets.QPushButton("+ ADD ROBOT"), f"add_robot_{title.split()[0].lower()}",
            f"Add a {title.split()[0].lower()} robot",
            "Adds another AI-controlled robot to this alliance, each running a strategy you "
            "pick from the dropdown that appears.",
            "Every robot you add gets its own tab under ROBOT CONFIG, so a fast robot and a "
            "slow one can share the field honestly.")
        add_button.clicked.connect(self._add_row)
        layout.addWidget(add_button)

    def _add_row(self) -> None:
        if not self._strategy_names:
            return
        row = RosterEntryRow(self._strategy_names)
        row.removed.connect(self._remove_row)
        self.rows_layout.addWidget(row)
        self._rows.append(row)
        config_tab = RobotConfigTab(show_alliance=False)
        self._config_tabs[row] = config_tab
        self.row_added.emit(row, config_tab)

    def _remove_row(self, row: RosterEntryRow) -> None:
        self._rows.remove(row)
        self._config_tabs.pop(row)
        self.row_removed.emit(row)
        row.setParent(None)
        row.deleteLater()

    def strategy_names(self) -> list[str]:
        return [row.strategy_name() for row in self._rows]

    def rows_with_config(self) -> list[tuple["RosterEntryRow", "RobotConfigTab"]]:
        return [(row, self._config_tabs[row]) for row in self._rows]


class RosterPanel(QtWidgets.QGroupBox):
    """Controls how many AI-strategy robots join the match beyond the
    single human/gamepad-controlled primary robot: a per-alliance roster
    of extra robots, plus a toggle to let a strategy drive the primary
    robot instead of the keyboard/gamepad.

    robot_added/robot_removed forward the alliance roster boxes' signals,
    tagged with which alliance they belong to, so the owning window can
    keep its ROBOT CONFIG tabs in sync with the roster."""

    robot_added = QtCore.Signal(str, object, object)  # (alliance, row, config_tab)
    robot_removed = QtCore.Signal(str, object)  # (alliance, row)

    def __init__(self, strategy_names, parent=None):
        super().__init__("ROSTER", parent)
        layout = QtWidgets.QVBoxLayout(self)

        self.ai_primary_check = document(
            QtWidgets.QCheckBox("AI drives primary robot"), "ai_primary",
            "AI drives primary robot",
            "Hands your robot -- the one WASD/gamepad normally controls -- to a strategy "
            "instead, so you can watch it drive itself.",
            "This is what turns MATCH into a place to watch a strategy rather than practice "
            "driving. Pick which strategy from the dropdown that appears once this is checked.")
        layout.addWidget(self.ai_primary_check)

        self.primary_strategy_combo = QtWidgets.QComboBox()
        self.primary_strategy_combo.addItems(strategy_names)
        self.primary_strategy_combo.setEnabled(False)
        self.ai_primary_check.toggled.connect(self.primary_strategy_combo.setEnabled)
        layout.addWidget(self.primary_strategy_combo)

        # Only meaningful once AI is driving -- with a human at the
        # keyboard/gamepad, running faster than real time would just make
        # the robot undrivable.
        self.fast_forward_check = document(
            QtWidgets.QCheckBox("Run as fast as possible"), "fast_forward",
            "Run as fast as possible",
            "Advances the sim as fast as the CPU allows instead of at real-time speed.",
            "Only available once the AI is driving -- a human at the keyboard can't keep up "
            "with a sped-up match, but a strategy doesn't care. Good for watching a whole "
            "match play out in a few seconds.")
        self.fast_forward_check.setEnabled(False)
        self.fast_forward_check.setToolTip("Advance the sim as fast as the CPU allows instead of at real-time speed.")
        self.ai_primary_check.toggled.connect(self._on_ai_primary_toggled)
        layout.addWidget(self.fast_forward_check)

        self.blue_roster = AllianceRosterBox("BLUE -- EXTRA ROBOTS", strategy_names)
        self.red_roster = AllianceRosterBox("RED -- EXTRA ROBOTS", strategy_names)
        self.blue_roster.row_added.connect(lambda row, tab: self.robot_added.emit("blue", row, tab))
        self.blue_roster.row_removed.connect(lambda row: self.robot_removed.emit("blue", row))
        self.red_roster.row_added.connect(lambda row, tab: self.robot_added.emit("red", row, tab))
        self.red_roster.row_removed.connect(lambda row: self.robot_removed.emit("red", row))
        layout.addWidget(self.blue_roster)
        layout.addWidget(self.red_roster)

        hint = QtWidgets.QLabel("Changes apply on RESET (below field).")
        hint.setStyleSheet(f"color: {theme.TEXT_DIM};")
        hint.setWordWrap(True)
        layout.addWidget(hint)

    def _on_ai_primary_toggled(self, checked: bool) -> None:
        self.fast_forward_check.setEnabled(checked)
        if not checked:
            self.fast_forward_check.setChecked(False)

    def ai_drives_primary(self) -> bool:
        return self.ai_primary_check.isChecked() and bool(STRATEGY_NAMES)

    def fast_forward_enabled(self) -> bool:
        return self.ai_drives_primary() and self.fast_forward_check.isChecked()

    def primary_strategy_name(self) -> str:
        return self.primary_strategy_combo.currentText()

    def roster_rows(self, alliance: str) -> list[tuple["RosterEntryRow", "RobotConfigTab"]]:
        box = self.blue_roster if alliance == "blue" else self.red_roster
        return box.rows_with_config()


class RobotRosterConfigPanel(QtWidgets.QWidget):
    """RosterPanel + the per-robot RobotConfigTab QTabWidget kept in sync
    -- the left-column block MATCH and SWEEP both need, extracted
    verbatim from MatchView.__init__ and its _on_robot_added/
    _on_robot_removed. sweep_mode=True hides "Run as fast as possible"
    and forces "AI drives primary" on (a sweep has no human driver)."""

    roster_changed = QtCore.Signal()

    def __init__(self, sweep_mode: bool = False, parent=None):
        super().__init__(parent)
        self.sweep_mode = sweep_mode

        self.roster_panel = RosterPanel(STRATEGY_NAMES)
        self.roster_panel.robot_added.connect(self._on_robot_added)
        self.roster_panel.robot_added.connect(lambda *_: self.roster_changed.emit())
        self.roster_panel.robot_removed.connect(self._on_robot_removed)
        self.roster_panel.robot_removed.connect(lambda *_: self.roster_changed.emit())

        self.primary_config_tab = RobotConfigTab(show_alliance=True)
        self._row_config_tabs: dict[object, RobotConfigTab] = {}
        self.config_tabs = QtWidgets.QTabWidget()
        self.config_tabs.addTab(self.primary_config_tab, "PRIMARY")

        self.config_box = document(
            CollapsibleBox("ROBOT CONFIG"), "robot_config", "Robot config",
            "One tab per robot in the match -- PRIMARY plus every AI robot you've added -- each "
            "holding that robot's own chassis settings, manipulators and scoring timing.",
            "Click the header to collapse this section once you're done editing; it takes up a "
            "lot of space with several robots on the roster.")
        config_box_layout = QtWidgets.QVBoxLayout()
        config_box_layout.setContentsMargins(0, 0, 0, 0)
        config_box_layout.addWidget(self.config_tabs)
        self.config_box.setContentLayout(config_box_layout)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.roster_panel)
        layout.addWidget(self.config_box)

        if sweep_mode:
            self.roster_panel.fast_forward_check.setVisible(False)
            self.roster_panel.ai_primary_check.setChecked(True)
            self.roster_panel.ai_primary_check.setEnabled(False)

    def _on_robot_added(self, alliance: str, row, config_tab: RobotConfigTab) -> None:
        self._row_config_tabs[row] = config_tab
        # -1: `row` is already appended to the roster by the time this
        # signal fires (see AllianceRosterBox._add_row), so the un-adjusted
        # count is one ahead of this row's own 0-based index -- the same
        # index robot_labels() and _spawn_roster_robot use for "BLUE 0",
        # "BLUE 1", etc. Without the -1 the very first extra robot's
        # ROBOT CONFIG tab reads "BLUE 1" while everything else calls it
        # "BLUE 0".
        index = len(self.roster_panel.roster_rows(alliance)) - 1
        self.config_tabs.addTab(config_tab, f"{alliance.upper()} {index}")

    def _on_robot_removed(self, alliance: str, row) -> None:
        config_tab = self._row_config_tabs.pop(row)
        tab_index = self.config_tabs.indexOf(config_tab)
        if tab_index >= 0:
            self.config_tabs.removeTab(tab_index)

    def robot_labels(self) -> list[str]:
        """Stable per-robot keys ("PRIMARY" / "BLUE 0" / "RED 0" / ...),
        BLUE/RED indices 0-based, matching each row's roster slot index."""
        labels = ["PRIMARY"]
        for alliance in ("blue", "red"):
            for i in range(len(self.roster_panel.roster_rows(alliance))):
                labels.append(f"{alliance.upper()} {i}")
        return labels

    def config_tab_for(self, label: str) -> RobotConfigTab:
        if label == "PRIMARY":
            return self.primary_config_tab
        alliance, index_str = label.split(" ", 1)
        row, config_tab = self.roster_panel.roster_rows(alliance.lower())[int(index_str)]
        return config_tab

    def robot_specs(self, strategy_override=None) -> list[RobotSpec]:
        """The linchpin: both MatchView._reset_match (via
        run_reefscape.py) and SweepTab._build_jobs go through this, so a
        config that scores X in MATCH scores X in a sweep.
        `strategy_override(label) -> str | dict | None`, if given, takes
        priority over a roster row's file-selected strategy -- lets an
        unsaved STRATEGY-tab edit be included in a sweep the same way it
        overrides MATCH on RESET."""
        specs = []
        overrides = self.primary_config_tab.characteristics_overrides()
        primary_chars = characteristics_to_spec(build_demo_characteristics(**overrides))
        primary_alliance = self.primary_config_tab.settings_panel.alliance()
        primary_strategy = self._resolve_strategy("PRIMARY", strategy_override, default=(
            self.roster_panel.primary_strategy_name() if self.roster_panel.ai_drives_primary() or self.sweep_mode else None
        ))
        specs.append(RobotSpec(
            label="PRIMARY", alliance=primary_alliance, roster_index=-1,
            characteristics=primary_chars, strategy=primary_strategy,
        ))

        for alliance in ("blue", "red"):
            for i, (row, config_tab) in enumerate(self.roster_panel.roster_rows(alliance)):
                label = f"{alliance.upper()} {i}"
                overrides = config_tab.characteristics_overrides()
                overrides["name"] = f"{alliance}-ai-{i}"
                chars = characteristics_to_spec(build_demo_characteristics(**overrides))
                strategy = self._resolve_strategy(label, strategy_override, default=row.strategy_name())
                specs.append(RobotSpec(label=label, alliance=alliance, roster_index=i, characteristics=chars, strategy=strategy))
        return specs

    @staticmethod
    def _resolve_strategy(label, strategy_override, default):
        if strategy_override is not None:
            override = strategy_override(label)
            if override is not None:
                return override
        return default
