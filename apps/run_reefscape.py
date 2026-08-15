"""
Interactive REEFSCAPE viewer -- the real 2025 field/pieces/scoring
(game_specific/reefscape) driven through the exact same gui_utils
FieldCanvas and control.input_sources plumbing as apps/run_match.py's
placeholder game. Nothing about the GUI or input layer changed to
support a real game; that's the point of this dry run.

Layout: a scrollable settings + controls-reference column on the left,
the live field view (with a play/pause/reset transport bar and a
held-piece count readout) in the center, and a telemetry + manual
scoring column on the right.

Controls: WASD translate (field-relative), LEFT/RIGHT rotate,
SPACE hold to intake, 1/2/3/4 select CORAL level L1-L4 (or X to cycle
levels, as a keyboard-only fallback), F hold to deposit at the selected
level. Holding ALGAE, the deposit target auto-switches to PROCESSOR/NET
whenever the robot is sitting in one of those zones, no manual selection
needed. Xbox controller (if connected): left stick drives, right stick X
rotates, A intakes, B deposits, X cycles CORAL level, Start pauses/resumes.
"""
from __future__ import annotations

import math
import random
import sys

from pyqtgraph.Qt import QtCore, QtWidgets

from common_sim.analysis.metrics import extract_metrics
from common_sim.control.input_sources import GamepadInput, KeyBindings, KeyboardInput
from common_sim.field.field_config import point_in_polygon
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig, Phase
from game_specific.reefscape.field import (
    build_field,
    coral_station_positions,
    reef_center,
)
from game_specific.reefscape.game_pieces import ALGAE_TYPE, CORAL_TYPE, spawn_algae, spawn_coral
from game_specific.reefscape.scoring import REEFSCAPE_SCORING_RULES
from common_sim.robot.characteristics import RobotCharacteristics, SideManipulators, SIDES
from gui_utils import theme
from gui_utils.console_panel import ConsolePanel
from gui_utils.field_canvas import FieldCanvas
from gui_utils.telemetry_panel import TelemetryPanel

Qt = QtCore.Qt

# Unit conversions for the GUI's display/entry fields -- RobotCharacteristics
# and everything else in common_sim stays in inches/radians/seconds; only
# what the user types/reads in RobotSettingsPanel is metric/degrees.
M_TO_IN = 39.3701
RAD_TO_DEG = 180.0 / math.pi

KEY_BINDINGS = KeyBindings(
    forward=Qt.Key_W, backward=Qt.Key_S, left=Qt.Key_A, right=Qt.Key_D,
    rotate_ccw=Qt.Key_Left, rotate_cw=Qt.Key_Right,
    intake=Qt.Key_Space, deposit=Qt.Key_F,
)
LEVEL_KEYS = {Qt.Key_1: "l1", Qt.Key_2: "l2", Qt.Key_3: "l3", Qt.Key_4: "l4"}
REEF_LEVELS = ("l1", "l2", "l3", "l4")
TOGGLE_REEF_LEVEL_KEY = Qt.Key_X
DEPOSIT_ACTIONS = (("l1", "L1"), ("l2", "L2"), ("l3", "L3"), ("l4", "L4"), ("processor", "PROCESSOR"), ("net", "NET"))
DEFAULT_DEPOSIT_TIMES = {"l1": 0.3, "l2": 0.6, "l3": 1.0, "l4": 1.8, "processor": 0.4, "net": 1.2}
DEFAULT_INTAKE_TIMES = {CORAL_TYPE: 0.4, ALGAE_TYPE: 0.4}
DEFAULT_PIECE_CAPACITY = {CORAL_TYPE: 1, ALGAE_TYPE: 1}

PIECE_TYPES = (CORAL_TYPE, ALGAE_TYPE)
# Default side layout, matching the example from the feature request: a
# back coral intake, front coral scoring, and a right-side algae intake +
# score -- gives a non-empty, visually distinct starting point rather than
# every checkbox unchecked.
DEFAULT_SIDE_CHECKS = {
    ("back", "in", CORAL_TYPE), ("front", "out", CORAL_TYPE),
    ("right", "in", ALGAE_TYPE), ("right", "out", ALGAE_TYPE),
}

GAMEPAD_BINDINGS = [
    ("Left Stick", "Drive"), ("Right Stick X", "Rotate"),
    ("A", "Intake"), ("B", "Deposit"), ("X", "Cycle CORAL level"), ("Start", "Pause / Resume"),
]
KEYBOARD_BINDINGS = [
    ("W A S D", "Drive"), ("Left / Right", "Rotate"),
    ("Space", "Intake"), ("F", "Deposit"), ("1 2 3 4", "Select CORAL level"),
    ("X", "Cycle CORAL level (fallback)"),
]


def build_demo_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="demo",
        max_speed=150.0, max_accel=400.0, max_angular_speed=6.0, max_angular_accel=20.0,
        width=28.0, length=28.0,
        piece_capacity_by_type=dict(DEFAULT_PIECE_CAPACITY),
        intake_time_by_type=dict(DEFAULT_INTAKE_TIMES), station_intake_time=0.6, intake_range=6.0,
        deposit_time=0.5,
        deposit_time_by_action=dict(DEFAULT_DEPOSIT_TIMES),
        accepted_piece_types=frozenset({"coral", "algae"}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def build_demo_match(alliance: str = "blue") -> Match:
    field = build_field()
    match = Match(field, REEFSCAPE_SCORING_RULES, MatchConfig(auto_duration=15, teleop_duration=135))

    rng = random.Random(0)
    center = reef_center(alliance)
    # Scatter pieces between this alliance's REEF and its own ALLIANCE
    # WALL (same side the robot starts on) -- toward -x for blue (wall
    # at x=0), toward +x for red (wall at x=FIELD_LENGTH).
    toward_wall = -1.0 if alliance == "blue" else 1.0
    for _ in range(6):
        spawn_coral(match, (center[0] + toward_wall * 60, center[1] + rng.uniform(-30, 30)))
    for i in range(3):
        spawn_algae(match, (center[0] + toward_wall * 40, center[1] - 60 + 40 * i))

    return match


class RobotSettingsPanel(QtWidgets.QGroupBox):
    """Robot characteristics + alliance selection. Values here only take
    effect once the transport bar's RESET button rebuilds the match --
    tuning a live robot's chassis limits mid-flight isn't a real
    scenario this sim needs to model.

    Displayed/entered in metric (m/s, m/s², deg/s) even though
    RobotCharacteristics itself stays in inches/radians internally --
    characteristics_overrides() does the conversion."""

    def __init__(self, parent=None):
        super().__init__("ROBOT SETTINGS", parent)
        form = QtWidgets.QFormLayout(self)

        self.alliance_combo = QtWidgets.QComboBox()
        self.alliance_combo.addItems(["Blue", "Red"])
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

        hint = QtWidgets.QLabel("Changes apply on RESET (below field).")
        hint.setStyleSheet(f"color: {theme.TEXT_DIM};")
        hint.setWordWrap(True)
        form.addRow(hint)

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

    def deposit_time_by_action(self) -> dict[str, float]:
        return {action: spin.value() for action, spin in self._deposit_spins.items()}


class ControlsPanel(QtWidgets.QGroupBox):
    """Plain-text reference for the active input source's bindings --
    replaces the old painted Xbox controller diagram with just the
    control list, since the graphic wasn't adding useful information."""

    def __init__(self, parent=None):
        super().__init__("CONTROLS", parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.label = QtWidgets.QLabel()
        self.label.setFont(theme.technical_font(9))
        self.label.setWordWrap(True)
        layout.addWidget(self.label)
        self.set_available(False)

    def set_available(self, available: bool) -> None:
        bindings = GAMEPAD_BINDINGS if available else KEYBOARD_BINDINGS
        heading = "GAMEPAD" if available else "KEYBOARD"
        lines = [f"{control}: {action}" for control, action in bindings]
        self.label.setText(f"{heading}\n" + "\n".join(lines))


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

        self._checks: dict[tuple[str, str, str], QtWidgets.QCheckBox] = {}
        for row, side in enumerate(SIDES, start=1):
            grid.addWidget(QtWidgets.QLabel(side.upper()), row, 0)
            for col, (piece_type, mode) in enumerate(columns, start=1):
                box = QtWidgets.QCheckBox()
                box.setChecked((side, mode, piece_type) in DEFAULT_SIDE_CHECKS)
                grid.addWidget(box, row, col, alignment=QtCore.Qt.AlignCenter)
                self._checks[(side, mode, piece_type)] = box

        hint = QtWidgets.QLabel("Changes apply on RESET (below field).")
        hint.setStyleSheet(f"color: {theme.TEXT_DIM};")
        hint.setWordWrap(True)
        grid.addWidget(hint, len(SIDES) + 1, 0, 1, len(columns) + 1)

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
            result[side] = SideManipulators(intake_piece_types=intake_types, score_piece_types=score_types)
        return result


class ScoringControlsPanel(QtWidgets.QGroupBox):
    """Manual scoring controls that mirror the keyboard/gamepad deposit
    action: pick a level with the buttons, hold SCORE to deposit -- same
    semantics as holding F on the keyboard."""

    def __init__(self, parent=None):
        super().__init__("SCORE CONTROLS", parent)
        layout = QtWidgets.QVBoxLayout(self)

        self.level_group = QtWidgets.QButtonGroup(self)
        self.level_group.setExclusive(True)
        level_row = QtWidgets.QGridLayout()
        for i, (action, label) in enumerate(DEPOSIT_ACTIONS):
            btn = QtWidgets.QPushButton(label)
            btn.setCheckable(True)
            btn.setProperty("action", action)
            self.level_group.addButton(btn)
            level_row.addWidget(btn, i // 2, i % 2)
        layout.addLayout(level_row)
        buttons = self.level_group.buttons()
        buttons[3].setChecked(True)  # default: L4

        self.score_button = QtWidgets.QPushButton("HOLD TO SCORE")
        self.score_button.setStyleSheet(f"font-weight: bold; color: {theme.ACCENT_AMBER};")
        self.score_button.setMinimumHeight(40)
        layout.addWidget(self.score_button)

    def selected_action(self) -> str:
        checked = self.level_group.checkedButton()
        return checked.property("action") if checked else "l4"

    def set_selected_action(self, action: str) -> None:
        for btn in self.level_group.buttons():
            if btn.property("action") == action:
                btn.setChecked(True)
                return


class TransportBar(QtWidgets.QWidget):
    """Playback controls under the field: play/pause toggle, a
    (non-seekable -- this is a live sim, not a replay) elapsed-time
    slider, and a reset button that starts a fresh match."""

    play_pause_clicked = QtCore.Signal()
    reset_clicked = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)

        self.play_pause_button = QtWidgets.QPushButton("⏸")
        self.play_pause_button.setFixedWidth(40)
        self.play_pause_button.clicked.connect(self.play_pause_clicked)
        layout.addWidget(self.play_pause_button)

        self.slider = QtWidgets.QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.setEnabled(False)  # progress display only -- live sim isn't seekable
        self.slider.setToolTip("Match progress (live simulation -- not seekable)")
        layout.addWidget(self.slider, stretch=1)

        self.time_label = QtWidgets.QLabel("0:00 / 0:00")
        self.time_label.setFont(theme.technical_font(10))
        self.time_label.setMinimumWidth(90)
        layout.addWidget(self.time_label)

        self.reset_button = QtWidgets.QPushButton("RESET")
        self.reset_button.clicked.connect(self.reset_clicked)
        layout.addWidget(self.reset_button)

    def set_progress(self, elapsed: float, total: float, paused: bool) -> None:
        frac = 0.0 if total <= 0 else max(0.0, min(1.0, elapsed / total))
        self.slider.setValue(int(frac * 1000))
        self.time_label.setText(f"{_fmt_time(elapsed)} / {_fmt_time(total)}")
        self.play_pause_button.setText("▶" if paused else "⏸")


def _fmt_time(seconds: float) -> str:
    seconds = max(0, int(seconds))
    return f"{seconds // 60}:{seconds % 60:02d}"


class ReefscapeWindow(QtWidgets.QMainWindow):
    TICK_HZ = 60

    def __init__(self):
        super().__init__()
        self.setWindowTitle("sparky-sim -- REEFSCAPE")

        self.settings_panel = RobotSettingsPanel()
        self.manipulator_panel = SideManipulatorPanel()
        self.timing_panel = TimingPanel()
        self.controls_panel = ControlsPanel()
        left_content = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_content)
        left_layout.addWidget(self.settings_panel)
        left_layout.addWidget(self.manipulator_panel)
        left_layout.addWidget(self.timing_panel)
        left_layout.addWidget(self.controls_panel)
        left_layout.addStretch(1)

        left_column = QtWidgets.QScrollArea()
        left_column.setWidget(left_content)
        left_column.setWidgetResizable(True)
        left_column.setFixedWidth(280)
        left_column.setFrameShape(QtWidgets.QFrame.NoFrame)

        self.canvas = FieldCanvas(None)  # match assigned by _reset_match()
        self.piece_count_label = QtWidgets.QLabel()
        self.piece_count_label.setAlignment(Qt.AlignCenter)
        self.piece_count_label.setFont(theme.technical_font(11, bold=True))
        self.piece_count_label.setStyleSheet(f"color: {theme.ACCENT_CYAN};")
        self.console = ConsolePanel()
        self.transport_bar = TransportBar()
        self.transport_bar.play_pause_clicked.connect(self._toggle_paused)
        self.transport_bar.reset_clicked.connect(self._reset_match)
        center_column = QtWidgets.QWidget()
        center_layout = QtWidgets.QVBoxLayout(center_column)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.addWidget(self.canvas, stretch=2)
        center_layout.addWidget(self.piece_count_label)
        center_layout.addWidget(self.console, stretch=1)
        center_layout.addWidget(self.transport_bar)

        self.telemetry_panel = TelemetryPanel("TELEMETRY")
        self.scoring_panel = ScoringControlsPanel()
        right_column = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_column)
        right_layout.addWidget(self.telemetry_panel)
        right_layout.addWidget(self.scoring_panel)
        right_layout.addStretch(1)
        right_column.setFixedWidth(240)

        central = QtWidgets.QWidget()
        central_layout = QtWidgets.QHBoxLayout(central)
        central_layout.addWidget(left_column)
        central_layout.addWidget(center_column, stretch=1)
        central_layout.addWidget(right_column)
        self.setCentralWidget(central)
        self.resize(1500, 760)

        self._pressed_keys: set[int] = set()
        self._prev_pressed_keys: set[int] = set()
        self.canvas.keyPressEvent = self._key_press
        self.canvas.keyReleaseEvent = self._key_release

        self.keyboard = KeyboardInput(pressed_keys=lambda: self._pressed_keys, bindings=KEY_BINDINGS)
        gamepad = GamepadInput()
        self.input_source = gamepad if gamepad.available else self.keyboard
        self.controls_panel.set_available(gamepad.available)
        self.setWindowTitle(self.windowTitle() + (" [gamepad]" if gamepad.available else " [keyboard]"))

        self.paused = False
        self._reset_match()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / self.TICK_HZ))
        self.canvas.setFocus()

    # -- match lifecycle -----------------------------------------------

    def _reset_match(self) -> None:
        alliance = self.settings_panel.alliance()
        self.match = build_demo_match(alliance)
        station = coral_station_positions(alliance)[0]
        facing = 0.0 if alliance == "blue" else 3.14159265
        start_pose = Pose2d(station[0] + (30.0 if alliance == "blue" else -30.0), station[1], facing)
        overrides = self.settings_panel.characteristics_overrides()
        overrides["side_manipulators"] = self.manipulator_panel.side_manipulators()
        overrides["deposit_time_by_action"] = self.timing_panel.deposit_time_by_action()
        characteristics = build_demo_characteristics(**overrides)
        self.robot = self.match.add_robot(characteristics, start_pose, alliance=alliance)
        self.canvas.match = self.match
        self.console.reset()
        self._logged_event_count = 0
        self.paused = False
        self._update_piece_counts()
        self.canvas.setFocus()

    def _toggle_paused(self) -> None:
        self.paused = not self.paused

    # -- input -----------------------------------------------------------

    def _key_press(self, event) -> None:
        self._pressed_keys.add(event.key())
        if event.key() in LEVEL_KEYS:
            self.scoring_panel.set_selected_action(LEVEL_KEYS[event.key()])

    def _key_release(self, event) -> None:
        self._pressed_keys.discard(event.key())

    def _cycle_reef_level(self) -> None:
        current = self.scoring_panel.selected_action()
        start = REEF_LEVELS.index(current) if current in REEF_LEVELS else -1
        self.scoring_panel.set_selected_action(REEF_LEVELS[(start + 1) % len(REEF_LEVELS)])

    def _tick(self) -> None:
        dt = 1.0 / self.TICK_HZ
        drive, operator = self.input_source.poll()

        # Edge-trigger off the same _pressed_keys set WASD driving polls,
        # rather than QKeyEvent.isAutoRepeat() -- on some platforms the
        # very first physical keypress can arrive already flagged as a
        # repeat, which silently swallowed this cycle before it ever ran.
        # Kept as a fallback alongside the gamepad's X button (the primary
        # binding) since input_source is keyboard-only when no gamepad is
        # connected.
        just_pressed = self._pressed_keys - self._prev_pressed_keys
        if TOGGLE_REEF_LEVEL_KEY in just_pressed or operator.cycle_level:
            self._cycle_reef_level()
        self._prev_pressed_keys = set(self._pressed_keys)

        if operator.pause_toggle:
            self._toggle_paused()

        if not self.paused:
            c = self.robot.characteristics
            self.robot.drive_field_relative(dt, drive.vx * c.max_speed, drive.vy * c.max_speed, drive.omega * c.max_angular_speed)
            self.robot.set_intake_active(operator.intake_active)
            deposit_active = operator.deposit_active or self.scoring_panel.score_button.isDown()
            action = self._effective_deposit_action()
            self.robot.set_deposit_active(deposit_active, action=action)
            self.match.step(dt)
            self._drain_new_events()

        self.transport_bar.set_progress(self.match.elapsed, self.match.config.total_duration, self.paused)
        self._update_telemetry()
        self._update_piece_counts()
        self.canvas.update()

    def _update_piece_counts(self) -> None:
        held = self.robot.held_pieces
        parts = []
        for piece_type in PIECE_TYPES:
            held_count = sum(1 for p in held if p.piece_type == piece_type)
            capacity = self.robot.characteristics.capacity_for(piece_type)
            parts.append(f"{piece_type.upper()}: {held_count}/{capacity}")
        self.piece_count_label.setText("   ".join(parts))

    def _effective_deposit_action(self) -> str:
        """The manually-selected level/action, unless the robot is
        currently sitting in a single-action scoring zone (PROCESSOR,
        NET) holding a piece that zone accepts -- ALGAE only ever scores
        at one of those two, so there's nothing to pick between and
        requiring an explicit GUI selection just adds friction. REEF
        faces offer 4 actions at once (L1-L4), so they're deliberately
        excluded from this and stay a manual/toggle (X) choice."""
        manual = self.scoring_panel.selected_action()
        pose = self.robot.pose
        held_types = {p.piece_type for p in self.robot.held_pieces}
        if not held_types:
            return manual
        for region in self.match.field.scoring_regions:
            if len(region.actions) != 1:
                continue
            if region.piece_types and not (held_types & region.piece_types):
                continue
            if point_in_polygon((pose.x, pose.y), region.vertices):
                return next(iter(region.actions))
        return manual

    def _drain_new_events(self) -> None:
        all_events = list(self.match.events)
        new_events = all_events[self._logged_event_count:]
        self._logged_event_count = len(all_events)
        if new_events:
            self.console.append_events(new_events)

    def _update_telemetry(self) -> None:
        metrics = extract_metrics(self.match)
        cycle = f"{metrics.mean_cycle_time:.1f}s" if metrics.mean_cycle_time is not None else "--"
        lines = [
            ("Phase", self.match.phase.value.upper()),
            ("Elapsed", f"{self.match.elapsed:.1f}s"),
        ]
        for alliance, score in sorted(self.match.scores.items()):
            lines.append((f"Score ({alliance})", f"{score:.0f}"))
        lines += [
            ("Held Pieces", str(len(self.robot.held_pieces))),
            ("Intaked", str(metrics.pieces_intaked)),
            ("Deposited", str(metrics.pieces_deposited)),
            ("Scored", str(metrics.pieces_scored)),
            ("Misses", str(metrics.misses)),
            ("Mean Cycle", cycle),
        ]
        self.telemetry_panel.set_lines(lines)


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    theme.apply_app_theme(app)
    window = ReefscapeWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
