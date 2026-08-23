"""
Interactive REEFSCAPE viewer -- the real 2025 field/pieces/scoring
(game_specific/reefscape) driven through the exact same gui_utils
FieldCanvas and control.input_sources plumbing as apps/run_match.py's
placeholder game. Nothing about the GUI or input layer changed to
support a real game; that's the point of this dry run.

Layout: a MATCH tab (MatchView) -- a scrollable settings +
controls-reference column on the left, the live field view (with a
play/pause/reset transport bar and a held-piece count readout) in the
center, and a telemetry + manual scoring column on the right -- next to
a STRATEGY tab (gui_utils/strategy_editor.py) where each robot's Rule
list and fallback tactic can be edited and staged for the next RESET.

Controls: WASD translate (field-relative), LEFT/RIGHT rotate,
SPACE hold to intake, 1/2/3/4 select CORAL level L1-L4 (or X to cycle
levels, as a keyboard-only fallback), F hold to deposit at the selected
level. Holding ALGAE, the deposit target auto-switches to PROCESSOR/NET
whenever the robot is sitting in one of those zones, no manual selection
needed. Xbox controller (if connected): left stick drives, right stick X
rotates, A intakes, RIGHT TRIGGER deposits, X cycles CORAL level, Start
pauses/resumes.
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

from pyqtgraph.Qt import QtCore, QtWidgets

from apps.reefscape_widgets import (
    AllianceRosterBox,
    CollapsibleBox,
    DEFAULT_DEPOSIT_TIMES,
    DEFAULT_INTAKE_TIMES,
    DEFAULT_PIECE_CAPACITY,
    DEFAULT_SIDE_CHECKS,
    DEPOSIT_ACTIONS,
    M_TO_IN,
    MatchSettingsPanel,
    PIECE_TYPES,
    RAD_TO_DEG,
    RobotConfigTab,
    RobotRosterConfigPanel,
    RobotSettingsPanel,
    RosterEntryRow,
    RosterPanel,
    SideManipulatorPanel,
    STRATEGIES_DIR,
    STRATEGY_FILES,
    STRATEGY_NAMES,
    UNIT_HINTS,
    build_demo_characteristics,
    build_demo_match,
)
from apps.search_tab import SearchTab
from apps.sweep_tab import SweepTab
from common_sim.analysis.metrics import extract_metrics
from common_sim.control import strategy_io
from common_sim.control.human import HumanController
from common_sim.control.input_sources import (
    CombinedInput, DriveCommand, GamepadInput, KeyBindings, KeyboardInput, OperatorCommand,
)
from common_sim.control.strategy import StrategyController
from common_sim.field.field_config import point_in_polygon
from common_sim.match.match import Match, MatchConfig, Phase
from common_sim.match.telemetry import TelemetryRecorder
from game_specific.reefscape import sweep_trial
from game_specific.reefscape.game_pieces import ALGAE_TYPE, CORAL_TYPE
from gui_utils import theme
from gui_utils.console_panel import ConsolePanel
from gui_utils.doc_tags import document
from gui_utils.field_camera import DRIVER, ELEVATED, orient_drive
from gui_utils.field_canvas import FieldCanvas
from gui_utils.match_sounds import MatchSoundboard
from gui_utils.scrub_slider import ScrubSlider
from gui_utils.strategy_editor import StrategyEditor
from gui_utils.strategy_graph import StrategyGraphPanel
from gui_utils.telemetry_panel import TelemetryPanel

Qt = QtCore.Qt

KEY_BINDINGS = KeyBindings(
    forward=Qt.Key_W, backward=Qt.Key_S, left=Qt.Key_A, right=Qt.Key_D,
    rotate_ccw=Qt.Key_Left, rotate_cw=Qt.Key_Right,
    intake=Qt.Key_Space, deposit=Qt.Key_F,
)
LEVEL_KEYS = {Qt.Key_1: "l1", Qt.Key_2: "l2", Qt.Key_3: "l3", Qt.Key_4: "l4"}
REEF_LEVELS = ("l1", "l2", "l3", "l4")
TOGGLE_REEF_LEVEL_KEY = Qt.Key_X

ASSETS_DIR = Path(__file__).resolve().parent.parent / "assets"
# Cosmetic only -- there is no ENDGAME phase or duration in common_sim/
# game_specific (see Phase in common_sim/match/match.py); this just picks
# when the "Start of End Game" cue plays, matching the real FRC 2025
# endgame length, and has no effect on scoring.
ENDGAME_SECONDS = 20.0

GAMEPAD_BINDINGS = [
    ("Left Stick", "Drive"), ("Right Stick X", "Rotate"),
    ("A", "Intake"), ("RT", "Deposit"), ("X", "Cycle CORAL level"), ("Start", "Pause / Resume"),
]
KEYBOARD_BINDINGS = [
    ("W A S D", "Drive"), ("Left / Right", "Rotate"),
    ("Space", "Intake"), ("F", "Deposit"), ("1 2 3 4", "Select CORAL level"),
    ("X", "Cycle CORAL level (fallback)"),
]


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
        document(
            self, "controls_reference", "Controls reference",
            "Which keys and gamepad buttons drive the robot. Both devices are live at the "
            "same time -- a connected controller never takes the keyboard away.",
            "This only matters while a human is driving PRIMARY -- once AI drives primary robot "
            "is checked in the roster panel, nothing here does anything.")

    def set_available(self, available: bool) -> None:
        """Lists the keyboard always, and the gamepad as well when one is
        connected. It used to show one *instead of* the other, which
        stopped being true when the window moved to CombinedInput -- and
        was the more misleading half anyway, since the panel claiming
        W A S D didn't exist was a driver's only clue about why nothing
        moved."""
        sections = [("KEYBOARD", KEYBOARD_BINDINGS)]
        if available:
            sections.append(("GAMEPAD", GAMEPAD_BINDINGS))
        blocks = [
            "\n".join([heading] + [f"{control}: {action}" for control, action in bindings])
            for heading, bindings in sections
        ]
        self.label.setText("\n\n".join(blocks))


VIEW_MODES = {
    "TOP-DOWN": (None, None),
    "DRIVER (BLUE)": ("blue", DRIVER),
    "DRIVER (RED)": ("red", DRIVER),
    "ELEVATED (BLUE)": ("blue", ELEVATED),
    "ELEVATED (RED)": ("red", ELEVATED),
}


class ViewModePanel(QtWidgets.QGroupBox):
    """Picks which of FieldCanvas's camera modes the field is drawn
    with -- the usual top-down strategy view, or a tilted driver-station
    perspective from behind either alliance's wall, for driver practice."""

    view_changed = QtCore.Signal(str)  # one of VIEW_MODES' keys

    def __init__(self, parent=None):
        super().__init__("FIELD VIEW", parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.combo = document(
            QtWidgets.QComboBox(), "view_mode", "Field view",
            "Switches the field between the usual top-down strategy view and a tilted "
            "driver-station perspective from behind an alliance wall.",
            "The perspective views are for driver practice, not analysis -- distance is much "
            "harder to judge than in top-down, which is the point: that's what a real driver "
            "has to deal with too. ELEVATED pulls the eye back and up for an easier, more "
            "coach's-eye version of the same idea.")
        self.combo.addItems(list(VIEW_MODES.keys()))
        self.combo.currentTextChanged.connect(self.view_changed.emit)
        layout.addWidget(self.combo)

    def current_mode(self) -> str:
        return self.combo.currentText()


PLAYER2_AI_OPTION = "AI (no second player)"


class Player2Panel(QtWidgets.QGroupBox):
    """Picks which non-PRIMARY robot, if any, a second gamepad drives --
    either a teammate (another robot on PRIMARY's alliance) or an
    opponent, for practicing against a human-driven defender. Options
    are refreshed from the current roster on every RESET (roster edits
    already only take effect on RESET elsewhere in this app)."""

    def __init__(self, parent=None):
        super().__init__("PLAYER 2 (GAMEPAD 2)", parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.combo = document(
            QtWidgets.QComboBox(), "player2_robot", "Player 2 robot",
            "Hands a second gamepad's input to whichever robot is picked here, instead of that "
            "robot's AI strategy. Pick a teammate to practice cycling together, or an opponent "
            "to practice against a human defender.",
            "Same bindings as the GAMEPAD control scheme, on a second physical controller. "
            "Takes effect on the next RESET.")
        self.combo.addItem(PLAYER2_AI_OPTION)
        layout.addWidget(self.combo)
        self.status_label = QtWidgets.QLabel()
        self.status_label.setFont(theme.technical_font(8))
        self.status_label.setStyleSheet(f"color: {theme.TEXT_DIM};")
        layout.addWidget(self.status_label)

    def set_options(self, labels: list[str]) -> None:
        current = self.combo.currentText()
        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItem(PLAYER2_AI_OPTION)
        self.combo.addItems(labels)
        index = self.combo.findText(current)
        self.combo.setCurrentIndex(index if index >= 0 else 0)
        self.combo.blockSignals(False)

    def selected_label(self) -> str | None:
        text = self.combo.currentText()
        return None if text == PLAYER2_AI_OPTION else text

    def set_gamepad_available(self, available: bool) -> None:
        self.status_label.setText("" if available else "No second gamepad detected")


class SplitScreenPanel(QtWidgets.QGroupBox):
    """Toggles a two-pane driver-station view -- Player 1's alliance
    wall on the left, Player 2's on the right -- so both humans drive
    from their own correct, opposing perspective on one screen at once.
    Only enabled when two physical gamepads are connected and Player 2
    is piloting a RED robot; otherwise there's no second, opposing
    driver view to show (see MatchView._update_split_screen_availability)."""

    toggled_by_user = QtCore.Signal(bool)

    def __init__(self, parent=None):
        super().__init__("SPLIT SCREEN", parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.checkbox = document(
            QtWidgets.QCheckBox("Split Screen (Face Off)"), "split_screen", "Split screen",
            "Shows two driver-station views side by side -- Player 1's alliance wall on the "
            "left, Player 2's on the right -- so both players drive from their own correct "
            "perspective at once.",
            "Needs two gamepads connected and Player 2 piloting a RED robot; otherwise there's "
            "no second, opposing driver view to show, and this stays disabled.")
        self.checkbox.toggled.connect(self.toggled_by_user.emit)
        layout.addWidget(self.checkbox)
        self.status_label = QtWidgets.QLabel()
        self.status_label.setFont(theme.technical_font(8))
        self.status_label.setStyleSheet(f"color: {theme.TEXT_DIM};")
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

    def set_available(self, available: bool, reason: str = "") -> None:
        self.checkbox.setEnabled(available)
        if not available:
            self.set_checked(False)
        self.status_label.setText(reason)

    def set_checked(self, checked: bool) -> None:
        if self.checkbox.isChecked() == checked:
            return
        self.checkbox.blockSignals(True)
        self.checkbox.setChecked(checked)
        self.checkbox.blockSignals(False)

    def is_checked(self) -> bool:
        return self.checkbox.isChecked()


class TransportBar(QtWidgets.QWidget):
    """Playback controls under the field: play/pause toggle, an
    elapsed-time slider, and a reset button that starts a fresh match.

    The slider tracks live progress (and is disabled) while the sim is
    running -- scrubbing a live physics sim mid-step doesn't mean
    anything. Once paused, MatchView enables it for review: dragging it
    jumps every robot to its recorded telemetry pose at that point in
    the match (see MatchView._enter_playback_at_fraction), same as
    scrubbing a video. Values are a 0-1000 fraction of the match's
    total_duration rather than a frame index, so the slider's meaning
    doesn't depend on how many telemetry frames happen to exist yet."""

    play_pause_clicked = QtCore.Signal()
    reset_clicked = QtCore.Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)

        self.play_pause_button = document(
            QtWidgets.QPushButton("⏸"), "play_pause", "Play / pause",
            "Starts or freezes the match clock. Everything else -- driving, telemetry, scoring "
            "-- only advances while this says playing.")
        self.play_pause_button.setFixedWidth(40)
        self.play_pause_button.clicked.connect(self.play_pause_clicked)
        layout.addWidget(self.play_pause_button)

        self.slider = document(
            ScrubSlider(Qt.Horizontal), "scrub_slider", "Scrub bar",
            "Shows how far into the match you are. Once paused, drag it to jump every robot "
            "back to its recorded position at that moment.",
            "This only works after pausing -- scrubbing a live physics simulation mid-step "
            "doesn't mean anything, so the bar is locked while playing. It's how you review a "
            "moment that already happened without re-running the whole match.")
        self.slider.setRange(0, 1000)
        self.slider.setEnabled(False)  # enabled by MatchView once paused with telemetry recorded
        self.slider.setToolTip("Match progress -- pause to scrub and review")
        layout.addWidget(self.slider, stretch=1)

        self.time_label = QtWidgets.QLabel("0:00 / 0:00")
        self.time_label.setFont(theme.technical_font(10))
        self.time_label.setMinimumWidth(90)
        layout.addWidget(self.time_label)

        self.reset_button = document(
            QtWidgets.QPushButton("RESET"), "reset", "Reset",
            "Throws away the current match and starts a fresh one with whatever roster and "
            "settings are configured right now.",
            "This is also when every setting you've changed on the left and right columns "
            "actually takes effect -- nothing there applies to a match already in progress.")
        self.reset_button.clicked.connect(self.reset_clicked)
        layout.addWidget(self.reset_button)

    def set_progress(self, elapsed: float, total: float, paused: bool, *, sync_slider: bool = True) -> None:
        """`sync_slider=False` updates the time label/button only,
        leaving the slider's position alone -- used while the user is
        actively scrubbing it, so this doesn't fight their drag every
        tick (see MatchView._tick)."""
        frac = 0.0 if total <= 0 else max(0.0, min(1.0, elapsed / total))
        if sync_slider:
            self.slider.setValue(int(frac * 1000))
        self.time_label.setText(f"{_fmt_time(elapsed)} / {_fmt_time(total)}")
        self.play_pause_button.setText("▶" if paused else "⏸")


def _fmt_time(seconds: float) -> str:
    seconds = max(0, int(seconds))
    return f"{seconds // 60}:{seconds % 60:02d}"


class MatchView(QtWidgets.QWidget):
    """Everything needed to run and watch one live match: roster/config
    controls, the field canvas, telemetry/scoring/controls, and the
    transport bar -- exactly what used to be ReefscapeWindow's central
    widget, extracted so it can sit in a QTabWidget next to STRATEGY
    (gui_utils/strategy_editor.py) instead of being the whole window.

    `strategy_provider(label) -> Strategy | None`, set by the owning
    window, lets the STRATEGY tab's edited/applied strategies override
    a robot's roster-selected strategy file on the next RESET; `label`
    is the same "PRIMARY" / "BLUE 0" / "RED 0" scheme `robot_labels()`
    reports, so the two stay in sync without MatchView importing
    anything from strategy_editor."""

    TICK_HZ = 60

    roster_changed = QtCore.Signal()
    match_reset = QtCore.Signal(object)  # Match
    tick_completed = QtCore.Signal()
    fullscreen_changed = QtCore.Signal(bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.strategy_provider = None
        self._robots_by_label: dict[str, object] = {}

        self.roster_config = RobotRosterConfigPanel(sweep_mode=False)
        self.roster_config.roster_changed.connect(lambda: self.roster_changed.emit())
        self.roster_config.roster_panel.fast_forward_check.toggled.connect(self._update_timer_interval)
        self.primary_config_tab = self.roster_config.primary_config_tab

        self.left_column = QtWidgets.QScrollArea()
        self.left_column.setWidget(self.roster_config)
        self.left_column.setWidgetResizable(True)
        self.left_column.setMinimumWidth(240)
        self.left_column.setFrameShape(QtWidgets.QFrame.NoFrame)

        self.canvas = document(
            FieldCanvas(None), "field", "The field",
            "The live match, drawn to scale: robots, game pieces, scoring regions, and (if "
            "enabled) each AI robot's current intent.",
            "Click it and use WASD/arrows to drive PRIMARY if a human is driving. This is the "
            "same rendering the SWEEP tab's replay and the SEARCH tab's confirmation runs would "
            "show if you replayed one of their matches here.")  # match assigned by _reset_match()
        # Player 2's pane -- only shown in split-screen mode, sharing the
        # same live Match as self.canvas but with its own driver-station
        # perspective (see SplitScreenPanel / _apply_split_screen_views).
        self.canvas2 = document(
            FieldCanvas(None), "field_p2", "Player 2's field view",
            "The same live match, drawn from Player 2's own driver-station wall.",
            "Only visible once Split Screen is enabled in the right column; hidden otherwise.")
        self.canvas2.setVisible(False)
        self._split_screen_active = False
        self.piece_count_label = QtWidgets.QLabel()
        self.piece_count_label.setAlignment(Qt.AlignCenter)
        self.piece_count_label.setFont(theme.technical_font(11, bold=True))
        self.piece_count_label.setStyleSheet(f"color: {theme.ACCENT_CYAN};")
        self.show_intent_check = document(
            QtWidgets.QCheckBox("Show AI Intent"), "show_intent", "Show AI intent",
            "Draws each AI robot's current target and active tactic name over the field, so you "
            "can see what a strategy is trying to do, not just what it's doing.",
            "The single best way to debug a strategy that looks like it's doing something odd -- "
            "watch its intent line and see whether it's aiming somewhere sensible.")
        self.show_intent_check.setChecked(True)
        self.show_intent_check.toggled.connect(self._on_show_intent_toggled)
        self.fullscreen_check = document(
            QtWidgets.QCheckBox("Full Screen Field"), "fullscreen_field", "Full screen field",
            "Hides every other panel and the tab bar so the field fills the window.",
            "Good for spectating or recording a match; the tab bar comes back the moment this "
            "is unchecked.")
        self.fullscreen_check.toggled.connect(self._set_fullscreen)
        self.console = ConsolePanel()
        self.transport_bar = TransportBar()
        self.transport_bar.play_pause_clicked.connect(self._toggle_paused)
        self.transport_bar.reset_clicked.connect(self._reset_match)
        self.transport_bar.slider.sliderPressed.connect(self._on_scrub_start)
        self.transport_bar.slider.valueChanged.connect(self._on_scrub_value_changed)
        center_column = QtWidgets.QWidget()
        center_layout = QtWidgets.QVBoxLayout(center_column)
        center_layout.setContentsMargins(0, 0, 0, 0)
        canvas_row = QtWidgets.QHBoxLayout()
        canvas_row.setContentsMargins(0, 0, 0, 0)
        canvas_row.addWidget(self.canvas, stretch=1)
        canvas_row.addWidget(self.canvas2, stretch=1)
        center_layout.addLayout(canvas_row, stretch=2)
        self.piece_count_widget = QtWidgets.QWidget()
        piece_count_row = QtWidgets.QHBoxLayout(self.piece_count_widget)
        piece_count_row.setContentsMargins(0, 0, 0, 0)
        piece_count_row.addStretch(1)
        piece_count_row.addWidget(self.piece_count_label)
        piece_count_row.addStretch(1)
        piece_count_row.addWidget(self.show_intent_check)
        piece_count_row.addWidget(self.fullscreen_check)
        center_layout.addWidget(self.piece_count_widget)
        center_layout.addWidget(self.console, stretch=1)
        center_layout.addWidget(self.transport_bar)
        self._center_column = center_column

        self.telemetry_panel = document(
            TelemetryPanel("TELEMETRY"), "telemetry", "Telemetry",
            "Live readouts for the currently selected robot: position, speed, what it's "
            "holding, and the current score.",
            "Numbers here update every tick while playing -- pause the match and scrub the bar "
            "above to see what they were at any earlier moment.")
        self.match_settings_panel = MatchSettingsPanel()
        self.controls_panel = ControlsPanel()
        self.view_mode_panel = ViewModePanel()
        self.view_mode_panel.view_changed.connect(self._on_view_mode_changed)
        self.player2_panel = Player2Panel()
        self.split_screen_panel = SplitScreenPanel()
        self.split_screen_panel.toggled_by_user.connect(self._on_split_screen_toggled)
        self.right_column = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(self.right_column)
        right_layout.addWidget(self.telemetry_panel)
        right_layout.addWidget(self.match_settings_panel)
        right_layout.addWidget(self.controls_panel)
        right_layout.addWidget(self.view_mode_panel)
        right_layout.addWidget(self.player2_panel)
        right_layout.addWidget(self.split_screen_panel)
        right_layout.addStretch(1)
        self.right_column.setMinimumWidth(220)

        main_splitter = QtWidgets.QSplitter(Qt.Horizontal)
        main_splitter.addWidget(self.left_column)
        main_splitter.addWidget(center_column)
        main_splitter.addWidget(self.right_column)
        main_splitter.setStretchFactor(0, 0)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setStretchFactor(2, 0)
        main_splitter.setSizes([360, 1000, 240])

        central_layout = QtWidgets.QHBoxLayout(self)
        central_layout.addWidget(main_splitter)
        self._fullscreen = False

        self._pressed_keys: set[int] = set()
        self._prev_pressed_keys: set[int] = set()
        self.canvas.keyPressEvent = self._key_press
        self.canvas.keyReleaseEvent = self._key_release

        self.keyboard = KeyboardInput(pressed_keys=lambda: self._pressed_keys, bindings=KEY_BINDINGS)
        self.gamepad = GamepadInput()
        # Both devices live at once -- see CombinedInput. This used to pick
        # one (gamepad whenever `available`), which meant a controller
        # merely plugged into the machine silently made W A S D do nothing,
        # with no message anywhere saying why.
        self.input_source = CombinedInput(self.keyboard, self.gamepad)
        self.gamepad_available = self.gamepad.available
        # This tick's already-polled input, read (not re-polled) by any
        # HumanController -- see that module's docstring for why it must
        # not own a second poll() call.
        self._latest_commands: tuple[DriveCommand, OperatorCommand] = (DriveCommand(), OperatorCommand())
        self.controls_panel.set_available(self.gamepad.available)

        # A second physical gamepad, wired to whichever roster robot
        # Player2Panel selects -- see _reset_match. Keyboard stays
        # single-player (it's already owned by the canvas' key events for
        # PRIMARY), so a second human is gamepad-only.
        self.gamepad2 = GamepadInput(index=1)
        self.player2_panel.set_gamepad_available(self.gamepad2.available)
        self._player2_commands: tuple[DriveCommand, OperatorCommand] = (DriveCommand(), OperatorCommand())
        self._player2_deposit_action = "l4"

        self.sounds = MatchSoundboard(ASSETS_DIR)

        self.paused = False
        self._selected_deposit_action = "l4"
        # Playback/scrub state -- see _on_scrub_start/_enter_playback_at_fraction.
        # playback_time is None while live; _live_snapshot holds each
        # robot's true physics state from the moment scrubbing started,
        # so resuming play can restore it exactly rather than continuing
        # from wherever the scrub display last parked the bodies.
        self.playback_time: float | None = None
        self._live_snapshot: dict[object, tuple] | None = None
        # (scores, region_scores, station_supply) captured the same moment
        # as _live_snapshot -- the scoring squares/badges FieldCanvas draws
        # read Match.region_scores/station_supply directly (see
        # _restore_state_at_time), so those need saving/restoring around a
        # scrub too, not just robot poses.
        self._live_match_snapshot: tuple | None = None
        # Free-piece positions from that same moment, keyed by piece object
        # (not id() -- these are live objects, not telemetry rows) so
        # _exit_playback can put them back rather than leaving them
        # stranded wherever the last scrub parked them.
        self._live_piece_positions: dict[object, tuple] | None = None
        self._updating_slider_programmatically = False
        self._reset_match()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / self.TICK_HZ))
        self.canvas.setFocus()

    @property
    def roster_panel(self) -> RosterPanel:
        return self.roster_config.roster_panel

    def _update_timer_interval(self) -> None:
        # Interval 0 makes Qt fire the timer again as soon as the event
        # loop is otherwise idle -- the sim then advances one fixed dt
        # per tick as fast as the CPU (and canvas repaint) allow, rather
        # than throttled to real time.
        fast = self.roster_panel.fast_forward_enabled()
        self.timer.setInterval(0 if fast else int(1000 / self.TICK_HZ))

    # -- match lifecycle -----------------------------------------------

    def _reset_match(self) -> None:
        self._set_fullscreen(False)
        alliance = self.primary_config_tab.settings_panel.alliance()
        self.match = build_demo_match(
            self.match_settings_panel.disable_friendly_collisions(),
            self.match_settings_panel.emit_coral_to_field(),
        )
        start_pose = sweep_trial.start_pose(alliance, -1)
        overrides = self.primary_config_tab.characteristics_overrides()
        characteristics = build_demo_characteristics(**overrides)
        self.robot = self.match.add_robot(characteristics, start_pose, alliance=alliance)
        self._robots_by_label = {"PRIMARY": self.robot}
        if self.roster_panel.ai_drives_primary():
            self._attach_strategy(self.robot, "PRIMARY", self.roster_panel.primary_strategy_name())
        else:
            # Unifies with the AI path: Match.step ticks every robot's
            # controller uniformly, rather than _tick special-casing "no
            # controller means drive it inline here".
            self.robot.controller = HumanController(
                command_provider=lambda: self._latest_commands,
                deposit_action_provider=self._effective_deposit_action,
            )

        for roster_alliance in ("blue", "red"):
            for i, (row, config_tab) in enumerate(self.roster_panel.roster_rows(roster_alliance)):
                self._spawn_roster_robot(roster_alliance, i, row.strategy_name(), config_tab)

        self.player2_panel.set_options([label for label in self.robot_labels() if label != "PRIMARY"])
        self._player2_commands = (DriveCommand(), OperatorCommand())
        self._player2_deposit_action = "l4"
        label2 = self.player2_panel.selected_label()
        robot2 = self._robots_by_label.get(label2) if label2 is not None else None
        if robot2 is not None:
            # Overwrites the AI controller _spawn_roster_robot just
            # attached -- see Stage 4 of the driver-practice plan for why
            # this doesn't touch _spawn_roster_robot/RosterEntryRow
            # themselves.
            robot2.controller = HumanController(
                command_provider=lambda: self._player2_commands,
                deposit_action_provider=lambda: self._effective_deposit_action_for(robot2, self._player2_deposit_action),
            )
        self._update_split_screen_availability(robot2)

        self._endgame_cue_played = False
        self.canvas.match = self.match
        self.canvas2.match = self.match
        self.console.reset()
        self._logged_event_count = 0
        self.paused = True
        self.telemetry = TelemetryRecorder(self.match)
        self.playback_time = None
        self._live_snapshot = None
        self._live_match_snapshot = None
        self._live_piece_positions = None
        self.canvas.playback_pieces = None
        self.canvas.playback_targets = None
        self.canvas.playback_tactics = None
        self.transport_bar.slider.setEnabled(False)
        self._update_piece_counts()
        self.canvas.setFocus()
        # Wall-clock pacing state for _advance_realtime -- reset here (not
        # just in __init__) so a fresh match never inherits a stale
        # last-tick timestamp from whatever the previous match was doing.
        self._last_tick_time = time.perf_counter()
        self._accumulator = 0.0
        self.match_reset.emit(self.match)

    def robot_labels(self) -> list[str]:
        """Stable per-robot keys ("PRIMARY" / "BLUE 0" / "RED 0" / ...)
        shared with StrategyEditor.set_robots() -- BLUE/RED indices are
        0-based, matching _spawn_roster_robot's own `index`, not the
        1-based numbering ROBOT CONFIG's tab labels happen to use."""
        labels = ["PRIMARY"]
        for alliance in ("blue", "red"):
            for i in range(len(self.roster_panel.roster_rows(alliance))):
                labels.append(f"{alliance.upper()} {i}")
        return labels

    def robot_for_label(self, label: str):
        """The live Robot currently spawned under `label`, or None --
        None either before the first RESET or if `label` names a roster
        slot that isn't spawned this match (stale STRATEGY tab
        selection after a roster edit)."""
        return self._robots_by_label.get(label)

    def _resolve_strategy(self, label: str, strategy_name: str):
        """A Strategy staged for `label` in the STRATEGY tab (Apply To
        Robot) takes priority over the roster's file-selected strategy
        -- that's what makes "edit here, RESET to try it" work."""
        if self.strategy_provider is not None:
            override = self.strategy_provider(label)
            if override is not None:
                return override
        return strategy_io.load_strategy(STRATEGY_FILES[strategy_name])

    def _attach_strategy(self, robot, label: str, strategy_name: str) -> None:
        strategy = self._resolve_strategy(label, strategy_name)
        robot.controller = StrategyController(strategy, robot)

    def _spawn_roster_robot(self, alliance: str, index: int, strategy_name: str, config_tab: RobotConfigTab) -> None:
        """Extra AI robots line up at that alliance's coral stations,
        staggered along the wall so they don't spawn stacked on top of
        each other (or on the primary robot, which starts at station 0)."""
        pose = sweep_trial.start_pose(alliance, index)
        overrides = config_tab.characteristics_overrides()
        overrides["name"] = f"{alliance}-ai-{index}"
        characteristics = build_demo_characteristics(**overrides)
        robot = self.match.add_robot(characteristics, pose, alliance=alliance)
        self._robots_by_label[f"{alliance.upper()} {index}"] = robot
        self._attach_strategy(robot, f"{alliance.upper()} {index}", strategy_name)

    def _toggle_paused(self) -> None:
        if self.paused and self.playback_time is not None:
            # Resuming from a scrub -- put the true live physics state
            # back before unpausing, discarding the scrub display's
            # positions (see _restore_state_at_time), so the sim
            # continues from where it actually left off rather than
            # jumping from wherever the user scrubbed to.
            self._exit_playback()
        self.paused = not self.paused
        if not self.paused:
            # Run just started (PLAY pressed) -- automatically expand
            # the field to fill the window. Ending the match or hitting
            # RESET contracts it back (see _tick / _reset_match).
            self._set_fullscreen(True)
            # Otherwise the wall-clock time spent paused (however long)
            # would show up as one huge frame_dt on the next tick and get
            # burned through as a burst of catch-up steps.
            self._last_tick_time = time.perf_counter()
            if self.match.elapsed == 0.0:
                # elapsed only reads exactly 0.0 at the one moment PLAY is
                # first pressed on a fresh match -- every later resume
                # (mid-match pause, or after scrubbing) has already
                # advanced past it.
                self.sounds.play("start_auto")
        elif self.match.elapsed > 0.0 and not self.match.ended:
            # Excludes the RESET-induced initial paused state (elapsed ==
            # 0) and the match-end auto-pause, which fires from _tick, not
            # here, and plays its own "match_end" cue instead.
            self.sounds.play("pause")

    def _on_show_intent_toggled(self, checked: bool) -> None:
        self.canvas.show_intent = checked

    def _on_view_mode_changed(self, mode: str) -> None:
        alliance, preset = VIEW_MODES[mode]
        self.canvas.set_driver_view(alliance, preset)

    def _update_split_screen_availability(self, robot2) -> None:
        """Split screen only makes sense with two physical gamepads and
        an opposing human to show: Player 2 piloting a RED robot, so
        each pane can show that player's own driver wall. Re-evaluated
        every RESET, since Player 2's robot (and its alliance) can
        change there."""
        gamepads_ok = self.gamepad_available and self.gamepad2.available
        robot2_red = robot2 is not None and robot2.alliance == "red"
        available = gamepads_ok and robot2_red
        if not gamepads_ok:
            reason = "Needs two gamepads connected"
        elif not robot2_red:
            reason = "Player 2 must be piloting a RED robot"
        else:
            reason = ""
        self.split_screen_panel.set_available(available, reason)
        if self._split_screen_active:
            if available:
                self._apply_split_screen_views(robot2)
            else:
                self._set_split_screen(False)

    def _apply_split_screen_views(self, robot2) -> None:
        self.canvas.set_driver_view(self.robot.alliance, DRIVER)
        self.canvas2.set_driver_view(robot2.alliance, DRIVER)

    def _on_split_screen_toggled(self, checked: bool) -> None:
        self._set_split_screen(checked)

    def _set_split_screen(self, enabled: bool) -> None:
        if enabled == self._split_screen_active:
            return
        self._split_screen_active = enabled
        self.canvas2.setVisible(enabled)
        # The single-canvas FIELD VIEW picker doesn't apply while both
        # panes are each showing their own alliance's driver wall.
        self.view_mode_panel.setEnabled(not enabled)
        if enabled:
            label2 = self.player2_panel.selected_label()
            robot2 = self._robots_by_label.get(label2) if label2 is not None else None
            if robot2 is not None:
                self._apply_split_screen_views(robot2)
        else:
            self._on_view_mode_changed(self.view_mode_panel.current_mode())
        self.split_screen_panel.set_checked(enabled)

    def _set_fullscreen(self, enabled: bool) -> None:
        """Hides the roster/telemetry/console panels so the field
        canvas can fill as much of a laptop screen as possible. Kept
        as an explicit toggle (rather than only the auto-trigger in
        _toggle_paused/_tick/_reset_match) so the user can also flip
        it by hand via the checkbox."""
        if enabled == self._fullscreen:
            return
        self._fullscreen = enabled
        self.left_column.setVisible(not enabled)
        self.right_column.setVisible(not enabled)
        self.console.setVisible(not enabled)
        if self.fullscreen_check.isChecked() != enabled:
            self.fullscreen_check.blockSignals(True)
            self.fullscreen_check.setChecked(enabled)
            self.fullscreen_check.blockSignals(False)
        self.fullscreen_changed.emit(enabled)

    # -- playback/scrub ----------------------------------------------------

    def _on_scrub_start(self) -> None:
        """User grabbed the slider handle (mouse-down, before any drag).
        A no-op if the slider is disabled (live/no telemetry yet) --
        Qt still emits sliderPressed for a disabled widget in some
        bindings, so this guards explicitly rather than relying on
        setEnabled(False) alone to block it."""
        if not self.transport_bar.slider.isEnabled():
            return
        if self._live_snapshot is None:
            self._capture_live_snapshot()
        self._enter_playback_at_fraction(self.transport_bar.slider.value() / 1000.0)

    def _on_scrub_value_changed(self, value: int) -> None:
        """Fires on every value change: dragging, a groove click, or
        arrow keys. Ignored while `_tick` is itself syncing the slider
        to live progress (see `_updating_slider_programmatically`) so
        that doesn't get misread as a user scrub."""
        if self._updating_slider_programmatically:
            return
        if not self.transport_bar.slider.isEnabled():
            return
        if self._live_snapshot is None:
            self._capture_live_snapshot()
        self._enter_playback_at_fraction(value / 1000.0)

    def _capture_live_snapshot(self) -> None:
        """Record each robot's true physics state before scrubbing
        overwrites it for display, so resuming play (_toggle_paused)
        can restore exactly where the live sim actually left off. Also
        captures match-level scoring state for the same reason -- see
        _live_match_snapshot."""
        self._live_snapshot = {
            robot: (
                robot.chassis.body.position, robot.chassis.body.angle,
                robot.chassis.body.velocity, robot.chassis.body.angular_velocity,
            )
            for robot in self.match.robots
        }
        self._live_match_snapshot = (
            dict(self.match.scores),
            {region: dict(actions) for region, actions in self.match.region_scores.items()},
            dict(self.match.station_supply),
        )
        self._live_piece_positions = {
            piece: piece.body.position for piece in self.match.active_pieces if piece.held_by is None
        }

    def _enter_playback_at_fraction(self, fraction: float) -> None:
        fraction = max(0.0, min(1.0, fraction))
        target_time = fraction * self.match.config.total_duration
        self.playback_time = target_time
        self._restore_state_at_time(target_time)
        total = self.match.config.total_duration
        self.transport_bar.time_label.setText(f"{_fmt_time(target_time)} / {_fmt_time(total)}")
        self.canvas.update()

    def _restore_state_at_time(self, target_time: float) -> None:
        """Pose/orientation/velocity for robots, rewound from telemetry.
        Game pieces are handed to FieldCanvas as a snapshot list
        (canvas.playback_pieces) rather than repositioned in place, because
        a scored piece is permanently removed from Match.active_pieces and
        pymunk (see Match.step) -- there's no live GamePiece left to move
        once that's happened, only its recorded PieceSnapshot rows (see
        common_sim/match/telemetry.py). This is only relied on for
        scrubbing a match that has already ended, where no new piece can
        spawn to collide with a stale piece_id.

        Held pieces are repositioned via sync_held_piece_positions() so
        they don't lag behind the chassis jump -- FieldCanvas still draws
        them from the same playback_pieces list, so that call matters for
        held-piece *physics state* (e.g. a subsequent live resume), not
        for what's drawn while scrubbed.

        Also rewinds match.scores/region_scores/station_supply from the
        matching MatchSnapshot, so FieldCanvas's REEF grid squares and
        station-remaining badges (which read those dicts directly, not
        telemetry) show the scrubbed-to moment instead of staying pinned
        at wherever the live/paused match actually left them."""
        playback_targets: dict[str, str | None] = {}
        playback_tactics: dict[str, str | None] = {}
        for robot in self.match.robots:
            snapshot = self.telemetry.get_robot_state_at_time(target_time, robot.characteristics.name)
            if snapshot is None:
                continue
            robot.chassis.body.position = (snapshot.position_x, snapshot.position_y)
            robot.chassis.body.angle = math.radians(snapshot.orientation_deg)
            robot.chassis.body.velocity = (snapshot.velocity_x, snapshot.velocity_y)
            robot.sync_held_piece_positions()
            playback_targets[robot.characteristics.name] = snapshot.target_name
            playback_tactics[robot.characteristics.name] = snapshot.tactic_name

        self.canvas.playback_pieces = self.telemetry.get_piece_states_at_time(target_time)
        self.canvas.playback_targets = playback_targets
        self.canvas.playback_tactics = playback_tactics

        match_snapshot = self.telemetry.get_match_state_at_time(target_time)
        if match_snapshot is not None:
            self.match.scores = dict(match_snapshot.alliance_scores)
            self.match.region_scores = {
                region: dict(actions) for region, actions in match_snapshot.region_scores.items()
            }
            locations_by_name = {location.name: location for location in self.match.field.intake_locations}
            for name, remaining in match_snapshot.station_supply.items():
                location = locations_by_name.get(name)
                if location is not None:
                    self.match.station_supply[location] = remaining

    def _exit_playback(self) -> None:
        if self._live_snapshot is not None:
            for robot, (position, angle, velocity, angular_velocity) in self._live_snapshot.items():
                robot.chassis.body.position = position
                robot.chassis.body.angle = angle
                robot.chassis.body.velocity = velocity
                robot.chassis.body.angular_velocity = angular_velocity
                robot.sync_held_piece_positions()
        if self._live_match_snapshot is not None:
            scores, region_scores, station_supply = self._live_match_snapshot
            self.match.scores = scores
            self.match.region_scores = region_scores
            self.match.station_supply = station_supply
        if self._live_piece_positions is not None:
            for piece, position in self._live_piece_positions.items():
                piece.body.position = position
        self.canvas.playback_pieces = None
        self.canvas.playback_targets = None
        self.canvas.playback_tactics = None
        self.playback_time = None
        self._live_snapshot = None
        self._live_match_snapshot = None
        self._live_piece_positions = None

    # -- input -----------------------------------------------------------

    def _key_press(self, event) -> None:
        self._pressed_keys.add(event.key())
        if event.key() in LEVEL_KEYS:
            self._selected_deposit_action = LEVEL_KEYS[event.key()]

    def _key_release(self, event) -> None:
        self._pressed_keys.discard(event.key())

    def _cycle_reef_level(self) -> None:
        self._selected_deposit_action = self._next_level(self._selected_deposit_action)

    def _next_level(self, current: str) -> str:
        start = REEF_LEVELS.index(current) if current in REEF_LEVELS else -1
        return REEF_LEVELS[(start + 1) % len(REEF_LEVELS)]

    # Wall-clock pacing guards for _advance_realtime: a frame_dt clamp so a
    # long stall (a slow repaint, or the process briefly losing the CPU)
    # can't inject one giant catch-up burst, and a steps-per-tick cap so
    # sustained overload degrades to "runs a bit slow" rather than a
    # spiral-of-death where each catch-up step costs more than it buys back.
    _MAX_FRAME_DT = 0.1
    _MAX_STEPS_PER_TICK = 4

    def _advance_one_step(self, dt: float) -> None:
        # Once the match has ended, Match.step is a no-op (elapsed stops
        # advancing) -- skip telemetry.tick() too, or it would keep
        # appending duplicate same-timestamp frames for as long as the
        # app sits open and unpaused.
        if self.match.ended:
            return
        self.match.step(dt)
        self._drain_new_events()
        self.telemetry.tick()

    def _advance_realtime(self) -> None:
        """Steps the sim by as many fixed dt=1/60 ticks as real wall-clock
        time actually elapsed since the last tick, instead of always
        exactly one -- otherwise a slower repaint (driver-station
        perspective costs more per vertex than the old orthographic path)
        would silently make the sim run in slow motion, which defeats the
        whole point of practicing cycle timing. Not used under
        fast-forward -- see _tick, which keeps that mode's original
        "exactly one fixed dt per timer fire, timer firing as fast as the
        event loop allows" behavior untouched."""
        now = time.perf_counter()
        frame_dt = min(now - self._last_tick_time, self._MAX_FRAME_DT)
        self._last_tick_time = now
        self._accumulator += frame_dt
        dt = 1.0 / self.TICK_HZ
        steps = 0
        while self._accumulator >= dt and steps < self._MAX_STEPS_PER_TICK:
            self._advance_one_step(dt)
            self._accumulator -= dt
            steps += 1
            if self.match.ended:
                break

    # Sentinel default for _orient_drive_command's `alliance` param -- None
    # is itself a meaningful value there (TOP-DOWN), so "not passed" needs
    # its own marker to fall back to self.canvas.driver_alliance.
    _CANVAS_ALLIANCE = object()

    def _orient_drive_command(self, drive: DriveCommand, alliance: str | None = _CANVAS_ALLIANCE) -> DriveCommand:
        """Remaps a polled stick reading from driver-relative axes (up =
        away from the driver, right = the driver's own right hand) to
        field-absolute vx/vy for `alliance`'s driver-station view.
        `alliance=None` (TOP-DOWN) passes `drive` through unchanged --
        its established convention (up=+y, right=+x) already matches the
        screen directly, and nobody's asked to relearn it there.

        `alliance` is whichever pane's perspective is actually driving
        this player right now: self.canvas.driver_alliance for Player 1
        always, but self.canvas2.driver_alliance for Player 2 while
        split screen is active (each player faces their own wall) --
        see the call sites in _tick. Omitted, it defaults to
        self.canvas.driver_alliance (the pre-split-screen behavior, and
        still correct for Player 1 and for the non-split-screen case).

        omega is negated too, not just vx/vy: a ground-level driver view
        is left-handed relative to the top-down map's screen convention
        (looking horizontally along the field's forward axis flips which
        way "right cross up" points, versus looking straight down from
        above), so the same field-CCW spin that reads as counter-clockwise
        on the map reads as clockwise from the driver's own eye. Without
        this, "rotate cw" swings the nose toward the driver's left, which
        is exactly backwards from every chase-view driving convention."""
        if alliance is self._CANVAS_ALLIANCE:
            alliance = self.canvas.driver_alliance
        if alliance is None:
            return drive
        vx, vy = orient_drive(alliance, up=drive.vy, right=drive.vx)
        return DriveCommand(vx=vx, vy=vy, omega=-drive.omega)

    def _tick(self) -> None:
        drive, operator = self.input_source.poll()
        drive = self._orient_drive_command(drive)
        self._latest_commands = (drive, operator)

        drive2, operator2 = self.gamepad2.poll()
        # Split screen: Player 2 faces their own pane's wall, not
        # Player 1's -- outside split screen there's only one shared
        # pane, so canvas's alliance is the only one there is.
        alliance2 = self.canvas2.driver_alliance if self._split_screen_active else self.canvas.driver_alliance
        drive2 = self._orient_drive_command(drive2, alliance2)
        self._player2_commands = (drive2, operator2)

        # Edge-trigger off the same _pressed_keys set WASD driving polls,
        # rather than QKeyEvent.isAutoRepeat() -- on some platforms the
        # very first physical keypress can arrive already flagged as a
        # repeat, which silently swallowed this cycle before it ever ran.
        # Both this and the gamepad's X button are always live now that
        # input_source is a CombinedInput, so either one cycles the level
        # regardless of what's plugged in.
        just_pressed = self._pressed_keys - self._prev_pressed_keys
        if TOGGLE_REEF_LEVEL_KEY in just_pressed or operator.cycle_level:
            self._cycle_reef_level()
        self._prev_pressed_keys = set(self._pressed_keys)

        if operator2.cycle_level:
            self._player2_deposit_action = self._next_level(self._player2_deposit_action)

        if operator.pause_toggle or operator2.pause_toggle:
            self._toggle_paused()

        if not self.paused:
            phase_before = self.match.phase
            if self.roster_panel.fast_forward_enabled():
                self._advance_one_step(1.0 / self.TICK_HZ)
            else:
                self._advance_realtime()
            if phase_before == Phase.AUTO and self.match.phase == Phase.TELEOP:
                self.sounds.play("start_teleop")
            if (
                not self._endgame_cue_played and not self.match.ended
                and self.match.phase == Phase.TELEOP
                and self.match.config.total_duration - self.match.elapsed <= ENDGAME_SECONDS
            ):
                self.sounds.play("start_endgame")
                self._endgame_cue_played = True
            # The match just ran out the clock -- drop into the same
            # paused state a manual pause would, so the transport bar's
            # play/pause button and scrub slider immediately reflect it
            # instead of sitting on "playing" over a sim that's actually
            # stopped advancing.
            if self.match.ended:
                self.paused = True
                self._set_fullscreen(False)
                self.sounds.play("match_end")

        # sync_slider=False while scrubbing: the slider already reflects
        # where the user dragged it (see _enter_playback_at_fraction),
        # and match.elapsed hasn't moved since we're paused -- syncing it
        # here every tick would just snap the handle back underneath
        # their drag before it ever completes. The programmatic-update
        # flag additionally guards _on_scrub_value_changed against
        # misreading this call's own setValue() as a user scrub.
        #
        # Likewise the displayed elapsed time must be playback_time (not
        # match.elapsed) while scrubbing -- match.elapsed is frozen at
        # wherever the sim was paused, so passing it here every tick was
        # overwriting the scrub position _enter_playback_at_fraction had
        # just written into the label, pinning the readout at e.g. "2:30 /
        # 2:30" (the paused end-of-match time) no matter where the slider
        # actually sat.
        displayed_elapsed = self.playback_time if self.playback_time is not None else self.match.elapsed
        self._updating_slider_programmatically = True
        self.transport_bar.set_progress(
            displayed_elapsed, self.match.config.total_duration, self.paused,
            sync_slider=self.playback_time is None,
        )
        self._updating_slider_programmatically = False
        self._update_scrub_availability()
        self._update_telemetry()
        self._update_piece_counts()
        self.canvas.update()
        if self._split_screen_active:
            self.canvas2.update()
        self.tick_completed.emit()

    def _update_scrub_availability(self) -> None:
        """The slider is only meaningful to drag once there's recorded
        telemetry to scrub through and the live sim isn't actively
        stepping (dragging mid-physics-step doesn't mean anything)."""
        can_scrub = self.paused and bool(self.telemetry.match_frames)
        if self.transport_bar.slider.isEnabled() != can_scrub:
            self.transport_bar.slider.setEnabled(can_scrub)

    def _update_piece_counts(self) -> None:
        held = self.robot.held_pieces
        parts = []
        for piece_type in PIECE_TYPES:
            held_count = sum(1 for p in held if p.piece_type == piece_type)
            capacity = self.robot.characteristics.capacity_for(piece_type)
            parts.append(f"{piece_type.upper()}: {held_count}/{capacity}")
        self.piece_count_label.setText("   ".join(parts))

    def _effective_deposit_action(self) -> str:
        return self._effective_deposit_action_for(self.robot, self._selected_deposit_action)

    def _effective_deposit_action_for(self, robot, manual: str) -> str:
        """The manually-selected level/action, unless `robot` is
        currently sitting in a single-action scoring zone (PROCESSOR,
        NET) holding a piece that zone accepts -- ALGAE only ever scores
        at one of those two, so there's nothing to pick between and
        requiring an explicit GUI selection just adds friction. REEF
        faces offer 4 actions at once (L1-L4), so they're deliberately
        excluded from this and stay a manual/toggle (X) choice."""
        pose = robot.pose
        held_types = {p.piece_type for p in robot.held_pieces}
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

    # -- sweep replay ------------------------------------------------------

    def load_replay(self, match, robots_by_label: dict, telemetry: TelemetryRecorder, *, title: str = "REPLAY") -> None:
        """Adopt a completed match (from game_specific.reefscape.
        sweep_trial.replay_trial) for scrubbing, reusing the existing
        _restore_state_at_time / _enter_playback_at_fraction machinery
        unchanged -- RESET afterwards discards the replay via the normal
        _reset_match path, a deliberate, documented exit back to a live
        match."""
        self.loaded_replay_title = title
        self.match = match
        self.robot = robots_by_label.get("PRIMARY") or next(iter(robots_by_label.values()))
        self._robots_by_label = dict(robots_by_label)
        self.telemetry = telemetry

        self._set_split_screen(False)
        self.canvas.match = match
        self.canvas2.match = match
        self.console.reset()
        all_events = list(match.events)
        self.console.append_events(all_events)
        self._logged_event_count = len(all_events)

        self.paused = True
        self.playback_time = None
        self._live_snapshot = None
        self._live_match_snapshot = None
        self._live_piece_positions = None
        self.canvas.playback_pieces = None
        self.canvas.playback_targets = None
        self.canvas.playback_tactics = None
        self.transport_bar.slider.setEnabled(bool(telemetry.match_frames))
        self._update_piece_counts()
        self.canvas.update()
        self.match_reset.emit(match)

    def set_ticking(self, active: bool) -> None:
        """Stop the 60 Hz timer while a sweep runs -- not optional
        polish: without it a sweep is measurably slower (this widget's
        own match keeps stepping/repainting every tick alongside it) and
        the UI stutters."""
        if active:
            if not self.timer.isActive():
                self.timer.start(int(1000 / self.TICK_HZ))
        else:
            self.timer.stop()


class ReefscapeWindow(QtWidgets.QMainWindow):
    """Thin shell: a MatchView tab and a STRATEGY tab -- StrategyEditor
    (gui_utils/strategy_editor.py) beside a live StrategyGraphPanel
    (gui_utils/strategy_graph.py) -- wired so the editor always knows
    the live roster, can stage a strategy for the next RESET, and the
    graph mirrors both in-progress edits and (while the STRATEGY tab's
    selected robot is actually running) the arbiter's real transitions."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("sparky-sim -- REEFSCAPE")
        self._graph_event_count = 0

        self.match_view = MatchView()
        self.strategy_editor = StrategyEditor()
        self.strategy_graph = StrategyGraphPanel()
        self.sweep_tab = SweepTab(strategy_provider=self.strategy_editor.strategy_for)
        self.match_view.strategy_provider = self.strategy_editor.strategy_for
        self.match_view.roster_changed.connect(self._sync_strategy_robots)
        self.match_view.match_reset.connect(self._on_match_reset)
        self.match_view.tick_completed.connect(self._on_tick_completed)
        self.match_view.fullscreen_changed.connect(self._on_fullscreen_changed)
        self.strategy_editor.changed.connect(self._refresh_strategy_graph)
        self.strategy_editor.robot_combo.currentTextChanged.connect(lambda *_: self._refresh_strategy_graph())
        self.strategy_graph.node_clicked.connect(self.strategy_editor.select_rule)
        self.sweep_tab.replay_requested.connect(self._on_replay_requested)
        self.sweep_tab.running_changed.connect(lambda running: self.match_view.set_ticking(not running))

        # SEARCH keeps its own roster too, for the same reason SWEEP does,
        # and pauses the live MATCH tick while it runs -- both tabs
        # saturate the CPU with worker processes and a ticking match
        # competing for those cores makes the whole window stutter.
        self.search_tab = SearchTab()
        self.search_tab.running_changed.connect(lambda running: self.match_view.set_ticking(not running))

        strategy_tab = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        strategy_tab.addWidget(self.strategy_editor)
        strategy_tab.addWidget(self.strategy_graph)
        strategy_tab.setSizes([560, 480])

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self.match_view, "MATCH")
        self.tabs.addTab(strategy_tab, "STRATEGY")
        self.tabs.addTab(self.sweep_tab, "SWEEP")
        self.tabs.addTab(self.search_tab, "SEARCH")
        self.setCentralWidget(self.tabs)
        self.resize(1600, 800)

        self.setWindowTitle(self.windowTitle() + (" [gamepad]" if self.match_view.gamepad_available else " [keyboard]"))
        self._sync_strategy_robots()
        self._on_match_reset(self.match_view.match)

    def _on_fullscreen_changed(self, enabled: bool) -> None:
        # Hides the MATCH/STRATEGY/SWEEP tab bar too, so the field
        # canvas gets the whole window rather than just the space
        # inside its own tab.
        self.tabs.tabBar().setVisible(not enabled)

    def _on_replay_requested(self, job) -> None:
        # SWEEP keeps its own roster, independent of MATCH by design -- an
        # in-flight sweep must not change because someone edited the MATCH
        # tab -- so replaying a run only ever touches the MATCH tab, never
        # the sweep's own state.
        QtWidgets.QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            match, robots_by_label, telemetry = sweep_trial.replay_trial(job)
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
        self.match_view.load_replay(match, robots_by_label, telemetry, title=f"Sweep run #{job.index}")
        self.tabs.setCurrentWidget(self.match_view)

    def closeEvent(self, event) -> None:
        # Required: a live ProcessPoolExecutor keeps the process alive
        # after the window closes otherwise.
        self.sweep_tab.shutdown()
        self.search_tab.shutdown()
        super().closeEvent(event)

    def _sync_strategy_robots(self) -> None:
        self.strategy_editor.set_robots(self.match_view.robot_labels())
        self._refresh_strategy_graph()

    def _on_match_reset(self, match) -> None:
        self.strategy_editor.set_field(match.field)
        self._graph_event_count = 0
        self._refresh_strategy_graph()

    def _refresh_strategy_graph(self) -> None:
        strategy = self.strategy_editor.current_strategy()
        if strategy is not None:
            self.strategy_graph.set_strategy(strategy)

    def _on_tick_completed(self) -> None:
        label = self.strategy_editor.current_label()
        robot = self.match_view.robot_for_label(label) if label is not None else None
        controller = robot.controller if robot is not None else None
        # The rule graph only means anything for a StrategyController --
        # a human-driven PRIMARY now always has *some* controller
        # (HumanController, see MatchView._reset_match), so this can no
        # longer use "controller is None" as the "nothing to show" guard.
        if not isinstance(controller, StrategyController):
            return
        self.strategy_graph.set_active_rule(controller.active_rule_name)

        all_events = list(self.match_view.match.events)
        for event in all_events[self._graph_event_count:]:
            if event.kind == "behavior_change" and event.data.get("robot") is robot:
                self.strategy_graph.record_transition(
                    event.timestamp, event.data.get("from"), event.data.get("to"), event.data.get("trigger"),
                )
        self._graph_event_count = len(all_events)


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    theme.apply_app_theme(app)
    window = ReefscapeWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
