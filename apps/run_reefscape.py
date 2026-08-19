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
from apps.sweep_tab import SweepTab
from common_sim.analysis.metrics import extract_metrics
from common_sim.control import strategy_io
from common_sim.control.input_sources import GamepadInput, KeyBindings, KeyboardInput
from common_sim.control.strategy import StrategyController
from common_sim.field.field_config import point_in_polygon
from common_sim.match.match import Match, MatchConfig, Phase
from common_sim.match.telemetry import TelemetryRecorder
from game_specific.reefscape import sweep_trial
from game_specific.reefscape.game_pieces import ALGAE_TYPE, CORAL_TYPE
from gui_utils import theme
from gui_utils.console_panel import ConsolePanel
from gui_utils.field_canvas import FieldCanvas
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

    def set_available(self, available: bool) -> None:
        bindings = GAMEPAD_BINDINGS if available else KEYBOARD_BINDINGS
        heading = "GAMEPAD" if available else "KEYBOARD"
        lines = [f"{control}: {action}" for control, action in bindings]
        self.label.setText(f"{heading}\n" + "\n".join(lines))


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

        self.play_pause_button = QtWidgets.QPushButton("⏸")
        self.play_pause_button.setFixedWidth(40)
        self.play_pause_button.clicked.connect(self.play_pause_clicked)
        layout.addWidget(self.play_pause_button)

        self.slider = ScrubSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.setEnabled(False)  # enabled by MatchView once paused with telemetry recorded
        self.slider.setToolTip("Match progress -- pause to scrub and review")
        layout.addWidget(self.slider, stretch=1)

        self.time_label = QtWidgets.QLabel("0:00 / 0:00")
        self.time_label.setFont(theme.technical_font(10))
        self.time_label.setMinimumWidth(90)
        layout.addWidget(self.time_label)

        self.reset_button = QtWidgets.QPushButton("RESET")
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

        self.canvas = FieldCanvas(None)  # match assigned by _reset_match()
        self.piece_count_label = QtWidgets.QLabel()
        self.piece_count_label.setAlignment(Qt.AlignCenter)
        self.piece_count_label.setFont(theme.technical_font(11, bold=True))
        self.piece_count_label.setStyleSheet(f"color: {theme.ACCENT_CYAN};")
        self.show_intent_check = QtWidgets.QCheckBox("Show AI Intent")
        self.show_intent_check.setChecked(True)
        self.show_intent_check.toggled.connect(self._on_show_intent_toggled)
        self.fullscreen_check = QtWidgets.QCheckBox("Full Screen Field")
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
        center_layout.addWidget(self.canvas, stretch=2)
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

        self.telemetry_panel = TelemetryPanel("TELEMETRY")
        self.match_settings_panel = MatchSettingsPanel()
        self.controls_panel = ControlsPanel()
        self.right_column = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(self.right_column)
        right_layout.addWidget(self.telemetry_panel)
        right_layout.addWidget(self.match_settings_panel)
        right_layout.addWidget(self.controls_panel)
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
        gamepad = GamepadInput()
        self.input_source = gamepad if gamepad.available else self.keyboard
        self.gamepad_available = gamepad.available
        self.controls_panel.set_available(gamepad.available)

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

        for roster_alliance in ("blue", "red"):
            for i, (row, config_tab) in enumerate(self.roster_panel.roster_rows(roster_alliance)):
                self._spawn_roster_robot(roster_alliance, i, row.strategy_name(), config_tab)

        self.canvas.match = self.match
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

    def _on_show_intent_toggled(self, checked: bool) -> None:
        self.canvas.show_intent = checked

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
        start = REEF_LEVELS.index(self._selected_deposit_action) if self._selected_deposit_action in REEF_LEVELS else -1
        self._selected_deposit_action = REEF_LEVELS[(start + 1) % len(REEF_LEVELS)]

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
            if self.robot.controller is None:
                c = self.robot.characteristics
                self.robot.drive_field_relative(dt, drive.vx * c.max_speed, drive.vy * c.max_speed, drive.omega * c.max_angular_speed)
                self.robot.set_intake_active(operator.intake_active)
                action = self._effective_deposit_action()
                self.robot.set_deposit_active(operator.deposit_active, action=action)
            # Once the match has ended, Match.step is a no-op (elapsed
            # stops advancing) -- skip telemetry.tick() too, or it would
            # keep appending duplicate same-timestamp frames for as long
            # as the app sits open and unpaused.
            if not self.match.ended:
                self.match.step(dt)
                self._drain_new_events()
                self.telemetry.tick()
            # The match just ran out the clock -- drop into the same
            # paused state a manual pause would, so the transport bar's
            # play/pause button and scrub slider immediately reflect it
            # instead of sitting on "playing" over a sim that's actually
            # stopped advancing.
            if self.match.ended:
                self.paused = True
                self._set_fullscreen(False)

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
        """The manually-selected level/action, unless the robot is
        currently sitting in a single-action scoring zone (PROCESSOR,
        NET) holding a piece that zone accepts -- ALGAE only ever scores
        at one of those two, so there's nothing to pick between and
        requiring an explicit GUI selection just adds friction. REEF
        faces offer 4 actions at once (L1-L4), so they're deliberately
        excluded from this and stay a manual/toggle (X) choice."""
        manual = self._selected_deposit_action
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

        self.canvas.match = match
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

        strategy_tab = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        strategy_tab.addWidget(self.strategy_editor)
        strategy_tab.addWidget(self.strategy_graph)
        strategy_tab.setSizes([560, 480])

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self.match_view, "MATCH")
        self.tabs.addTab(strategy_tab, "STRATEGY")
        self.tabs.addTab(self.sweep_tab, "SWEEP")
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
        if controller is None:
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
