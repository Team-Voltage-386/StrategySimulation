"""
Minimal interactive single-robot viewer: a keyboard-driven swerve
chassis over a small placeholder field. This exercises the full
common_sim + gui_utils.FieldCanvas pipeline end-to-end before any real
game_specific package exists (see ARCHITECTURE.md's build sequencing).

Controls: WASD translate (field-relative), LEFT/RIGHT rotate,
SPACE hold to intake, F hold to deposit -- or an Xbox controller if one
is plugged in (left stick translate, right stick X rotate, A intake,
B deposit), auto-preferred over keyboard when available.
"""
from __future__ import annotations

import sys

from pyqtgraph.Qt import QtCore, QtWidgets

from common_sim.control.input_sources import GamepadInput, KeyBindings, KeyboardInput
from common_sim.field.field_config import FieldConfig, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.match.telemetry import TelemetryRecorder
from common_sim.robot.characteristics import RobotCharacteristics
from gui_utils import theme
from gui_utils.field_canvas import FieldCanvas
from gui_utils.scrub_slider import ScrubSlider

Qt = QtCore.Qt

DEPOSIT_ACTION = "score"

KEY_BINDINGS = KeyBindings(
    forward=Qt.Key_W, backward=Qt.Key_S, left=Qt.Key_A, right=Qt.Key_D,
    rotate_ccw=Qt.Key_Left, rotate_cw=Qt.Key_Right,
    intake=Qt.Key_Space, deposit=Qt.Key_F,
)

PIECE_TYPE = "widget"


def build_demo_match() -> Match:
    field = FieldConfig(
        width=300,
        height=200,
        scoring_regions=(
            ScoringRegion(
                name="goal",
                vertices=((240, 60), (290, 60), (290, 140), (240, 140)),
                actions=frozenset({DEPOSIT_ACTION}),
                piece_types=frozenset({PIECE_TYPE}),
            ),
        ),
    )
    rules = TableScoringRules({(DEPOSIT_ACTION, "auto"): 3, (DEPOSIT_ACTION, "teleop"): 2})
    match = Match(field, rules, MatchConfig(auto_duration=15, teleop_duration=135))
    for pos in [(150, 40), (150, 100), (150, 160)]:
        match.spawn_piece(PIECE_TYPE, pos)
    return match


def build_demo_characteristics() -> RobotCharacteristics:
    return RobotCharacteristics(
        name="demo",
        max_speed=150,
        max_accel=400,
        max_angular_speed=6,
        max_angular_accel=20,
        width=28,
        length=28,
        piece_capacity=1,
        intake_time=0.4,
        deposit_time=0.4,
        intake_range=6,
        accepted_piece_types=frozenset({PIECE_TYPE}),
    )


class MatchWindow(QtWidgets.QMainWindow):
    TICK_HZ = 60

    def __init__(self):
        super().__init__()
        self.setWindowTitle("sparky-sim -- demo match")

        self.match = build_demo_match()
        self.robot = self.match.add_robot(
            build_demo_characteristics(), Pose2d(30, 100, 0), alliance="blue",
        )

        self.telemetry = TelemetryRecorder(self.match)
        self.playback_time: float | None = None  # None = live, otherwise scrubbed to this time
        self._updating_slider_programmatically = False

        self.canvas = FieldCanvas(self.match)
        self.control_panel = self._build_control_panel()

        main_widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(main_widget)
        layout.addWidget(self.canvas, stretch=1)
        layout.addWidget(self.control_panel)
        self.setCentralWidget(main_widget)
        self.resize(900, 750)

        self._pressed_keys: set[int] = set()
        self.canvas.keyPressEvent = self._key_press
        self.canvas.keyReleaseEvent = self._key_release

        keyboard = KeyboardInput(
            pressed_keys=lambda: self._pressed_keys, bindings=KEY_BINDINGS, deposit_action=DEPOSIT_ACTION,
        )
        gamepad = GamepadInput(deposit_action=DEPOSIT_ACTION)
        self.input_source = gamepad if gamepad.available else keyboard
        self.setWindowTitle(self.windowTitle() + (" [gamepad]" if gamepad.available else " [keyboard]"))

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / self.TICK_HZ))
        self.canvas.setFocus()

    def _build_control_panel(self) -> QtWidgets.QWidget:
        """Build the control panel for match playback."""
        panel = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(panel)

        self.time_label = QtWidgets.QLabel("Time: 0.00s / 0.00s")
        layout.addWidget(self.time_label, stretch=0)

        self.time_slider = ScrubSlider(panel)
        self.time_slider.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.time_slider.setMinimum(0)
        self.time_slider.setMaximum(0)
        # sliderPressed catches "grabbed the handle, haven't moved yet" (a
        # plain click); valueChanged catches every value change, whether
        # from dragging, a groove click, or arrow keys -- together they
        # cover every way a user can scrub. Both are no-ops with zero
        # frames recorded (checked in the handlers) rather than disabling
        # the widget, so scrubbing works from the first recorded tick
        # instead of only after the ~150s (real-time) match ends.
        self.time_slider.sliderPressed.connect(self._on_scrub)
        self.time_slider.valueChanged.connect(self._on_slider_value_changed)
        layout.addWidget(self.time_slider, stretch=1)

        self.replay_button = QtWidgets.QPushButton("Resume Live")
        self.replay_button.clicked.connect(self._on_resume_live)
        self.replay_button.setEnabled(False)
        layout.addWidget(self.replay_button, stretch=0)

        self.export_button = QtWidgets.QPushButton("Export CSV")
        self.export_button.clicked.connect(self._on_export)
        self.export_button.setEnabled(False)
        layout.addWidget(self.export_button, stretch=0)

        return panel

    def _on_scrub(self) -> None:
        """User grabbed the slider handle (mouse-down, before any drag) --
        freeze the sim at whatever time that index currently maps to."""
        self._enter_playback_at_index(self.time_slider.value())

    def _on_slider_value_changed(self, value: int) -> None:
        """Fires on every value change: dragging, a groove click, or arrow
        keys. Ignored while `_tick` is itself repositioning the slider to
        track live playback (see `_updating_slider_programmatically`) so
        that doesn't get misread as a user scrub."""
        if self._updating_slider_programmatically:
            return
        self._enter_playback_at_index(value)

    def _enter_playback_at_index(self, index: int) -> None:
        match_frames = self.telemetry.match_frames
        if not match_frames:
            return
        index = max(0, min(index, len(match_frames) - 1))
        self.playback_time = match_frames[index].time
        self._restore_state_at_time(self.playback_time)
        self.replay_button.setEnabled(True)
        self._update_display()

    def _on_resume_live(self) -> None:
        """Leave playback mode and let the sim keep advancing from where
        it left off (not a rewind -- `match.elapsed` never stopped)."""
        self.playback_time = None
        self.replay_button.setEnabled(False)
        self._update_display()

    def _on_export(self) -> None:
        """Export telemetry to CSV files."""
        timestamp = int(self.match.elapsed * 1000)
        robot_file = f"telemetry_robots_{timestamp}.csv"
        match_file = f"telemetry_match_{timestamp}.csv"

        robot_df = self.telemetry.to_robot_dataframe()
        match_df = self.telemetry.to_match_dataframe()

        if not robot_df.empty:
            robot_df.to_csv(robot_file, index=False)
            print(f"Exported robot telemetry to {robot_file}")

        if not match_df.empty:
            match_df.to_csv(match_file, index=False)
            print(f"Exported match telemetry to {match_file}")

    def _restore_state_at_time(self, target_time: float) -> None:
        """Restore robot positions from telemetry at a specific time."""
        import math

        for robot in self.match.robots:
            snapshot = self.telemetry.get_robot_state_at_time(target_time, robot.characteristics.name)
            if snapshot is not None:
                # Update robot pose and velocity from telemetry
                heading_rad = math.radians(snapshot.orientation_deg)
                robot.chassis.body.position = (snapshot.position_x, snapshot.position_y)
                robot.chassis.body.angle = heading_rad
                robot.chassis.body.velocity = (snapshot.velocity_x, snapshot.velocity_y)

    def _update_display(self) -> None:
        """Update time label to show current time."""
        current_time = self.playback_time if self.playback_time is not None else self.match.elapsed
        total_time = self.match.config.total_duration
        self.time_label.setText(f"Time: {current_time:.2f}s / {total_time:.2f}s")

    def _key_press(self, event) -> None:
        self._pressed_keys.add(event.key())

    def _key_release(self, event) -> None:
        self._pressed_keys.discard(event.key())

    def _tick(self) -> None:
        dt = 1.0 / self.TICK_HZ

        # Only advance the simulation if we're in live mode (not scrubbed
        # into playback) -- the robot keeps driving and telemetry keeps
        # recording even mid-match, so the slider is scrubbable from the
        # first recorded frame rather than only once the match ends. Once
        # the match itself has ended, Match.step becomes a no-op (elapsed
        # stops advancing) so recording also stops -- otherwise telemetry
        # would keep appending duplicate same-timestamp frames forever for
        # as long as the window stays open.
        if self.playback_time is None and not self.match.ended:
            c = self.robot.characteristics
            drive, operator = self.input_source.poll()
            self.robot.drive_field_relative(
                dt, drive.vx * c.max_speed, drive.vy * c.max_speed, drive.omega * c.max_angular_speed,
            )
            self.robot.set_intake_active(operator.intake_active)
            self.robot.set_deposit_active(operator.deposit_active, action=operator.deposit_action)

            self.match.step(dt)
            self.telemetry.tick()
            self._track_slider_to_live()

        self._update_display()
        self.canvas.update()

    def _track_slider_to_live(self) -> None:
        """Keep the slider's range/handle following the live recording
        head. Guarded by `_updating_slider_programmatically` so this
        doesn't get misread by `_on_slider_value_changed` as the user
        scrubbing."""
        total_frames = len(self.telemetry.match_frames)
        if total_frames == 0:
            return
        self._updating_slider_programmatically = True
        self.time_slider.setMaximum(total_frames - 1)
        self.time_slider.setSliderPosition(total_frames - 1)
        self._updating_slider_programmatically = False
        if not self.export_button.isEnabled():
            self.export_button.setEnabled(True)


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    theme.apply_app_theme(app)
    window = MatchWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
