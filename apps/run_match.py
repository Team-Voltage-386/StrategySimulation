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
from common_sim.robot.characteristics import RobotCharacteristics
from gui_utils import theme
from gui_utils.field_canvas import FieldCanvas

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

        self.canvas = FieldCanvas(self.match)
        self.setCentralWidget(self.canvas)
        self.resize(900, 650)

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

    def _key_press(self, event) -> None:
        self._pressed_keys.add(event.key())

    def _key_release(self, event) -> None:
        self._pressed_keys.discard(event.key())

    def _tick(self) -> None:
        dt = 1.0 / self.TICK_HZ
        c = self.robot.characteristics
        drive, operator = self.input_source.poll()
        self.robot.drive_field_relative(
            dt, drive.vx * c.max_speed, drive.vy * c.max_speed, drive.omega * c.max_angular_speed,
        )
        self.robot.set_intake_active(operator.intake_active)
        self.robot.set_deposit_active(operator.deposit_active, action=operator.deposit_action)

        self.match.step(dt)
        self.canvas.update()


def main() -> None:
    app = QtWidgets.QApplication(sys.argv)
    theme.apply_app_theme(app)
    window = MatchWindow()
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()
