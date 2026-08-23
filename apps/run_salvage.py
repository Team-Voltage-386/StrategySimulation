"""
Drive a robot around SALVAGE, the dry-run game (see DRY_RUN_LOG.md).

A deliberately small window: the field, one human-driven robot, three AI
robots, and a readout. It is not `run_reefscape.py` -- there is no
STRATEGY, SWEEP or SEARCH tab here, and no roster editor. Those are
worth having and are not what this is for.

What this *is* for, besides being fun: SALVAGE's rules are the kind you
cannot read off a static field drawing, because most of them are about
timing. The deep hold pays 8 in TELEOP and 3 in AUTO. The wall hold pays
4 then 2. The REACTOR pays 10 then 7, but there are only ten CELLs on
the field all match and both alliances draw from the same two depot
mouths. The BEACON holds six CRATEs total, shared -- filling it scores
and denies at the same time. So the SCORING panel shows every action's
value *as of right now*, and what is left of everything finite, and the
FIELD panel names the zone you are standing in and whether a deposit
would actually land. Watching those numbers move while you drive is the
whole explanation of the game.

Deposits need no action selection here, unlike REEFSCAPE's REEF: every
SALVAGE scoring region offers exactly one action, so standing in a zone
holding something it accepts is unambiguous.

Run: `python -m apps.run_salvage`
"""
from __future__ import annotations

import argparse
import sys

from pyqtgraph.Qt import QtCore, QtWidgets

from common_sim.analysis.sweep_spec import (
    MatchSpec, RobotSpec, TrialJob, characteristics_to_spec,
)
from common_sim.analysis.variability import VariabilityModel
from common_sim.control.input_sources import (
    DriveCommand, GamepadInput, KeyBindings, KeyboardInput, OperatorCommand,
)
from common_sim.field.field_config import point_in_polygon
from common_sim.match.match import Phase
from game_specific.salvage import sweep_trial
from game_specific.salvage.field import build_field
from game_specific.salvage.robot import build_characteristics
from game_specific.salvage.scoring import (
    DEFAULT_SCORING_RELIABILITY_BY_ACTION, SALVAGE_SCORING_RULES,
)
from gui_utils import theme
from gui_utils.field_canvas import FieldCanvas
from gui_utils.telemetry_panel import TelemetryPanel

Qt = QtCore.Qt

KEY_BINDINGS = KeyBindings(
    forward=Qt.Key_W, backward=Qt.Key_S, left=Qt.Key_A, right=Qt.Key_D,
    rotate_ccw=Qt.Key_Left, rotate_cw=Qt.Key_Right,
    intake=Qt.Key_Space, deposit=Qt.Key_F,
)
PAUSE_KEY = Qt.Key_P
RESET_KEY = Qt.Key_R

KEYBOARD_BINDINGS = [
    ("W A S D", "Drive (field-relative)"),
    ("Left / Right", "Rotate"),
    ("Space", "Intake -- hold it while you sit on a depot or bay"),
    ("F", "Deposit -- hold it while you sit in a scoring zone"),
    ("P", "Pause / resume"),
    ("R", "Restart the match"),
]
# Every keyboard action has a controller equivalent, so a pad is a
# complete way to play rather than a way to drive and nothing else.
GAMEPAD_BINDINGS = [
    ("Left stick", "Drive (field-relative)"),
    ("Right stick X", "Rotate"),
    ("LB / RB", "Rotate, held rate -- easier for squaring up"),
    ("A", "Intake"),
    ("RT", "Deposit (half-press is enough)"),
    ("Start", "Pause / resume"),
    ("Back", "Restart the match"),
]

# You drive B0. B1 runs the plan that won the bench (see
# apps/run_salvage_bench.py) so there is something worth watching on your
# own alliance, and red cycles CRATEs -- the static plan every other one
# is measured against.
YOUR_LABEL = "B0"
TEAMMATE_PLAN = "pursue_scarce"
OPPONENT_PLAN = "cycle_crates"

# Matches the benches, so a match you drive is the same match they
# measure -- a bare VariabilityModel() perturbs nothing at all.
VARIABILITY = VariabilityModel(
    enabled=True, intake_time_pct=0.10, deposit_time_pct=0.10,
    max_speed_pct=0.08, max_accel_pct=0.08,
    start_pose_xy_in=4.0, start_pose_heading_deg=5.0, piece_scatter_in=3.0,
)

# Order the SCORING panel lists actions in. Not alphabetical and not the
# points order, because neither is stable: the whole point of the panel
# is that the ranking *changes* at t=15s, and a list that re-sorts itself
# underneath you hides exactly that.
ACTION_ORDER = ("hold_low", "hold_high", "reactor", "airlock", "beacon")
ACTION_LABELS = {
    "hold_low": "HOLD LOW  (crate)",
    "hold_high": "HOLD HIGH (crate)",
    "reactor": "REACTOR   (cell)",
    "airlock": "AIRLOCK   (scrap)",
    "beacon": "BEACON    (crate)",
}


def _clamp(value: float) -> float:
    return max(-1.0, min(1.0, value))


class CombinedInput:
    """Keyboard and gamepad at the same time, summed.

    `run_reefscape.py` picks one -- gamepad if `GamepadInput.available`,
    keyboard otherwise -- and that is a trap worth not repeating here.
    `available` means "pygame found a joystick device", not "a human is
    holding it", so any controller plugged into the machine (or a ghost
    device some wireless dongles leave behind) silently makes W A S D do
    nothing at all, with no message saying why. There is no reason to
    choose: an idle stick reads zero, an unpressed key reads zero, and
    adding two zeros is still zero.

    Sums the axes and ORs the buttons, so whichever device is actually
    being touched wins without either being switched off."""

    def __init__(self, keyboard, gamepad):
        self.keyboard = keyboard
        self.gamepad = gamepad

    def poll(self):
        drive, operator = self.keyboard.poll()
        if not self.gamepad.available:
            return drive, operator
        pad_drive, pad_operator = self.gamepad.poll()
        drive = DriveCommand(
            vx=_clamp(drive.vx + pad_drive.vx),
            vy=_clamp(drive.vy + pad_drive.vy),
            omega=_clamp(drive.omega + pad_drive.omega),
        )
        operator = OperatorCommand(
            intake_active=operator.intake_active or pad_operator.intake_active,
            deposit_active=operator.deposit_active or pad_operator.deposit_active,
            # Edge-triggered on the pad side only -- the keyboard's
            # equivalents are edge-detected by the window against its own
            # pressed-key set, since KeyboardInput has no memory between
            # polls and so cannot tell a new press from a held one.
            pause_toggle=pad_operator.pause_toggle,
            reset=pad_operator.reset,
        )
        return drive, operator


def _panel(title: str) -> tuple[QtWidgets.QGroupBox, QtWidgets.QLabel]:
    box = QtWidgets.QGroupBox(title)
    layout = QtWidgets.QVBoxLayout(box)
    label = QtWidgets.QLabel("")
    label.setFont(theme.technical_font(9))
    label.setStyleSheet(f"color: {theme.TEXT_PRIMARY};")
    label.setWordWrap(True)
    layout.addWidget(label)
    return box, label


class SalvageWindow(QtWidgets.QWidget):
    TICK_HZ = 60

    def __init__(self):
        super().__init__()
        self.setWindowTitle("SALVAGE 2027 -- dry-run game")
        self._pressed_keys: set = set()
        self._prev_pressed_keys: set = set()
        self.paused = False

        self.canvas = FieldCanvas(None)
        self.canvas.setFocusPolicy(Qt.NoFocus)  # this window owns the keys

        self.telemetry = TelemetryPanel("MATCH")
        self.scoring_box, self.scoring_label = _panel("SCORING (right now)")
        self.field_box, self.field_label = _panel("WHERE YOU ARE")
        self.supply_box, self.supply_label = _panel("WHAT IS LEFT")
        self.controls_box, self.controls_label = _panel("CONTROLS")

        side = QtWidgets.QVBoxLayout()
        side.addWidget(self.telemetry)
        side.addWidget(self.field_box)
        side.addWidget(self.scoring_box)
        side.addWidget(self.supply_box)
        side.addWidget(self.controls_box)
        side.addStretch(1)
        side_widget = QtWidgets.QWidget()
        side_widget.setLayout(side)
        side_widget.setFixedWidth(340)

        root = QtWidgets.QHBoxLayout(self)
        root.addWidget(self.canvas, stretch=1)
        root.addWidget(side_widget)

        self.gamepad = GamepadInput()
        self.keyboard = KeyboardInput(pressed_keys=lambda: self._pressed_keys, bindings=KEY_BINDINGS)
        self.input_source = CombinedInput(self.keyboard, self.gamepad)
        bindings = list(KEYBOARD_BINDINGS)
        if self.gamepad.available:
            bindings += [("", ""), ("gamepad also live:", "")] + GAMEPAD_BINDINGS
        self.controls_label.setText("\n".join(
            f"{key:<14}{what}".rstrip() for key, what in bindings
        ))

        self._reset_match()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / self.TICK_HZ))
        self.setFocusPolicy(Qt.StrongFocus)
        self.resize(1400, 820)

    # -- setup ---------------------------------------------------------

    def _reset_match(self) -> None:
        """Build the match through the *same* builder the benches use, so
        what you drive is the match they measure. Seeded off the wall
        clock so consecutive resets are not the same match."""
        characteristics = characteristics_to_spec(build_characteristics())
        robots = [
            RobotSpec(label="B0", alliance="blue", roster_index=0,
                      characteristics=characteristics, strategy=None),
            RobotSpec(label="B1", alliance="blue", roster_index=1,
                      characteristics=characteristics, strategy=TEAMMATE_PLAN),
            RobotSpec(label="R0", alliance="red", roster_index=0,
                      characteristics=characteristics, strategy=OPPONENT_PLAN),
            RobotSpec(label="R1", alliance="red", roster_index=1,
                      characteristics=characteristics, strategy=OPPONENT_PLAN),
        ]
        job = TrialJob(
            index=0, seed=QtCore.QDateTime.currentMSecsSinceEpoch() % 100000,
            params={}, robots=tuple(robots),
            match=MatchSpec(auto_duration=15.0, teleop_duration=135.0),
            variability=VARIABILITY, strategies_dir=str(sweep_trial.STRATEGIES_DIR),
            dt=sweep_trial.SWEEP_DT,
        )
        self.match, self.robots_by_label, _ = sweep_trial.build_match_for_job(job)
        # `strategy=None` above leaves this robot with no controller, which
        # is what makes it drivable -- Match never touches a robot that
        # has none, so the only thing moving it is the input source below.
        self.robot = self.robots_by_label[YOUR_LABEL]
        self.canvas.match = self.match
        self.paused = False

    # -- input ---------------------------------------------------------

    def keyPressEvent(self, event) -> None:
        self._pressed_keys.add(event.key())

    def keyReleaseEvent(self, event) -> None:
        self._pressed_keys.discard(event.key())

    def _deposit_action(self) -> str | None:
        """Which action a deposit would attempt from where the robot is
        standing.

        Every SALVAGE scoring region offers exactly one action, so this is
        never a choice and never needs a key -- unlike a REEFSCAPE REEF
        face, which offers L1-L4 at one spot and makes the player pick.
        Returns None when the robot is not in a zone that would take
        anything it is holding, which is also what the WHERE YOU ARE
        panel reports."""
        held = {p.piece_type for p in self.robot.held_pieces}
        if not held:
            return None
        position = (self.robot.pose.x, self.robot.pose.y)
        for region in self.match.field.scoring_regions:
            if region.alliance is not None and region.alliance != self.robot.alliance:
                continue
            if region.piece_types and not (held & region.piece_types):
                continue
            if point_in_polygon(position, region.vertices):
                return next(iter(region.actions))
        return None

    # -- loop ----------------------------------------------------------

    def _tick(self) -> None:
        dt = 1.0 / self.TICK_HZ
        drive, operator = self.input_source.poll()

        just_pressed = self._pressed_keys - self._prev_pressed_keys
        self._prev_pressed_keys = set(self._pressed_keys)
        if PAUSE_KEY in just_pressed or operator.pause_toggle:
            self.paused = not self.paused
        if RESET_KEY in just_pressed or operator.reset:
            self._reset_match()

        if not self.paused and not self.match.ended:
            characteristics = self.robot.characteristics
            self.robot.drive_field_relative(
                dt,
                drive.vx * characteristics.max_speed,
                drive.vy * characteristics.max_speed,
                drive.omega * characteristics.max_angular_speed,
            )
            self.robot.set_intake_active(operator.intake_active)
            self.robot.set_deposit_active(operator.deposit_active, action=self._deposit_action())
            self.match.step(dt)

        self._update_panels()
        self.canvas.update()

    # -- readouts ------------------------------------------------------

    def _update_panels(self) -> None:
        match = self.match
        remaining = max(0.0, match.config.total_duration - match.elapsed)
        state = "ENDED" if match.ended else ("PAUSED" if self.paused else match.phase.value.upper())
        self.telemetry.set_lines([
            ("State", state),
            ("Remaining", f"{remaining:5.1f}s"),
            ("Blue", f"{match.scores.get('blue', 0.0):.0f}"),
            ("Red", f"{match.scores.get('red', 0.0):.0f}"),
            ("You hold", ", ".join(p.piece_type for p in self.robot.held_pieces) or "nothing"),
        ])
        self._update_scoring()
        self._update_field()
        self._update_supply()

    def _update_scoring(self) -> None:
        """Every action's value as of this instant, so the AUTO->TELEOP
        reordering is something you watch happen rather than something
        you read about. `x P` is how often this robot's mechanism lands
        that attempt -- the deep hold pays four times the wall hold in
        TELEOP and misses roughly a fifth of the time, which is the trade
        the whole point table is built around."""
        phase = self.match.phase.value
        lines = [f"{'action':<19}{'pts':>4}{'P':>7}"]
        for action in ACTION_ORDER:
            points = SALVAGE_SCORING_RULES.points_for(action, phase)
            reliability = DEFAULT_SCORING_RELIABILITY_BY_ACTION.get(action, 1.0)
            lines.append(f"{ACTION_LABELS[action]:<19}{points:4.0f}{reliability:7.2f}")
        if self.match.phase is Phase.AUTO:
            lines.append("")
            lines.append("AUTO. These change at t=15s --")
            lines.append("HOLD HIGH goes 3 -> 8, HOLD LOW")
            lines.append("4 -> 2, REACTOR 10 -> 7.")
        self.scoring_label.setText("\n".join(lines))

    def _update_field(self) -> None:
        position = (self.robot.pose.x, self.robot.pose.y)
        held = {p.piece_type for p in self.robot.held_pieces}
        lines = []

        for station in self.match.field.intake_locations:
            if station.alliance is not None and station.alliance != self.robot.alliance:
                continue
            if point_in_polygon(position, station.vertices):
                supply = self.match.station_supply.get(station)
                left = "unlimited" if supply is None else f"{supply} left"
                lines.append(f"IN {station.name}")
                lines.append(f"  dispenses {station.piece_type} ({left})")
                lines.append("  hold SPACE to load")

        for region in self.match.field.scoring_regions:
            if region.alliance is not None and region.alliance != self.robot.alliance:
                continue
            if not point_in_polygon(position, region.vertices):
                continue
            action = next(iter(region.actions))
            lines.append(f"IN {region.name}")
            if self.match.region_full(region, action):
                lines.append("  FULL -- nothing more scores here")
            elif not held:
                lines.append(f"  wants {'/'.join(sorted(region.piece_types)) or 'anything'}")
            elif region.piece_types and not (held & region.piece_types):
                lines.append(f"  wants {'/'.join(sorted(region.piece_types))}, you have "
                             f"{'/'.join(sorted(held))}")
            else:
                points = SALVAGE_SCORING_RULES.points_for(action, self.match.phase.value)
                lines.append(f"  hold F to score {action} for {points:.0f}")

        self.field_label.setText("\n".join(lines) or "Open field.\nNothing to load or score here.")

    def _update_supply(self) -> None:
        """The finite things, which is most of what makes this game a
        game. The depot is neutral -- red draws its CELLs from the same
        two mouths you do -- and the BEACON's six slots are shared, so
        every CRATE either alliance puts in one is a slot the other
        cannot have."""
        lines = []
        cells = sum(
            supply for station, supply in self.match.station_supply.items()
            if station.alliance is None
        )
        lines.append(f"{'DEPOT (neutral)':<18}{cells:2d} cells left")
        beacon = next((r for r in self.match.field.scoring_regions if r.name == "beacon"), None)
        if beacon is not None:
            used = self.match.region_scores.get("beacon", {}).get("beacon", 0)
            cap = (beacon.capacity_by_action or {}).get("beacon", 0)
            lines.append(f"{'BEACON (shared)':<18}{cap - used:2d} of {cap} slots free")
        for alliance in ("blue", "red"):
            region = next(
                (r for r in self.match.field.scoring_regions if r.name == f"{alliance}_hold_high"), None
            )
            if region is None:
                continue
            used = self.match.region_scores.get(region.name, {}).get("hold_high", 0)
            cap = (region.capacity_by_action or {}).get("hold_high", 0)
            lines.append(f"{alliance.upper() + ' HOLD HIGH':<18}{cap - used:2d} of {cap} slots free")
        self.supply_label.setText("\n".join(lines))


def field_errors() -> list:
    """Any reason this field cannot be played, from
    `common_sim.field.validation`. Better a message than a match that
    quietly cannot be finished -- which is exactly the failure mode that
    module was written for."""
    from common_sim.field.validation import ERROR, validate_field

    characteristics = build_characteristics()
    return [
        p for p in validate_field(
            build_field(), robot_width=characteristics.width, robot_length=characteristics.length,
            scoring_rules=SALVAGE_SCORING_RULES,
        )
        if p.severity == ERROR
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive a robot around SALVAGE.")
    parser.add_argument(
        "--check", action="store_true",
        help="build the window, report the field and the input devices, and exit "
             "without showing anything -- what run_salvage.bat --check runs, and "
             "the quickest way to find out whether a controller is being seen",
    )
    args = parser.parse_args()

    from common_sim.field.validation import describe_problems

    app = QtWidgets.QApplication(sys.argv)
    theme.apply_app_theme(app)
    problems = field_errors()
    if problems:
        print("SALVAGE field has errors; not launching:\n" + describe_problems(problems))
        sys.exit(1)

    window = SalvageWindow()
    if args.check:
        pad = window.gamepad
        print("SALVAGE field OK.")
        print(f"keyboard: ready ({len(KEYBOARD_BINDINGS)} bindings)")
        print("gamepad: " + ("detected -- both it and the keyboard are live"
                             if pad.available else "none detected -- keyboard only"))
        window._tick()
        print(f"one tick ran; match clock at {window.match.elapsed:.2f}s")
        return
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
