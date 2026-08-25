"""The robot's joystick-to-chassis mapping, and its inverse.

The strategy layer speaks velocities: `drive_field_relative(dt, vx, vy,
omega)`. The wire speaks joystick axes. This module is the translation,
and it is not a scale factor -- `DriveCommands.joystickDrive` deadbands
the stick, *squares* it, and halves the whole drivetrain while the feeder
is running. Getting any of those wrong produces a robot that drives, just
not where it was told, which is the hardest kind of wrong to see in a
campaign report.

So the forward model is written out explicitly, transcribed from
`DriveCommands.java`, and `axes_for` is its exact inverse. `calibrate`
then checks the pair against the running robot rather than trusting
either: command a stick value, read what the drive was actually told
through `SwerveChassisSpeeds/Setpoints`, and compare. That also *measures*
the maximum speeds instead of transcribing them, which matters because
`DriveConstants` picks them from an `isReefscape` branch at class-init
time and the number that wins is not obvious from reading the file.

Everything here is in the robot's units -- metres per second and radians
per second, blue-origin -- and in fractions of maximum. Converting to
sparky-sim's inches is the adapter's job, in `match_view.py`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from bridge import operator as op

# DriveCommands.DEADBAND. Applied to the *magnitude* of the translation
# stick, not to each axis, and separately to the rotation axis.
DEADBAND = 0.06

# DriveCommands.joystickDrive: `speedMultiplier` is 0.5 while
# `spindexer::isFeederOn`, i.e. for exactly as long as a shot is being
# fed. A caller that ignores this asks for full speed during every shot
# and gets half, which reads as a robot that mysteriously slows down
# whenever it scores.
SHOOTING_SPEED_MULTIPLIER = 0.5


def apply_deadband(value: float, deadband: float = DEADBAND) -> float:
    """`MathUtil.applyDeadband(value, deadband)` with maxMagnitude 1.

    Note it *rescales* rather than clipping: the surviving range is
    stretched back out to [0, 1], so the inverse has to undo the stretch
    as well as re-add the offset.
    """
    if abs(value) <= deadband:
        return 0.0
    return (value - math.copysign(deadband, value)) / (1.0 - deadband)


def undo_deadband(fraction: float, deadband: float = DEADBAND) -> float:
    """The stick value that `apply_deadband` maps to `fraction`.

    Exact, not approximate. Zero maps to zero rather than to the deadband
    edge, so a request to stop is a genuine stop.
    """
    if fraction == 0.0:
        return 0.0
    return math.copysign(abs(fraction) * (1.0 - deadband) + deadband, fraction)


@dataclass(frozen=True)
class DriveLimits:
    """What the drive does with a full stick. Measured, not declared --
    see `calibrate`."""

    max_speed_mps: float
    max_omega_rad_s: float

    # DriveConstants for the branch this robot builds; kept only as the
    # starting guess for calibration and as something to compare a
    # measurement against.
    NOMINAL_MAX_SPEED_MPS = 5.3


def forward(axes: dict[int, float], *, shooting: bool = False,
            limits: DriveLimits | None = None) -> tuple[float, float, float]:
    """What the drive will be commanded, given these stick positions.

    Transcribed from `DriveCommands.getLinearVelocityFromJoysticks` and
    `joystickDrive`. Returns field-relative (vx, vy, omega) as fractions
    of maximum, or in m/s and rad/s when `limits` is given.

    The binding layer is part of this: `RobotContainer` passes
    `xSupplier = -getLeftY()` and `ySupplier = -getLeftX()`, so the
    forward field axis is the *negated* left-stick Y and the leftward
    field axis is the negated left-stick X.
    """
    x = -axes.get(op.AXIS_LEFT_Y, 0.0)
    y = -axes.get(op.AXIS_LEFT_X, 0.0)
    omega_stick = -axes.get(op.AXIS_RIGHT_X, 0.0)

    magnitude = apply_deadband(math.hypot(x, y))
    magnitude *= magnitude  # squared "for more precise control"
    direction = math.atan2(y, x)

    omega = apply_deadband(omega_stick)
    omega = math.copysign(omega * omega, omega)

    multiplier = SHOOTING_SPEED_MULTIPLIER if shooting else 1.0
    vx = magnitude * math.cos(direction) * multiplier
    vy = magnitude * math.sin(direction) * multiplier

    if limits is None:
        return vx, vy, omega
    return vx * limits.max_speed_mps, vy * limits.max_speed_mps, omega * limits.max_omega_rad_s


@dataclass(frozen=True)
class DriveRequest:
    """Stick positions, plus what had to be given up to produce them."""

    axes: dict[int, float]
    saturated: bool  # the request exceeded what the drivetrain can do

    def apply(self, link: op.OperatorLink) -> None:
        for axis, value in self.axes.items():
            link.set_axis(axis, value)


def axes_for(
    vx: float, vy: float, omega: float, limits: DriveLimits, *, shooting: bool = False
) -> DriveRequest:
    """Stick positions that make the drive do (vx, vy, omega) in m/s and rad/s.

    The exact inverse of `forward`. Translation is inverted as a
    magnitude and a direction rather than per-axis, because the squaring
    is applied to the magnitude -- undoing it component-wise would rotate
    the commanded direction, sending the robot somewhere near where it
    was told to go and nowhere exactly.

    A request beyond the drivetrain is clipped to full stick in the
    requested *direction* and reported as `saturated`. Clipping the
    direction as well would be the worse failure: a robot that cannot go
    as fast as asked still goes the right way, whereas one whose heading
    quietly bends looks like a navigation bug.
    """
    multiplier = SHOOTING_SPEED_MULTIPLIER if shooting else 1.0
    speed = math.hypot(vx, vy)
    fraction = speed / (limits.max_speed_mps * multiplier) if limits.max_speed_mps else 0.0
    omega_fraction = omega / limits.max_omega_rad_s if limits.max_omega_rad_s else 0.0

    saturated = fraction > 1.0 or abs(omega_fraction) > 1.0
    fraction = min(fraction, 1.0)
    omega_fraction = max(-1.0, min(1.0, omega_fraction))

    # Undo the square, then the deadband. `magnitude` is the deadbanded
    # value the robot would compute; `stick` is what has to be sent for
    # it to compute that.
    stick = undo_deadband(math.sqrt(fraction))
    direction = math.atan2(vy, vx) if speed > 0.0 else 0.0
    omega_stick = undo_deadband(math.copysign(math.sqrt(abs(omega_fraction)), omega_fraction))

    # Back through the binding: x = -leftY, y = -leftX, omega = -rightX.
    return DriveRequest(
        axes={
            op.AXIS_LEFT_Y: -stick * math.cos(direction),
            op.AXIS_LEFT_X: -stick * math.sin(direction),
            op.AXIS_RIGHT_X: -omega_stick,
        },
        saturated=saturated,
    )


@dataclass(frozen=True)
class Calibration:
    """One commanded stick position weighed against what the drive did."""

    label: str
    axes: dict[int, float]
    expected_fraction: tuple[float, float, float]
    measured: tuple[float, float, float]  # m/s, m/s, rad/s -- robot-relative
    limits: DriveLimits

    @property
    def predicted(self) -> tuple[float, float, float]:
        fx, fy, fomega = self.expected_fraction
        return (fx * self.limits.max_speed_mps, fy * self.limits.max_speed_mps,
                fomega * self.limits.max_omega_rad_s)

    @property
    def speed_error(self) -> float:
        """Difference in commanded speed, m/s."""
        px, py, _ = self.predicted
        mx, my, _ = self.measured
        return abs(math.hypot(px, py) - math.hypot(mx, my))

    @property
    def omega_error(self) -> float:
        return abs(self.predicted[2] - self.measured[2])

    @property
    def direction_error_deg(self) -> float:
        """Difference in commanded heading, degrees, or 0 when too slow to mean anything.

        Split out from the speed error rather than folded into one
        per-component number, because the two have different noise floors
        and only one of them indicates a broken model.

        `joystickDrive` builds field-relative speeds and immediately
        converts them to robot-relative using the robot's heading;
        `measure` converts back using a pose read a moment later. Any
        heading change between those two instants shows up here as a
        direction error, scaled by how fast the robot is going -- which is
        why the full-stick probe shows a couple of degrees and the
        half-stick ones show none. That is sampling skew, not a wrong
        model, and a single lumped tolerance either has to be loose
        enough to hide a real error at full stick or tight enough to fail
        on this every time.
        """
        px, py, _ = self.predicted
        mx, my, _ = self.measured
        if math.hypot(px, py) < DIRECTION_MEANINGFUL_MPS:
            return 0.0
        delta = math.atan2(my, mx) - math.atan2(py, px)
        return abs(math.degrees(math.atan2(math.sin(delta), math.cos(delta))))

    def __str__(self) -> str:
        px, py, pw = self.predicted
        mx, my, mw = self.measured
        return (f"{self.label}: predicted ({px:+.2f}, {py:+.2f}) m/s {pw:+.2f} rad/s  "
                f"measured ({mx:+.2f}, {my:+.2f}) {mw:+.2f}  "
                f"[speed {self.speed_error:.3f}, dir {self.direction_error_deg:.1f}deg, "
                f"omega {self.omega_error:.3f}]")


# Below this, a commanded direction is arbitrary -- the robot is being
# asked to hold still and any residual points somewhere meaningless.
DIRECTION_MEANINGFUL_MPS = 0.5


# Stick positions to probe. Deliberately not just full stick: the mapping
# is quadratic, so a model fitted at 1.0 and checked at 1.0 would confirm
# a scale factor and miss the squaring entirely. The half-stick cases are
# the ones that would catch it.
_PROBES: tuple[tuple[str, dict[int, float]], ...] = (
    ("forward full", {op.AXIS_LEFT_Y: -1.0}),
    ("forward half", {op.AXIS_LEFT_Y: -0.5}),
    ("left half", {op.AXIS_LEFT_X: -0.5}),
    ("diagonal", {op.AXIS_LEFT_Y: -0.5, op.AXIS_LEFT_X: -0.5}),
    ("rotate half", {op.AXIS_RIGHT_X: -0.5}),
)


def _release(link: op.OperatorLink) -> None:
    for axis in (op.AXIS_LEFT_X, op.AXIS_LEFT_Y, op.AXIS_RIGHT_X):
        link.set_axis(axis, 0.0)


def measure(link: op.OperatorLink, state, axes: dict[int, float], settle: float = 0.4):
    """Hold `axes`, read back what the drive was commanded, then undo the move.

    Returns field-relative (vx, vy, omega) in m/s and rad/s.
    `SwerveChassisSpeeds/Setpoints` is robot-relative -- `joystickDrive`
    builds field-relative speeds and immediately converts -- so the
    heading has to be rotated back out. Doing that here rather than
    calibrating at heading zero means the check still works after the
    robot has been driven around.

    **The mirrored second half is not optional.** Calibrating means
    holding full stick, and full stick on this drivetrain is 4.45 m/s: a
    handful of probes walks the robot several metres before anything else
    has run. The first live strategy run spent forty-five seconds wedged
    against the red HUB and it was the *calibration* that put it there,
    with the strategy layer inheriting a robot that was already stuck.
    Holding the negated stick for the same duration returns it to roughly
    where it started, so calibration is something the caller can do
    without having to think about where the robot ends up.
    """
    import time

    from bridge.robot_state import CHASSIS_SETPOINT, POSE_TRUTH

    _release(link)
    for axis, value in axes.items():
        link.set_axis(axis, value)
    time.sleep(settle)

    speeds = state.chassis_speeds(CHASSIS_SETPOINT)
    pose = state.pose(POSE_TRUTH)

    for axis, value in axes.items():
        link.set_axis(axis, -value)
    time.sleep(settle)
    _release(link)
    time.sleep(0.2)  # let the drivetrain come to rest before the next probe

    if speeds is None or pose is None:
        return None

    heading = pose.theta
    return (
        speeds.vx * math.cos(heading) - speeds.vy * math.sin(heading),
        speeds.vx * math.sin(heading) + speeds.vy * math.cos(heading),
        speeds.omega,
    )


def calibrate(link: op.OperatorLink, state) -> tuple[DriveLimits, list[Calibration]]:
    """Measure the drivetrain's maxima, then check the model against them.

    Measuring rather than transcribing, for a specific reason:
    `DriveConstants` sets `maxSpeed` and `driveBaseRadius` inside an
    `if (isReefscape)` branch resolved at class-init, and which values
    win is not obvious from reading the file. A wrong constant here does
    not fail loudly -- it produces a robot that drives at the wrong speed
    and therefore arrives somewhere other than where the navigator
    believed it would, which reads as a navigation bug.

    Full stick gives the maxima directly, since at magnitude 1 the
    deadband rescale and the squaring are both identities. The remaining
    probes then test the model at intermediate stick, where those two
    steps actually do something.
    """
    forward_full = measure(link, state, {op.AXIS_LEFT_Y: -1.0})
    rotate_full = measure(link, state, {op.AXIS_RIGHT_X: -1.0})
    if forward_full is None or rotate_full is None:
        raise RuntimeError(
            "the drive published no chassis setpoint while a full stick was held, so the "
            "joystick-to-velocity model cannot be calibrated or checked"
        )

    limits = DriveLimits(
        max_speed_mps=math.hypot(forward_full[0], forward_full[1]),
        max_omega_rad_s=abs(rotate_full[2]),
    )

    checks = []
    for label, axes in _PROBES:
        measured = measure(link, state, axes)
        if measured is None:
            continue
        checks.append(Calibration(
            label=label, axes=axes,
            expected_fraction=forward(axes),
            measured=measured,
            limits=limits,
        ))
    return limits, checks
