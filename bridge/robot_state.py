"""Robot -> Python: read what AdvantageKit already publishes, over NT4.

Nothing here needs a change to robot code. `Robot.java` adds an `NT4Publisher`
data receiver in SIM mode, so every `Logger.recordOutput` and every
`@AutoLogOutput` is already on the wire under `/AdvantageKit/RealOutputs/...`,
and every `Logger.processInputs` under `/AdvantageKit/<table>/...`.

Poses are decoded from WPILib's struct encoding by hand rather than through
`wpimath`. The layout is fixed and trivial -- Pose2d is three little-endian
doubles, being Translation2d(x, y) followed by Rotation2d(radians) -- and
skipping the dependency keeps the bridge installable next to pymunk without
dragging a second copy of the WPILib math stack into the environment.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass

try:
    import ntcore  # pyntcore
except ImportError as exc:  # pragma: no cover - depends on the install
    # The topic names and the struct decoders are plain data and arithmetic,
    # and have no business requiring NetworkTables. Keeping this module
    # importable without pyntcore is what lets the pose decoding and the
    # world-state helpers be unit-tested in CI, where there is no JVM and no
    # robot project -- the same reason bridge.oracles imports without it.
    # `RobotStateLink` still fails, but on construction rather than on import.
    ntcore = None  # type: ignore[assignment]
    _NTCORE_ERROR = exc
else:
    _NTCORE_ERROR = None

DEFAULT_SERVER = "127.0.0.1"
DEFAULT_CLIENT_NAME = "sparky-bridge"

# AdvantageKit's namespaces.
OUTPUTS = "/AdvantageKit/RealOutputs"
INPUTS = "/AdvantageKit"

# Ground truth from maple-sim's dyn4j world (SimContainer.java).
POSE_TRUTH = f"{OUTPUTS}/FieldSimulation/RobotPosition"

# Every FUEL loose on the field, also from SimContainer.simulationPeriodic.
# This is what makes a world-state reader possible without predicting where
# pieces went: maple-sim owns the piece physics and publishes the answer.
FUEL_POSES = f"{OUTPUTS}/FieldSimulation/Fuel"

# How many FUEL the robot is carrying, from IntakeIOSim.updateInputs.
#
# Worth being precise about, because a note in an earlier session said intake
# state was unobservable and that was half wrong. The *IntakeIOInputs* -- the
# deployed flag, the position, the intaking state -- really are invisible:
# IntakeSubsystem holds a bare inputs object and never calls
# Logger.processInputs on it. But `Intake/BallCount` is published separately by
# a `Logger.recordOutput` inside the same `updateInputs`, and `periodic` calls
# that every cycle. So possession *is* readable, and it is the one signal the
# strategy layer cannot work without: collect-versus-score turns on it.
#
# Only in simulation. IntakeIOSim reads it from maple-sim's IntakeSimulation,
# and the real IO has no such sensor.
BALL_COUNT = f"{OUTPUTS}/Intake/BallCount"

# The intake arm's commanded angle, out of the Mechanism2d IntakeIOSim draws:
# 90 degrees is stowed, 0 is deployed (IntakeConstants.retractedAngle /
# .extendedAngle). The nearest thing to an observable deploy state -- the
# IntakeIOInputs that carry the real `deployed` flag are never run through
# Logger.processInputs, so this drawing is what there is. Worth having,
# because "the robot collected nothing" has two very different causes and
# this separates them: the intake never came down, or it came down and
# missed.
INTAKE_ARM_ANGLE = f"{OUTPUTS}/Intake/Mech/Intake/IntakeArm/angle"

# Whether the turret is tracking the HUB by itself. Bound to manip Start as a
# *toggle* (`turret.toggleAutoAimCommand`), which is why the published state
# matters: a toggle pressed blind is as likely to turn the thing off as on.
# Reading it first is what makes the press idempotent.
AUTO_AIM_ENABLED = f"{OUTPUTS}/Shooter/Turret/autoAimEnabled"

#: Whether the feeder is running. The one signal two *other* subsystems
#: branch on: `DriveCommands.joystickDrive` halves the whole drivetrain
#: while it is true, and `TurretIOSim` only launches fuel while it is
#: true. Neither could be inverted without inferring this, and inferring
#: it from the last button sent is wrong exactly when an edge is lost.
FEEDER_ON = f"{OUTPUTS}/Spindexer/FeederOn"

# What the robot code *believes*, from its own odometry (Drive.java:312).
#
# Careful: in this sim configuration these two are the same number, not two
# independent estimates. SimContainer feeds maple-sim's ground-truth pose to
# `addVisionMeasurement` with standard deviations of (0, 0, 0) -- infinite
# confidence -- so the pose estimator is pinned to truth and re-snaps to it
# every cycle. Anything that writes odometry (`drive.setPose`) is silently
# undone within one loop, and an odometry-vs-truth divergence oracle would
# read zero forever here. Treat this key as truth-with-extra-steps until the
# vision std devs are made realistic.
POSE_ODOMETRY = f"{OUTPUTS}/Odometry/Robot"

# AdvantageKit logs DriverStation and HID values automatically, which makes
# them an echo of what the operator link just sent -- a way to confirm the
# wire independently of whether any binding reacted.
DS_ENABLED = f"{INPUTS}/DriverStation/Enabled"
DS_AUTONOMOUS = f"{INPUTS}/DriverStation/Autonomous"
DS_ATTACHED = f"{INPUTS}/DriverStation/DSAttached"

# AdvantageKit's own cycle counter, in microseconds. The cheapest possible
# "is robot code still running" signal: if this stops advancing while NT is
# still connected, the loop is wedged rather than merely idle.
TIMESTAMP = f"{INPUTS}/Timestamp"

# What the drive was last *told* to do, downstream of the binding layer
# (Drive.java:218). Separate from what the stick said, on purpose -- the gap
# between the two is its own kind of failure.
CHASSIS_SETPOINT = f"{OUTPUTS}/SwerveChassisSpeeds/Setpoints"
CHASSIS_MEASURED = f"{OUTPUTS}/SwerveChassisSpeeds/Measured"

# Note the doubled slash: the recordOutput key really does begin with "/".
FLYWHEEL_SETPOINT_RPM = f"{OUTPUTS}//Shooter/Flywheel/VelocitySetpoint"
FLYWHEEL_SPEED_RAD_S = f"{INPUTS}/Shooter/Flywheel/Inputs/FlywheelSpeed"

#: Per-module drive current. This is what separates a robot that is *pushing*
#: against something from one that is not being driven at all -- measured on
#: this field: 0.0 A idle, 9.4 A driving freely, 58.2 A pinned against a wall.
#: Wheel speed does not separate them; maple-sim stalls a blocked drivetrain
#: rather than letting it slip, so the encoders read zero in both cases.
DRIVE_CURRENT = tuple(f"{INPUTS}/Drive/Module{i}/DriveCurrentAmps" for i in range(4))

#: Ground-truth poses of the five robots sparky-sim drives through
#: BridgeRobots.java. Same publisher and same units as POSE_TRUTH -- from the
#: oracles' point of view these are simply five more robots to hold to the
#: same invariants, which is most of what a populated field buys.
BRIDGE_ROBOT_POSES = f"{OUTPUTS}/FieldSimulation/BridgeRobots"

LOOP_CYCLE_MS = f"{OUTPUTS}/LoggedRobot/FullCycleMS"
BROWNED_OUT = f"{INPUTS}/SystemStats/BrownedOut"
BATTERY_VOLTAGE = f"{INPUTS}/SystemStats/BatteryVoltage"

# AdvantageKit's Alert API, already used by PathPlanner and the vision code.
# A structured second source for oracle 01 that needs no log parsing.
ALERT_GROUPS = ("Alerts", "PathPlanner")

_CHASSIS_SPEEDS = struct.Struct("<ddd")


def joystick_axes_key(joystick: int = 0) -> str:
    return f"{INPUTS}/DriverStation/Joystick{joystick}/AxisValues"


def joystick_buttons_key(joystick: int = 0) -> str:
    """Topic holding a bitfield: bit (n-1) is WPILib button n."""
    return f"{INPUTS}/DriverStation/Joystick{joystick}/ButtonValues"

_POSE2D = struct.Struct("<ddd")


@dataclass(frozen=True)
class ChassisSpeeds:
    vx: float  # m/s
    vy: float  # m/s
    omega: float  # rad/s

    @classmethod
    def decode(cls, raw: bytes) -> "ChassisSpeeds":
        if len(raw) != _CHASSIS_SPEEDS.size:
            raise ValueError(f"expected {_CHASSIS_SPEEDS.size} bytes for ChassisSpeeds, got {len(raw)}")
        return cls(*_CHASSIS_SPEEDS.unpack(raw))

    @property
    def linear(self) -> float:
        return (self.vx**2 + self.vy**2) ** 0.5

    def is_moving(self, linear_min: float = 0.05, omega_min: float = 0.1) -> bool:
        return self.linear >= linear_min or abs(self.omega) >= omega_min


@dataclass(frozen=True)
class Pose2d:
    x: float
    y: float
    theta: float  # radians, CCW-positive, blue-origin

    @classmethod
    def decode(cls, raw: bytes) -> "Pose2d":
        if len(raw) != _POSE2D.size:
            raise ValueError(f"expected {_POSE2D.size} bytes for a Pose2d struct, got {len(raw)}")
        return cls(*_POSE2D.unpack(raw))

    @classmethod
    def decode_array(cls, raw: bytes) -> list["Pose2d"]:
        """Decode a `struct:Pose2d[]` payload -- see `Pose3d.decode_array`."""
        if len(raw) % _POSE2D.size:
            raise ValueError(
                f"{len(raw)} bytes is not a whole number of {_POSE2D.size}-byte Pose2d structs"
            )
        return [cls.decode(raw[i:i + _POSE2D.size]) for i in range(0, len(raw), _POSE2D.size)]

    def distance_to(self, other: "Pose2d") -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5

    def __str__(self) -> str:  # metres and degrees read better in a report
        import math

        return f"({self.x:+.3f} m, {self.y:+.3f} m, {math.degrees(self.theta):+.1f} deg)"


# Translation3d(x, y, z) followed by Rotation3d, which serialises as a
# Quaternion(w, x, y, z) -- seven little-endian doubles, 56 bytes.
_POSE3D = struct.Struct("<ddddddd")


@dataclass(frozen=True)
class Pose3d:
    """A pose from a `struct:Pose3d` topic, carrying only what a floor
    plan needs.

    The rotation is dropped on decode rather than converted to Euler
    angles. Everything read as a Pose3d here is a game piece or a fixed
    goal, and neither a sphere's orientation nor a bolted-down HUB's has
    any bearing on where a robot should drive. Adding the conversion
    would mean either a quaternion-to-yaw implementation to maintain or
    the wpimath dependency this module exists to avoid.
    """

    x: float
    y: float
    z: float

    @classmethod
    def decode(cls, raw: bytes) -> "Pose3d":
        if len(raw) != _POSE3D.size:
            raise ValueError(f"expected {_POSE3D.size} bytes for a Pose3d struct, got {len(raw)}")
        x, y, z = _POSE3D.unpack(raw)[:3]
        return cls(x, y, z)

    @classmethod
    def decode_array(cls, raw: bytes) -> list["Pose3d"]:
        """Decode a `struct:Pose3d[]` payload.

        WPILib serialises a struct array as its elements back to back with
        no header, so the count is the length. A payload that is not a
        whole number of poses means the topic is not what it claims to be
        -- worth an exception rather than a truncated read, because the
        caller would otherwise get a plausible-looking short list.
        """
        if len(raw) % _POSE3D.size:
            raise ValueError(
                f"{len(raw)} bytes is not a whole number of {_POSE3D.size}-byte Pose3d structs"
            )
        return [cls.decode(raw[i:i + _POSE3D.size]) for i in range(0, len(raw), _POSE3D.size)]

    def __str__(self) -> str:
        return f"({self.x:+.3f} m, {self.y:+.3f} m, {self.z:+.3f} m)"


class RobotStateLink:
    """An NT4 client subscribed to the robot's published state.

    Usable as a context manager. Subscriptions are created lazily and cached,
    so a caller can read an arbitrary key without declaring it up front -- which
    matters while nobody yet knows the full set of keys worth watching.
    """

    def __init__(
        self,
        server: str = DEFAULT_SERVER,
        client_name: str = DEFAULT_CLIENT_NAME,
        first_read_timeout: float = 2.0,
    ):
        if ntcore is None:  # pragma: no cover - depends on the install
            raise RuntimeError(
                "pyntcore is not installed, so nothing can be read back from the robot. "
                "pip install -r bridge/requirements.txt"
            ) from _NTCORE_ERROR
        self.server = server
        self.first_read_timeout = first_read_timeout
        self._inst = ntcore.NetworkTableInstance.create()
        self._inst.startClient4(client_name)
        self._inst.setServer(server, ntcore.NetworkTableInstance.kDefaultPort4)
        self._subs: dict[tuple[str, str], ntcore.Subscriber] = {}
        self._pubs: dict[tuple[str, str], object] = {}

    def close(self) -> None:
        for sub in self._subs.values():
            sub.close()
        self._subs.clear()
        for pub in self._pubs.values():
            pub.close()
        self._pubs.clear()
        self._inst.stopClient()
        ntcore.NetworkTableInstance.destroy(self._inst)

    def __enter__(self) -> "RobotStateLink":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    @property
    def connected(self) -> bool:
        return self._inst.isConnected()

    def wait_for_connection(self, timeout: float = 60.0, poll: float = 0.25) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._inst.isConnected():
                return
            time.sleep(poll)
        raise TimeoutError(f"no NetworkTables server at {self.server} after {timeout:.0f}s")

    def wait_for_topic(self, name: str, timeout: float = 60.0, poll: float = 0.25) -> None:
        """Block until `name` exists and has a value.

        Connection is not readiness: NT answers as soon as the server socket is
        up, which on this robot happens well before `RobotContainer` has
        finished building subsystems and PathPlanner has warmed up. Waiting for
        a specific topic to carry a value is the honest "robot is running" test.
        """
        deadline = time.monotonic() + timeout
        sub = self._raw_sub(name)
        while time.monotonic() < deadline:
            if sub.getAtomic().time != 0:
                return
            time.sleep(poll)
        raise TimeoutError(f"topic {name} never published a value within {timeout:.0f}s")

    # -- reads -------------------------------------------------------------

    def pose(self, name: str = POSE_TRUTH) -> Pose2d | None:
        value = self._raw_sub(name).getAtomic()
        if value.time == 0:
            return None
        return Pose2d.decode(bytes(value.value))

    def truth_pose(self) -> Pose2d | None:
        """Where the robot *is*, per maple-sim's physics."""
        return self.pose(POSE_TRUTH)

    def odometry_pose(self) -> Pose2d | None:
        """Where the robot *thinks* it is. See POSE_ODOMETRY -- currently pinned to truth."""
        return self.pose(POSE_ODOMETRY)

    def pose2d_array(self, name: str) -> list[Pose2d] | None:
        """Decode a `struct:Pose2d[]` topic, or None if it has never published.

        None means "nobody is publishing this", `[]` means "published, and
        there are none". The extra robots make that distinction load-bearing:
        an empty list is a match with no opponents in it, while None is a
        robot build that predates them.
        """
        value = self._raw_sub(name, "struct:Pose2d[]").getAtomic()
        if value.time == 0:
            return None
        return Pose2d.decode_array(bytes(value.value))

    def pose3d(self, name: str) -> Pose3d | None:
        value = self._raw_sub(name, "struct:Pose3d").getAtomic()
        if value.time == 0:
            return None
        return Pose3d.decode(bytes(value.value))

    def pose3d_array(self, name: str) -> list[Pose3d] | None:
        """Decode a `struct:Pose3d[]` topic, or None if it has never published.

        None and `[]` are different answers and both happen here: the fuel
        array is legitimately empty once every piece has been collected,
        while None means nothing has been published at all. A caller
        counting pieces must not read the second as the first.
        """
        value = self._raw_sub(name, "struct:Pose3d[]").getAtomic()
        if value.time == 0:
            return None
        return Pose3d.decode_array(bytes(value.value))

    def chassis_speeds(self, name: str = CHASSIS_SETPOINT) -> ChassisSpeeds | None:
        value = self._raw_sub(name, "struct:ChassisSpeeds").getAtomic()
        if value.time == 0:
            return None
        return ChassisSpeeds.decode(bytes(value.value))

    def drive_current(self) -> float | None:
        """Mean drive-motor current across the four modules in amps, or None.

        None rather than 0.0 when nothing has published, because those are very
        different facts and `number()` cannot tell them apart -- an unresolved
        topic returns its default, which here would read as "the motors are
        drawing nothing" and turn every pinned robot into a reported fault.
        That is the exact bug this signal was added to fix, so it must not be
        reintroduced by a topic name going stale.
        """
        readings = [abs(self.number(key)) for key in DRIVE_CURRENT if self.has_value(key)]
        return sum(readings) / len(readings) if readings else None

    def has_value(self, name: str) -> bool:
        """Whether `name` has ever carried a value, as opposed to reading as its default."""
        return self._double_sub(name).getAtomic().time != 0

    def string_array(self, name: str) -> list[str]:
        return list(self._string_array_sub(name).get([]))

    def alerts(self, group: str, level: str = "errors") -> list[str]:
        """AdvantageKit Alert strings, e.g. alerts("PathPlanner", "errors").

        A structured second source for the fault oracle: these are already
        published, already categorised by severity, and need no log parsing.
        """
        return self.string_array(f"{OUTPUTS}/{group}/{level}")

    def number(self, name: str, default: float = 0.0) -> float:
        return self._double_sub(name).get(default)

    def boolean(self, name: str, default: bool = False) -> bool:
        return self._bool_sub(name).get(default)

    def integer(self, name: str, default: int = 0) -> int:
        return self._int_sub(name).get(default)

    def double_array(self, name: str) -> list[float]:
        """A `double[]` topic. Distinct from `float_array`, which subscribes
        as float32 and would quietly halve the precision of anything read
        through it."""
        return list(self._double_array_sub(name).get([]))

    def integer_array(self, name: str) -> list[int]:
        return [int(v) for v in self._int_array_sub(name).get([])]

    def float_array(self, name: str) -> list[float]:
        return list(self._float_array_sub(name).get([]))

    def joystick_axes(self, joystick: int = 0) -> list[float]:
        """What the robot code sees on that stick -- our own input, echoed back."""
        return self.float_array(joystick_axes_key(joystick))

    def joystick_buttons(self, joystick: int = 0) -> set[int]:
        """Currently-pressed WPILib button numbers (1-indexed), from the bitfield."""
        bits = self.integer(joystick_buttons_key(joystick))
        return {n for n in range(1, 33) if bits & (1 << (n - 1))}

    def topic_info(self, prefix: str = "", settle: float = 1.0) -> dict[str, str]:
        """Map of topic name -> NT type string for everything under `prefix`.

        An NT4 client is only told about topics it has asked for, so a bare
        `getTopics()` returns this client's own subscriptions and nothing else
        -- which reads as "the robot is publishing two things" and is wrong.
        A topics-only `MultiSubscriber` on the prefix is what makes the server
        announce the rest; `settle` is the round trip for those to land.

        The type string is worth carrying: it is the difference between
        guessing that `flywheelSpeed` is a double and knowing it.
        """
        sub = ntcore.MultiSubscriber(self._inst, [prefix], ntcore.PubSubOptions(topicsOnly=True))
        try:
            time.sleep(settle)
            return {
                info.name: info.type_str
                for info in self._inst.getTopicInfo()
                if info.name.startswith(prefix)
            }
        finally:
            sub.close()

    def topic_names(self, prefix: str = "", settle: float = 1.0) -> list[str]:
        return sorted(self.topic_info(prefix, settle))

    # -- writes ------------------------------------------------------------
    #
    # This module's name and its docstring both say "robot -> Python", and
    # for four steps that was the whole of it: everything the robot had to
    # say was already on the wire, and everything Python had to say went
    # through the HALSim link as a joystick.
    #
    # Step 6 broke that symmetry, because the other robots on the field have
    # no joystick. There is exactly one DriverStation in a WPILib simulation
    # and the robot code under test owns it; an opponent is a body in
    # maple-sim with nothing driving it, so the only channel left is the one
    # already open. Hence a publisher here rather than a second NT client in
    # `opponents.py` -- one connection, one place that knows the server
    # address, and one thing to close.
    #
    # `keepDuplicates` on every publisher, deliberately. A robot asked to
    # hold still sends the same three zeros every tick, and a wire that
    # coalesces those looks identical to a wire that has gone quiet -- which
    # is the exact signal the far side's staleness watchdog reads.

    def publish_double_array(self, name: str, value) -> None:
        self._pub("double[]", name, lambda: self._inst.getDoubleArrayTopic(name).publish(
            ntcore.PubSubOptions(keepDuplicates=True))).set(list(value))

    def publish_boolean(self, name: str, value: bool) -> None:
        self._pub("bool", name, lambda: self._inst.getBooleanTopic(name).publish(
            ntcore.PubSubOptions(keepDuplicates=True))).set(bool(value))

    def publish_integer(self, name: str, value: int) -> None:
        self._pub("int", name, lambda: self._inst.getIntegerTopic(name).publish(
            ntcore.PubSubOptions(keepDuplicates=True))).set(int(value))

    def flush(self) -> None:
        """Push what has been set, rather than waiting for the next network
        period.

        NT4 batches at 100 Hz by default. A strategy tick that publishes five
        robots' velocities and then sleeps is a tick whose commands arrive
        somewhere in the next 10 ms -- fine on average and jittery in a way
        that shows up as opponents that move in small lurches. Flushing at
        the end of a tick costs one syscall and makes the timing the loop's
        own rather than the transport's.
        """
        self._inst.flush()

    # -- internals ---------------------------------------------------------

    def _sub(self, kind: str, name: str, make):
        """Cache one subscriber per (kind, name), primed before first use.

        Keyed on kind as well as name because the same topic can legitimately
        be read two ways -- a pose as raw bytes, say, while something else
        wants its timestamp -- and a name-only cache would hand back a
        subscriber of the wrong type.

        The priming wait is the part that matters. A brand-new subscription
        has no value until the server answers, so a create-then-read in one
        call returns the *default* -- silently, and indistinguishably from a
        real reading. That produced a false failure ("robot reports itself
        enabled") on a topic whose real value was the opposite, which is the
        worst way for this to go wrong: the default happens to be plausible.
        Blocking once, here, means no caller has to remember to sleep first.
        """
        key = (kind, name)
        if key not in self._subs:
            sub = make()
            self._subs[key] = sub
            deadline = time.monotonic() + self.first_read_timeout
            while time.monotonic() < deadline and sub.getAtomic().time == 0:
                time.sleep(0.02)
        return self._subs[key]

    def _raw_sub(self, name: str, type_str: str = "struct:Pose2d") -> ntcore.RawSubscriber:
        return self._sub(f"raw:{type_str}", name, lambda: self._inst.getRawTopic(name).subscribe(type_str, b""))

    def _double_sub(self, name: str) -> ntcore.DoubleSubscriber:
        return self._sub("double", name, lambda: self._inst.getDoubleTopic(name).subscribe(0.0))

    def _bool_sub(self, name: str) -> ntcore.BooleanSubscriber:
        return self._sub("bool", name, lambda: self._inst.getBooleanTopic(name).subscribe(False))

    def _int_sub(self, name: str) -> ntcore.IntegerSubscriber:
        return self._sub("int", name, lambda: self._inst.getIntegerTopic(name).subscribe(0))

    def _float_array_sub(self, name: str) -> ntcore.FloatArraySubscriber:
        return self._sub("float[]", name, lambda: self._inst.getFloatArrayTopic(name).subscribe([]))

    def _double_array_sub(self, name: str) -> ntcore.DoubleArraySubscriber:
        return self._sub("double[]", name, lambda: self._inst.getDoubleArrayTopic(name).subscribe([]))

    def _int_array_sub(self, name: str) -> ntcore.IntegerArraySubscriber:
        return self._sub("int[]", name, lambda: self._inst.getIntegerArrayTopic(name).subscribe([]))

    def _string_array_sub(self, name: str) -> ntcore.StringArraySubscriber:
        return self._sub("string[]", name, lambda: self._inst.getStringArrayTopic(name).subscribe([]))

    def _pub(self, kind: str, name: str, make):
        """Cache one publisher per (kind, name).

        No priming wait, unlike `_sub`: a publisher is ready the moment it
        exists, and there is nothing to read back.
        """
        key = (kind, name)
        if key not in self._pubs:
            self._pubs[key] = make()
        return self._pubs[key]
