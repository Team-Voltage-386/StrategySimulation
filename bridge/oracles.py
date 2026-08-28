"""Oracles 01, 02, 03 and 05: how a fuzz run learns to fail.

A campaign is only as good as its ability to recognise a failure. Without
these, 190 unattended matches produce 190 matches that ran.

    01  FaultOracle      -- hard faults. Stack traces, DriverStation.reportError,
                            scheduler faults, loop overruns. All of it is already
                            on the console; the work is deciding what counts.
    02  LivenessMonitor  -- the frozen-robot family. Commanded but not moving,
                            requested but never reaching setpoint, a robot loop
                            that stopped advancing at all.
    03  InvariantMonitor -- things that must never be true, of any robot on the
                            field. NaN, off the field, teleported, driven while
                            disabled, commanded past the drivetrain, holding an
                            impossible number of pieces.
    05  CoverageOracle   -- which of the robot's own code the campaign never
                            entered, and whether it has stopped reaching
                            anything new.

(04, differential scoring, is deliberately not here. See bridge/README.md.)

02 and 03 are the two halves of the usual split: 02 is liveness, something that
should happen and has not; 03 is safety, something that should never happen and
has. Keeping them apart is not tidiness -- they debounce differently, they care
about opposite sides of the enable, and their evidence is a different shape.

05 is not that shape at all. 01 to 03 judge a match; 05 judges the campaign,
and neither thing it can say belongs to any single seed. It is also the only
one that can report that a night was wasted while every match passed.

The design constraint on all three is false positives, not misses. The strategy
sim will command things no human would, and some reported failures will be
"nobody would ever do that". Untriaged, that noise kills the habit of reading
the morning report inside a week -- at which point a working detector and a
broken one are worth the same. So:

* every muffled pattern carries a written reason, not just a regex, so the next
  person can re-litigate it instead of guessing why it is there;
* every detector over a continuous quantity is debounced by *duration*, because
  a single sample of "not moving" is a scheduling hiccup and two seconds of it
  is a bug. The exceptions are oracle 03's, and they are exceptions on purpose:
  a NaN or a teleport is never jitter, and waiting to be sure means missing it;
* every detector fires once per episode and re-arms only after the condition
  clears, so one wedged mechanism is one finding rather than four hundred.

And, as with the Python-side stall audit: these are **detectors, not
watchdogs**. Nothing here intervenes, resets, or nudges the robot back into
motion. A detector that fixes the thing it detects destroys its own evidence.
"""
from __future__ import annotations

import math
import re
import time
from dataclasses import dataclass

from bridge import jacoco

try:
    from bridge import robot_state as rs
except ImportError as exc:  # pragma: no cover - depends on the install
    # Oracle 01 is pure text processing and has no business requiring pyntcore,
    # a JVM, or a robot project. Keeping it importable without them is what
    # lets its rules be unit-tested in CI, where none of that exists -- and an
    # untested fault oracle is one nobody finds out is broken.
    rs = None  # type: ignore[assignment]
    _ROBOT_STATE_ERROR = exc
else:
    _ROBOT_STATE_ERROR = None

# ---------------------------------------------------------------------------
# findings
# ---------------------------------------------------------------------------

ERROR = "error"
WARNING = "warning"

#: The two ways "commanded but not moving" gets reported. Callers that want to
#: know whether the stuck detector fired at all -- the preflight, mainly --
#: should test against this rather than against one kind, since which one comes
#: back depends on the drive current at the moment it fired.
STUCK_KINDS = ("frozen-robot", "robot-pinned")

#: The possession reader's "nothing published here", kept far away from any
#: number a robot could really publish. It cannot be -1: a robot that genuinely
#: reports -1 pieces held is exactly the violation oracle 03 exists to catch,
#: and a sentinel that swallows it would hide the bug it was standing in for.
NO_COUNT = -(2 ** 31)


@dataclass(frozen=True)
class Finding:
    """One thing worth a human's attention, from either oracle."""

    oracle: str  # "faults" | "liveness" | "invariants" | "coverage"
    kind: str  # short slug, stable enough to group by across runs
    severity: str  # ERROR | WARNING
    message: str
    where: str  # "console:142" or "t=37.4s"
    detail: str = ""

    def __str__(self) -> str:
        head = f"[{self.severity.upper():7}] {self.kind:24} {self.where:16} {self.message}"
        if not self.detail:
            return head
        body = "\n".join(f"           | {line}" for line in self.detail.splitlines())
        return f"{head}\n{body}"


def summarize(findings: list[Finding]) -> str:
    if not findings:
        return "no findings"
    errors = sum(1 for f in findings if f.severity == ERROR)
    warnings = len(findings) - errors
    kinds = sorted({f.kind for f in findings})
    return f"{errors} error(s), {warnings} warning(s) across {len(kinds)} kind(s): {', '.join(kinds)}"


# ---------------------------------------------------------------------------
# oracle 01 -- hard faults
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Muffle:
    """A console pattern that is known-benign, and why.

    The reason is the load-bearing half. A bare regex list rots into a thing
    nobody dares touch because nobody remembers what it was hiding.
    """

    pattern: str
    reason: str

    def matches(self, line: str) -> bool:
        return re.search(self.pattern, line) is not None


DEFAULT_MUFFLES = (
    Muffle(
        r"^> Task :|^> Configure project|^BUILD SUCCESSFUL|^BUILD FAILED|^\d+ actionable task",
        "gradle's own build output, not the robot's",
    ),
    Muffle(
        r"warning: \[removal\]|warning: \[deprecation\]|^\d+ warnings?$|^Note: ",
        "javac warnings about the robot source, surfaced at build time and not at runtime",
    ),
    Muffle(
        r"If you receive errors loading the JNI dependencies",
        "boilerplate advice printed by every sim launch; contains the word 'errors' "
        "but reports nothing",
    ),
    Muffle(
        r"NT4? socket error: operation canceled|NT: .*connection closed",
        "our own teardown closing the NT socket from the Python side",
    ),
    Muffle(
        r"Warning at .*DriverStation.*Joystick .* not available",
        "emitted for the joystick ports the bridge does not populate; the ports it "
        "does populate are covered by the smoke test's ECHO check",
    ),
    Muffle(
        r"Warning at edu\.wpi\.first\.wpilibj\.Tracer\.",
        "the per-epoch breakdown WPILib prints immediately after a loop overrun "
        "(printLoopOverrunMessage calls printEpochs, and every epoch goes out as its "
        "own reportWarning). The overrun is already counted; these lines are detail "
        "on that one event, not a dozen separate findings",
    ),
)

# Stack-trace heads, in the two shapes the JVM prints them.
_EXCEPTION_HEAD = re.compile(
    r"(?:^|\s)(?:Exception in thread .*?)?"
    r"((?:[a-z][\w.]*\.)?[A-Z]\w*(?:Exception|Error))(?::\s*(.*))?$"
)
_STACK_FRAME = re.compile(r"^\s+(?:at |\.\.\. \d+ more|Caused by: |Suppressed: )")

# DriverStation.reportError / reportWarning, as WPILib formats them:
#
#   Error at frc.robot.Robot.teleopInit(Robot.java:140): intake never deployed
#
# The location contains a colon of its own, inside the parentheses, so
# splitting on the first colon puts "140): " at the front of every message.
# Prefer a location ending at its closing paren; fall back to a colon-free
# location for the rarer form that has no source reference.
_DS_REPORT = re.compile(r"^(Error|Warning) at (.*?\)|[^:]*):\s*(.*)$")

_LOOP_OVERRUN = re.compile(r"Loop time of ([\d.]+)s overrun")

# The ways a robot program dies rather than merely complains.
_HARD_STOP = re.compile(
    r"The robot program quit unexpectedly|Robots should not quit|"
    r"Error at frc\.robot|robot program had an unhandled exception|"
    r"Unhandled exception",
    re.IGNORECASE,
)


class FaultOracle:
    """Oracle 01: read a captured console and say what went wrong.

    Post-hoc rather than streaming, because a stack trace is several lines and
    the interesting part (`Caused by:`) is at the bottom.
    """

    def __init__(
        self,
        muffles: tuple[Muffle, ...] = DEFAULT_MUFFLES,
        max_loop_overruns: int = 5,
        report_ds_warnings: bool = True,
    ):
        self.muffles = muffles
        # A couple of overruns while PathPlanner warms up and the JIT settles is
        # normal on any robot. A *sustained* stream of them is a real finding,
        # and the count is the only thing separating the two in a console log
        # with no timestamps. Oracle 02 catches the timed version properly.
        self.max_loop_overruns = max_loop_overruns
        self.report_ds_warnings = report_ds_warnings
        self.muffled_count = 0
        self.loop_overrun_count = 0

    def scan_lines(self, lines: list[str]) -> list[Finding]:
        self.muffled_count = 0
        self.loop_overrun_count = 0
        findings: list[Finding] = []

        index = 0
        while index < len(lines):
            line = lines[index].rstrip()
            number = index + 1
            index += 1

            if not line.strip():
                continue
            if any(m.matches(line) for m in self.muffles):
                self.muffled_count += 1
                continue

            overrun = _LOOP_OVERRUN.search(line)
            if overrun:
                self.loop_overrun_count += 1
                continue

            report = _DS_REPORT.match(line)
            if report:
                level, where, message = report.groups()
                severity = ERROR if level == "Error" else WARNING
                if severity == WARNING and not self.report_ds_warnings:
                    continue
                findings.append(
                    Finding(
                        oracle="faults",
                        kind=f"ds-{level.lower()}",
                        severity=severity,
                        message=message.strip(),
                        where=f"console:{number}",
                        detail=f"reported from {where.strip()}",
                    )
                )
                continue

            head = _EXCEPTION_HEAD.search(line)
            if head:
                # Absorb the frames underneath so the finding carries the whole
                # trace, and so the frames are not each reported as a fault.
                frames: list[str] = [line.strip()]
                while index < len(lines) and _STACK_FRAME.match(lines[index]):
                    frames.append(lines[index].rstrip())
                    index += 1
                exception, detail = head.group(1), (head.group(2) or "").strip()
                findings.append(
                    Finding(
                        oracle="faults",
                        kind="exception",
                        severity=ERROR,
                        message=f"{exception}{': ' + detail if detail else ''}",
                        where=f"console:{number}",
                        detail="\n".join(frames[: 1 + 12]),
                    )
                )
                continue

            if _HARD_STOP.search(line):
                findings.append(
                    Finding(
                        oracle="faults",
                        kind="robot-stopped",
                        severity=ERROR,
                        message=line.strip(),
                        where=f"console:{number}",
                    )
                )

        if self.loop_overrun_count > self.max_loop_overruns:
            findings.append(
                Finding(
                    oracle="faults",
                    kind="loop-overrun",
                    severity=WARNING,
                    message=f"{self.loop_overrun_count} loop overruns "
                    f"(threshold {self.max_loop_overruns})",
                    where="console",
                    detail="A handful at startup is normal. This many suggests the robot "
                    "loop is genuinely not keeping up.",
                )
            )
        return findings

    def scan_file(self, path) -> list[Finding]:
        from pathlib import Path

        text = Path(path).read_text(encoding="utf-8", errors="replace")
        return self.scan_lines(text.splitlines())

    def scan_alerts(self, state: "rs.RobotStateLink") -> list[Finding]:
        """The structured half: AdvantageKit's own Alert arrays.

        Cheap, already categorised by severity, and it catches things that
        never reach the console at all.
        """
        findings: list[Finding] = []
        for group in rs.ALERT_GROUPS:
            for level, severity in (("errors", ERROR), ("warnings", WARNING)):
                for message in state.alerts(group, level):
                    findings.append(
                        Finding(
                            oracle="faults",
                            kind=f"alert-{level[:-1]}",
                            severity=severity,
                            message=message,
                            where=f"alerts:{group}",
                        )
                    )
        return findings


# ---------------------------------------------------------------------------
# oracle 02 -- liveness
# ---------------------------------------------------------------------------


@dataclass
class Sample:
    """One observation of the robot, taken at `t` seconds since arming."""

    t: float
    enabled: bool
    timestamp_us: int
    truth: rs.Pose2d | None
    commanded: rs.ChassisSpeeds | None
    stick_magnitude: float
    flywheel_setpoint_rpm: float
    flywheel_speed_rad_s: float
    cycle_ms: float
    browned_out: bool
    #: None when no module has published a current reading -- distinct from a
    #: genuine zero, which means the motors are not being driven.
    drive_current: float | None = None


class _Latch:
    """Fires once when a condition has held continuously for `duration`.

    Re-arms only after the condition clears, which is what keeps one wedged
    mechanism from producing a finding on every one of 20 samples per second.
    """

    def __init__(self, duration: float):
        self.duration = duration
        self._since: float | None = None
        self._fired = False

    def update(self, condition: bool, now: float) -> bool:
        if not condition:
            self._since = None
            self._fired = False
            return False
        if self._since is None:
            self._since = now
            return False
        if self._fired or now - self._since < self.duration:
            return False
        self._fired = True
        return True

    @property
    def held_for(self) -> float:
        return 0.0 if self._since is None else time.monotonic() - self._since


RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def classify_stuck(drive_current: float | None, threshold: float) -> tuple[str, str]:
    """Decide which kind of "commanded but not moving" this is.

    Pulled out as a plain function because it is the highest-stakes line in the
    oracle: it alone decides whether a match passes or fails, and getting it
    wrong in either direction is expensive. Too eager and a third of matches
    fail on contact a driver would not notice; too shy and a drivetrain that
    stopped working gets waved through as "just pushing".

    Returns (kind, severity).
    """
    if drive_current is None:
        # Blind. Fail toward the benign reading rather than manufacture an
        # error out of a missing measurement.
        return "robot-pinned", WARNING
    if drive_current >= threshold:
        return "robot-pinned", WARNING
    return "frozen-robot", ERROR


@dataclass
class LivenessThresholds:
    """Every number oracle 02 decides on, in one place.

    Durations are all >= 1 s deliberately. Anything shorter picks up ordinary
    scheduling jitter, and a fuzz campaign that cries wolf is worse than none.
    """

    # A robot loop that has not advanced its own clock is wedged, not idle.
    code_stall_seconds: float = 1.5

    # Stick pushed but the drive never commanded anything: the binding layer
    # or the scheduler dropped it.
    input_ignored_seconds: float = 2.0
    stick_deadband: float = 0.2

    # Commanded but not moving. Two different things wear this signature, and
    # only one of them is a fault -- see `pinned_current_amps`.
    frozen_seconds: float = 2.0
    commanded_linear_min: float = 0.15  # m/s
    commanded_omega_min: float = 0.3  # rad/s
    moved_min_metres: float = 0.10
    turned_min_radians: float = 0.10

    # Mean per-module drive current above which "not moving" means the robot is
    # *pushing* on something rather than failing to drive at all.
    #
    # Deliberately a floor, not a high-water mark. Stall current scales with
    # applied voltage, so how hard a pinned robot pulls depends entirely on how
    # hard it was told to go: 58 A leaning on a wall at 2.06 m/s commanded, but
    # only 14 A nudging the hub at 0.50 m/s. A threshold picked from the first
    # number calls the second one a fault. What actually separates the two
    # cases is much simpler -- a drivetrain that is not being driven draws
    # nothing at all (measured: 0.0 A idle), so any real current means the
    # motors are working and something is holding the robot back.
    pinned_current_amps: float = 5.0

    # Requested but never reaching setpoint.
    mechanism_seconds: float = 3.0
    mechanism_follow_fraction: float = 0.5

    # Sustained overrun, the timed version of oracle 01's count.
    overrun_seconds: float = 2.0
    overrun_cycle_ms: float = 25.0

    brownout_seconds: float = 0.5


class LivenessMonitor:
    """Oracle 02: sample the robot while it runs and notice what stops.

    Runs on the caller's thread via `poll()`, or on its own via `start()`.
    Findings accumulate in `findings` and are safe to read at any point.
    """

    def __init__(
        self,
        state: "rs.RobotStateLink",
        operator=None,
        thresholds: LivenessThresholds | None = None,
        sample_hz: float = 20.0,
    ):
        if rs is None:  # pragma: no cover - depends on the install
            raise RuntimeError(
                "the liveness oracle needs NetworkTables: pip install -r bridge/requirements.txt"
            ) from _ROBOT_STATE_ERROR
        self.state = state
        self.operator = operator
        self.th = thresholds or LivenessThresholds()
        self.sample_hz = sample_hz
        self.findings: list[Finding] = []
        self.samples: list[Sample] = []

        self._t0 = time.monotonic()
        self._latches = {
            "robot-code-stalled": _Latch(self.th.code_stall_seconds),
            "input-ignored": _Latch(self.th.input_ignored_seconds),
            "frozen-robot": _Latch(self.th.frozen_seconds),
            "mechanism-not-following": _Latch(self.th.mechanism_seconds),
            "loop-overrun-sustained": _Latch(self.th.overrun_seconds),
            "brownout": _Latch(self.th.brownout_seconds),
        }
        self._last_timestamp_us = 0
        self._timestamp_changed_at = self._t0
        # Anchor for "has the robot moved lately". Reset whenever it does move,
        # so the question is always "since it last moved", not "since arming".
        self._anchor: rs.Pose2d | None = None
        self._thread = None
        self._stop = None

    # -- sampling ----------------------------------------------------------

    def sample(self) -> Sample:
        # Read the stick off the robot's own DriverStation log rather than off
        # the operator link. It is deliberately what the *robot* sees: that
        # makes "input-ignored" a claim about the binding layer and the
        # scheduler, with the transport already excluded.
        axes = self.state.joystick_axes(0)
        stick = max(abs(axes[0]), abs(axes[1]), abs(axes[4])) if len(axes) > 4 else 0.0
        return Sample(
            t=time.monotonic() - self._t0,
            enabled=self.state.boolean(rs.DS_ENABLED),
            timestamp_us=self.state.integer(rs.TIMESTAMP),
            truth=self.state.truth_pose(),
            commanded=self.state.chassis_speeds(rs.CHASSIS_SETPOINT),
            stick_magnitude=stick,
            flywheel_setpoint_rpm=self.state.number(rs.FLYWHEEL_SETPOINT_RPM),
            flywheel_speed_rad_s=self.state.number(rs.FLYWHEEL_SPEED_RAD_S),
            cycle_ms=self.state.number(rs.LOOP_CYCLE_MS),
            browned_out=self.state.boolean(rs.BROWNED_OUT),
            drive_current=self.state.drive_current(),
        )

    def poll(self) -> list[Finding]:
        """Take one sample and return any findings it produced."""
        sample = self.sample()
        self.samples.append(sample)
        new = self._evaluate(sample)
        self.findings.extend(new)
        return new

    def start(self) -> None:
        import threading

        self._stop = threading.Event()

        def loop():
            period = 1.0 / self.sample_hz
            while not self._stop.is_set():
                try:
                    self.poll()
                except Exception as exc:  # a dead NT link is not this thread's problem
                    self.findings.append(
                        Finding(
                            oracle="liveness",
                            kind="monitor-error",
                            severity=WARNING,
                            message=f"sampling failed: {exc!r}",
                            where=f"t={time.monotonic() - self._t0:.1f}s",
                        )
                    )
                    break
                self._stop.wait(period)

        self._thread = threading.Thread(target=loop, name="liveness", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._stop is not None:
            self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def __enter__(self) -> "LivenessMonitor":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def reset_episode(self) -> None:
        """Clear latch state at a phase boundary, keeping findings so far.

        Called between scenario phases so that, say, a deliberate wall pin does
        not leave a latch armed into the next phase.
        """
        for latch in self._latches.values():
            latch.update(False, time.monotonic())
        self._anchor = None
        self._timestamp_changed_at = time.monotonic()

    # -- detectors ---------------------------------------------------------

    def _evaluate(self, s: Sample) -> list[Finding]:
        now = time.monotonic()
        out: list[Finding] = []

        def fire(kind: str, severity: str, message: str, detail: str = "") -> None:
            out.append(
                Finding(
                    oracle="liveness",
                    kind=kind,
                    severity=severity,
                    message=message,
                    where=f"t={s.t:.1f}s",
                    detail=detail,
                )
            )

        # 1. Robot code stalled. Checked whether enabled or not: a wedged loop
        #    while disabled is every bit as broken, and easier to miss.
        advanced = s.timestamp_us != self._last_timestamp_us
        self._last_timestamp_us = s.timestamp_us
        if advanced:
            self._timestamp_changed_at = now
        # The latch supplies the duration, so the condition is just "did not
        # advance on this sample" -- checking elapsed time here as well would
        # silently double the threshold.
        if self._latches["robot-code-stalled"].update(s.timestamp_us != 0 and not advanced, now):
            fire(
                "robot-code-stalled",
                ERROR,
                f"AdvantageKit timestamp frozen at {s.timestamp_us} for "
                f"{now - self._timestamp_changed_at:.1f}s while NT is still connected",
                "The robot loop is not advancing. This is a hang in robot code, not an idle robot.",
            )

        # Everything below is only meaningful while the robot is enabled --
        # commands do not run when it is not, and the last-published setpoint
        # would otherwise sit there looking like a live command forever.
        if not s.enabled:
            for kind in ("input-ignored", "frozen-robot", "mechanism-not-following"):
                self._latches[kind].update(False, now)
            self._anchor = None
        else:
            out.extend(self._evaluate_enabled(s, now, fire))

        # 5. Sustained loop overrun.
        if self._latches["loop-overrun-sustained"].update(
            s.cycle_ms > self.th.overrun_cycle_ms, now
        ):
            fire(
                "loop-overrun-sustained",
                WARNING,
                f"loop cycle above {self.th.overrun_cycle_ms:.0f} ms for "
                f"{self.th.overrun_seconds:.1f}s (currently {s.cycle_ms:.1f} ms)",
            )

        # 6. Brownout.
        if self._latches["brownout"].update(s.browned_out, now):
            fire("brownout", ERROR, "robot reports a brownout")

        return out

    def _evaluate_enabled(self, s: Sample, now: float, fire) -> list[Finding]:
        out: list[Finding] = []
        commanded = s.commanded
        commanding = commanded is not None and (
            commanded.linear >= self.th.commanded_linear_min
            or abs(commanded.omega) >= self.th.commanded_omega_min
        )

        # 2. Input ignored: the stick is pushed, the drive commands nothing.
        stick_pushed = s.stick_magnitude >= self.th.stick_deadband
        if self._latches["input-ignored"].update(stick_pushed and not commanding, now):
            fire(
                "input-ignored",
                ERROR,
                f"stick at {s.stick_magnitude:.2f} for {self.th.input_ignored_seconds:.1f}s "
                f"but the drive commanded nothing",
                "Input reached the DriverStation but produced no chassis setpoint -- the "
                "binding layer or the scheduler dropped it. Contrast with frozen-robot, "
                "where the command exists and the robot does not follow it.",
            )

        # 3. Frozen robot: commanded, but the world is not changing.
        if not commanding or s.truth is None:
            self._latches["frozen-robot"].update(False, now)
            self._anchor = s.truth
        else:
            if self._anchor is None:
                self._anchor = s.truth
            moved = self._anchor.distance_to(s.truth)
            turned = abs(_wrap(s.truth.theta - self._anchor.theta))
            if moved >= self.th.moved_min_metres or turned >= self.th.turned_min_radians:
                self._anchor = s.truth  # it moved; the clock restarts
                self._latches["frozen-robot"].update(False, now)
            elif self._latches["frozen-robot"].update(True, now):
                # Same symptom, two very different causes, told apart by
                # whether the motors are drawing current at all. A robot
                # leaning on the hub is doing physics; a robot whose drive is
                # commanded but drawing nothing never got the command to the
                # motors. Reporting both as errors made a third of matches fail
                # on contact that a driver would not even notice.
                symptom = (
                    f"commanded {commanded.linear:.2f} m/s, {commanded.omega:+.2f} rad/s for "
                    f"{self.th.frozen_seconds:.1f}s but moved {moved:.3f} m / "
                    f"{math.degrees(turned):.1f} deg"
                )
                kind, severity = classify_stuck(s.drive_current, self.th.pinned_current_amps)
                if s.drive_current is None:
                    reading = "; drive current unavailable, so this could not be classified"
                    why = (
                        f"{rs.DRIVE_CURRENT[0]} and its siblings published nothing, so "
                        f"pinned-on-geometry and drive-not-working cannot be told apart. "
                        f"Check the topic names against the robot project before trusting "
                        f"any stuck finding from this run."
                    )
                elif kind == "robot-pinned":
                    reading = f", drawing {s.drive_current:.0f} A"
                    why = (
                        "The drivetrain is straining, so it is pushing on something -- a "
                        "wall, a field element, another robot. Physical, not a code fault, "
                        "but a match spent wedged is still a strategy problem worth seeing."
                    )
                else:
                    reading = f", drawing only {s.drive_current:.1f} A"
                    why = (
                        "The drive is commanded but the motors are not working, so nothing "
                        "is holding the robot back -- it simply is not being driven. This "
                        "is the one worth waking up for."
                    )
                fire(kind, severity, symptom + reading, f"Robot is at {s.truth}. {why}")

        # 4. Mechanism requested but not following.
        wanted = s.flywheel_setpoint_rpm * RPM_TO_RAD_S
        following = wanted <= 0 or s.flywheel_speed_rad_s >= wanted * self.th.mechanism_follow_fraction
        if self._latches["mechanism-not-following"].update(not following, now):
            fire(
                "mechanism-not-following",
                ERROR,
                f"flywheel commanded {s.flywheel_setpoint_rpm:.0f} RPM ({wanted:.0f} rad/s) for "
                f"{self.th.mechanism_seconds:.1f}s but reached only {s.flywheel_speed_rad_s:.0f} rad/s",
                "The command is running. The mechanism is not doing what it says.",
            )
        return out


def _wrap(radians: float) -> float:
    """Shortest signed angle, so 359 deg -> 1 deg reads as 2 deg and not 358."""
    return (radians + math.pi) % (2.0 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# oracle 03 -- invariants
# ---------------------------------------------------------------------------
#
# Oracle 02 asks whether something that should happen did. This one asks
# whether something that should never happen has. That is the classic
# liveness/safety split, and it is worth keeping as two oracles rather than ten
# more detectors in one, because almost every design decision differs:
#
#   * 02 debounces by duration -- a single sample of "not moving" is jitter.
#     Several invariants here must fire on a *single* sample, because a NaN or
#     a teleport is never jitter and is frequently gone by the next read.
#   * 02 is only meaningful while enabled. Half of these are most interesting
#     while disabled.
#   * 02's evidence is a stretch of time. An invariant's evidence is one
#     instant, and the useful detail is the value that broke it.
#
# All of it is game-agnostic on purpose. Nothing below knows what a FUEL is,
# where the HUB stands, or which alliance is which -- these are properties of
# *any* robot on *any* field, which is the whole point of building them against
# the game this bridge is only ever proved against.


@dataclass
class Snapshot:
    """One instant of everything oracle 03 has an opinion about."""

    t: float
    enabled: bool
    truth: "rs.Pose2d | None"
    commanded: "rs.ChassisSpeeds | None"
    #: Ground-truth poses of the other five robots, empty on a solo run.
    extras: list
    held: int
    drive_current: float | None
    battery_volts: float
    flywheel_setpoint_rpm: float

    def robots(self) -> list[tuple[str, "rs.Pose2d"]]:
        """Every robot on the field, named. Ours first, then the extras.

        The invariants below do not care which one is under test: a robot
        ejected through the field wall invalidates the match whoever it was.
        """
        out = [("ours", self.truth)] if self.truth is not None else []
        out += [(f"extra{i}", p) for i, p in enumerate(self.extras) if p is not None]
        return out


class _OneShot:
    """Fires the first time a condition is true, and re-arms when it clears.

    The undebounced sibling of `_Latch`, for the violations where waiting two
    seconds to be sure would mean never seeing it: a NaN that propagates for
    one cycle and is overwritten, a body the physics engine ejected and then
    settled. Same re-arming discipline, so a stuck violation is still one
    finding and not twenty per second.
    """

    def __init__(self) -> None:
        self._fired = False

    def update(self, condition: bool) -> bool:
        if not condition:
            self._fired = False
            return False
        if self._fired:
            return False
        self._fired = True
        return True


@dataclass
class InvariantThresholds:
    """Every number oracle 03 decides on, in one place.

    The field size defaults to the FRC field, which has been these dimensions
    for every game this bridge could plausibly be pointed at. It is a
    threshold rather than an import from `bridge.arena` so that this module
    keeps its one useful property: it is pure arithmetic over samples, and
    imports without pyntcore, a JVM, or a robot project.
    """

    field_length_m: float = 16.541
    field_width_m: float = 8.052

    #: How far outside the field a robot's *centre* may be before this counts
    #: as ejected rather than as a bumper resting on the wall. A robot pressed
    #: flat against the border still has its centre half a footprint inside,
    #: so anything past the line at all is already suspicious; the margin is
    #: for the sim's own contact tolerance, not for legitimate driving.
    off_field_margin_m: float = 0.30
    off_field_seconds: float = 0.5

    #: Above this, a change in truth pose between two samples did not happen by
    #: driving. Set well clear of any FRC drivetrain (the fastest are under
    #: 6 m/s) so that legitimate motion cannot reach it no matter how the
    #: sampling jitters -- a teleport is a physics ejection or a direct pose
    #: write, and both are orders of magnitude past this, not marginally past.
    teleport_speed_mps: float = 12.0
    #: Sample gaps outside this range are not evidence of anything. A long gap
    #: is the sampler being descheduled, and dividing a real displacement by a
    #: wrong dt is how a teleport detector invents teleports.
    teleport_dt_range: tuple[float, float] = (0.01, 0.5)

    #: Motors drawing current with the DriverStation reporting disabled. The
    #: same floor oracle 02 uses to separate pinned from frozen, and for the
    #: same reason: an undriven drivetrain draws nothing at all, so any real
    #: current means something is commanding it.
    disabled_current_amps: float = 5.0
    #: Long enough to clear the disable itself -- the modules are still
    #: spinning for a moment afterwards, and regeneration is not a fault.
    disabled_current_seconds: float = 1.0

    #: How far past the *measured* drive limits a commanded speed may go before
    #: it is a bug rather than a rounding difference. Generous, because the
    #: limits come from a calibration probe and not from a constant.
    command_overrange_factor: float = 1.25
    command_overrange_seconds: float = 0.5

    #: Pieces held. `None` means the capacity is unknown and only the negative
    #: half of the invariant is checked -- see `InvariantMonitor.inactive`.
    piece_capacity: int | None = None

    #: A battery reading outside this is not a brownout, it is a broken
    #: reading. Oracle 02 already owns brownouts; this catches the sensor.
    battery_range_v: tuple[float, float] = (0.0, 14.0)


class InvariantMonitor:
    """Oracle 03: things that must never be true, on every robot on the field.

    Same shape as `LivenessMonitor` -- `poll()` on the caller's thread or
    `start()` on its own, findings accumulate and are safe to read at any
    point, and nothing here ever intervenes.
    """

    #: Every invariant this oracle knows, so a caller can report on the ones
    #: that are switched off instead of quietly getting fewer checks than it
    #: thinks. "No findings" from a detector that was never active and "no
    #: findings" from a clean run are the same empty list.
    KINDS = (
        "not-a-number",
        "off-the-field",
        "teleport",
        "driven-while-disabled",
        "command-out-of-range",
        "possession-impossible",
    )

    def __init__(
        self,
        state: "rs.RobotStateLink",
        limits=None,
        thresholds: InvariantThresholds | None = None,
        sample_hz: float = 20.0,
    ):
        # No NetworkTables guard here, unlike oracle 02. Every detector below
        # is arithmetic over a `Snapshot` and touches nothing else; only
        # `sample()` needs a live link. Keeping the constructor free of that
        # requirement is what lets all six invariants be proved in CI, where
        # there is no pyntcore -- and an unproved detector is the one failure
        # mode this whole file exists to avoid.
        self.state = state
        #: A `drive_model.DriveLimits`, or None. Without it there is no answer
        #: to "how fast is too fast", so that one invariant stands down rather
        #: than guessing at a constant.
        self.limits = limits
        self.th = thresholds or InvariantThresholds()
        self.sample_hz = sample_hz
        self.findings: list[Finding] = []
        #: How many snapshots were judged. A count and not the snapshots
        #: themselves, unlike oracle 02: at 20 Hz over a 150 s match this is
        #: three thousand of them, and a campaign holds two hundred matches.
        #: Kept at all because a monitor that never sampled and a monitor that
        #: saw nothing wrong report the same empty list.
        self.samples_taken = 0
        #: Every reason an invariant stood down at any point while sampling.
        #: Accumulated rather than read at the end, because `limits` can arrive
        #: mid-match -- the harness calibrates during the first one -- and a
        #: monitor asked afterwards would report full coverage for a match that
        #: spent part of itself checking five invariants out of six.
        self._ever_inactive: set[str] = set()
        #: The most robots this monitor ever judged at once, ours included.
        #: `off-the-field` and `teleport` are held per robot, so a contested
        #: campaign where this stayed at 1 checked one robot and reported the
        #: silence of five as a clean field. The same trap as an oracle that
        #: has never fired, one level down: an invariant applied to an empty
        #: list is not an invariant that held.
        self.robots_seen = 0

        self._t0 = time.monotonic()
        self._nan = _OneShot()
        self._battery = _OneShot()
        self._possession = _OneShot()
        self._teleports: dict = {}
        self._off_field: dict = {}
        self._disabled_drive = _Latch(self.th.disabled_current_seconds)
        self._overrange = _Latch(self.th.command_overrange_seconds)
        #: Last (t, pose) per robot, for the teleport test.
        self._previous: dict = {}
        self._thread = None
        self._stop = None

    # -- what is switched off ----------------------------------------------

    @property
    def inactive(self) -> list[str]:
        """Invariants that cannot fire *right now*, and why.

        Reported rather than inferred. A detector standing down for a good
        reason is fine; a detector standing down silently is how a campaign
        spends eight hours checking less than the report claims.
        """
        out = []
        if self.limits is None:
            out.append("command-out-of-range: no calibrated drive limits were supplied")
        if self.th.piece_capacity is None:
            out.append("possession-impossible: no piece capacity, so only a negative count fires")
        return out

    @property
    def stood_down(self) -> list[str]:
        """Invariants that were inactive at any point while this was sampling.

        The one to report after the fact. `inactive` answers "what is switched
        off now", which for a monitor that has already finished is a different
        and more flattering question.
        """
        return sorted(self._ever_inactive)

    # -- sampling ----------------------------------------------------------

    def sample(self) -> Snapshot:
        if rs is None:  # pragma: no cover - depends on the install
            raise RuntimeError(
                "sampling needs NetworkTables: pip install -r bridge/requirements.txt"
            ) from _ROBOT_STATE_ERROR
        return Snapshot(
            t=time.monotonic() - self._t0,
            enabled=self.state.boolean(rs.DS_ENABLED),
            truth=self.state.truth_pose(),
            commanded=self.state.chassis_speeds(rs.CHASSIS_SETPOINT),
            extras=self.state.pose2d_array(rs.BRIDGE_ROBOT_POSES) or [],
            held=self.state.integer(rs.BALL_COUNT, NO_COUNT),
            drive_current=self.state.drive_current(),
            battery_volts=self.state.number(rs.BATTERY_VOLTAGE),
            flywheel_setpoint_rpm=self.state.number(rs.FLYWHEEL_SETPOINT_RPM),
        )

    def poll(self) -> list[Finding]:
        snapshot = self.sample()
        self.samples_taken += 1
        self._ever_inactive.update(self.inactive)
        self.robots_seen = max(self.robots_seen, len(snapshot.robots()))
        new = self.evaluate(snapshot)
        self.findings.extend(new)
        return new

    def start(self) -> None:
        import threading

        self._stop = threading.Event()

        def loop():
            period = 1.0 / self.sample_hz
            while not self._stop.is_set():
                try:
                    self.poll()
                except Exception as exc:  # a dead NT link is not this thread's problem
                    self.findings.append(
                        Finding(
                            oracle="invariants",
                            kind="monitor-error",
                            severity=WARNING,
                            message=f"sampling failed: {exc!r}",
                            where=f"t={time.monotonic() - self._t0:.1f}s",
                        )
                    )
                    break
                self._stop.wait(period)

        self._thread = threading.Thread(target=loop, name="invariants", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._stop is not None:
            self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def __enter__(self) -> "InvariantMonitor":
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()

    def reset_episode(self) -> None:
        """Clear per-episode state at a phase boundary, keeping findings.

        The teleport history goes too: a phase boundary is exactly where the
        field legitimately gets rearranged, and carrying a stale pose across it
        turns a fresh start into a reported teleport.
        """
        now = time.monotonic()
        for latch in (self._disabled_drive, self._overrange, *self._off_field.values()):
            latch.update(False, now)
        for shot in (self._nan, self._battery, self._possession, *self._teleports.values()):
            shot.update(False)
        self._previous.clear()

    # -- detectors ---------------------------------------------------------

    def evaluate(self, s: Snapshot) -> list[Finding]:
        """Judge one snapshot. Pure, apart from the monitor's own latches.

        Public because it is the whole oracle: a caller holding a synthetic
        snapshot can prove every detector fires without a JVM, and the tests
        do exactly that.
        """
        now = time.monotonic()
        out: list[Finding] = []

        def fire(kind: str, severity: str, message: str, detail: str = "") -> None:
            out.append(
                Finding(
                    oracle="invariants",
                    kind=kind,
                    severity=severity,
                    message=message,
                    where=f"t={s.t:.1f}s",
                    detail=detail,
                )
            )

        self._check_numbers(s, fire)
        self._check_field(s, now, fire)
        self._check_teleport(s, fire)
        self._check_disabled(s, now, fire)
        self._check_command(s, now, fire)
        self._check_possession(s, fire)
        return out

    # 1. Nothing published is NaN or infinite.
    #
    # The classic FRC source is a swerve module optimising its angle from a
    # zero-length velocity vector, and the classic symptom is a robot that
    # works until the stick returns to centre. NaN also survives every
    # comparison quietly: `speed > limit` is false for NaN, so a value that
    # broke arithmetic slips silently past every other check in this file.
    # Which is why it is checked first, and why it fires on a single sample.
    def _check_numbers(self, s: Snapshot, fire) -> None:
        bad = []
        for name, pose in s.robots():
            for label, value in (("x", pose.x), ("y", pose.y), ("theta", pose.theta)):
                if not math.isfinite(value):
                    bad.append(f"{name}.{label}={value}")
        if s.commanded is not None:
            for label, value in (
                ("vx", s.commanded.vx),
                ("vy", s.commanded.vy),
                ("omega", s.commanded.omega),
            ):
                if not math.isfinite(value):
                    bad.append(f"commanded.{label}={value}")
        for label, value in (
            ("flywheel-setpoint", s.flywheel_setpoint_rpm),
            ("battery", s.battery_volts),
        ):
            if not math.isfinite(value):
                bad.append(f"{label}={value}")

        if self._nan.update(bool(bad)):
            fire(
                "not-a-number",
                ERROR,
                f"non-finite value published: {', '.join(bad)}",
                "NaN compares false against everything, so it defeats every threshold "
                "downstream of it as well as breaking whatever consumed it. In FRC code "
                "the usual source is a division or an atan2 on a zero-length vector.",
            )

        low, high = self.th.battery_range_v
        impossible = math.isfinite(s.battery_volts) and not low <= s.battery_volts <= high
        if self._battery.update(impossible):
            fire(
                "not-a-number",
                ERROR,
                f"battery reads {s.battery_volts:.2f} V, outside [{low:.1f}, {high:.1f}]",
                "Not a brownout -- oracle 02 owns those. This is the reading itself being "
                "impossible, which makes every current- and voltage-based judgement in "
                "this run untrustworthy.",
            )

    # 2. Every robot's centre stays on the field.
    #
    # Held per robot, because the interesting version of this is one of the
    # extras being squeezed out through a wall by a contact the physics engine
    # resolved badly -- which a solo run could not produce at all.
    def _check_field(self, s: Snapshot, now: float, fire) -> None:
        margin = self.th.off_field_margin_m
        for name, pose in s.robots():
            outside = (
                pose.x < -margin
                or pose.y < -margin
                or pose.x > self.th.field_length_m + margin
                or pose.y > self.th.field_width_m + margin
            )
            latch = self._off_field.setdefault(name, _Latch(self.th.off_field_seconds))
            if latch.update(outside, now):
                fire(
                    "off-the-field",
                    ERROR,
                    f"{name} is at {pose}, outside the "
                    f"{self.th.field_length_m:.2f} x {self.th.field_width_m:.2f} m field "
                    f"for {self.th.off_field_seconds:.1f}s",
                    "A robot cannot drive through the border, so the physics put it there. "
                    "Everything else this match reports about positions is suspect.",
                )

    # 3. No robot moves further between two samples than it could have driven.
    def _check_teleport(self, s: Snapshot, fire) -> None:
        low, high = self.th.teleport_dt_range
        for name, pose in s.robots():
            previous = self._previous.get(name)
            self._previous[name] = (s.t, pose)
            shot = self._teleports.setdefault(name, _OneShot())
            if previous is None:
                continue
            dt = s.t - previous[0]
            if not low <= dt <= high:
                shot.update(False)
                continue
            moved = previous[1].distance_to(pose)
            speed = moved / dt
            if shot.update(speed > self.th.teleport_speed_mps):
                fire(
                    "teleport",
                    ERROR,
                    f"{name} moved {moved:.2f} m in {dt * 1000:.0f} ms "
                    f"({speed:.0f} m/s) -- from {previous[1]} to {pose}",
                    "Faster than any drivetrain, so it was not driven there. Either the "
                    "physics engine resolved an overlap by ejecting a body, or something "
                    "wrote a pose directly.",
                )

    # 4. Nothing drives the motors while the DriverStation says disabled.
    #
    # A rule as well as a code invariant, and one of the few faults here a fuzz
    # campaign can genuinely produce: a subsystem that writes its outputs from
    # `periodic` rather than from a command does not stop when the match does,
    # and on a real field that is how a robot moves during a stoppage.
    def _check_disabled(self, s: Snapshot, now: float, fire) -> None:
        current = s.drive_current
        drawing = current is not None and current >= self.th.disabled_current_amps
        if self._disabled_drive.update(not s.enabled and drawing, now):
            fire(
                "driven-while-disabled",
                ERROR,
                f"drive drawing {current:.1f} A for "
                f"{self.th.disabled_current_seconds:.1f}s with the robot disabled",
                "An undriven drivetrain draws nothing, so something is still commanding "
                "the motors after the DriverStation disabled the robot.",
            )

    # 5. The commanded chassis speed stays inside what the drive can do.
    def _check_command(self, s: Snapshot, now: float, fire) -> None:
        if self.limits is None or s.commanded is None:
            self._overrange.update(False, now)
            return
        factor = self.th.command_overrange_factor
        speed_cap = self.limits.max_speed_mps * factor
        omega_cap = self.limits.max_omega_rad_s * factor
        linear = s.commanded.linear
        omega = abs(s.commanded.omega)
        if self._overrange.update(linear > speed_cap or omega > omega_cap, now):
            fire(
                "command-out-of-range",
                WARNING,
                f"commanded {linear:.2f} m/s / {omega:.2f} rad/s against a measured "
                f"maximum of {self.limits.max_speed_mps:.2f} / "
                f"{self.limits.max_omega_rad_s:.2f} (x{factor:g} allowed)",
                "The drive is being asked for more than it has. Usually input scaling, or "
                "two command sources summing; occasionally the calibration is stale, which "
                "is why this is a warning and not an error.",
            )

    # 6. The robot's own count of what it is carrying is possible.
    #
    # About the robot's bookkeeping, not the sim's: this reads the counter the
    # robot code publishes and believes, and the strategy layer's
    # collect-versus-score decision turns on it. A count that has gone negative
    # or past the hopper means the sensor handling has drifted from reality,
    # and every decision made from it afterwards is made from a wrong number.
    def _check_possession(self, s: Snapshot, fire) -> None:
        held = s.held
        if held == NO_COUNT:  # the reader's "nothing published", not a real count
            self._possession.update(False)
            return
        capacity = self.th.piece_capacity
        impossible = held < 0 or (capacity is not None and held > capacity)
        if self._possession.update(impossible):
            bound = f"[0, {capacity}]" if capacity is not None else "[0, inf)"
            fire(
                "possession-impossible",
                ERROR,
                f"the robot believes it is holding {held} pieces, outside {bound}",
                "The robot's own possession counter, which is what its decisions are made "
                "from. Every collect-or-score choice after this point was made from a "
                "number that cannot be true.",
            )


# -- proving oracle 03 -------------------------------------------------------


def _synthetic(pose, speeds, t: float, **overrides) -> Snapshot:
    """An ordinary instant, with one thing about it deliberately impossible."""
    base = dict(
        t=t,
        enabled=True,
        truth=pose(8.0, 4.0, 0.0),
        commanded=speeds(1.0, 0.0, 0.0),
        extras=[],
        held=4,
        drive_current=9.0,
        battery_volts=12.4,
        flywheel_setpoint_rpm=0.0,
    )
    base.update(overrides)
    return Snapshot(**base)


def _hold_synthetic(monitor, pose, speeds, seconds: float, step: float = 0.05, **overrides):
    """Feed one synthetic snapshot for a while and collect what it produced.

    Real sleeping, because the debounced invariants read the monotonic clock
    and not the snapshot's own `t`. Faking that clock would prove the detectors
    fire under a fake clock, which is not the claim being made.
    """
    out: list[Finding] = []
    t = 0.0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        t += step
        out.extend(monitor.evaluate(_synthetic(pose, speeds, t, **overrides)))
        time.sleep(step)
    return out


def prove_invariants(
    thresholds: InvariantThresholds,
    limits=None,
    *,
    pose=None,
    speeds=None,
) -> dict[str, list[Finding]]:
    """Push a deliberate violation of each invariant through oracle 03.

    Returns what each detector produced, keyed by the invariant it was aimed
    at. A key missing from the result was not attempted and says so; a key
    present but empty is a detector that stayed silent on a violation, which is
    the thing worth failing a campaign over.

    This is the weaker sibling of the wall pin, and the difference is worth
    being plain about. The wall pin is a real robot genuinely wedged. Nothing
    here is: these are synthetic snapshots pushed through the detectors by
    hand, because making real robot code publish a NaN or drive its motors
    while disabled would mean breaking it on purpose. That is oracle 01's
    trade, and it comes out the same way.

    What it buys over the unit tests, which prove the same six detectors in CI,
    is narrow and real: it runs the *shipped* thresholds and whatever drive
    limits the caller just measured, on the machine about to spend the night. A
    threshold edited to something unreachable passes the unit tests, which
    supply their own, and fails here.

    `pose` and `speeds` default to the NetworkTables structs, and are
    injectable so that this function is itself testable where those cannot be
    imported -- which is the same place the invariants themselves are tested.
    """
    if pose is None or speeds is None:
        if rs is None:  # pragma: no cover - depends on the install
            raise RuntimeError(
                "prove_invariants needs either pyntcore or explicit pose/speeds factories"
            ) from _ROBOT_STATE_ERROR
        pose = pose or rs.Pose2d
        speeds = speeds or rs.ChassisSpeeds

    results: dict[str, list[Finding]] = {}

    def attempt(kind: str, run) -> None:
        # A fresh monitor per invariant, so a latch tripped by one injection
        # cannot arm or mask the next, and so the order of this list does not
        # quietly become part of what is being proved.
        results[kind] = run(
            InvariantMonitor(state=None, limits=limits, thresholds=thresholds)
        )

    def snap(t, **over):
        return _synthetic(pose, speeds, t, **over)

    def hold(monitor, seconds, **over):
        return _hold_synthetic(monitor, pose, speeds, seconds, **over)

    attempt("not-a-number", lambda m: m.evaluate(snap(0.05, battery_volts=float("nan"))))

    attempt(
        "off-the-field",
        lambda m: hold(m, thresholds.off_field_seconds + 0.4, truth=pose(-4.0, 4.0, 0.0)),
    )

    def teleport(m):
        m.evaluate(snap(0.05, truth=pose(8.0, 4.0, 0.0)))
        return m.evaluate(snap(0.10, truth=pose(14.0, 4.0, 0.0)))

    attempt("teleport", teleport)

    attempt(
        "driven-while-disabled",
        lambda m: hold(m, thresholds.disabled_current_seconds + 0.4,
                       enabled=False, drive_current=40.0),
    )

    # Skipped rather than faked when the drive was never calibrated. A
    # violation measured against a made-up maximum would prove the arithmetic
    # and nothing about this robot, and the caller already reports the gap
    # through `InvariantMonitor.inactive`.
    if limits is not None:
        attempt(
            "command-out-of-range",
            lambda m: hold(m, thresholds.command_overrange_seconds + 0.4,
                           commanded=speeds(limits.max_speed_mps * 3.0, 0.0, 0.0)),
        )

    attempt("possession-impossible", lambda m: m.evaluate(snap(0.05, held=-1)))
    return results


def unproven_invariants(proof: dict[str, list[Finding]]) -> list[str]:
    """Which invariants were injected and stayed silent. Empty is the good case.

    Deliberately not the same as "which were not attempted": a detector that
    was skipped for a stated reason is a smaller problem than one that was
    handed a violation and shrugged.
    """
    return [
        kind for kind, found in proof.items()
        if not any(f.kind == kind for f in found)
    ]


# ---------------------------------------------------------------------------
# oracle 05 -- coverage
# ---------------------------------------------------------------------------


@dataclass
class CoverageThresholds:
    """When silence from the coverage numbers is worth reporting."""

    #: Consecutive matches that reached nothing new before the campaign is told
    #: it has stopped learning. Ten is roughly half an hour of wall clock, which
    #: is long enough that the answer is about the scenario generator rather
    #: than about one unlucky seed.
    plateau_matches: int = 10

    #: A dump naming fewer classes than this is treated as a broken measurement
    #: rather than a startling result. A robot that booted at all has executed
    #: hundreds of probes across dozens of classes; "two classes ran" means the
    #: agent attached to something that then died, and reporting the other
    #: hundred as never-run would be the loudest wrong answer available.
    minimum_classes: int = 10


#: Read this before believing a name on the never-run list, because two
#: entirely ordinary things land on it.
#:
#: JaCoCo places a class's probes so that each executes at the *end* of the
#: basic block it belongs to -- which for a straight-line method means just
#: before it returns. A method that has not returned yet has therefore recorded
#: nothing. `frc.robot.Main` is the standing example: `main` calls
#: `RobotBase.startRobot`, which blocks for the entire match, so the entry
#: point of the program appears in every never-run list this will ever produce.
#: Verified directly rather than reasoned about -- a class whose only method
#: was parked in `Thread.sleep` at dump time reported zero hits.
#:
#: And a class can hold code that genuinely never runs and never should: a
#: generated constants holder's implicit constructor, a hardware IO
#: implementation on a robot that is being simulated. Those entries are true
#: rather than false, and the way to stop reading them every morning is the
#: excludes filter, which the agent and this oracle share.
NEVER_RUN_CAVEAT = (
    "Two ordinary things land on this list. A method that has not returned "
    "records no probe, so an entry point that blocks for the whole match "
    "(frc.robot.Main) always appears here. And hardware IO classes cannot run "
    "on a simulated robot. Narrow the list with --coverage-excludes rather "
    "than by rereading it every morning."
)


class CoverageOracle:
    """Oracle 05: which of the robot's own code a campaign actually entered.

    Different in shape from 01-03, and the difference is not incidental. Those
    three judge a match: they watch one robot for 150 seconds and say whether
    something went wrong. This one judges the *campaign* -- neither of its
    findings can be attributed to a seed, because "no match ever entered
    `ClimbCommand`" is a fact about all of them at once. So it is owned by
    `Campaign` rather than `MatchRunner`, and it reports into its own section
    of the morning report rather than into any match's findings.

    It is also the only oracle that can say a night was wasted while every
    match passed, which is the thing a fuzzing harness is otherwise structurally
    unable to notice.
    """

    ORACLE = "coverage"

    KINDS = ("code-never-run", "coverage-plateau", "coverage-build-mismatch")

    def __init__(
        self,
        expected: set[str] | None = None,
        thresholds: CoverageThresholds | None = None,
    ):
        #: Every class the agent would have instrumented, whether it ran or not
        #: -- from the build output, because a dump names only what was hit.
        #: Empty means the question cannot be asked, not that the answer is
        #: "nothing was missed".
        self.expected = set(expected or ())
        self.thresholds = thresholds or CoverageThresholds()
        self.totals = jacoco.Coverage()
        #: New probes per match, in order. The zeros are the interesting part.
        self.gains: list[int] = []
        self._reasons: list[str] = []

    # -- collecting --------------------------------------------------------

    def observe(self, dump: "jacoco.Dump") -> int:
        """Merge one match's coverage in. Returns the probes new to the campaign."""
        gained = self.totals.add(dump)
        self.gains.append(gained)
        return gained

    def stand_down(self, reason: str) -> None:
        """Record a match that produced no coverage at all, and why.

        Not the same as a match that gained nothing: this one was never
        measured. Keeping them apart is the whole of the difference between
        "the campaign has stopped learning" and "the campaign stopped looking".
        """
        if reason not in self._reasons:
            self._reasons.append(reason)

    @property
    def matches_measured(self) -> int:
        return len(self.gains)

    @property
    def stood_down(self) -> list[str]:
        """Why coverage is missing or unreliable. Empty is the good case."""
        reasons = list(self._reasons)
        if not self.expected:
            reasons.append(
                "code-never-run: no compiled classes found under "
                f"{jacoco.CLASSES_DIR} -- build the robot project, or every "
                "class will look covered"
            )
        if self.totals.conflicts:
            sample = ", ".join(sorted(self.totals.conflicts)[:3])
            reasons.append(
                f"coverage totals: {len(self.totals.conflicts)} class(es) changed probe "
                f"count mid-campaign ({sample}) -- the robot was rebuilt while it ran, "
                "so these totals mix two builds"
            )
        return reasons

    # -- judging -----------------------------------------------------------

    @property
    def never_run(self) -> set[str]:
        return self.expected - self.totals.classes if self.expected else set()

    @property
    def unexpected(self) -> set[str]:
        return self.totals.classes - self.expected if self.expected else set()

    def findings(self) -> list[Finding]:
        """What the campaign's coverage is worth saying. Call once, at the end."""
        found: list[Finding] = []
        if not self.matches_measured:
            return found

        classes_seen = len(self.totals.classes)
        where = f"{self.matches_measured} matches"

        if classes_seen < self.thresholds.minimum_classes:
            # Below this the numbers describe a failed measurement, and every
            # judgement built on them would be confidently wrong. Say so and
            # stop, rather than reporting a hundred never-run classes because
            # the agent attached to a JVM that died on boot.
            return [
                Finding(
                    oracle=self.ORACLE,
                    kind="coverage-build-mismatch",
                    severity=WARNING,
                    message=(
                        f"only {classes_seen} class(es) ever executed, across "
                        f"{self.matches_measured} matches -- the coverage agent attached "
                        f"but measured almost nothing"
                    ),
                    where=where,
                    detail=(
                        "Expected hundreds of probes across dozens of classes from a robot "
                        "that booted. Check the -PbridgeCoverageIncludes filter, and whether "
                        "the JVM survived long enough to be asked."
                    ),
                ),
            ]

        if self.unexpected:
            sample = ", ".join(sorted(self.unexpected)[:6])
            found.append(
                Finding(
                    oracle=self.ORACLE,
                    kind="coverage-build-mismatch",
                    severity=WARNING,
                    message=(
                        f"{len(self.unexpected)} class(es) executed that are not in the "
                        f"build output -- the never-run list below is unreliable"
                    ),
                    where=where,
                    detail=(
                        f"{sample}\n"
                        f"The robot most likely ran against a different build than the one "
                        f"in {jacoco.CLASSES_DIR}. Rebuild and rerun before trusting either list."
                    ),
                )
            )

        if self.never_run:
            listing = "\n".join(sorted(self.never_run)[:40])
            more = len(self.never_run) - 40
            found.append(
                Finding(
                    oracle=self.ORACLE,
                    kind="code-never-run",
                    severity=WARNING,
                    message=(
                        f"{len(self.never_run)} of {len(self.expected)} classes were never "
                        f"entered by any match"
                    ),
                    where=where,
                    detail=(
                        listing
                        + (f"\n... and {more} more" if more > 0 else "")
                        + "\n\n"
                        + NEVER_RUN_CAVEAT
                    ),
                )
            )

        plateau = self.thresholds.plateau_matches
        if len(self.gains) >= plateau and not any(self.gains[-plateau:]):
            last_gain = max(
                (i for i, g in enumerate(self.gains) if g), default=None
            )
            since = (
                f"nothing new since match {last_gain}"
                if last_gain is not None
                else "nothing was ever reached"
            )
            found.append(
                Finding(
                    oracle=self.ORACLE,
                    kind="coverage-plateau",
                    severity=WARNING,
                    message=(
                        f"the last {plateau} matches reached no code the campaign had not "
                        f"already reached"
                    ),
                    where=where,
                    detail=(
                        f"{since}. More matches of this shape will keep passing and keep "
                        f"finding nothing; the thing to change is the scenario generator, "
                        f"not the match count."
                    ),
                )
            )
        return found

    def summary(self) -> str:
        """One line for the report header."""
        if not self.matches_measured:
            return "not measured"
        covered = len(self.totals.classes)
        text = (
            f"{self.totals.probes_hit}/{self.totals.probes_total} probes in "
            f"{covered} class(es)"
        )
        if self.expected:
            text += f", {covered}/{len(self.expected)} of the classes that exist"
        return text
