"""Oracles 01 and 02: how a fuzz run learns to fail.

A campaign is only as good as its ability to recognise a failure. Without
these, 190 unattended matches produce 190 matches that ran.

    01  FaultOracle     -- hard faults. Stack traces, DriverStation.reportError,
                           scheduler faults, loop overruns. All of it is already
                           on the console; the work is deciding what counts.
    02  LivenessMonitor -- the frozen-robot family. Commanded but not moving,
                           requested but never reaching setpoint, a robot loop
                           that stopped advancing at all.

The design constraint on both is false positives, not misses. The strategy sim
will command things no human would, and some reported failures will be "nobody
would ever do that". Untriaged, that noise kills the habit of reading the
morning report inside a week -- at which point a working detector and a broken
one are worth the same. So:

* every muffled pattern carries a written reason, not just a regex, so the next
  person can re-litigate it instead of guessing why it is there;
* every detector is debounced by *duration*, because a single sample of "not
  moving" is a scheduling hiccup and two seconds of it is a bug;
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


@dataclass(frozen=True)
class Finding:
    """One thing worth a human's attention, from either oracle."""

    oracle: str  # "faults" | "liveness"
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

    # Commanded but not moving: the frozen-robot signature.
    frozen_seconds: float = 2.0
    commanded_linear_min: float = 0.15  # m/s
    commanded_omega_min: float = 0.3  # rad/s
    moved_min_metres: float = 0.10
    turned_min_radians: float = 0.10

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
                fire(
                    "frozen-robot",
                    ERROR,
                    f"commanded {commanded.linear:.2f} m/s, {commanded.omega:+.2f} rad/s for "
                    f"{self.th.frozen_seconds:.1f}s but moved {moved:.3f} m / "
                    f"{math.degrees(turned):.1f} deg",
                    f"Robot is at {s.truth}. Pinned, wedged, or commanding a mechanism that "
                    f"cannot act. Worth reporting either way -- a driver would notice.",
                )

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
