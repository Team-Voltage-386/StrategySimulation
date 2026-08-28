"""The overnight harness: N seeded matches, and a morning report.

This is the product. Steps 1 and 2 built a loop that closes and oracles that
fire; this is the thing that runs while nobody is watching and leaves behind a
page worth reading over coffee.

Four decisions, each of which cost something to get right:

**A fresh JVM per match.** Restarting costs ~40 s of gradle and boot per match,
which is real overhead against a 150 s match. It buys isolation: the deliverable
here is a crash or a hang, and a robot that has wedged itself must not quietly
poison every match after it. A campaign of 190 matches where match 12 hung and
matches 13-190 were garbage is worth less than 12 good matches.

**Three statuses, not two.** `pass`, `fail`, and `error` -- where `error` means
the harness could not run the match at all (the sim never booted, the link
dropped). Folding that into either of the others is how a night of nothing gets
reported as a night of clean runs.

**Preflight the oracles.** Before spending eight hours, run step 2's wall-pin
provocation once and confirm `frozen-robot` still fires. An oracle that has
never fired is not known to work, and finding that out at 7am is finding out too
late.

**Keep failures, delete passes.** Each match writes a ~45 MB WPILOG. Across an
overnight run that is ~8.5 GB of logs, nearly all of them from matches where
nothing happened. Passing matches keep one line in a JSONL; failing matches keep
everything, because that is the run somebody is about to debug.
"""
from __future__ import annotations

import json
import shutil
import time
import traceback
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

from bridge import operator as op
from bridge import scenario
from bridge.oracles import ERROR, FaultOracle, Finding, LivenessMonitor
from bridge.sim_process import RobotSim

#: How the robot is driven during a match.
#:
#: `scripted` is step 3's seeded operator: a weighted random walk of button
#: presses that never asks where anything is. It reaches strange states
#: cheaply and it is what the false-positive work was tuned against.
#:
#: `strategy` is step 4's -- sparky-sim's own `StrategyController` reading
#: the live field. It reaches *plausible* states rather than strange ones,
#: which is a different and complementary kind of coverage: a scripted
#: operator will never drive a full scoring cycle by accident, and a
#: strategy will never mash two contradictory buttons on purpose.
#:
#: Both are worth running. Neither subsumes the other.
SCRIPTED = "scripted"
STRATEGY = "strategy"
DRIVERS = (SCRIPTED, STRATEGY)

try:
    from bridge.robot_state import POSE_TRUTH, RobotStateLink
except ImportError as exc:  # pragma: no cover - depends on the install
    # `render_report` and `Campaign` are the product of this module and neither
    # needs NetworkTables. The report especially: it is the thing somebody
    # reads at 7am, and the path that renders a *failure* is the one that has
    # by definition never run during a clean night. It has to be testable
    # without standing up a JVM, or it stays untested until it matters.
    POSE_TRUTH = "/AdvantageKit/RealOutputs/FieldSimulation/RobotPosition"
    RobotStateLink = None  # type: ignore[assignment]
    _ROBOT_STATE_ERROR = exc
else:
    _ROBOT_STATE_ERROR = None

PASS = "pass"
FAIL = "fail"
HARNESS_ERROR = "error"

# Below this many matches, "appeared in every match" is not evidence of
# anything -- in a one-match campaign it is true of every finding, including
# the genuine one-off failure somebody is about to investigate. Calling that
# environmental is worse than saying nothing, because it tells them to ignore
# it.
MIN_MATCHES_FOR_ENVIRONMENTAL = 3

# Where -Pbridge tells the robot to write its replay logs (Robot.java).
BRIDGE_LOG_DIR = Path("logs") / "bridge"


@dataclass
class MatchResult:
    index: int
    seed: int
    status: str
    started_at: str
    wall_seconds: float = 0.0
    boot_seconds: float = 0.0
    match_seconds: float = 0.0
    actions: int = 0
    findings: list[dict] = field(default_factory=list)
    artifacts: dict[str, str] = field(default_factory=dict)
    note: str = ""

    @property
    def kinds(self) -> set[str]:
        return {f["kind"] for f in self.findings}


def cycle_fuel(shoot_at: int):
    """Collect fuel until there is enough of it and a HUB will take it,
    and stand somewhere useful when neither is possible.

    About as simple as a strategy gets, and that is the point -- what is
    being tested is the adapter, not the strategy. `shoot_at` is a
    strategy parameter and not a physical one: the intake really does
    hold 40, and `RobotCharacteristics.piece_capacity` says so, but no
    driver waits for 40 before scoring.

    The `ScoringAvailable` half of the shoot trigger is where REBUILT's
    25-second HUB clock enters. It reads through `world_view` to
    `MapleMatchView.region_blocked`, so a robot with plenty of fuel and
    no live HUB does not go and stand in the goal for twenty seconds --
    it keeps collecting, and shoots when the HUB comes back.

    `collect_fuel`'s trigger is "not literally full" rather than the
    complement of the shoot threshold, so it stays the thing to do
    whenever the higher-priority rule is not firing, for either reason.

    `wait_at_the_goal` is the lowest-priority rule on purpose, one step
    above the fallback, and it is what closes the hole the other two
    leave: a robot that is *both* full and unable to score has no rule
    at all, and drops to `Idle` wherever it stopped. Measured at
    eighteen to twenty seconds of a sixty-second run, at midfield, on
    the wrong side of a wall with two gaps in it. Ranked any higher it
    would answer a dead HUB by parking a half-full robot, which is
    worse: fuel on the floor is always worth collecting.

    Imports inside the function on purpose -- `bridge.harness` is
    imported by the scripted driver and by rules tests that have neither
    pymunk nor common_sim available, and this is the only part of it
    that needs them.
    """
    from bridge import arena
    from bridge import match_view as mv
    from common_sim.control.strategy import Rule, Strategy
    from common_sim.control.tactics import Collect, Idle, Score, Stage
    from common_sim.control.triggers import AllOf, PiecesHeld, ScoringAvailable

    return Strategy(
        name="cycle_fuel",
        rules=[
            Rule(
                name="shoot_fuel",
                trigger=AllOf(triggers=(
                    PiecesHeld(piece_type=arena.PIECE_TYPE, min_count=shoot_at),
                    ScoringAvailable(),
                )),
                tactic=Score(action=mv.SHOOT),
                priority=10,
            ),
            Rule(
                name="collect_fuel",
                trigger=PiecesHeld(piece_type=arena.PIECE_TYPE, max_count=mv.INTAKE_CAPACITY - 1),
                tactic=Collect(piece_type=arena.PIECE_TYPE, mode="nearest"),
                priority=5,
            ),
            Rule(
                name="wait_at_the_goal",
                trigger=PiecesHeld(piece_type=arena.PIECE_TYPE, min_count=1),
                tactic=Stage(),
                priority=1,
            ),
        ],
        fallback=Idle(),
    )


class MatchRunner:
    """Runs one match end to end and returns a MatchResult."""

    def __init__(
        self,
        repo: Path,
        workdir: Path,
        match_seconds: float = 150.0,
        auto_seconds: float = 15.0,
        boot_timeout: float = 300.0,
        gui: bool = False,
        driver: str = SCRIPTED,
        shoot_at: int = 20,
        tick_hz: float = 20.0,
        limits=None,
        opponents: int = 0,
        partners: int = 0,
        defenders: int = 1,
    ):
        self.repo = Path(repo)
        self.workdir = Path(workdir)
        self.match_seconds = match_seconds
        self.auto_seconds = auto_seconds
        self.boot_timeout = boot_timeout
        self.gui = gui
        if driver not in DRIVERS:
            raise ValueError(f"unknown driver {driver!r}; expected one of {', '.join(DRIVERS)}")
        self.driver = driver
        self.shoot_at = shoot_at
        self.tick_hz = tick_hz
        #: How many of the other five robots to put on the field, and how
        #: many of the opponents play defence rather than cycling.
        #:
        #: Zero by default, so an existing campaign runs exactly the
        #: matches it ran before this option existed. A populated field is
        #: a different experiment, not a better version of the same one --
        #: it reaches states a solo run cannot and it also spends a lot of
        #: every match with somebody wedged against somebody else, so a
        #: night of it and a night without are worth comparing rather than
        #: silently swapping.
        #:
        #: Scripted matches ignore this. The extras need a strategy layer
        #: to decide anything, and the scripted driver deliberately has
        #: none -- five more robots mashing random buttons is noise, not
        #: opposition.
        self.opponents = opponents
        self.partners = partners
        self.defenders = defenders
        #: Measured once and reused across matches. The drive model is a
        #: property of the robot code, not of a match, and re-measuring it
        #: every time would spend four seconds a match confirming a
        #: constant. Lazily filled on the first strategy match, or handed
        #: in by a caller that measured it during preflight.
        self.limits = limits

    def run(self, index: int, seed: int) -> MatchResult:
        if RobotStateLink is None:  # pragma: no cover - depends on the install
            raise RuntimeError(
                "running matches needs NetworkTables: pip install -r bridge/requirements.txt"
            ) from _ROBOT_STATE_ERROR
        started = time.monotonic()
        result = MatchResult(
            index=index,
            seed=seed,
            status=HARNESS_ERROR,
            started_at=datetime.now().isoformat(timespec="seconds"),
        )
        scratch = self.workdir / f"match-{index:04d}-seed{seed}"
        scratch.mkdir(parents=True, exist_ok=True)
        console_path = scratch / "console.log"

        gradle_args = ("simulateJava", "-Pbridge", "--no-daemon")
        if self.gui:
            gradle_args += ("-PbridgeGui",)

        sim = RobotSim(self.repo, console_path, gradle_args=gradle_args)
        findings: list[Finding] = []
        fault_oracle = FaultOracle()
        # Snapshot the log directory before the JVM starts, so whatever appears
        # afterwards is unambiguously this match's. Exact, and it does not
        # depend on a console line surviving in a bounded tail buffer -- which
        # matters most for a match that crashed before AdvantageKit renamed
        # its log, i.e. exactly the log worth keeping.
        logs_before = self._wpilogs()

        try:
            sim.start()
            boot_start = time.monotonic()
            link = op.OperatorLink(connect_timeout=self.boot_timeout)
            with link, RobotStateLink() as state:
                state.wait_for_connection(timeout=self.boot_timeout)
                state.wait_for_topic(POSE_TRUTH, timeout=self.boot_timeout)
                result.boot_seconds = time.monotonic() - boot_start

                monitor = LivenessMonitor(state, link)
                with monitor:
                    result.actions = self._play(link, state, seed)
                findings.extend(monitor.findings)
                findings.extend(fault_oracle.scan_alerts(state))

                link.neutral()
                link.disable()
                time.sleep(0.5)
            result.status = PASS  # provisional; the oracles get a vote below
        except Exception as exc:
            result.note = f"{type(exc).__name__}: {exc}"
            (scratch / "traceback.txt").write_text(traceback.format_exc(), encoding="utf-8")
        finally:
            sim.stop()
            result.match_seconds = self.match_seconds
            result.wall_seconds = time.monotonic() - started

        # Oracle 01 reads the console after teardown, so it sees anything the
        # JVM printed on its way down -- which is where a fatal error lands.
        if console_path.is_file():
            findings.extend(fault_oracle.scan_file(console_path))
        result.findings = [asdict(f) for f in findings]

        if result.status != HARNESS_ERROR:
            result.status = FAIL if any(f.severity == ERROR for f in findings) else PASS

        self._collect_artifacts(logs_before, scratch, result)
        return result

    def _wpilogs(self) -> set[Path]:
        directory = self.repo / BRIDGE_LOG_DIR
        return set(directory.glob("*.wpilog")) if directory.is_dir() else set()

    # -- the match itself --------------------------------------------------

    def _play(self, link: op.OperatorLink, state: RobotStateLink, seed: int) -> int:
        """Drive the robot for `match_seconds`, in auto then teleop."""
        if self.driver == STRATEGY:
            return self._play_strategy(link, state, seed)
        return self._play_scripted(link, state, seed)

    # -- the strategy driver -----------------------------------------------

    def _play_strategy(self, link: op.OperatorLink, state: RobotStateLink, seed: int) -> int:
        """Drive with sparky-sim's own strategy layer instead of a script.

        The seed still matters, but it means something different here. A
        scripted match replays the same button sequence; a strategy match
        replays the same *starting conditions* and then reacts to a field
        that no longer unfolds identically. That is the honest thing for
        it to mean -- the physics does not repeat bit-for-bit across
        processes anyway (see the reproduction note in
        `run_bridge_overnight.py`), and the kept WPILOG is the record.

        Calibration happens once, before the first match's autonomous, in
        a short teleop window. Once, because the joystick-to-velocity
        model is a property of the robot code rather than of a match;
        before autonomous, because the probes drive the robot and would
        otherwise be fighting whatever auto is doing.
        """
        # Imported here rather than at module scope on purpose. The
        # strategy path pulls in pymunk and half of common_sim; the
        # scripted path needs none of it, and neither does the campaign
        # report or any of the rules tests. Keeping `bridge.harness`
        # light is what lets those run where the strategy sim's
        # dependencies are not installed.
        from bridge import drive_model as dm
        from bridge import match_view as mv
        from bridge import world_state as ws
        from common_sim.control.behavior import BehaviorContext
        from common_sim.control.strategy import StrategyController
        from common_sim.match.match import Phase

        link.neutral()
        if self.limits is None:
            link.teleop_enable(station="blue1")
            time.sleep(0.5)
            self.limits, checks = dm.calibrate(link, state)
            self._require_model_agrees(checks)
            link.neutral()
            link.disable()
            time.sleep(0.3)

        reader = ws.WorldStateReader(state, alliance="blue")
        robot = mv.MapleRobot(link, self.limits, alliance="blue")
        view = mv.MapleMatchView(robot, reader)
        controller = StrategyController(self.strategy(seed), robot)
        robot.controller = controller

        cast = self._deploy_cast(state, view)

        # Warm every subscription before the match clock starts. A first
        # tick creates a dozen of them, each of which blocks until it
        # carries a value, and left inside the loop that spent the first
        # thirteen seconds of a match in NT handshakes -- with `elapsed`
        # running and the robots doing nothing. It reads as a match that
        # started late rather than as a match that was slow.
        view.sync(0.0, Phase.AUTO)
        if cast is not None:
            cast.link.poses()
            cast.link.speeds()
            cast.link.held()

        link.autonomous_enable(station="blue1")
        link.set_match_time(self.match_seconds)
        started = time.monotonic()
        ticks = self._tick_until(
            view, robot, controller, BehaviorContext, Phase.AUTO,
            started, started + self.auto_seconds, cast,
        )

        link.teleop_enable()
        ticks += self._tick_until(
            view, robot, controller, BehaviorContext, Phase.TELEOP,
            started, started + self.match_seconds, cast,
        )

        # Same buzzer semantics as the scripted path: disable interrupts
        # whatever was running rather than being handed a tidy release.
        # The extras get an explicit stop rather than the same treatment,
        # because nothing disables them: their watchdog would stop them
        # half a second later, and that half second is long enough to
        # matter to whatever reads the final poses.
        if cast is not None:
            cast.stand_down()
        link.disable()
        time.sleep(0.5)
        return ticks

    #: How far the joystick model may disagree with the drive before the
    #: campaign is called off. Same budgets as `run_bridge_strategy.py`.
    SPEED_TOLERANCE_MPS = 0.15
    OMEGA_TOLERANCE_RAD_S = 0.15
    DIRECTION_TOLERANCE_DEG = 8.0

    def _require_model_agrees(self, checks) -> None:
        """Abort rather than run the night against a wrong drive model.

        Raised, not recorded as a finding, and the distinction matters. A
        finding says "this match went wrong"; this says "every match from
        here will be commanding the wrong velocities", which is a harness
        error. Three of those in a row end the campaign -- which is the
        behaviour wanted, because the alternative is eight hours of
        matches that all look plausible and all navigated to the wrong
        places.
        """
        bad = [
            f"{c.label} (speed {c.speed_error:.3f} m/s, dir {c.direction_error_deg:.1f}deg, "
            f"omega {c.omega_error:.3f} rad/s)"
            for c in checks
            if c.speed_error > self.SPEED_TOLERANCE_MPS
            or c.omega_error > self.OMEGA_TOLERANCE_RAD_S
            or c.direction_error_deg > self.DIRECTION_TOLERANCE_DEG
        ]
        if bad:
            raise RuntimeError(
                "the joystick model disagrees with the drive at: " + "; ".join(bad)
                + ". bridge/drive_model.py no longer matches DriveCommands.joystickDrive, so "
                "every strategy match would command the wrong velocity and arrive somewhere "
                "the navigator did not plan for."
            )

    def _deploy_cast(self, state, view):
        """Put the other robots on the field, or return None for a solo match.

        Imported here for the same reason the strategy stack is: the
        scripted driver and the rules tests import this module without
        pymunk, and the extras are a `Robot` subclass.

        A roster the simulation refuses is a hard error and not a match
        that quietly runs solo. The whole reason to populate a field is
        that the states worth reaching need somebody else on it, and a
        night of "contested" matches that were all solo is the same
        failure as an oracle that never fires.
        """
        if not (self.opponents or self.partners):
            return None
        from bridge import opponents as opp

        roster = opp.default_roster(
            opponents=self.opponents, partners=self.partners, defenders=self.defenders
        )
        cast = opp.OpponentCast(opp.OpponentLink(state), roster, self.limits)
        cast.deploy(timeout=30.0)
        # Attached before the first tick, and the ordering is
        # load-bearing: `Defend` picks its mark from the robots the view
        # knows about, so a controller built against an empty field starts
        # by deciding there is nobody to defend against.
        from common_sim.control.strategy import StrategyController

        cast.attach(view, StrategyController)
        return cast

    def _tick_until(self, view, robot, controller, BehaviorContext, phase,
                    started: float, deadline: float, cast=None) -> int:
        dt = 1.0 / self.tick_hz
        ticks = 0
        while time.monotonic() < deadline:
            elapsed = time.monotonic() - started
            view.sync(elapsed, phase)
            context = BehaviorContext(robot=robot, dt=dt, elapsed=elapsed, match=view)
            controller.tick(context)
            if cast is not None:
                # Every extra robot decides against the same instant of the
                # same world our robot just did -- see `OpponentCast.tick`.
                cast.tick(context)
            ticks += 1
            time.sleep(dt)
        return ticks

    def strategy(self, seed: int):
        """The strategy a strategy-driven match plays.

        A method so a caller can subclass or monkeypatch it -- the point
        of the harness is to run *a* strategy many times, not to own which
        one. Overriding this is how a future campaign compares two.
        """
        return cycle_fuel(self.shoot_at)

    # -- the scripted driver -----------------------------------------------

    def _play_scripted(self, link: op.OperatorLink, state: RobotStateLink, seed: int) -> int:
        gen = scenario.ScenarioGenerator(seed)
        link.neutral()

        # Autonomous first, then a transition into teleop *while something is
        # running*. "A phase transition landing mid-sequence" is one of the
        # brief's named categories and it costs nothing to reach: teleopInit
        # cancels the auto command, stops motors, and schedules three more.
        link.autonomous_enable(station="blue1")
        link.set_match_time(self.match_seconds)
        deadline = time.monotonic() + self.auto_seconds
        count = self._play_until(link, state, gen, deadline)

        link.teleop_enable()
        deadline = time.monotonic() + max(0.0, self.match_seconds - self.auto_seconds)
        count += self._play_until(link, state, gen, deadline)

        # Match end arriving mid-sequence: disable without neutralising first,
        # so whatever was running is interrupted by the disable rather than by
        # a tidy release. That is what the buzzer does.
        link.disable()
        time.sleep(0.5)
        return count

    #: Displacement below which a drive action is judged to have gone nowhere.
    #: Well above pose noise, well below what any real move covers.
    STUCK_METRES = 0.08
    #: Only judge actions long enough for the answer to mean something.
    STUCK_MIN_ACTION_SECONDS = 0.5
    #: After this many failed back-offs, stop helping. A robot that will not
    #: reverse is genuinely wedged, and that is the finding worth having.
    #:
    #: Being generous here sharpens the signal instead of blunting it: a robot
    #: held by field geometry comes free given a few tries, and one whose code
    #: has wedged never does. Two was measured too few -- the hub is big enough
    #: to hold a robot through two short reverses, and it produced a false
    #: `frozen-robot` in roughly a third of matches.
    MAX_CONSECUTIVE_BACKOFFS = 4

    def _play_until(self, link, state, gen: scenario.ScenarioGenerator, deadline: float) -> int:
        count = 0
        previous_pose = None
        previous_action = None
        backoffs = 0

        while time.monotonic() < deadline:
            pose = state.truth_pose()
            action, backoffs = self.choose_action(gen, pose, previous_pose, previous_action, backoffs)
            self._apply(link, action)
            time.sleep(min(action.seconds, max(0.0, deadline - time.monotonic())))
            previous_pose, previous_action = pose, action
            count += 1
        return count

    def choose_action(self, gen, pose, previous_pose, previous_action, backoffs):
        """Decide what the operator does next. Pure, so it can be tested.

        This is where the campaign's false-positive rate is actually set. The
        detector thresholds are not the lever: tightening those hides real
        wedges. Making the operator behave like a person -- backing away from
        walls, reversing out of things it has run into, and then *giving up* --
        removes the noise at its source and leaves the real signal intact.
        """
        stuck = (
            pose is not None
            and previous_pose is not None
            and previous_action is not None
            and previous_action.commands_drive
            and previous_action.seconds >= self.STUCK_MIN_ACTION_SECONDS
            and previous_pose.distance_to(pose) < self.STUCK_METRES
        )

        if pose is not None and scenario.near_wall(pose.x, pose.y):
            return gen.recover_toward_centre(pose.x, pose.y), 0

        if stuck and backoffs < self.MAX_CONSECUTIVE_BACKOFFS:
            # Run into a field element -- the hub, most often. A driver
            # reverses; a fuzzer that leans on it produces a true and useless
            # frozen-robot finding every match.
            return gen.back_off(previous_action, backoffs), backoffs + 1

        # Either not stuck, or stuck and out of patience. Out of patience is
        # the important half: a robot that will not reverse twice running is
        # genuinely wedged, and the detector should be allowed to say so.
        return gen.next_action(), backoffs if stuck else 0

    @staticmethod
    def _apply(link: op.OperatorLink, action) -> None:
        """Set the operator's hands to exactly this action, releasing the rest.

        Releasing everything not named is what makes the boundary between two
        actions produce genuine button edges rather than a monotonically
        growing pile of held buttons.
        """
        link.neutral()
        for axis, value in action.axes.items():
            link.set_axis(axis, value)
        for button in action.buttons:
            link.set_button(button, True, joystick=0)
        for button in action.manip_buttons:
            link.set_button(button, True, joystick=1)

    # -- artifacts ---------------------------------------------------------

    def _collect_artifacts(self, logs_before: set[Path], scratch: Path, result: MatchResult) -> None:
        produced = sorted(self._wpilogs() - logs_before)

        if result.status == PASS:
            # ~45 MB each. Across a night that is most of ~8.5 GB, nearly all of
            # it from matches where nothing happened. Delete every one of them,
            # including any the match left behind unclaimed -- an unreaped log
            # is a slow leak that fills the disk somewhere around 3am.
            for path in produced:
                path.unlink(missing_ok=True)
            shutil.rmtree(scratch, ignore_errors=True)
            return

        # Failures keep everything: this is the run somebody is about to debug.
        for number, path in enumerate(produced):
            target = scratch / ("robot.wpilog" if len(produced) == 1 else f"robot-{number}.wpilog")
            try:
                shutil.move(str(path), target)
                result.artifacts[target.stem] = str(target)
            except OSError as exc:
                result.note += f" (could not keep {path.name}: {exc})"

        result.artifacts["console"] = str(scratch / "console.log")
        (scratch / "findings.json").write_text(
            json.dumps(result.findings, indent=2), encoding="utf-8"
        )
        result.artifacts["findings"] = str(scratch / "findings.json")


class Campaign:
    """A whole night of matches, and the report that comes out of it."""

    def __init__(
        self,
        runner: MatchRunner,
        workdir: Path,
        matches: int,
        first_seed: int,
        max_hours: float | None = None,
        abort_after_consecutive_errors: int = 3,
        on_result=None,
    ):
        self.runner = runner
        self.workdir = Path(workdir)
        self.matches = matches
        self.first_seed = first_seed
        self.max_hours = max_hours
        self.abort_after_consecutive_errors = abort_after_consecutive_errors
        self.on_result = on_result
        self.results: list[MatchResult] = []
        self.stopped_early: str | None = None

    @property
    def jsonl_path(self) -> Path:
        return self.workdir / "campaign.jsonl"

    def run(self) -> list[MatchResult]:
        self.workdir.mkdir(parents=True, exist_ok=True)
        started = time.monotonic()
        consecutive_errors = 0

        with self.jsonl_path.open("w", encoding="utf-8") as sink:
            for index in range(self.matches):
                if self.max_hours is not None and (time.monotonic() - started) / 3600 >= self.max_hours:
                    self.stopped_early = f"time budget of {self.max_hours:g} h reached"
                    break
                try:
                    result = self.runner.run(index, self.first_seed + index)
                except KeyboardInterrupt:
                    self.stopped_early = "interrupted"
                    break

                self.results.append(result)
                sink.write(json.dumps(asdict(result)) + "\n")
                sink.flush()  # the report must survive the machine going down
                if self.on_result is not None:
                    self.on_result(result)

                consecutive_errors = consecutive_errors + 1 if result.status == HARNESS_ERROR else 0
                if consecutive_errors >= self.abort_after_consecutive_errors:
                    # Something is systemically wrong -- a broken build, a port
                    # held by a leftover JVM. Burning the rest of the night on
                    # it produces nothing but a longer report of the same error.
                    self.stopped_early = (
                        f"{consecutive_errors} consecutive harness errors; "
                        f"something is wrong with the setup, not with the robot"
                    )
                    break
        return self.results


# ---------------------------------------------------------------------------
# the morning report
# ---------------------------------------------------------------------------


def render_report(campaign: Campaign, preflight: str = "") -> str:
    results = campaign.results
    lines: list[str] = []

    def out(text: str = "") -> None:
        lines.append(text)

    passed = [r for r in results if r.status == PASS]
    failed = [r for r in results if r.status == FAIL]
    errored = [r for r in results if r.status == HARNESS_ERROR]
    wall = sum(r.wall_seconds for r in results)

    out("=" * 72)
    out("  BRIDGE CAMPAIGN REPORT")
    out("=" * 72)
    out(f"  when       : {datetime.now().isoformat(timespec='seconds')}")
    out(f"  matches    : {len(results)} run"
        + (f" of {campaign.matches} planned" if len(results) != campaign.matches else ""))
    out(f"  outcome    : {len(passed)} pass, {len(failed)} FAIL, {len(errored)} harness error")
    if wall:
        overhead = sum(r.boot_seconds for r in results) / wall * 100
        out(f"  wall clock : {wall / 60:.1f} min total, {wall / max(1, len(results)):.0f} s/match "
            f"({overhead:.0f}% of it boot overhead)")
    if preflight:
        out(f"  preflight  : {preflight}")
    if campaign.stopped_early:
        out(f"  STOPPED    : {campaign.stopped_early}")
    out(f"  raw        : {campaign.jsonl_path}")
    out()

    if not results:
        out("  Nothing ran. Do not read this as a clean night.")
        return "\n".join(lines)

    # Findings by kind, with the seeds that produced them. A kind that appears
    # in every single match is a property of the environment, not a regression
    # -- saying so is what stops it drowning the report every morning.
    by_kind: dict[str, list[MatchResult]] = {}
    for result in results:
        for kind in result.kinds:
            by_kind.setdefault(kind, []).append(result)

    out("-" * 72)
    out("  FINDINGS BY KIND")
    out("-" * 72)
    if not by_kind:
        out("  none")
    for kind, hits in sorted(by_kind.items(), key=lambda kv: -len(kv[1])):
        everywhere = (
            len(hits) == len(results) and len(results) >= MIN_MATCHES_FOR_ENVIRONMENTAL
        )
        tag = "  [environmental -- present in every match]" if everywhere else ""
        seeds = ", ".join(str(h.seed) for h in hits[:8])
        more = f" (+{len(hits) - 8} more)" if len(hits) > 8 else ""
        out(f"  {kind:26} {len(hits):4}/{len(results)} matches{tag}")
        out(f"  {'':26} seeds: {seeds}{more}")
    out()

    if failed or errored:
        out("-" * 72)
        out("  WHAT TO LOOK AT")
        out("-" * 72)
        for result in (failed + errored)[:20]:
            label = "FAIL " if result.status == FAIL else "ERROR"
            out(f"  {label} match {result.index:04d}  seed {result.seed}"
                + (f"  -- {result.note}" if result.note else ""))
            for finding in result.findings:
                if finding["severity"] == ERROR:
                    out(f"        {finding['kind']:24} {finding['where']:14} {finding['message']}")
            for name, path in result.artifacts.items():
                out(f"        {name:24} {path}")
            out(f"        reproduce            "
                f"python apps/run_bridge_overnight.py --matches 1 --first-seed {result.seed}")
            out()
    else:
        out("  No failures. Every match ran to completion with no error-level finding.")
        out()

    return "\n".join(lines)
