"""Step 3 of the maple-sim bridge: the overnight harness.

Runs N seeded matches against the real robot code, unattended, and leaves a
morning report. This is the product the first two steps were prerequisites for.

    python apps/run_bridge_overnight.py --matches 40
    python apps/run_bridge_overnight.py --max-hours 8 --matches 500

Reproducing one reported failure:

    python apps/run_bridge_overnight.py --matches 1 --first-seed 4711
    python apps/run_bridge_overnight.py --matches 1 --first-seed 4711 --gui

A seed reproduces the *script* -- the same moves in the same order -- not the
run. Wall recovery is closed-loop and the physics does not repeat bit-for-bit
across processes, so the robot will not land on the same coordinates. The kept
WPILOG is the authoritative record of what happened, and it replays through the
robot code deterministically in AdvantageScope.

Before the campaign starts, the oracles are preflighted: one deliberate wall
pin, which must produce `frozen-robot`, and a deliberate violation of each of
oracle 03's invariants, all of which must fire. Eight hours against a detector
that cannot detect is eight hours of rubber stamp, and the campaign refuses to
start rather than find that out at 7am.
"""
from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bridge import drive_model as dm
from bridge import harness as hz
from bridge import operator as op
from bridge import oracles
from bridge import robot_state as rs
from bridge.robot_state import POSE_TRUTH, RobotStateLink
from bridge.sim_process import find_robot_repo, RobotSim


def _log(msg: str = "") -> None:
    print(msg, flush=True)


def preflight(repo: Path, workdir: Path, boot_timeout: float, seconds: float = 6.0):
    """Prove oracles 02 and 03 still fire, before committing to the night.

    Two provocations, of deliberately unequal strength. Oracle 02 gets the real
    one, the same as `run_bridge_oracles.py`: push the robot into the field wall
    while still commanding it forward. Oracle 03 gets synthetic snapshots
    through `prove_invariants`, because making real robot code publish a NaN
    would mean breaking it on purpose.

    Also measures the drivetrain's maxima on the way out, and hands them back
    for the campaign to reuse. Two things fall out of that and both matter:
    oracle 03's command-range invariant is active from the first sample of the
    first match rather than from wherever that match happens to calibrate, and
    it can be injected here, which is the difference between preflighting five
    invariants and preflighting six.

    Returns `(status, limits)`, or raises if any detector stayed quiet. The
    limits are None if the measurement failed, which is not fatal: it costs the
    campaign four seconds in its first match and nothing else.
    """
    console = workdir / "preflight-console.log"
    sim = RobotSim(repo, console, gradle_args=("simulateJava", "-Pbridge", "--no-daemon"))
    logs_before = set((repo / hz.BRIDGE_LOG_DIR).glob("*.wpilog")) if (repo / hz.BRIDGE_LOG_DIR).is_dir() else set()
    try:
        sim.start()
        link = op.OperatorLink(connect_timeout=boot_timeout)
        with link, RobotStateLink() as state:
            state.wait_for_connection(timeout=boot_timeout)
            state.wait_for_topic(POSE_TRUTH, timeout=boot_timeout)
            link.neutral()
            link.teleop_enable(station="blue1")
            time.sleep(1.0)

            monitor = oracles.LivenessMonitor(state, link)
            amps = None
            with monitor:
                # Positive leftX is -y; the lower wall is at y=0 and the robot
                # starts within about half a metre of it.
                link.set_axis(op.AXIS_LEFT_X, 0.7)
                time.sleep(seconds * 0.6)
                # Read the current straight off the link while the robot is
                # definitely pinned. `robot-pinned` is also what the classifier
                # returns when the current signal is *missing*, so the kind
                # alone cannot tell a working classifier from a blind one --
                # and a blind one silently retires the frozen-robot error path
                # for the whole campaign.
                amps = state.drive_current()
                time.sleep(seconds * 0.4)
                link.set_axis(op.AXIS_LEFT_X, 0.0)

            # After the pin, never before it. Calibration drives, and the
            # provocation above depends on the robot being where the field put
            # it -- a check that rearranges the field has changed the
            # experiment it was meant to validate. Reading the *setpoint*
            # rather than the motion is also why being wedged does not spoil
            # the measurement.
            limits = None
            try:
                limits, _ = dm.calibrate(link, state)
            except Exception:
                pass  # reported through the status line; the campaign copes

            link.neutral()
            link.disable()
            time.sleep(0.5)

        detected = [f for f in monitor.findings if f.kind in oracles.STUCK_KINDS]
        if not detected:
            raise RuntimeError(
                "preflight: a deliberate wall pin produced no stuck finding at all. "
                "Oracle 02 cannot be trusted, so the campaign would be a rubber stamp. "
                f"Console: {console}"
            )
        kind = detected[0].kind
        reading = "no current reading" if amps is None else f"{amps:.0f} A while pinned"
        status = f"ok -- wall pin detected as {kind} at {detected[0].where}, {reading}"
        if amps is None:
            raise RuntimeError(
                "preflight: the wall pin was detected, but no drive-current reading was "
                "available, so pinned-on-geometry cannot be told from drive-not-working. "
                "Every stuck finding this campaign produced would be a warning and the "
                f"frozen-robot error path would never fire. Check {rs.DRIVE_CURRENT[0]} "
                f"against the robot project. Console: {console}"
            )
        if kind != "robot-pinned":
            # It fired and the signal is live, so the campaign is not blind and
            # there is no reason to refuse the night. But a wall pin *is* a
            # pin, so the threshold is suspect and every finding it produces
            # should be read with that in mind.
            status += "  [!] expected robot-pinned; the current threshold looks wrong"

        # Oracle 03, on the shipped thresholds and the limits just measured.
        # No robot needed for this part, which is the whole reason it is only
        # as strong as it is -- see `prove_invariants`.
        from bridge import match_view as mv

        thresholds = oracles.InvariantThresholds(piece_capacity=mv.INTAKE_CAPACITY)
        proof = oracles.prove_invariants(thresholds, limits)
        silent = oracles.unproven_invariants(proof)
        if silent:
            raise RuntimeError(
                f"preflight: oracle 03 invariant(s) stayed silent on a deliberate "
                f"violation: {', '.join(silent)}. The campaign would run all night "
                f"reporting nothing from a detector that cannot detect."
            )
        status += (
            f"; oracle 03 fired on {len(proof)}/"
            f"{len(oracles.InvariantMonitor.KINDS)} injected invariants"
        )
        if limits is None:
            status += "  [!] drive limits unmeasured; command-out-of-range not injected"
        else:
            status += f"; drive measured at {limits.max_speed_mps:.2f} m/s"
        return status, limits
    finally:
        sim.stop()
        for path in (set((repo / hz.BRIDGE_LOG_DIR).glob("*.wpilog")) - logs_before):
            path.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--repo", type=Path, default=None,
                        help="robot project root (default: the sibling checkout, or $SPARKY_ROBOT_REPO)")
    parser.add_argument("--matches", type=int, default=10)
    parser.add_argument("--first-seed", type=int, default=None,
                        help="default: random, and printed so the campaign can be repeated")
    parser.add_argument("--match-seconds", type=float, default=150.0)
    parser.add_argument("--auto-seconds", type=float, default=15.0,
                        help="autonomous before the transition into teleop")
    parser.add_argument("--max-hours", type=float, default=None)
    parser.add_argument("--boot-timeout", type=float, default=300.0)
    parser.add_argument("--driver", choices=hz.DRIVERS, default=hz.SCRIPTED,
                        help="scripted: step 3's seeded button-masher. "
                             "strategy: step 4's live strategy layer.")
    parser.add_argument("--shoot-at", type=int, default=20,
                        help="strategy driver: fuel held before going to score")
    parser.add_argument("--opponents", type=int, default=0,
                        help="strategy driver: opposing robots, also driven by sparky-sim "
                             "(0-3; default 0, which is the solo field this ran on before)")
    parser.add_argument("--partners", type=int, default=0,
                        help="strategy driver: robots on our own alliance (0-2)")
    parser.add_argument("--defenders", type=int, default=1,
                        help="how many opponents play defence rather than cycling")
    parser.add_argument("--gui", action="store_true",
                        help="keep the Sim GUI up; for watching one seed, useless overnight")
    parser.add_argument("--no-preflight", action="store_true",
                        help="skip the oracle self-test (leaves the campaign unverified)")
    parser.add_argument("--out", type=Path, default=None,
                        help="campaign directory (default: build/bridge/runs/<timestamp>)")
    args = parser.parse_args(argv)

    # Argument validation first, and specifically before the robot project is
    # located. Whether two flags contradict each other is a fact about the
    # command line and nothing else -- making it wait on a filesystem search
    # means a contradictory command on a machine with no robot checkout
    # reports "no robot project found", which is true and is not the problem.
    if (args.opponents or args.partners) and args.driver != hz.STRATEGY:
        # Refused rather than ignored. The extras need a strategy layer to
        # decide anything, so a scripted campaign asked for a contested
        # field would run a solo one and report it as contested -- and
        # nothing in the output would say otherwise.
        parser.error(
            f"--opponents/--partners need --driver {hz.STRATEGY}: the extra robots are driven "
            f"by the strategy layer, and the {hz.SCRIPTED} driver deliberately has none"
        )

    # Resolved once, here, so the run prints the project it actually
    # picked and a missing one fails before a JVM is started.
    args.repo = find_robot_repo(args.repo)

    first_seed = args.first_seed if args.first_seed is not None else random.randrange(1, 10_000)
    workdir = args.out or Path("build/bridge/runs") / time.strftime("%Y%m%d-%H%M%S")
    workdir.mkdir(parents=True, exist_ok=True)

    _log("=" * 72)
    _log("  BRIDGE CAMPAIGN")
    _log("=" * 72)
    _log(f"  robot repo   : {args.repo}")
    _log(f"  matches      : {args.matches} x {args.match_seconds:.0f}s "
         f"({args.auto_seconds:.0f}s auto, then teleop)")
    _log(f"  driver       : {args.driver}")
    if args.opponents or args.partners:
        _log(f"  field        : {args.opponents} opponents ({args.defenders} defending), "
             f"{args.partners} partners, all driven by sparky-sim")
    elif args.driver == hz.STRATEGY:
        _log("  field        : solo -- pass --opponents/--partners for a contested one")
    _log(f"  seeds        : {first_seed}..{first_seed + args.matches - 1}")
    _log(f"  output       : {workdir}")
    if args.max_hours:
        _log(f"  time budget  : {args.max_hours:g} h")
    _log()

    preflight_status = "skipped -- oracles 02 and 03 UNVERIFIED for this campaign"
    measured_limits = None
    if not args.no_preflight:
        _log("-- preflight: proving oracles 02 and 03 still fire " + "-" * 19)
        try:
            preflight_status, measured_limits = preflight(
                args.repo, workdir, args.boot_timeout
            )
            _log(f"   {preflight_status}")
        except Exception as exc:
            _log(f"   ABORT: {exc}")
            return 2
    else:
        _log(f"   {preflight_status}")

    runner = hz.MatchRunner(
        repo=args.repo,
        workdir=workdir,
        match_seconds=args.match_seconds,
        auto_seconds=args.auto_seconds,
        boot_timeout=args.boot_timeout,
        gui=args.gui,
        driver=args.driver,
        shoot_at=args.shoot_at,
        opponents=args.opponents,
        partners=args.partners,
        defenders=args.defenders,
        # Measured during preflight, on a JVM that is already dead by now. The
        # drive model is a property of the robot code and not of a match, so
        # measuring it once is right -- and doing it before the first match
        # rather than during it is what keeps oracle 03's command-range
        # invariant active from that match's first sample.
        limits=measured_limits,
    )

    # The count means different things per driver -- discrete button
    # actions for the scripted operator, control-loop ticks for the
    # strategy layer, which differ by more than an order of magnitude.
    # Printing both as "actions" invites a comparison that is meaningless.
    unit = "ticks  " if args.driver == hz.STRATEGY else "actions"

    def announce(result: hz.MatchResult) -> None:
        mark = {hz.PASS: "  ok  ", hz.FAIL: " FAIL ", hz.HARNESS_ERROR: "ERROR "}[result.status]
        kinds = ", ".join(sorted(result.kinds)) or "-"
        _log(f"   [{mark}] match {result.index:04d} seed {result.seed:<6} "
             f"{result.wall_seconds:5.0f}s wall  {result.actions:5d} {unit}  {kinds}")

    campaign = hz.Campaign(
        runner=runner,
        workdir=workdir,
        matches=args.matches,
        first_seed=first_seed,
        max_hours=args.max_hours,
        on_result=announce,
    )

    _log()
    _log("-- running " + "-" * 60)
    try:
        campaign.run()
    except KeyboardInterrupt:
        campaign.stopped_early = "interrupted"

    report = hz.render_report(campaign, preflight_status)
    report_path = workdir / "report.txt"
    report_path.write_text(report, encoding="utf-8")

    _log()
    _log(report)
    _log(f"  report kept at {report_path}")

    if not campaign.results:
        return 1
    return 1 if any(r.status != hz.PASS for r in campaign.results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
