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
pin, which must produce `frozen-robot`. Eight hours against a detector that
cannot detect is eight hours of rubber stamp, and the campaign refuses to start
rather than find that out at 7am.
"""
from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bridge import harness as hz
from bridge import operator as op
from bridge import oracles
from bridge import robot_state as rs
from bridge.robot_state import POSE_TRUTH, RobotStateLink
from bridge.sim_process import find_robot_repo, RobotSim


def _log(msg: str = "") -> None:
    print(msg, flush=True)


def preflight(repo: Path, workdir: Path, boot_timeout: float, seconds: float = 6.0) -> str:
    """Prove oracle 02 still fires, before committing to the night.

    Same provocation as `run_bridge_oracles.py`: push the robot into the field
    wall while still commanding it forward. Returns a short status string, or
    raises if the detector stayed quiet.
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
        return status
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
    parser.add_argument("--gui", action="store_true",
                        help="keep the Sim GUI up; for watching one seed, useless overnight")
    parser.add_argument("--no-preflight", action="store_true",
                        help="skip the oracle self-test (leaves the campaign unverified)")
    parser.add_argument("--out", type=Path, default=None,
                        help="campaign directory (default: build/bridge/runs/<timestamp>)")
    args = parser.parse_args(argv)
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
    _log(f"  seeds        : {first_seed}..{first_seed + args.matches - 1}")
    _log(f"  output       : {workdir}")
    if args.max_hours:
        _log(f"  time budget  : {args.max_hours:g} h")
    _log()

    preflight_status = "skipped -- oracle 02 UNVERIFIED for this campaign"
    if not args.no_preflight:
        _log("-- preflight: proving oracle 02 still fires " + "-" * 26)
        try:
            preflight_status = preflight(args.repo, workdir, args.boot_timeout)
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
