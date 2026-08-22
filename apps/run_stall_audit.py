"""
Frozen-robot audit: find robots that stop moving while they still have
somewhere to be, across the same match grid the defense bench runs.

Written because the same defect kept arriving wearing different clothes.
Three separate times a robot has committed to a target it could not
reach and held that commitment for most of a match -- a wedged bumper on
the REEF, a station emptied out from under it, an opposing robot buying
a feeder for the price of announcing it. Each cost 100+ seconds of a 150
second match, and each was found the same laborious way: notice a mean
sag, discover the distribution is bimodal rather than shifted, bisect to
a seed, then trace one robot tick by tick.

The point of this tool is that none of that should be how the *next* one
gets found. A robot standing still with a live target is a symptom with
one obvious signature, so measure the signature directly and let it
surface on its own.

Deliberately a detector and not a fix. The tempting generalisation --
have the navigation layer release any commitment that stops making
progress -- is wrong, and the third instance is why. That robot was
frozen because an opponent's *declared intent* was being counted as a
claimant on an alliance-scoped feeder; a watchdog would have released
the station, re-picked, and left the defender still collecting free
denial. The stall would have gone quiet and the bug would have stayed.
Standing still is also sometimes correct: waiting out a defender at the
only feeder beats touring the field, and that decision is measured (see
`test_collect_keeps_the_only_station_however_long_it_takes`). Only the
tactic knows which it is, so this reports and never intervenes.

Read the output as a lead, not a verdict. A long stall is a place to
point a trace.

Run: `python -m apps.run_stall_audit [--seeds N] [--blue PLAN] [--threshold SECONDS]`
"""
from __future__ import annotations

import argparse
import math
from collections import defaultdict

from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import TrialJob, TrialOutcome
from game_specific.reefscape.sweep_trial import build_match_for_job

from apps.run_defense_bench import BLUE_PLANS, RED_PLANS, build_job

# A robot that moves less than this in a tick is treated as stopped. Well
# under the distance a drivetrain covers in a tick at any real speed, and
# well over the jitter a robot parked against something shows while its
# drive fights the contact solver.
STILL_EPSILON = 0.05

# How long a robot must be stopped before the stall is worth reporting.
# Long enough to clear every legitimate pause -- an intake cycle, a
# scoring action, a queue behind a teammate who is loading -- and short
# enough that a commitment which is never going to resolve still shows up
# with most of the match left to save.
DEFAULT_THRESHOLD = 20.0


def audit_trial(job: TrialJob) -> TrialOutcome:
    """Step the match directly rather than through
    `run_match_to_completion`, because the whole measurement is per-tick
    and that helper has no hook. Picklable by qualified name for the
    process pool, like `run_defense_bench.run_trial`."""
    match, _, _ = build_match_for_job(job)
    robots = list(match.robots)
    last = [(r.pose.x, r.pose.y) for r in robots]
    still = [0.0] * len(robots)
    longest = [0.0] * len(robots)
    frozen_at = [None] * len(robots)

    while not match.ended:
        match.step(job.dt)
        for i, robot in enumerate(robots):
            moved = math.hypot(robot.pose.x - last[i][0], robot.pose.y - last[i][1])
            last[i] = (robot.pose.x, robot.pose.y)
            if moved > STILL_EPSILON:
                still[i] = 0.0
                continue
            still[i] += job.dt
            if still[i] > longest[i]:
                longest[i] = still[i]
                frozen_at[i] = (round(robot.pose.x, 1), round(robot.pose.y, 1))

    stalls = [
        {"robot": job.robots[i].label, "alliance": robots[i].alliance,
         "seconds": round(longest[i], 1), "at": frozen_at[i]}
        for i in range(len(robots))
    ]
    return TrialOutcome(
        index=job.index, seed=job.seed,
        params={**job.params, "stalls": stalls,
                "blue_points": match.scores.get("blue", 0.0)},
        metrics=None,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=24)
    parser.add_argument("--seed-base", type=int, default=5000)
    parser.add_argument("--blue", default="pursue_tuned",
                        help=f"blue plan to audit; one of {sorted(BLUE_PLANS)}")
    parser.add_argument("--per-side", type=int, default=2)
    parser.add_argument("--defenders", type=int, default=1)
    parser.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD,
                        help="report stalls at least this long, in seconds")
    args = parser.parse_args()

    if args.blue not in BLUE_PLANS:
        parser.error(f"unknown blue plan {args.blue!r}; have {sorted(BLUE_PLANS)}")

    jobs = [
        build_job(index, args.seed_base + seed, red, args.blue, args.per_side, args.defenders)
        for index, (red, seed) in enumerate(
            (red, seed) for red in RED_PLANS for seed in range(args.seeds)
        )
    ]
    outcomes = run_all(audit_trial, jobs, parallel=True)

    # Only ours: a defender standing still is doing its job, and a robot
    # on the alliance we are not grading is not what this is looking for.
    findings = [
        (stall["seconds"], out.params["red"], out.seed, stall["robot"],
         stall["at"], out.params["blue_points"])
        for out in outcomes
        for stall in out.params["stalls"]
        if stall["alliance"] == "blue" and stall["seconds"] >= args.threshold
    ]
    findings.sort(reverse=True)

    total = len(outcomes)
    print(f"blue plan {args.blue!r}: {total} matches, "
          f"{len(RED_PLANS)} red plans x {args.seeds} seeds, "
          f"stalls >= {args.threshold:g}s\n")
    if not findings:
        print("no blue robot stood still that long in any match.")
        return

    print(f"{'secs':>6}  {'red plan':<16} {'seed':>6}  {'robot':<6} "
          f"{'frozen at':<16} {'blue pts':>8}")
    for seconds, red, seed, robot, at, points in findings:
        where = f"({at[0]:.0f},{at[1]:.0f})" if at else "-"
        print(f"{seconds:6.1f}  {red:<16} {seed:>6}  {robot:<6} {where:<16} {points:8.0f}")

    by_row: dict[str, int] = defaultdict(int)
    for _, red, seed, _, _, _ in findings:
        by_row[red] += 1
    print(f"\n{len(findings)} stall(s) over {total} matches. By red plan:")
    for red in RED_PLANS:
        print(f"  {red:<16} {by_row[red]:3d}")
    print("\nTrace the worst before reading anything into the means: a row "
          "whose deficit is a handful of frozen matches is not a row that "
          "wants tuning.")


if __name__ == "__main__":
    main()
