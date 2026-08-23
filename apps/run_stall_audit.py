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

Ported off REEFSCAPE onto `common_sim.analysis.game_bench` (DRY_RUN_LOG.md,
F6): this used to import `game_specific.reefscape.sweep_trial` at module
scope, so it could only ever audit one game. `--game` now picks between
REEFSCAPE and SALVAGE; each game's plan table is imported lazily, inside
the matching `_*_config` function below, so choosing one never pulls in
the other game's strategy files or match builder.

Run: `python -m apps.run_stall_audit [--game reefscape|salvage] [--seeds N] [--blue PLAN] [--threshold SECONDS]`
"""
from __future__ import annotations

import argparse
from collections import defaultdict

from common_sim.analysis import game_bench
from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import MatchSpec, TrialJob, TrialOutcome


def _reefscape_config():
    """Reuses `run_defense_bench`'s own `RED_PLANS`/`BLUE_PLANS` rather
    than re-declaring them here, so there is still exactly one place
    that names REEFSCAPE's defense-bench plan table. Imported lazily --
    this is the only place in the module `game_specific.reefscape` gets
    named, and only when `--game reefscape` (the default) is chosen."""
    from apps.run_defense_bench import BLUE_PLANS, GAME, RED_BASELINE, RED_PLANS
    return GAME, RED_PLANS, BLUE_PLANS, RED_BASELINE


def _salvage_config():
    from apps.run_salvage_bench import BASELINE_BLUE, BLUE_PLANS, GAME, RED_PLANS
    blue_lineups = {name: (name,) for name in BLUE_PLANS}
    return GAME, RED_PLANS, blue_lineups, BASELINE_BLUE


GAME_CONFIGS = {"reefscape": _reefscape_config, "salvage": _salvage_config}


def _audit_trial_reefscape(job: TrialJob) -> TrialOutcome:
    """One module-level wrapper per game, not one generic function
    taking a builder -- ProcessPoolExecutor needs `fn` picklable by
    qualified name, which means a real module-level function, and the
    game-specific import has to happen somewhere; here, lazily, inside
    the one wrapper a worker will actually call."""
    from game_specific.reefscape.sweep_trial import build_match_for_job
    return game_bench.run_stall_trial(job, build_match_for_job)


def _audit_trial_salvage(job: TrialJob) -> TrialOutcome:
    from game_specific.salvage.sweep_trial import build_match_for_job
    return game_bench.run_stall_trial(job, build_match_for_job)


AUDIT_TRIAL = {"reefscape": _audit_trial_reefscape, "salvage": _audit_trial_salvage}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--game", choices=sorted(GAME_CONFIGS), default="reefscape")
    parser.add_argument("--seeds", type=int, default=24)
    parser.add_argument("--seed-base", type=int, default=5000)
    parser.add_argument("--blue", default="pursue_tuned",
                        help="blue plan to audit; depends on --game")
    parser.add_argument("--per-side", type=int, default=2)
    parser.add_argument("--defenders", type=int, default=1)
    parser.add_argument("--threshold", type=float, default=game_bench.DEFAULT_STALL_THRESHOLD,
                        help="report stalls at least this long, in seconds")
    args = parser.parse_args()

    game, red_plans, blue_lineups, red_baseline = GAME_CONFIGS[args.game]()
    if args.blue not in blue_lineups:
        parser.error(f"unknown blue plan {args.blue!r} for --game {args.game}; have {sorted(blue_lineups)}")

    jobs = [
        game_bench.build_defense_job(
            game, index=index, seed=args.seed_base + seed, red_plan=red,
            blue_plan=args.blue, blue_lineup=blue_lineups[args.blue], red_baseline=red_baseline,
            per_side=args.per_side, defenders=args.defenders,
            match=MatchSpec(auto_duration=15.0, teleop_duration=135.0),
        )
        for index, (red, seed) in enumerate((red, seed) for red in red_plans for seed in range(args.seeds))
    ]
    outcomes = run_all(AUDIT_TRIAL[args.game], jobs, parallel=True)

    # Only ours: a defender standing still is doing its job, and a robot
    # on the alliance we are not grading is not what this is looking for.
    findings = [
        (stall["seconds"], out.params["red"], out.seed, stall["robot"],
         stall["at"], out.params["blue_points"], stall["commanded"],
         stall["spin"], stall["duration"] - stall["ended"])
        for out in outcomes
        for stall in out.params["stalls"]
        if stall["alliance"] == "blue" and stall["seconds"] >= args.threshold
    ]
    findings.sort(reverse=True)

    total = len(outcomes)
    print(f"{args.game} blue plan {args.blue!r}: {total} matches, "
          f"{len(red_plans)} red plans x {args.seeds} seeds, "
          f"stalls >= {args.threshold:g}s\n")
    if not findings:
        print("no blue robot stood still that long in any match.")
        return

    print(f"{'secs':>6}  {'red plan':<16} {'seed':>6}  {'robot':<6} "
          f"{'frozen at':<16} {'asking':>7} {'spin':>6} {'left':>6}  {'blue pts':>8}")
    for seconds, red, seed, robot, at, points, asked, spin, left in findings:
        where = f"({at[0]:.0f},{at[1]:.0f})" if at else "-"
        print(f"{seconds:6.1f}  {red:<16} {seed:>6}  {robot:<6} {where:<16} "
              f"{asked:7.1f} {spin:6.2f} {left:6.1f}  {points:8.0f}")

    by_row: dict[str, int] = defaultdict(int)
    for _, red, seed, _, _, _, _, _, _ in findings:
        by_row[red] += 1
    print(f"\n{len(findings)} stall(s) over {total} matches. By red plan:")
    for red in red_plans:
        print(f"  {red:<16} {by_row[red]:3d}")
    print("\n`asking` is mean commanded speed, in/s, and `spin` the mean")
    print("commanded rotation, rad/s, over the frozen window. Together they")
    print("split the two things a stall can be. Both near zero is a robot")
    print("that chose to wait -- sometimes correct, and not this tool's")
    print("business. Either one large is a robot being held: it is asking")
    print("for the drivetrain and getting nothing, which is a physical fact")
    print("rather than a judgement about strategy, and is where the last two")
    print("bugs actually lived. Sort your attention by these columns, not by")
    print("which tactic was running -- the corner pin looked like a `Score`")
    print("bug for an afternoon and was two bugs in the match rules.")
    print("")
    print("Read `spin` before trusting a low `asking`. The column exists")
    print("because it was missing: a robot parked exactly on its scoring")
    print("pose, unable to *rotate* into the heading its deposit needed,")
    print("commands no translation at all and so reported asking 0.0 while")
    print("being held as hard as a robot can be. That is how the SALVAGE")
    print("dry run lost 110 seconds of a match (DRY_RUN_LOG.md), and it")
    print("means every earlier `asking 0.0` verdict on this grid was drawn")
    print("with a blind spot in it.")
    print("\nTrace the worst before reading anything into the means: a row "
          "whose deficit is a handful of frozen matches is not a row that "
          "wants tuning.")


if __name__ == "__main__":
    main()
