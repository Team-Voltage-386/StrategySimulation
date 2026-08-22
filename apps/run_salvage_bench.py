"""
Head-to-head benchmark on SALVAGE, the dry-run game (see
DRY_RUN_LOG.md).

Two reasons this exists rather than a SALVAGE arm being added to
`apps/run_defense_bench.py`. The first is that a paired design is what
this bench is *for*: REEFSCAPE's grid resolves about +/-2-3 points per
row at 80 seeds while real policy differences there are 1-2 points, so
it stopped being usable as a gradient. Running both alliances on the
same seeds and reporting the per-seed difference costs nothing and
resolves an order of magnitude better.

The second reason is a finding, not a preference: `run_defense_bench`
imports `game_specific.reefscape.sweep_trial` at module scope, and
`run_stall_audit` imports `run_defense_bench` for its job builder, so
neither of the two most valuable measurement tools in the repo can be
pointed at a second game. This file is the duplicate that proves it --
compare the two and the shared surface is obvious: a match builder, a
strategies directory, a plan table, and `run_all`. See DRY_RUN_LOG.md,
F6. That extraction is worth doing and was deliberately not done at the
same time as the behavior fixes it would be measuring.

Run: `python -m apps.run_salvage_bench [--seeds N] [--per-side N]`
"""
from __future__ import annotations

import argparse
import math
import statistics
from collections import defaultdict

from common_sim.analysis.metrics import extract_metrics
from common_sim.analysis.monte_carlo import run_match_to_completion
from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import (
    MatchSpec, RobotSpec, TrialJob, TrialOutcome, characteristics_to_spec,
)
from common_sim.analysis.variability import VariabilityModel
from common_sim.control import strategy_io
from game_specific.salvage.robot import build_characteristics
from game_specific.salvage.sweep_trial import STRATEGIES_DIR, SWEEP_DT, build_match_for_job

VARIABILITY = VariabilityModel(
    enabled=True, intake_time_pct=0.10, deposit_time_pct=0.10,
    max_speed_pct=0.08, max_accel_pct=0.08,
    start_pose_xy_in=4.0, start_pose_heading_deg=5.0, piece_scatter_in=3.0,
)

# What red does. "none" is the control -- red simply cycles -- and every
# other row gives one red robot over to full-time Defend, with its
# `mode`/`deny` overridden the way run_defense_bench does it.
RED_PLANS = ("none", "block/scoring", "block/supply", "shadow/any")

# Blue's candidates. All four score the same game with the same robot;
# what differs is how the choice of what to do next is made.
BLUE_PLANS = ("cycle_crates", "rush_reactor", "pursue", "pursue_tuned", "pursue_scarce")

# Read every other blue plan against this one. `cycle_crates` is the
# static policy: collect crates, score crates, never reconsider. On
# REEFSCAPE its analogue is very hard to beat, because "cycle CORAL" is
# very nearly the optimal constant policy and an arbiter can at best tie
# it. SALVAGE is built so that a constant policy is *wrong* -- the
# valuable target moves at the end of AUTO, the depot runs dry, and the
# BEACON fills -- so this is the comparison REEFSCAPE could not make.
BASELINE_BLUE = "cycle_crates"


def _load(name: str) -> dict:
    return strategy_io.to_dict(strategy_io.load_strategy(STRATEGIES_DIR / f"{name}.json"))


def _full_time_defender(plan_name: str) -> dict:
    mode, _, deny = plan_name.partition("/")
    plan = _load("full_defense")
    for rule in plan["rules"]:
        if rule["tactic"]["type"] == "Defend":
            rule["tactic"]["mode"] = mode
            if deny:
                rule["tactic"]["deny"] = deny
    return plan


def build_job(index: int, seed: int, red_plan: str, blue_plan: str, per_side: int, defenders: int) -> TrialJob:
    characteristics = characteristics_to_spec(build_characteristics())
    robots = [
        RobotSpec(label=f"B{i}", alliance="blue", roster_index=i,
                  characteristics=characteristics, strategy=_load(blue_plan))
        for i in range(per_side)
    ]
    robots += [
        RobotSpec(label=f"R{i}", alliance="red", roster_index=i, characteristics=characteristics,
                  strategy=_full_time_defender(red_plan) if red_plan != "none" and i < defenders
                  else _load(BASELINE_BLUE))
        for i in range(per_side)
    ]
    return TrialJob(
        index=index, seed=seed, params={"red": red_plan, "blue": blue_plan}, robots=tuple(robots),
        match=MatchSpec(auto_duration=15.0, teleop_duration=135.0),
        variability=VARIABILITY, strategies_dir=str(STRATEGIES_DIR), dt=SWEEP_DT,
    )


def run_trial(job: TrialJob) -> TrialOutcome:
    """Module-level and picklable by qualified name -- a
    ProcessPoolExecutor sends it by reference."""
    match, _, _ = build_match_for_job(job)
    run_match_to_completion(match, dt=job.dt)
    return TrialOutcome(index=job.index, seed=job.seed, params=job.params, metrics=extract_metrics(match))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=24)
    parser.add_argument("--seed-base", type=int, default=7000)
    parser.add_argument("--per-side", type=int, default=2, help="robots per alliance")
    parser.add_argument("--defenders", type=int, default=1, help="how many red robots defend full time")
    args = parser.parse_args()

    jobs = [
        build_job(index, args.seed_base + seed, red, blue, args.per_side, args.defenders)
        for index, (red, blue, seed) in enumerate(
            (red, blue, seed)
            for red in RED_PLANS for blue in BLUE_PLANS for seed in range(args.seeds)
        )
    ]
    outcomes = run_all(run_trial, jobs, parallel=True)

    failed = [o for o in outcomes if o.error is not None]
    if failed:
        print(f"{len(failed)} trial(s) failed, e.g.:\n{failed[0].error}")

    # (red, blue) -> seed -> blue points. Keyed by seed, not appended, so
    # the comparison below can be paired: the same seed is the same
    # field, the same perturbations and the same red behaviour, and
    # differencing within it removes all of that variance instead of
    # averaging over it.
    points: dict[tuple[str, str], dict[int, float]] = defaultdict(dict)
    for outcome in outcomes:
        if outcome.metrics is not None:
            points[(outcome.params["red"], outcome.params["blue"])][outcome.seed] = \
                outcome.metrics.final_scores.get("blue", 0.0)

    print(f"\nSALVAGE {args.per_side}v{args.per_side}, {args.defenders} full-time red defender(s), "
          f"{args.seeds} seeds, {len(outcomes)} matches")
    print(f"blue points, paired against {BASELINE_BLUE!r} on the same seeds\n")
    print(f"{'red plan':<16}{'blue plan':<16}{'mean':>8}{'sd':>7}{'min':>6}"
          f"{'vs baseline':>26}  W/L")

    for red in RED_PLANS:
        baseline = points.get((red, BASELINE_BLUE), {})
        for blue in BLUE_PLANS:
            runs = points.get((red, blue))
            if not runs:
                continue
            values = [runs[s] for s in sorted(runs)]
            line = (f"{red:<16}{blue:<16}{statistics.mean(values):8.1f}"
                    f"{statistics.stdev(values) if len(values) > 1 else 0.0:7.1f}{min(values):6.0f}")
            if blue == BASELINE_BLUE:
                print(f"{line}{'--':>26}")
                continue
            print(f"{line}{_paired(baseline, runs):>26}")
        print()

    print("Read the paired column, not the means. Two arms differing by a")
    print("point or two on unpaired means over a couple of dozen seeds are")
    print("not distinguishable; the same two on paired seeds usually are,")
    print("because the seed *is* most of the variance. The REEFSCAPE grid")
    print("was tuned against for months before that was noticed.")


def _paired(baseline: dict[int, float], arm: dict[int, float]) -> str:
    seeds = sorted(set(baseline) & set(arm))
    if len(seeds) < 2:
        return "n/a"
    diffs = [arm[s] - baseline[s] for s in seeds]
    mean = statistics.mean(diffs)
    sem = statistics.stdev(diffs) / math.sqrt(len(diffs))
    wins = sum(1 for d in diffs if d > 0)
    losses = sum(1 for d in diffs if d < 0)
    sigma = mean / sem if sem > 0 else 0.0
    return f"{mean:+7.2f} +/-{sem:5.2f} ({sigma:+5.1f}) {wins:3d}/{losses}"


if __name__ == "__main__":
    main()
