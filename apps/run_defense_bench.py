"""
Head-to-head defense benchmark: how much does a dedicated defender cost
the alliance it is defending, and how much of that does the defended
alliance win back?

A strategy sweep (`apps/run_strategy_sweep.py`) varies one alliance and
reads its own score. That cannot measure defense at all: a defender
scores nothing by design, so a sweep grades it as the worst strategy in
the file no matter how well it plays. Defense is only visible as
somebody *else's* rate falling, which needs both alliances configured at
once and the metrics read per alliance (see
`analysis/metrics.py`'s `*_by_alliance`).

So this runs a grid of (red plan) x (blue plan) matches and prints blue's
production under each. Blue is always the alliance being defended; red
either cycles normally ("none", the control -- what blue does when left
alone) or gives one robot over to full-time Defend.

Determinism note: `VariabilityModel()`'s defaults perturb *nothing*, so
running N seeds with it produces N bit-identical matches and an N-of-1
sample wearing a mean's clothing. `VARIABILITY` below is deliberately
non-trivial for that reason.

Run: `python -m apps.run_defense_bench [--seeds N] [--per-side N]`
"""
from __future__ import annotations

import argparse
import statistics
from collections import defaultdict

from common_sim.analysis.metrics import extract_metrics
from common_sim.analysis.monte_carlo import run_match_to_completion
from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import (
    MatchSpec,
    RobotSpec,
    TrialJob,
    TrialOutcome,
    characteristics_to_spec,
)
from common_sim.analysis.variability import VariabilityModel
from common_sim.control import strategy_io
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, SWEEP_DT, build_match_for_job
from apps.run_strategy_sweep import build_characteristics

VARIABILITY = VariabilityModel(
    enabled=True, intake_time_pct=0.10, deposit_time_pct=0.10,
    max_speed_pct=0.08, max_accel_pct=0.08,
    start_pose_xy_in=4.0, start_pose_heading_deg=5.0, piece_scatter_in=3.0,
)

# Red's plans. "none" is the control -- no defender at all -- and every
# other row is read against it, since the only meaningful statement about
# a defender is how far it moved blue off what blue does unopposed.
#
# A plan is "<mode>" or "<mode>/<deny>", the two axes of Defend that this
# bench compares: where the defender stands relative to its mark, and
# which half of the opponent's cycle -- scoring, supply, or whichever it
# is currently headed for -- it is willing to attack.
RED_PLANS = (
    "none",
    "block", "block/supply", "block/any",
    "shadow", "shadow/supply", "shadow/any",
)

# Blue's plans, by strategy file. Both cycle CORAL; the evasive one adds
# a BeingDefended rule on top (see the strategies directory).
BLUE_PLANS = ("cycle_coral", "cycle_coral_evasive")


def _load(name: str) -> dict:
    """Strategies travel to a worker as plain dicts, not live Strategy
    objects -- see sweep_trial._resolve_strategy."""
    return strategy_io.to_dict(strategy_io.load_strategy(STRATEGIES_DIR / f"{name}.json"))


def _full_time_defender(plan_name: str) -> dict:
    """The `full_defense` strategy file with its Defend `mode` (and
    `deny`, if the plan names one) overridden -- a robot that does
    nothing but Defend, from the first tick, as a bound on how much
    denial is even available.

    Loaded from the file rather than built here so the plan these numbers
    are measured against is the same object you can open in the STRATEGY
    tab and sweep by name. Only the compared axes are overridden; edit
    the file to change anything else and both the bench and the GUI see
    it."""
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
                  else _load("cycle_coral"))
        for i in range(per_side)
    ]
    return TrialJob(
        index=index, seed=seed, params={"red": red_plan, "blue": blue_plan}, robots=tuple(robots),
        match=MatchSpec(auto_duration=15.0, teleop_duration=135.0),
        variability=VARIABILITY, strategies_dir=str(STRATEGIES_DIR), dt=SWEEP_DT,
    )


def run_trial(job: TrialJob) -> TrialOutcome:
    """Module-level and picklable by qualified name, like
    sweep_trial.run_trial -- ProcessPoolExecutor sends it by reference."""
    match, _, _ = build_match_for_job(job)
    run_match_to_completion(match, dt=job.dt)
    return TrialOutcome(index=job.index, seed=job.seed, params=job.params, metrics=extract_metrics(match))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, default=8)
    parser.add_argument("--per-side", type=int, default=2, help="robots per alliance")
    parser.add_argument("--defenders", type=int, default=1, help="how many red robots defend full time")
    args = parser.parse_args()

    jobs = [
        build_job(index, 1000 + seed, red, blue, args.per_side, args.defenders)
        for index, (red, blue, seed) in enumerate(
            (red, blue, seed)
            for red in RED_PLANS for blue in BLUE_PLANS for seed in range(args.seeds)
        )
    ]
    outcomes = run_all(run_trial, jobs, parallel=True)

    failed = [o for o in outcomes if o.error is not None]
    if failed:
        print(f"{len(failed)} trial(s) failed, e.g.:\n{failed[0].error}")

    grouped: dict[tuple[str, str], list] = defaultdict(list)
    for outcome in outcomes:
        if outcome.metrics is not None:
            grouped[(outcome.params["red"], outcome.params["blue"])].append(outcome.metrics)

    print(f"{args.per_side}v{args.per_side}, {args.defenders} full-time red defender(s), "
          f"{args.seeds} seeds, {len(outcomes)} matches")
    # "red foul" is protected-zone contact by red, counted per call (see
    # field_config.ProtectedZone). Without it a defender that simply
    # parks on top of a protected scorer looks like it is denying, when
    # what it is actually doing is donating.
    print(f"{'red plan':<14} {'blue plan':<22}{'blue pts':>9} {'blue pcs':>9} "
          f"{'blue cyc':>9} {'red pts':>9} {'red foul':>9}")
    for red in RED_PLANS:
        for blue in BLUE_PLANS:
            runs = grouped.get((red, blue))
            if not runs:
                continue
            print(
                f"{red:<14} {blue:<22} "
                f"{_mean(m.final_scores.get('blue', 0.0) for m in runs):>9.1f} "
                f"{_mean(m.pieces_scored_by_alliance.get('blue', 0) for m in runs):>9.1f} "
                f"{_mean(m.mean_cycle_time_by_alliance.get('blue') for m in runs):>9.2f} "
                f"{_mean(m.final_scores.get('red', 0.0) for m in runs):>9.1f} "
                f"{_mean(m.protection_fouls_by_alliance.get('red', 0) for m in runs):>9.1f}"
            )


def _mean(values) -> float:
    present = [v for v in values if v is not None]
    return statistics.mean(present) if present else float("nan")


if __name__ == "__main__":
    main()
