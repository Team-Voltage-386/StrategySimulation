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

Genericized onto `common_sim.analysis.game_bench` (DRY_RUN_LOG.md, F6):
what's left here is exactly the REEFSCAPE-specific surface -- the match
builder, the strategies directory, the reference robot, and the plan
table. `apps/run_salvage_bench.py` plugs the same shared core into a
second game; `apps/run_stall_audit.py` reuses this file's `RED_PLANS`/
`BLUE_PLANS` to audit either one.

Run: `python -m apps.run_defense_bench [--seeds N] [--per-side N]`
"""
from __future__ import annotations

import argparse
import statistics
from collections import defaultdict

from common_sim.analysis import game_bench
from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import MatchSpec, TrialJob, TrialOutcome
from game_specific.reefscape.robot import build_characteristics
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, SWEEP_DT, build_match_for_job

GAME = game_bench.BenchGame(
    build_match_for_job=build_match_for_job, strategies_dir=STRATEGIES_DIR, dt=SWEEP_DT,
    build_characteristics=build_characteristics,
)

# What a non-defending red robot runs -- red_plan="none" reads as
# "everybody just cycles".
RED_BASELINE = "cycle_coral"

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
    "block/scoring", "block/supply", "block/any",
    "shadow/scoring", "shadow/supply", "shadow/any",
)

# Blue's plans, as the strategy file each blue robot runs. A plan is a
# *lineup*, not one strategy, because a defender's effect depends on what
# the alliance it is defending is trying to do: robot i runs entry i, and
# the last entry repeats if the alliance is wider than the lineup.
#
# The mixed row is not a nicety. The two uniform CORAL rows cannot see
# any behavior that only appears when an alliance runs out of its own
# supply or goes after loose pieces -- which is most of Collect. The
# corner-piece stall (ARCHITECTURE.md, "Every commitment needs an
# expiry") was invisible to this bench for exactly that reason: both
# CORAL rows were bit-identical across the fix. A grid that varies the
# defense while holding one offensive shape fixed measures that shape,
# not the tactic.
BLUE_PLANS: dict[str, tuple[str, ...]] = {
    "cycle_coral": ("cycle_coral",),
    "pursue": ("pursue",),
    "pursue_tuned": ("pursue_tuned",),
    "cycle_coral_evasive": ("cycle_coral_evasive",),
    "algae+coral": ("algae_processor", "cycle_coral"),
}


def build_job(index: int, seed: int, red_plan: str, blue_plan: str, per_side: int, defenders: int) -> TrialJob:
    return game_bench.build_defense_job(
        GAME, index=index, seed=seed, red_plan=red_plan, blue_plan=blue_plan,
        blue_lineup=BLUE_PLANS[blue_plan], red_baseline=RED_BASELINE,
        per_side=per_side, defenders=defenders,
        match=MatchSpec(auto_duration=15.0, teleop_duration=135.0),
    )


def run_trial(job: TrialJob) -> TrialOutcome:
    """Module-level and picklable by qualified name, like
    sweep_trial.run_trial -- ProcessPoolExecutor sends it by reference."""
    return game_bench.run_defense_trial(job, build_match_for_job)


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
    # "red pin" is pin violations by red (see field_config.PinRule),
    # which reads the opposite way to the column beside it: a protection
    # foul means red touched somebody it may not touch at all, a pin
    # foul means red's defense *worked* and it held on past the limit.
    print(f"{'red plan':<14} {'blue plan':<22}{'blue pts':>9} {'blue pcs':>9} "
          f"{'blue cyc':>9} {'red pts':>9} {'red foul':>9} {'red pin':>9}")
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
                f"{_mean(m.protection_fouls_by_alliance.get('red', 0) for m in runs):>9.1f} "
                f"{_mean(m.pin_fouls_by_alliance.get('red', 0) for m in runs):>9.1f}"
            )


def _mean(values) -> float:
    present = [v for v in values if v is not None]
    return statistics.mean(present) if present else float("nan")


if __name__ == "__main__":
    main()
