"""Timestep fidelity study.

The question is NOT "does a coarser dt reproduce dt=1/60 exactly" -- it
cannot, and does not need to. A search evaluator only needs to *rank*
configurations the same way. So this measures rank preservation directly,
against the noise floor the seeds themselves impose.

Seeds are `base_seed + index` in expand_jobs and independent of dt, so
every configuration is compared to itself on the same seed: paired
samples, not two independent draws.

Run:  python dt_study.py
"""
from __future__ import annotations

import itertools
import math
import statistics
import time

from common_sim.analysis.results import to_dataframe
from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import (
    MatchSpec, RobotSpec, SweepVariable, characteristics_to_spec, expand_jobs,
)
from common_sim.analysis.variability import VariabilityModel
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, run_trial
from apps.run_strategy_sweep import build_characteristics

DTS = [1 / 60, 1 / 40, 1 / 30, 1 / 20]
REPS = 24
STRATEGIES = ("cycle_coral", "cycle_coral_evasive", "algae_processor", "auto_then_cycle")
SPEEDS = (130.0, 150.0, 170.0)

VARIABILITY = VariabilityModel(
    enabled=True, intake_time_pct=0.10, deposit_time_pct=0.10, max_speed_pct=0.08,
    max_accel_pct=0.08, start_pose_xy_in=2.0, start_pose_heading_deg=3.0, piece_scatter_in=3.0,
)


def build(dt):
    char = characteristics_to_spec(build_characteristics())
    robots = [
        RobotSpec(label="PRIMARY", alliance="blue", roster_index=-1,
                  characteristics=dict(char), strategy="cycle_coral"),
        RobotSpec(label="PARTNER", alliance="blue", roster_index=0,
                  characteristics=dict(char), strategy="cycle_coral"),
        RobotSpec(label="OPPONENT", alliance="red", roster_index=0,
                  characteristics=dict(char), strategy="cycle_coral"),
    ]
    variables = [
        SweepVariable(target="PRIMARY", path="strategy", values=STRATEGIES),
        SweepVariable(target="PRIMARY", path="max_speed", values=SPEEDS),
    ]
    return expand_jobs(robots, MatchSpec(auto_duration=15.0, teleop_duration=135.0),
                       VARIABILITY, variables, repetitions=REPS,
                       strategies_dir=STRATEGIES_DIR, dt=dt)


def spearman(a, b):
    """Rank correlation without a scipy dependency."""
    def rank(xs):
        order = sorted(range(len(xs)), key=lambda i: xs[i])
        r = [0.0] * len(xs)
        i = 0
        while i < len(order):
            j = i
            while j + 1 < len(order) and xs[order[j + 1]] == xs[order[i]]:
                j += 1
            avg = (i + j) / 2.0 + 1.0
            for k in range(i, j + 1):
                r[order[k]] = avg
            i = j + 1
        return r

    ra, rb = rank(a), rank(b)
    ma, mb = statistics.mean(ra), statistics.mean(rb)
    num = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    den = math.sqrt(sum((x - ma) ** 2 for x in ra) * sum((y - mb) ** 2 for y in rb))
    return num / den if den else float("nan")


def main():
    results = {}
    for dt in DTS:
        jobs = build(dt)
        t0 = time.perf_counter()
        outcomes = run_all(run_trial, jobs, parallel=True)
        wall = time.perf_counter() - t0
        errs = [o for o in outcomes if o.error is not None]
        if errs:
            print(f"!! {len(errs)} failed at dt=1/{1/dt:.0f}; first:\n{errs[0].error[:800]}")
        ok = [o for o in outcomes if o.error is None]
        # (strategy, speed) -> {seed: blue score}. expand_jobs assigns
        # seed = base_seed + index over the same config x repetition order
        # at every dt, so the same seed is the same config on the same
        # draw: keying by it pairs the arms exactly.
        table = {}
        for o in ok:
            key = (o.params["PRIMARY.strategy"], o.params["PRIMARY.max_speed"])
            table.setdefault(key, {})[o.seed] = o.metrics.final_scores.get("blue", 0.0)
        results[dt] = (table, wall, len(ok))
        print(f"dt=1/{1/dt:>3.0f}  {len(ok):>4} matches  {wall:7.1f}s wall  "
              f"({wall / max(1, len(ok)):.2f}s/match effective)")

    base_table, base_wall, _ = results[DTS[0]]
    configs = sorted(base_table)

    def means(table):
        return [statistics.mean(table[c].values()) for c in configs]

    base_means = means(base_table)
    # Noise floor: how much one config's score moves between seeds at 1/60.
    within = [statistics.pstdev(base_table[c].values()) for c in configs]
    print(f"\nnoise floor at dt=1/60: seed-to-seed sd within a config "
          f"= {statistics.mean(within):.1f} pts (mean over {len(configs)} configs)")
    print(f"spread across configs:  sd of config means = {statistics.pstdev(base_means):.1f} pts")

    print(f"\n{'dt':>6} {'speedup':>8} {'spearman':>9} {'pair agree':>11} "
          f"{'mean |Δ| vs 1/60':>17} {'bias':>7}")
    for dt in DTS:
        table, wall, _ = results[dt]
        m = means(table)
        rho = spearman(base_means, m)

        # Pairwise agreement, restricted to pairs that dt=1/60 actually
        # separates by more than the noise floor -- agreeing about a pair
        # it cannot distinguish is not evidence of anything.
        agree = total = 0
        for i, j in itertools.combinations(range(len(configs)), 2):
            gap = base_means[i] - base_means[j]
            if abs(gap) < statistics.mean(within) / math.sqrt(REPS) * 2:
                continue
            total += 1
            if (m[i] - m[j]) * gap > 0:
                agree += 1
        agree_pct = agree / total * 100 if total else float("nan")

        # Paired per-seed absolute difference.
        diffs = [abs(table[c][s] - base_table[c][s])
                 for c in configs for s in base_table[c] if s in table[c]]
        bias = statistics.mean([table[c][s] - base_table[c][s]
                                for c in configs for s in base_table[c] if s in table[c]])
        print(f"  1/{1/dt:<3.0f} {base_wall / wall:7.2f}x {rho:9.3f} "
              f"{agree_pct:9.0f}%* {statistics.mean(diffs):15.1f} {bias:+7.1f}")
    print(f"\n* over the {total} of {len(configs) * (len(configs) - 1) // 2} config pairs "
          f"that dt=1/60 separates beyond its own standard error")


if __name__ == "__main__":
    main()
