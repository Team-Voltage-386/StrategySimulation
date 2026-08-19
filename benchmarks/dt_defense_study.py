"""Timestep fidelity study, defensive arm.

The first study gridded strategy x speed with no defender on the field.
Contact is where a coarser command rate is most likely to matter: a
defender that re-decides at 20Hz may lose or win shoving matches
differently, and pin/protection fouls are charged on per-tick contact
tests. So this grids blue strategy x red DEFENSE level and reads blue's
production -- a defender scores nothing itself, so its effect is only
visible as somebody else's rate falling.

Raw per-match rows are persisted to JSON so follow-up questions do not
need a re-run.
"""
from __future__ import annotations

import itertools
import json
import math
import statistics
import time
from pathlib import Path

from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import (
    MatchSpec, RobotSpec, SweepVariable, characteristics_to_spec, expand_jobs,
)
from common_sim.analysis.variability import VariabilityModel
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, run_trial
from apps.run_strategy_sweep import build_characteristics

OUT = Path(__file__).with_name("dt_defense_rows.json")

DTS = [1 / 60, 1 / 30, 1 / 20]
REPS = 24
BLUE = ("cycle_coral", "cycle_coral_evasive", "algae_processor")
RED = ("cycle_coral", "endgame_defense", "full_defense")   # control -> partial -> full defense

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
        SweepVariable(target="PRIMARY", path="strategy", values=BLUE),
        SweepVariable(target="OPPONENT", path="strategy", values=RED),
    ]
    return expand_jobs(robots, MatchSpec(auto_duration=15.0, teleop_duration=135.0),
                       VARIABILITY, variables, repetitions=REPS,
                       strategies_dir=STRATEGIES_DIR, dt=dt)


def main():
    rows = []
    tables, walls = {}, {}
    for dt in DTS:
        jobs = build(dt)
        t0 = time.perf_counter()
        outcomes = run_all(run_trial, jobs, parallel=True)
        wall = time.perf_counter() - t0
        bad = [o for o in outcomes if o.error is not None]
        if bad:
            print(f"!! {len(bad)} failed at dt=1/{1/dt:.0f}:\n{bad[0].error[:800]}")
        ok = [o for o in outcomes if o.error is None]
        table = {}
        for o in ok:
            m = o.metrics
            key = (o.params["PRIMARY.strategy"], o.params["OPPONENT.strategy"])
            blue = m.final_scores.get("blue", 0.0)
            table.setdefault(key, {})[o.seed] = blue
            rows.append({
                "dt": dt, "seed": o.seed, "blue_strategy": key[0], "red_strategy": key[1],
                "blue": blue, "red": m.final_scores.get("red", 0.0),
                "pieces_scored_by_alliance": m.pieces_scored_by_alliance,
                "protection_fouls": m.protection_fouls_by_alliance,
                "pin_fouls": m.pin_fouls_by_alliance,
            })
        tables[dt], walls[dt] = table, wall
        print(f"dt=1/{1/dt:>3.0f}  {len(ok):>4} matches  {wall:7.1f}s wall  "
              f"({len(ok)/wall:.2f} matches/s)")

    OUT.write_text(json.dumps(rows, indent=1))
    print(f"\npersisted {len(rows)} rows -> {OUT}")

    base = tables[DTS[0]]
    configs = sorted(base)
    base_means = [statistics.mean(base[c].values()) for c in configs]
    within = [statistics.pstdev(base[c].values()) for c in configs]
    se = statistics.mean(within) / math.sqrt(REPS)
    print(f"\nnoise floor at 1/60: within-config sd {statistics.mean(within):.1f} pts, "
          f"standard error {se:.2f} pts")
    print(f"spread across matchups: sd of means {statistics.pstdev(base_means):.1f} pts")

    print("\nblue score by matchup at dt=1/60 (defence should lower it):")
    for c, m in sorted(zip(configs, base_means), key=lambda t: -t[1]):
        print(f"    {c[0]:<22} vs {c[1]:<18} {m:7.1f}")

    # Contact-fidelity check: fouls are charged on per-tick contact tests,
    # so they are the metric most exposed to a coarser command rate.
    print("\ncontact-sensitive metrics by dt (totals across all matches):")
    for dt in DTS:
        sel = [r for r in rows if r["dt"] == dt]
        pf = sum(sum(r["protection_fouls"].values()) for r in sel)
        pin = sum(sum(r["pin_fouls"].values()) for r in sel)
        print(f"    1/{1/dt:<3.0f}  protection fouls {pf:>5}   pin fouls {pin:>5}")

    print(f"\n{'dt':>6} {'speedup':>8} {'pair agree':>11} {'bias':>7}   agreement by separation")
    for dt in DTS:
        table = tables[dt]
        m = [statistics.mean(table[c].values()) for c in configs]
        buckets = {"2-4 SE": [0, 0], "4-8 SE": [0, 0], ">8 SE": [0, 0]}
        agree = total = 0
        for i, j in itertools.combinations(range(len(configs)), 2):
            gap = base_means[i] - base_means[j]
            if abs(gap) < 2 * se:
                continue
            total += 1
            ok_pair = (m[i] - m[j]) * gap > 0
            agree += ok_pair
            n_se = abs(gap) / se
            b = "2-4 SE" if n_se < 4 else ("4-8 SE" if n_se < 8 else ">8 SE")
            buckets[b][1] += 1
            buckets[b][0] += ok_pair
        bias = statistics.mean([table[c][s] - base[c][s]
                                for c in configs for s in base[c] if s in table[c]])
        detail = "  ".join(f"{k}: {v[0]}/{v[1]}" for k, v in buckets.items() if v[1])
        print(f"  1/{1/dt:<3.0f} {walls[DTS[0]]/walls[dt]:7.2f}x "
              f"{agree/total*100 if total else float('nan'):9.0f}% {bias:+7.1f}   {detail}")
    print(f"\n(over {total} of {len(configs)*(len(configs)-1)//2} matchup pairs separated beyond 2 SE)")


if __name__ == "__main__":
    main()
