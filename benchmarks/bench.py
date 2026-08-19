"""Phase 1 harness: throughput + a determinism fingerprint.

The fingerprint is the point. Any change that alters the fingerprint has
changed what the simulator *does*, not just how fast it does it, and is
not a valid optimization no matter what the clock says.

Usage:  python bench.py [label]
"""
import hashlib
import json
import sys
import time

from common_sim.analysis.metrics import extract_metrics
from common_sim.analysis.monte_carlo import run_match_to_completion
from common_sim.analysis.sweep_spec import MatchSpec, RobotSpec, characteristics_to_spec, expand_jobs
from common_sim.analysis.variability import VariabilityModel
from game_specific.reefscape.sweep_trial import STRATEGIES_DIR, SWEEP_DT, build_match_for_job
from apps.run_strategy_sweep import build_characteristics

_SCALAR = (str, int, float, bool, type(None))

# Deliberately non-trivial: perturbs characteristics, start poses and piece
# scatter, so the fingerprint covers code paths a bit-identical default run
# would never reach.
VARIABILITY = VariabilityModel(
    enabled=True, intake_time_pct=0.10, deposit_time_pct=0.10, max_speed_pct=0.08,
    max_accel_pct=0.08, start_pose_xy_in=2.0, start_pose_heading_deg=3.0, piece_scatter_in=3.0,
)

# Mixed rosters so Collect / Score / Defend and the contention paths all run.
CONFIGS = [
    ("cycle_v_cycle",   ["cycle_coral", "cycle_coral"],           ["cycle_coral"]),
    ("evasive_v_def",   ["cycle_coral_evasive", "algae_processor"], ["full_defense"]),
    ("auto_v_endgame",  ["auto_then_cycle", "cycle_coral"],       ["endgame_defense"]),
]


def fingerprint(match) -> str:
    """Stable hash of everything the match did: the full event stream
    (scalar fields only -- object identities are not stable across runs)
    plus final scores and final robot poses to 6dp."""
    h = hashlib.sha256()
    for event in match.events:
        data = {k: v for k, v in event.data.items() if isinstance(v, _SCALAR)}
        h.update(json.dumps(
            [round(event.timestamp, 6), event.kind, sorted(data.items(), key=lambda kv: kv[0])],
            sort_keys=True, default=str,
        ).encode())
    h.update(json.dumps(sorted(match.scores.items()), sort_keys=True).encode())
    for robot in match.robots:
        pose = robot.pose
        h.update(json.dumps([robot.alliance, round(pose.x, 6), round(pose.y, 6),
                             round(pose.heading, 6)]).encode())
    return h.hexdigest()[:16]


def build_jobs():
    char = characteristics_to_spec(build_characteristics())
    jobs = []
    for name, blue, red in CONFIGS:
        robots = []
        for i, strat in enumerate(blue):
            robots.append(RobotSpec(label=f"B{i}", alliance="blue", roster_index=i - 1,
                                    characteristics=dict(char), strategy=strat))
        for i, strat in enumerate(red):
            robots.append(RobotSpec(label=f"R{i}", alliance="red", roster_index=i,
                                    characteristics=dict(char), strategy=strat))
        spec = MatchSpec(auto_duration=15.0, teleop_duration=135.0)
        built = expand_jobs(robots, spec, VARIABILITY, [], repetitions=2,
                            strategies_dir=STRATEGIES_DIR, dt=SWEEP_DT)
        jobs.extend((name, job) for job in built)
    return jobs


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "run"
    jobs = build_jobs()
    print(f"[{label}] {len(jobs)} matches\n")

    total = 0.0
    prints = []
    for name, job in jobs:
        t0 = time.perf_counter()
        match, _, _ = build_match_for_job(job)
        run_match_to_completion(match, dt=job.dt)
        elapsed = time.perf_counter() - t0
        total += elapsed
        m = extract_metrics(match)
        prints.append(f"  {name:<16} seed={job.seed:<4} {elapsed:6.2f}s  "
                      f"fp={fingerprint(match)}  scored={m.pieces_scored:<3} "
                      f"blue={match.scores.get('blue', 0):.0f} red={match.scores.get('red', 0):.0f}")
    print("\n".join(prints))

    combined = hashlib.sha256("".join(p.split("fp=")[1].split()[0] for p in prints).encode()).hexdigest()[:16]
    print(f"\n[{label}] TOTAL {total:.2f}s   mean {total / len(jobs):.2f}s/match")
    print(f"[{label}] COMBINED FINGERPRINT {combined}")


if __name__ == "__main__":
    main()
