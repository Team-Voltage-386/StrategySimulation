"""Interleaved A/B so thermal drift hits both arms equally.

The three cached functions expose `__wrapped__`, so the uncached arm is
the original function with no other difference -- same process, same
build, alternating rounds.
"""
import statistics
import sys
import time

sys.argv = ["x"]
from bench import build_jobs  # noqa: E402
from common_sim.analysis.monte_carlo import run_match_to_completion  # noqa: E402
from common_sim.control import navigation  # noqa: E402
from game_specific.reefscape.sweep_trial import build_match_for_job  # noqa: E402

CACHED = {n: getattr(navigation, n) for n in ("_inflate", "_clearance_for_goal", "polygon_distance")}
RAW = {n: f.__wrapped__ for n, f in CACHED.items()}


def use(mapping):
    for name, fn in mapping.items():
        setattr(navigation, name, fn)


def one_round():
    total = 0.0
    for _, job in build_jobs():
        t0 = time.perf_counter()
        match, _, _ = build_match_for_job(job)
        run_match_to_completion(match, dt=job.dt)
        total += time.perf_counter() - t0
    return total


ROUNDS = 4
off, on = [], []
for r in range(ROUNDS):
    use(RAW)
    for f in CACHED.values():
        f.cache_clear()
    off.append(one_round())
    use(CACHED)
    for f in CACHED.values():
        f.cache_clear()
    on.append(one_round())
    print(f"round {r + 1}: uncached {off[-1]:6.2f}s   cached {on[-1]:6.2f}s   "
          f"{off[-1] / on[-1]:.3f}x")

mo, mn = statistics.median(off), statistics.median(on)
print(f"\nuncached median {mo:.2f}s  (min {min(off):.2f}, max {max(off):.2f})")
print(f"cached   median {mn:.2f}s  (min {min(on):.2f}, max {max(on):.2f})")
print(f"speedup on medians: {mo / mn:.3f}x   ({(1 - mn / mo) * 100:.1f}% faster)")
print(f"per-round speedups: {', '.join(f'{a / b:.3f}' for a, b in zip(off, on))}")
