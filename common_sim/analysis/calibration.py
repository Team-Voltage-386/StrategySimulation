"""How much simulation this particular machine can do, and how long a
given batch will take on it.

Sweeps and searches are sized in matches, but a match costs wildly
different amounts of wall time on different hardware -- a laptop that
throttles under sustained load and a desktop with a higher sustained
clock differ by more than their core counts suggest, and this project is
run by whoever on the team has a machine free. Rather than tie any batch
size to one reference machine, measure the machine in front of the user
and report what it can realistically finish.

Game-agnostic by construction: the caller supplies the worker callable
and the reference jobs, so nothing here knows what a match is. See
`apps/run_calibration.py` for the REEFSCAPE entry point.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from common_sim.analysis.runner import run_all


@dataclass(frozen=True)
class Throughput:
    """What one machine measured, and how confident that measurement is.

    `sample_size` is carried because a calibration run short enough to be
    worth waiting for is also short enough to be noisy -- a caller
    printing an estimate should be able to say how thin the evidence is.
    """
    matches_per_second: float
    sample_size: int
    wall_seconds: float
    failures: int = 0

    @property
    def matches_per_hour(self) -> float:
        return self.matches_per_second * 3600.0

    @property
    def seconds_per_match(self) -> float:
        return 1.0 / self.matches_per_second if self.matches_per_second > 0 else float("inf")


def measure(run_fn, jobs: list, *, parallel: bool = True) -> Throughput:
    """Run `jobs` and time them end to end.

    The whole batch is timed rather than each job summed, because the
    number worth reporting is throughput *including* pool startup and the
    tail where workers run dry -- that is what the user actually waits
    through. Sum the per-job durations instead and a 12-core box looks
    twelve times better than it behaves.

    Failed trials are counted, not raised: a machine that cannot run the
    reference workload at all should report that clearly rather than
    crash a progress dialog.
    """
    if not jobs:
        raise ValueError("calibration needs at least one job")
    start = time.perf_counter()
    outcomes = run_all(run_fn, jobs, parallel=parallel)
    wall = time.perf_counter() - start
    failures = sum(1 for o in outcomes if getattr(o, "error", None) is not None)
    completed = len(outcomes) - failures
    rate = completed / wall if wall > 0 and completed else 0.0
    return Throughput(matches_per_second=rate, sample_size=len(jobs),
                      wall_seconds=wall, failures=failures)


def estimate_seconds(n_matches: int, throughput: Throughput) -> float:
    if throughput.matches_per_second <= 0:
        return float("inf")
    return n_matches / throughput.matches_per_second


def humanize(seconds: float) -> str:
    """Duration at the precision a person planning an evening cares
    about -- nobody schedules around 4h 37m 12s."""
    if seconds == float("inf"):
        return "unknown"
    if seconds < 90:
        return f"{seconds:.0f} seconds"
    minutes = seconds / 60.0
    if minutes < 90:
        return f"{minutes:.0f} minutes"
    hours = minutes / 60.0
    if hours < 36:
        return f"{hours:.1f} hours"
    return f"{hours / 24.0:.1f} days"


# Batch sizes the team actually runs, as (label, matches). Kept here so
# one edit changes every place that sets expectations.
REFERENCE_BATCHES = (
    ("a quick comparison", 200),
    ("a design sweep", 2_000),
    ("one search generation", 10_000),
    ("an overnight search", 50_000),
)

# Past this, a batch has stopped being something you wait for and started
# being something you plan around -- the point to suggest another machine.
_LONG_RUN_SECONDS = 8 * 3600.0


def report(throughput: Throughput, batches=REFERENCE_BATCHES) -> str:
    """A short plain-text capability summary to show the user before they
    commit to a long run."""
    lines = [
        f"This machine: {throughput.matches_per_hour:,.0f} matches/hour "
        f"({throughput.seconds_per_match:.2f}s per match), "
        f"measured over {throughput.sample_size} matches in "
        f"{humanize(throughput.wall_seconds)}.",
    ]
    if throughput.failures:
        lines.append(f"WARNING: {throughput.failures} of {throughput.sample_size} "
                     f"reference matches failed -- treat this estimate as unreliable.")
    lines.append("")
    slow = False
    for label, n in batches:
        seconds = estimate_seconds(n, throughput)
        flag = ""
        if seconds > _LONG_RUN_SECONDS:
            flag = "   <-- consider a faster machine"
            slow = True
        lines.append(f"  {n:>7,} matches  ({label:<22}) {humanize(seconds):>12}{flag}")
    if slow:
        lines.append("")
        lines.append("Runs marked above are long enough to be worth handing to whoever "
                     "on the team has the fastest desktop free.")
    return "\n".join(lines)
