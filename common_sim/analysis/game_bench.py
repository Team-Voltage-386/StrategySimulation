"""
Game-agnostic core shared by the defense bench and the stall audit --
the fix for DRY_RUN_LOG.md's F6.

Before this, `apps/run_defense_bench.py` imported
`game_specific.reefscape.sweep_trial` at module scope, and
`apps/run_stall_audit.py` imported `run_defense_bench` for its job
builder -- so neither of the two most valuable measurement tools in the
repo could be pointed at a second game. `apps/run_salvage_bench.py` was
written as a deliberate near-duplicate of `run_defense_bench.py` to
prove the shared surface: a match builder, a strategies directory, a
reference robot, and a plan table (see that file's own docstring). This
module is that shared surface, extracted.

No Qt, no game_specific -- see ARCHITECTURE.md's import contract and
`common_sim/analysis/runner.py`. A game plugs in by building a
`BenchGame` from its own `sweep_trial.build_match_for_job` and
`robot.build_characteristics`; everything here operates on those
callables and never imports the module they came from.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from common_sim.analysis.metrics import extract_metrics
from common_sim.analysis.monte_carlo import run_match_to_completion
from common_sim.analysis.sweep_spec import MatchSpec, RobotSpec, TrialJob, TrialOutcome, characteristics_to_spec
from common_sim.analysis.variability import VariabilityModel
from common_sim.control import strategy_io
from common_sim.robot.characteristics import RobotCharacteristics

# `VariabilityModel()`'s defaults perturb *nothing*, so running N seeds
# with it produces N bit-identical matches and an N-of-1 sample wearing
# a mean's clothing -- this is deliberately non-trivial. Shared by every
# defense-bench-shaped tool so a game doesn't have to remember to set it.
VARIABILITY = VariabilityModel(
    enabled=True, intake_time_pct=0.10, deposit_time_pct=0.10,
    max_speed_pct=0.08, max_accel_pct=0.08,
    start_pose_xy_in=4.0, start_pose_heading_deg=5.0, piece_scatter_in=3.0,
)


@dataclass(frozen=True)
class BenchGame:
    """What a game must supply to plug into the shared defense bench /
    stall audit: its match builder (from that game's `sweep_trial.py`),
    its strategies directory and sweep dt, and its reference robot (from
    that game's `robot.py`)."""
    build_match_for_job: Callable[[TrialJob], tuple]
    strategies_dir: Path
    dt: float
    build_characteristics: Callable[..., RobotCharacteristics]


def load_strategy(game: BenchGame, name: str) -> dict:
    """Strategies travel to a worker as plain dicts, not live Strategy
    objects -- see sweep_trial._resolve_strategy."""
    return strategy_io.to_dict(strategy_io.load_strategy(game.strategies_dir / f"{name}.json"))


def full_time_defender(game: BenchGame, plan_name: str, base: str = "full_defense") -> dict:
    """The `base` strategy file with its Defend `mode` (and `deny`, if
    the plan names one) overridden -- a robot that does nothing but
    Defend, from the first tick, as a bound on how much denial is even
    available.

    Loaded from the file rather than built here so the plan these
    numbers are measured against is the same object you can open in the
    STRATEGY tab and sweep by name."""
    mode, _, deny = plan_name.partition("/")
    plan = load_strategy(game, base)
    for rule in plan["rules"]:
        if rule["tactic"]["type"] == "Defend":
            rule["tactic"]["mode"] = mode
            if deny:
                rule["tactic"]["deny"] = deny
    return plan


def build_defense_job(
    game: BenchGame, *, index: int, seed: int, red_plan: str,
    blue_plan: str, blue_lineup: tuple, red_baseline: str,
    per_side: int, defenders: int, match: MatchSpec,
    variability: VariabilityModel = VARIABILITY,
) -> TrialJob:
    """One (red plan, blue plan) trial. `blue_lineup` is a strategy name
    per blue robot slot -- a *lineup*, not one strategy, because a
    defender's effect depends on what the alliance it is defending is
    trying to do (the last entry repeats if the alliance is wider than
    the lineup). `red_baseline` is what a non-defending red robot runs,
    so red_plan="none" reads as "everybody just cycles"."""
    characteristics = characteristics_to_spec(game.build_characteristics())
    robots = [
        RobotSpec(label=f"B{i}", alliance="blue", roster_index=i, characteristics=characteristics,
                  strategy=load_strategy(game, blue_lineup[min(i, len(blue_lineup) - 1)]))
        for i in range(per_side)
    ]
    robots += [
        RobotSpec(label=f"R{i}", alliance="red", roster_index=i, characteristics=characteristics,
                  strategy=full_time_defender(game, red_plan) if red_plan != "none" and i < defenders
                  else load_strategy(game, red_baseline))
        for i in range(per_side)
    ]
    return TrialJob(
        index=index, seed=seed, params={"red": red_plan, "blue": blue_plan}, robots=tuple(robots),
        match=match, variability=variability, strategies_dir=str(game.strategies_dir), dt=game.dt,
    )


def run_defense_trial(job: TrialJob, build_match_for_job: Callable) -> TrialOutcome:
    """The trial body every game's `run_trial` wrapper delegates to. The
    wrapper itself still has to live in a game-specific, module-level
    function -- ProcessPoolExecutor needs something picklable by
    qualified name, and that name has to resolve in a module whose
    import graph already includes this game's `sweep_trial`."""
    match, _, _ = build_match_for_job(job)
    run_match_to_completion(match, dt=job.dt)
    return TrialOutcome(index=job.index, seed=job.seed, params=job.params, metrics=extract_metrics(match))


# -- stall audit ----------------------------------------------------------

# A robot that moves less than this in a tick is treated as stopped. Well
# under the distance a drivetrain covers in a tick at any real speed, and
# well over the jitter a robot parked against something shows while its
# drive fights the contact solver.
STILL_EPSILON = 0.05

# How long a robot must be stopped before the stall is worth reporting.
DEFAULT_STALL_THRESHOLD = 20.0


def run_stall_trial(job: TrialJob, build_match_for_job: Callable) -> TrialOutcome:
    """Steps the match directly rather than through
    `run_match_to_completion`, because the whole measurement is per-tick
    and that helper has no hook. See `apps/run_stall_audit.py` for what
    the result means and how to read it."""
    match, _, _ = build_match_for_job(job)
    robots = list(match.robots)
    last = [(r.pose.x, r.pose.y) for r in robots]
    still = [0.0] * len(robots)
    asking = [0.0] * len(robots)
    spinning = [0.0] * len(robots)
    longest = [0.0] * len(robots)
    frozen_at = [None] * len(robots)
    commanded = [0.0] * len(robots)
    commanded_spin = [0.0] * len(robots)
    ended_at = [0.0] * len(robots)
    elapsed = 0.0

    while not match.ended:
        match.step(job.dt)
        elapsed += job.dt
        for i, robot in enumerate(robots):
            moved = math.hypot(robot.pose.x - last[i][0], robot.pose.y - last[i][1])
            last[i] = (robot.pose.x, robot.pose.y)
            if moved > STILL_EPSILON:
                still[i] = 0.0
                asking[i] = 0.0
                spinning[i] = 0.0
                continue
            still[i] += job.dt
            asking[i] += robot.commanded_speed * job.dt
            spinning[i] += robot.commanded_angular_speed * job.dt
            if still[i] > longest[i]:
                longest[i] = still[i]
                frozen_at[i] = (round(robot.pose.x, 1), round(robot.pose.y, 1))
                commanded[i] = asking[i] / still[i]
                commanded_spin[i] = spinning[i] / still[i]
                ended_at[i] = elapsed

    stalls = [
        {"robot": job.robots[i].label, "alliance": robots[i].alliance,
         "seconds": round(longest[i], 1), "at": frozen_at[i],
         "commanded": round(commanded[i], 1),
         "spin": round(commanded_spin[i], 2),
         "ended": round(ended_at[i], 1), "duration": round(elapsed, 1)}
        for i in range(len(robots))
    ]
    return TrialOutcome(
        index=job.index, seed=job.seed,
        params={**job.params, "stalls": stalls,
                "blue_points": match.scores.get("blue", 0.0)},
        metrics=None,
    )
