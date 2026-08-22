"""
SALVAGE's Qt-free worker entry point, the twin of
`game_specific/reefscape/sweep_trial.py`.

It is deliberately written as a near-copy of REEFSCAPE's rather than
being factored into a shared base first: the dry run's job is to find
out how much of this file is actually game-specific, and you cannot see
that by writing the abstraction you assumed. What the diff between the
two shows is that only `_stage_pieces`, `start_pose` and the three
imports (field / scoring / piece spawners) carry any game content --
`_resolve_strategy`, `build_match_for_job`, `run_trial` and
`replay_trial` are identical modulo those. See DRY_RUN_LOG.md.

Same constraints as REEFSCAPE's: no Qt import anywhere on this path
(Windows `spawn` re-imports the module in every worker), and the same
determinism contract -- every draw of randomness is a `substream` off
`job.seed`.
"""
from __future__ import annotations

import time
import traceback
from pathlib import Path

from common_sim.analysis.metrics import extract_metrics
from common_sim.analysis.monte_carlo import run_match_to_completion
from common_sim.analysis.sweep_spec import TrialJob, TrialOutcome, characteristics_from_spec
from common_sim.analysis.variability import perturb_characteristics, perturb_pose, scatter_offset, substream
from common_sim.control import strategy_io
from common_sim.control.strategy import StrategyController
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.telemetry import TelemetryRecorder
from game_specific.salvage.field import (
    build_field, crate_staging_positions, scrap_staging_positions, start_pose_positions,
)
from game_specific.salvage.game_pieces import spawn_crate, spawn_scrap
from game_specific.salvage.scoring import SALVAGE_SCORING_RULES

SWEEP_DT = 1.0 / 60.0
SEARCH_DT = 1.0 / 30.0

STRATEGIES_DIR = Path(__file__).resolve().parent / "strategies"

# How many starting spots each alliance's wall is divided into. Fixed
# rather than derived from the roster so that a 2v2 and a 3v3 do not
# place their first robot in different places -- a bench that compares
# across roster sizes needs the shared robots to start identically.
START_SPOTS = 3


def start_pose(alliance: str, roster_index: int) -> Pose2d:
    """Where robot `roster_index` of `alliance` starts. Index -1 is the
    GUI's single primary robot and takes the middle spot."""
    spots = start_pose_positions(alliance, START_SPOTS)
    index = (START_SPOTS // 2) if roster_index < 0 else (roster_index % START_SPOTS)
    x, y = spots[index]
    facing = 0.0 if alliance == "blue" else 3.141592653589793
    return Pose2d(x, y, facing)


def _resolve_strategy(strategy, strategies_dir):
    if strategy is None:
        return None
    if isinstance(strategy, dict):
        return strategy_io.from_dict(strategy)
    return strategy_io.load_strategy(Path(strategies_dir) / f"{strategy}.json")


def _stage_pieces(match: Match, seed: int, model) -> None:
    """Pre-match floor pieces: four SCRAP and two CRATEs per alliance
    zone. CELLs are deliberately absent -- the only way to get one is to
    go to the neutral depot."""
    rng = substream(seed, "pieces")
    for side in ("blue", "red"):
        for x, y in scrap_staging_positions(side):
            dx, dy = scatter_offset(model, rng)
            spawn_scrap(match, (x + dx, y + dy))
        for x, y in crate_staging_positions(side):
            dx, dy = scatter_offset(model, rng)
            spawn_crate(match, (x + dx, y + dy))


def build_match_for_job(job: TrialJob, recorder_cls=None):
    field = build_field()
    match_config = MatchConfig(
        auto_duration=job.match.auto_duration, teleop_duration=job.match.teleop_duration,
        disable_friendly_collisions=job.match.disable_friendly_collisions,
        emit_coral_to_field=True,
    )
    match = Match(field, SALVAGE_SCORING_RULES, match_config, rng=substream(job.seed, "scoring"))
    _stage_pieces(match, job.seed, job.variability)

    robots_by_label = {}
    for robot_spec in job.robots:
        char_spec = perturb_characteristics(
            robot_spec.characteristics, job.variability, substream(job.seed, f"chars:{robot_spec.label}"),
        )
        characteristics = characteristics_from_spec(char_spec)

        pose = start_pose(robot_spec.alliance, robot_spec.roster_index)
        x, y, heading = perturb_pose(
            pose.x, pose.y, pose.heading, job.variability, substream(job.seed, f"pose:{robot_spec.label}"),
        )

        robot = match.add_robot(characteristics, Pose2d(x, y, heading), alliance=robot_spec.alliance)
        robots_by_label[robot_spec.label] = robot

        strategy = _resolve_strategy(robot_spec.strategy, job.strategies_dir)
        if strategy is not None:
            robot.controller = StrategyController(strategy, robot)

    recorder = recorder_cls(match) if recorder_cls is not None else None
    return match, robots_by_label, recorder


def run_trial(job: TrialJob) -> TrialOutcome:
    start = time.perf_counter()
    try:
        match, _, _ = build_match_for_job(job)
        run_match_to_completion(match, dt=job.dt)
        metrics = extract_metrics(match)
        return TrialOutcome(
            index=job.index, seed=job.seed, params=job.params,
            metrics=metrics, error=None, duration_s=time.perf_counter() - start,
        )
    except Exception:
        return TrialOutcome(
            index=job.index, seed=job.seed, params=job.params,
            metrics=None, error=traceback.format_exc(), duration_s=time.perf_counter() - start,
        )


def replay_trial(job: TrialJob):
    match, robots_by_label, recorder = build_match_for_job(job, recorder_cls=TelemetryRecorder)
    while not match.ended:
        match.step(job.dt)
        recorder.tick()
    return match, robots_by_label, recorder
