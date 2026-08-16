"""
The Qt-free worker entry point for a Monte Carlo sweep trial, plus the
match builder the MATCH-tab replay path shares with it -- so the two
cannot drift.

Windows uses the `spawn` start method, so a ProcessPoolExecutor worker
re-imports this module fresh. It must therefore import NO Qt (enforced
by test/game_specific/test_reefscape_sweep.py's subprocess check) --
gui_utils/sweep_panel.py sends `run_trial` across the process boundary
by qualified name, so this module is all a worker process ever loads.

Determinism contract: `run_trial(job)` is exact for a fixed
(TrialJob, seed) -- every scoring interaction is a pymunk broad+narrow
phase result and every draw of randomness (config perturbation, piece
scatter, scoring-reliability rolls) is a substream seeded off `job.seed`,
so `run_trial` and
`replay_trial` agree bit-for-bit (see
test_reefscape_sweep.test_replay_matches_run_trial), *within* one
machine/pymunk build -- not guaranteed bit-identical across machines.

Determinism trap: run_match_to_completion defaults to dt=1/20, but
TelemetryRecorder assumes 60 Hz and MatchView ticks at 1/60. A sweep run
at 1/20 and replayed at 1/60 is a *different* simulation and its score
would not match the results table row -- SWEEP_DT is pinned to 1/60 and
threaded explicitly through TrialJob.dt into both run_trial and
replay_trial so that can't happen by accident.
"""
from __future__ import annotations

import math
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
from game_specific.reefscape.field import build_field, coral_station_positions, reef_center
from game_specific.reefscape.game_pieces import spawn_algae, spawn_coral
from game_specific.reefscape.scoring import REEFSCAPE_SCORING_RULES

SWEEP_DT = 1.0 / 60.0
STRATEGIES_DIR = Path(__file__).resolve().parent / "strategies"


def start_pose(alliance: str, roster_index: int) -> Pose2d:
    """Byte-for-byte the geometry apps/run_reefscape.py's MatchView
    _reset_match (roster_index < 0, the primary robot) and
    _spawn_roster_robot (roster_index >= 0) compute inline --
    run_reefscape.py calls this same function so MATCH and SWEEP place
    robots identically."""
    if roster_index < 0:
        station = coral_station_positions(alliance)[0]
        facing = 0.0 if alliance == "blue" else 3.14159265
        return Pose2d(station[0] + (30.0 if alliance == "blue" else -30.0), station[1], facing)

    station = coral_station_positions(alliance)[roster_index % 2]
    facing = 0.0 if alliance == "blue" else math.pi
    offset = 30.0 + 40.0 * (roster_index // 2 + 1)
    along_wall = offset if roster_index % 2 == 0 else -offset
    return Pose2d(station[0] + (30.0 if alliance == "blue" else -30.0), station[1] + along_wall, facing)


def _resolve_strategy(strategy, strategies_dir):
    """`strategy` travels as a name (str, worker loads the JSON) or a
    strategy_io.to_dict() payload (dict) -- never as a live Strategy
    object, which holds Trigger/Tactic instances and isn't picklable in
    a way that survives the process boundary reliably. The dict form is
    what lets an unsaved STRATEGY-tab edit be swept."""
    if strategy is None:
        return None
    if isinstance(strategy, dict):
        return strategy_io.from_dict(strategy)
    return strategy_io.load_strategy(Path(strategies_dir) / f"{strategy}.json")


def _scatter_pieces(match: Match, alliance: str, seed: int, model) -> None:
    """Same 6-coral/3-algae scatter apps/run_reefscape.py's
    build_demo_match does, with its `random.Random(0)` jitter folded
    into substream(seed, "pieces") so the default (variability-disabled)
    layout is preserved, plus an optional per-piece variability.
    scatter_offset on top."""
    rng = substream(seed, "pieces")
    center = reef_center(alliance)
    toward_wall = -1.0 if alliance == "blue" else 1.0
    for _ in range(6):
        x, y = center[0] + toward_wall * 60, center[1] + rng.uniform(-30, 30)
        dx, dy = scatter_offset(model, rng)
        spawn_coral(match, (x + dx, y + dy))
    for i in range(3):
        x, y = center[0] + toward_wall * 40, center[1] - 60 + 40 * i
        dx, dy = scatter_offset(model, rng)
        spawn_algae(match, (x + dx, y + dy))


def build_match_for_job(job: TrialJob, recorder_cls=None):
    """Builds the field + MatchConfig + scattered pieces, then adds each
    robot in `job.robots` order (spawn order affects physics) with its
    characteristics/pose perturbed and its strategy attached.
    `recorder_cls`, if given (e.g. TelemetryRecorder), is instantiated
    against the built match and returned as the third element -- None
    (the default, used by run_trial, which doesn't need telemetry)
    otherwise."""
    field = build_field()
    match_config = MatchConfig(
        auto_duration=job.match.auto_duration, teleop_duration=job.match.teleop_duration,
        disable_friendly_collisions=job.match.disable_friendly_collisions,
    )
    match = Match(field, REEFSCAPE_SCORING_RULES, match_config, rng=substream(job.seed, "scoring"))
    _scatter_pieces(match, job.match.scatter_alliance, job.seed, job.variability)

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
    """THE worker entry point -- module-level and picklable by qualified
    name, sent to a ProcessPoolExecutor by gui_utils/sweep_panel.py.
    Catches every exception so one bad config cannot kill the pool."""
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
    """Re-runs the exact same job with a TelemetryRecorder attached, for
    the MATCH tab to scrub through. Steps at job.dt (== SWEEP_DT for any
    job built by expand_jobs) -- see the determinism trap in this
    module's docstring."""
    match, robots_by_label, recorder = build_match_for_job(job, recorder_cls=TelemetryRecorder)
    while not match.ended:
        match.step(job.dt)
        recorder.tick()
    return match, robots_by_label, recorder
