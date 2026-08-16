from __future__ import annotations

import subprocess
import sys

from common_sim.analysis.runner import run_all
from common_sim.analysis.sweep_spec import (
    MatchSpec,
    RobotSpec,
    characteristics_to_spec,
)
from common_sim.analysis.variability import VariabilityModel
from game_specific.reefscape.sweep_trial import (
    SWEEP_DT,
    STRATEGIES_DIR,
    build_match_for_job,
    replay_trial,
    run_trial,
)
from apps.run_reefscape import build_demo_characteristics
from common_sim.analysis.sweep_spec import TrialJob


def _job(seed=0, variability=None, robots=None, strategy="cycle_coral", teleop_duration=3.0):
    char_spec = characteristics_to_spec(build_demo_characteristics())
    if robots is None:
        robots = [RobotSpec(label="PRIMARY", alliance="blue", roster_index=-1, characteristics=char_spec, strategy=strategy)]
    return TrialJob(
        index=0, seed=seed, params={}, robots=tuple(robots),
        match=MatchSpec(auto_duration=1.0, teleop_duration=teleop_duration),
        variability=variability or VariabilityModel(), strategies_dir=str(STRATEGIES_DIR), dt=SWEEP_DT,
    )


def test_run_trial_deterministic_variability_off():
    job = _job(seed=5)
    a = run_trial(job)
    b = run_trial(job)
    assert a.error is None and b.error is None
    assert a.metrics == b.metrics


def test_run_trial_deterministic_variability_on():
    model = VariabilityModel(enabled=True, intake_time_pct=0.2, deposit_time_pct=0.2, start_pose_xy_in=2.0)
    job = _job(seed=5, variability=model)
    a = run_trial(job)
    b = run_trial(job)
    assert a.error is None and b.error is None
    assert a.metrics == b.metrics


def test_different_seeds_with_variability_differ():
    model = VariabilityModel(enabled=True, start_pose_xy_in=10.0, intake_time_pct=0.3, deposit_time_pct=0.3)
    match_a, robots_a, _ = build_match_for_job(_job(seed=1, variability=model))
    match_b, robots_b, _ = build_match_for_job(_job(seed=2, variability=model))
    pose_a = robots_a["PRIMARY"].pose
    pose_b = robots_b["PRIMARY"].pose
    assert (pose_a.x, pose_a.y) != (pose_b.x, pose_b.y)


def test_run_all_parallel_matches_sequential():
    jobs = [_job(seed=s) for s in range(4)]
    sequential = [run_trial(j) for j in jobs]
    parallel = run_all(run_trial, jobs, parallel=True, max_workers=2)
    assert [o.metrics for o in sequential] == [o.metrics for o in parallel]


def test_bogus_strategy_returns_error_not_raise():
    job = _job(seed=0, strategy="not_a_real_strategy_file")
    outcome = run_trial(job)
    assert outcome.error is not None
    assert outcome.metrics is None


def test_replay_trial_matches_run_trial_metrics():
    from common_sim.analysis.metrics import extract_metrics

    job = _job(seed=9)
    run_outcome = run_trial(job)
    replay_match, _, _ = replay_trial(job)
    replay_metrics = extract_metrics(replay_match)
    assert run_outcome.metrics == replay_metrics


def test_sweep_trial_imports_no_qt():
    script = (
        "import sys; "
        "import game_specific.reefscape.sweep_trial; "
        "bad = [m for m in sys.modules if 'PyQt' in m or 'pyqtgraph' in m]; "
        "assert not bad, bad; "
        "print('ok')"
    )
    result = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, cwd=".")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "ok" in result.stdout
