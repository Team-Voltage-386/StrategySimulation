from common_sim.analysis.metrics import extract_metrics
from common_sim.analysis.monte_carlo import ParameterSweep, TrialResult, run_match_to_completion, run_monte_carlo
from common_sim.analysis.results import summarize, to_dataframe
from common_sim.control.behavior import BehaviorContext, Repeat, RunManipulator
from common_sim.field.field_config import FieldConfig, ScoringRegion
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"


def _make_field() -> FieldConfig:
    # Scoring region covers the whole field -- these trials are about
    # deposit timing/capacity, not navigation, so pieces preload right
    # on top of the goal.
    # passive_scoring=True: these trials are about deposit timing/capacity,
    # not robot placement/engagement geometry -- a piece landing anywhere
    # in the (whole-field) region should count, matching the pre-existing
    # behavior these trials were written against.
    region = ScoringRegion(
        name="goal", vertices=((0, 0), (300, 0), (300, 200), (0, 200)),
        actions=frozenset({"score"}), piece_types=frozenset({WIDGET}), passive_scoring=True,
    )
    return FieldConfig(width=300, height=200, scoring_regions=(region,))


def build_trial_match(params: dict) -> Match:
    """Module-level (not a closure) so it stays picklable for the
    parallel=True / multiprocessing path. A robot preloaded with pieces
    repeatedly deposits until the match ends -- deposit_time and
    piece_capacity directly gate how many of its preloaded pieces it
    gets through in the fixed match window, which is what this trial is
    designed to make visible to a parameter sweep."""
    field = _make_field()
    rules = TableScoringRules({("score", "auto"): 1.0, ("score", "teleop"): 1.0})
    match = Match(field, rules, MatchConfig(auto_duration=0.0, teleop_duration=params.get("match_duration", 2.0)))

    characteristics = RobotCharacteristics(
        piece_capacity=params.get("piece_capacity", 3),
        starting_piece_count=params.get("piece_capacity", 3),
        deposit_time=params["deposit_time"],
        accepted_piece_types=frozenset({WIDGET}),
    )
    robot = match.add_robot(characteristics, Pose2d(150, 100, 0))

    dt = 1.0 / 60.0
    routine = Repeat(RunManipulator("score", timeout=params["deposit_time"] + 1.0))
    ctx = BehaviorContext(robot=robot, dt=dt)
    while not match.ended:
        ctx.dt = dt
        routine.tick(ctx)
        match.step(dt)
        ctx.elapsed += dt
    return match


# -- ParameterSweep ----------------------------------------------------------


def test_parameter_sweep_full_factorial():
    sweep = ParameterSweep({"a": [1, 2], "b": ["x", "y", "z"]})
    configs = sweep.configs()
    assert len(configs) == 6
    assert {"a": 1, "b": "x"} in configs
    assert {"a": 2, "b": "z"} in configs


def test_parameter_sweep_empty_values_yields_one_empty_config():
    sweep = ParameterSweep({})
    assert sweep.configs() == [{}]


# -- metrics -------------------------------------------------------------


def test_extract_metrics_counts_events_and_cycle_times():
    match = build_trial_match({"deposit_time": 0.1, "piece_capacity": 3, "match_duration": 2.0})
    metrics = extract_metrics(match)
    assert metrics.pieces_scored == 3  # all 3 preloaded pieces deposited within the window
    assert metrics.pieces_deposited == 3
    assert metrics.misses == 0
    assert metrics.final_scores["blue"] == 3.0
    assert len(metrics.cycle_times) == 2  # 3 scores -> 2 gaps between them
    assert metrics.mean_cycle_time is not None
    assert metrics.mean_cycle_time > 0


# -- run_monte_carlo -----------------------------------------------------


def test_run_monte_carlo_sequential_runs_every_config_times_repetitions():
    sweep = ParameterSweep({"deposit_time": [0.1, 1.5]})
    results = run_monte_carlo(build_trial_match, sweep, repetitions=2)
    assert len(results) == 4
    assert all(isinstance(r, TrialResult) for r in results)
    param_counts = {}
    for r in results:
        param_counts[r.params["deposit_time"]] = param_counts.get(r.params["deposit_time"], 0) + 1
    assert param_counts == {0.1: 2, 1.5: 2}


def test_faster_deposit_time_scores_more_pieces_in_a_fixed_window():
    sweep = ParameterSweep({"deposit_time": [0.1, 1.5]})
    results = run_monte_carlo(build_trial_match, sweep, repetitions=1)
    by_deposit_time = {r.params["deposit_time"]: r.metrics.pieces_scored for r in results}
    assert by_deposit_time[0.1] > by_deposit_time[1.5]


def test_run_monte_carlo_parallel_matches_sequential_for_deterministic_trial():
    sweep = ParameterSweep({"deposit_time": [0.2]})
    sequential = run_monte_carlo(build_trial_match, sweep, repetitions=2, parallel=False)
    parallel = run_monte_carlo(build_trial_match, sweep, repetitions=2, parallel=True, max_workers=2)
    assert [r.metrics.pieces_scored for r in sequential] == [r.metrics.pieces_scored for r in parallel]


# -- results ---------------------------------------------------------------


def test_to_dataframe_has_expected_columns():
    sweep = ParameterSweep({"deposit_time": [0.1, 1.5]})
    results = run_monte_carlo(build_trial_match, sweep, repetitions=2)
    df = to_dataframe(results)
    assert len(df) == 4
    for col in ["deposit_time", "total_score", "score_blue", "pieces_scored", "misses", "mean_cycle_time"]:
        assert col in df.columns


def test_summarize_groups_by_swept_parameter():
    sweep = ParameterSweep({"deposit_time": [0.1, 1.5]})
    results = run_monte_carlo(build_trial_match, sweep, repetitions=3)
    df = to_dataframe(results)
    summary = summarize(df, ["deposit_time"], metric="pieces_scored")
    assert len(summary) == 2
    fast_row = summary[summary["deposit_time"] == 0.1].iloc[0]
    slow_row = summary[summary["deposit_time"] == 1.5].iloc[0]
    assert fast_row["mean"] > slow_row["mean"]
    assert fast_row["count"] == 3
