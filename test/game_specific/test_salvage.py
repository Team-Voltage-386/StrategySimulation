"""
SALVAGE dry-run tests -- see DRY_RUN_LOG.md for what this game is and
why it exists.

Two jobs. First, keep the invented field honest: a dry run whose own
field is broken measures the field, not the framework, and both bugs
this exercise found presented as "a strategy is losing" rather than as
anything obviously geometric. Second, hold the line on both fields
validating clean, so that the next game-shaped change to `common_sim`
has something to fail against.
"""
import pytest

from common_sim.analysis.metrics import extract_metrics
from common_sim.analysis.monte_carlo import run_match_to_completion
from common_sim.analysis.sweep_spec import (
    MatchSpec, RobotSpec, TrialJob, characteristics_to_spec,
)
from common_sim.analysis.variability import VariabilityModel
from common_sim.control import strategy_io
from common_sim.field.validation import ERROR, describe_problems, validate_field
from game_specific.salvage import sweep_trial
from game_specific.salvage.field import (
    BEACON_CAPACITY, DEPOT_CELLS_PER_SIDE, FIELD_LENGTH, FIELD_WIDTH, build_field, rotate,
)
from game_specific.salvage.game_pieces import CELL_TYPE, CRATE_TYPE, SCRAP_TYPE
from game_specific.salvage.robot import build_characteristics
from game_specific.salvage.scoring import SALVAGE_SCORING_RULES

# Non-trivial on purpose: VariabilityModel()'s defaults perturb nothing,
# so N seeds with it is N bit-identical matches wearing a mean's clothing.
VARIABILITY = VariabilityModel(
    enabled=True, intake_time_pct=0.10, deposit_time_pct=0.10,
    max_speed_pct=0.08, max_accel_pct=0.08,
    start_pose_xy_in=4.0, start_pose_heading_deg=5.0, piece_scatter_in=3.0,
)


def errors_on(field, rules):
    characteristics = build_characteristics()
    problems = validate_field(
        field, robot_width=characteristics.width, robot_length=characteristics.length,
        scoring_rules=rules,
    )
    return [p for p in problems if p.severity == ERROR]


def test_the_salvage_field_validates_clean():
    errors = errors_on(build_field(), SALVAGE_SCORING_RULES)
    assert not errors, describe_problems(errors)


def test_the_reefscape_field_validates_clean():
    """The checker is only worth anything if it agrees with the game
    that already works. A false alarm here is a bug in the checker."""
    from game_specific.reefscape.field import build_field as reefscape_field
    from game_specific.reefscape.scoring import REEFSCAPE_SCORING_RULES

    errors = errors_on(reefscape_field(), REEFSCAPE_SCORING_RULES)
    assert not errors, describe_problems(errors)


def test_the_field_is_rotationally_symmetric():
    """A head-to-head is only fair if both alliances get the same field,
    and the check is worth writing down because the first SALVAGE match
    scored 243-73 with identical strategies on both sides -- which looks
    exactly like an asymmetric field and was in fact two framework bugs.
    Ruling the field out first is what made the rest of that trace
    tractable.

    Rotational, not mirrored: the BEACON sits on the centre line and is
    shared, so it is excluded rather than paired."""
    field = build_field()
    for group in (field.scoring_regions, field.intake_locations, field.protected_zones):
        by_name = {f.name: f for f in group}
        for name, feature in by_name.items():
            if not name.startswith("blue"):
                continue
            twin = by_name.get(name.replace("blue", "red", 1))
            assert twin is not None, f"{name} has no red counterpart"
            expected = sorted(round(v, 6) for v in _flatten(rotate(p) for p in feature.vertices))
            actual = sorted(round(v, 6) for v in _flatten(twin.vertices))
            assert expected == actual, f"{name} and {twin.name} are not 180-degree rotations"


def _flatten(points):
    for x, y in points:
        yield x
        yield y


def test_the_depot_is_neutral_and_finite():
    """The one mechanic REEFSCAPE cannot express: a supply that belongs
    to nobody and runs out. `Pursue.scarcity_weight` has been inert at
    0.0 because a CORAL STATION never empties."""
    field = build_field()
    depots = [s for s in field.intake_locations if s.piece_type == CELL_TYPE]
    assert len(depots) == 2
    assert all(s.alliance is None for s in depots)
    assert all(s.starting_pieces == DEPOT_CELLS_PER_SIDE for s in depots)


def test_the_beacon_is_shared_capacity():
    field = build_field()
    beacon = next(r for r in field.scoring_regions if r.name == "beacon")
    assert beacon.alliance is None
    assert beacon.capacity_by_action == {"beacon": BEACON_CAPACITY}


def test_cells_never_start_on_the_floor():
    """A piece type reachable only by going to a contested station --
    REEFSCAPE has no such piece, so `Collect`'s station path has never
    been the *only* way to obtain something."""
    job = _job(("cycle_crates", "cycle_crates"))
    match, _, _ = sweep_trial.build_match_for_job(job)
    staged = {p.piece_type for p in match.active_pieces}
    assert staged == {CRATE_TYPE, SCRAP_TYPE}


def test_the_deep_hold_is_worth_more_in_teleop_than_auto():
    """Every REEFSCAPE action is worth the same or more in AUTO, so
    nothing has ever checked that a planner reads the phase rather than
    assuming the early game is the valuable one."""
    auto = SALVAGE_SCORING_RULES.points_for("hold_high", "auto")
    teleop = SALVAGE_SCORING_RULES.points_for("hold_high", "teleop")
    assert teleop > auto


def _job(lineup, seed=5000, per_side=2):
    characteristics = characteristics_to_spec(build_characteristics())
    robots = [
        RobotSpec(label=f"B{i}", alliance="blue", roster_index=i,
                  characteristics=characteristics, strategy=lineup[0])
        for i in range(per_side)
    ] + [
        RobotSpec(label=f"R{i}", alliance="red", roster_index=i,
                  characteristics=characteristics, strategy=lineup[1])
        for i in range(per_side)
    ]
    return TrialJob(
        index=0, seed=seed, params={}, robots=tuple(robots),
        match=MatchSpec(auto_duration=15.0, teleop_duration=135.0),
        variability=VARIABILITY, strategies_dir=str(sweep_trial.STRATEGIES_DIR),
        dt=sweep_trial.SEARCH_DT,
    )


@pytest.mark.parametrize("name", [
    "cycle_crates", "pursue", "pursue_tuned", "pursue_scarce", "full_defense", "rush_reactor",
])
def test_every_strategy_file_loads_and_runs(name):
    """These files double as the strategy format's documentation. A
    short match is enough to catch a name that no longer resolves."""
    strategy = strategy_io.load_strategy(sweep_trial.STRATEGIES_DIR / f"{name}.json")
    assert strategy.name == name

    job = _job((name, "cycle_crates"), per_side=1)
    match, _, _ = sweep_trial.build_match_for_job(job)
    for _ in range(600):
        match.step(job.dt)
    assert not match.ended  # 20s of a 150s match


def test_a_full_match_scores_on_both_sides():
    """The regression that matters most: for a long time this match was
    243-73 with the same plan on both sides, because a robot could park
    on a scoring pose it was unable to rotate into and re-choose that
    same target until the buzzer (DRY_RUN_LOG.md, F1/F2). Both
    alliances producing is the cheapest possible statement that neither
    failure is back."""
    job = _job(("cycle_crates", "cycle_crates"))
    match, _, _ = sweep_trial.build_match_for_job(job)
    run_match_to_completion(match, dt=job.dt)
    metrics = extract_metrics(match)

    blue = metrics.final_scores.get("blue", 0.0)
    red = metrics.final_scores.get("red", 0.0)
    assert blue > 100.0 and red > 100.0, metrics.final_scores
    # Same plan, same field (rotated) -- a large gap means one side is
    # stuck, not out-played.
    assert abs(blue - red) < 0.5 * max(blue, red), metrics.final_scores


def test_the_field_is_the_shape_the_module_says_it_is():
    field = build_field()
    assert (field.width, field.height) == (FIELD_LENGTH, FIELD_WIDTH)
    # Seven, against REEFSCAPE's two -- the navigator and every
    # measurement built on it had only ever seen two.
    assert len(field.obstacles) == 7
