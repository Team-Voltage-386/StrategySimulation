"""
field/validation.py tests.

Each check gets a field that trips it and the clean field beside it, for
the reason `test_import_contract.py` gives: a check that has never been
seen to fail is not known to work. The whole value of this module is on
game-reveal day, when nobody will have time to wonder whether it is
actually looking.
"""
import math

from common_sim.field.field_config import (
    EmitterRegion, FieldConfig, IntakeLocation, Obstacle, ScoringRegion,
)
from common_sim.field.game_piece import GamePieceSpec, register_piece_spec
from common_sim.field.validation import (
    ERROR, NOTE, WARNING, describe_problems, open_reference_point, validate_field,
)
from common_sim.match.scoring import TableScoringRules

WIDGET = "validation-widget"
register_piece_spec(WIDGET, GamePieceSpec(radius=3.0, mass=0.3, color="white"))

RULES = TableScoringRules({("stow", "auto"): 3.0, ("stow", "teleop"): 2.0})

GOAL = ScoringRegion(
    name="goal", vertices=((200, 90), (240, 90), (240, 130), (200, 130)),
    actions=frozenset({"stow"}), piece_types=frozenset({WIDGET}),
)
FEEDER = IntakeLocation(
    name="feeder", vertices=((20, 90), (60, 90), (60, 130), (20, 130)), piece_type=WIDGET,
)


def clean_field(**kwargs) -> FieldConfig:
    kwargs.setdefault("scoring_regions", (GOAL,))
    kwargs.setdefault("intake_locations", (FEEDER,))
    return FieldConfig(width=400, height=220, **kwargs)


def check(field, **kwargs):
    kwargs.setdefault("robot_width", 28.0)
    kwargs.setdefault("robot_length", 28.0)
    kwargs.setdefault("scoring_rules", RULES)
    return validate_field(field, **kwargs)


def problems_of(problems, severity):
    return [p for p in problems if p.severity == severity]


def test_a_well_formed_field_is_clean():
    assert check(clean_field()) == []


def test_duplicate_names_are_an_error():
    twin = Obstacle(name="goal", vertices=((300, 20), (340, 20), (340, 60), (300, 60)))
    problems = problems_of(check(clean_field(obstacles=(twin,))), ERROR)
    assert [p.where for p in problems] == ["goal"]
    assert "name is used by both" in problems[0].problem


def test_a_degenerate_polygon_is_an_error():
    sliver = ScoringRegion(
        name="sliver", vertices=((100, 100), (140, 100), (180, 100)),
        actions=frozenset({"stow"}), piece_types=frozenset({WIDGET}),
    )
    problems = problems_of(check(clean_field(scoring_regions=(GOAL, sliver))), ERROR)
    assert any("encloses no area" in p.problem for p in problems)


def test_vertices_outside_the_field_are_a_warning():
    """Not an error: a real game does this on purpose. REEFSCAPE's
    corner CORAL STATIONs are three quarters outside the field, and play
    fine, because a robot only needs to reach the part that is in."""
    spilling = IntakeLocation(
        name="corner", vertices=((-10, -10), (30, -10), (30, 30), (-10, 30)), piece_type=WIDGET,
    )
    problems = problems_of(check(clean_field(intake_locations=(spilling,))), WARNING)
    assert [p.where for p in problems] == ["corner"]


def test_an_unregistered_piece_type_is_an_error():
    orphan = ScoringRegion(
        name="orphan", vertices=((300, 90), (340, 90), (340, 130), (300, 130)),
        actions=frozenset({"stow"}), piece_types=frozenset({"nobody-registered-this"}),
    )
    problems = problems_of(check(clean_field(scoring_regions=(GOAL, orphan))), ERROR)
    assert any("no registered GamePieceSpec" in p.problem for p in problems)


def test_an_action_worth_nothing_is_a_warning():
    """The failure this catches is a typo on either side of the table.
    `points_for` returns 0.0 for an unknown action rather than raising,
    so the region simply never gets chosen and nothing says why."""
    mistyped = ScoringRegion(
        name="mistyped", vertices=((300, 90), (340, 90), (340, 130), (300, 130)),
        actions=frozenset({"stowe"}), piece_types=frozenset({WIDGET}),
    )
    problems = problems_of(check(clean_field(scoring_regions=(GOAL, mistyped))), WARNING)
    assert any("worth 0 points in every phase" in p.problem for p in problems)


def test_a_capacity_on_an_action_the_region_does_not_offer_is_an_error():
    mismatched = ScoringRegion(
        name="mismatched", vertices=((300, 90), (340, 90), (340, 130), (300, 130)),
        actions=frozenset({"stow"}), piece_types=frozenset({WIDGET}),
        capacity_by_action={"hoist": 2},
    )
    problems = problems_of(check(clean_field(scoring_regions=(GOAL, mismatched))), ERROR)
    assert any("which this region does not offer" in p.problem for p in problems)


def test_an_emitter_linked_to_a_missing_station_is_an_error():
    emitter = EmitterRegion(
        name="drop", vertices=((150, 20), (170, 20), (170, 40), (150, 40)),
        piece_type=WIDGET, linked_collection_region="no-such-feeder",
    )
    problems = problems_of(check(clean_field(emitter_regions=(emitter,))), ERROR)
    assert any("is not an intake location" in p.problem for p in problems)


def test_an_emitter_that_both_links_and_carries_stock_is_an_error():
    emitter = EmitterRegion(
        name="drop", vertices=((150, 20), (170, 20), (170, 40), (150, 40)),
        piece_type=WIDGET, linked_collection_region="feeder", initial_capacity=5,
    )
    problems = problems_of(check(clean_field(emitter_regions=(emitter,))), ERROR)
    assert any("mutually exclusive" in p.hint for p in problems)


def test_a_region_walled_in_on_every_side_is_an_error():
    """The case the whole module exists for: geometry that reads fine on
    a drawing and cannot be worked by a real chassis. Nothing at runtime
    complains -- `plan_path` hands back a straight line, the robot drives
    into the wall around it and pushes, and the symptom is a strategy
    that scores badly."""
    boxed = ScoringRegion(
        name="boxed", vertices=((190, 100), (210, 100), (210, 120), (190, 120)),
        actions=frozenset({"stow"}), piece_types=frozenset({WIDGET}),
    )
    # The pocket left in the middle is 20x20 -- smaller than the robot,
    # so there is nowhere legal to work the region from at any bearing.
    walls = (
        Obstacle(name="w_left", vertices=((160, 70), (190, 70), (190, 150), (160, 150))),
        Obstacle(name="w_right", vertices=((210, 70), (240, 70), (240, 150), (210, 150))),
        Obstacle(name="w_top", vertices=((160, 120), (240, 120), (240, 150), (160, 150))),
        Obstacle(name="w_bottom", vertices=((160, 70), (240, 70), (240, 100), (160, 100))),
    )
    problems = problems_of(check(clean_field(scoring_regions=(boxed,), obstacles=walls)), ERROR)
    boxed_problems = [p for p in problems if p.where == "boxed"]
    assert [p.problem for p in boxed_problems] and "no pose exists" in boxed_problems[0].problem
    # This field also reports the feeder as unroutable, and that report is
    # correct rather than collateral: the four walls sit closer together
    # than twice a robot radius, so their inflated outlines overlap, and
    # `plan_path` will hand back a route straight through them. See
    # DRY_RUN_LOG.md, F9 -- the checker is reporting what the navigator
    # actually does, which is the only thing it should ever report.
    assert {p.where for p in problems} == {"boxed", "feeder"}


def test_a_gap_narrower_than_the_robot_is_a_note():
    """A pinch point is legal geometry, so this is a NOTE. It earns its
    place because it is the shape of a mistake that is very hard to see:
    a pocket that looks open, is closed to a real chassis, and has
    something a robot wants on the far side. The SALVAGE dry run had
    exactly one, 38in against a 39.6in diagonal."""
    pillar = Obstacle(name="pillar", vertices=((100, 190), (140, 190), (140, 210), (100, 210)))
    problems = problems_of(check(clean_field(obstacles=(pillar,))), NOTE)
    assert [p.where for p in problems] == ["pillar"]
    assert "less than a" in problems[0].problem
    # 220 - 210 = 10in of headroom against a 39.6in diagonal
    assert "10.0in" in problems[0].problem


def test_a_wide_gap_is_not_reported():
    pillar = Obstacle(name="pillar", vertices=((100, 120), (140, 120), (140, 160), (100, 160)))
    assert problems_of(check(clean_field(obstacles=(pillar,))), NOTE) == []


def test_the_route_origin_avoids_a_structure_on_the_field_centre():
    """SALVAGE puts its depot on the exact centre of the field, which
    made the first run of this check report every single feature as
    unroutable -- the route started inside an obstacle, so of course it
    never left one."""
    middle = Obstacle(name="middle", vertices=((160, 70), (240, 70), (240, 150), (160, 150)))
    origin = open_reference_point(clean_field(obstacles=(middle,)), 28.0, 28.0)
    assert origin is not None
    assert not (160 <= origin[0] <= 240 and 70 <= origin[1] <= 150)


def test_describe_problems_orders_by_severity():
    twin = Obstacle(name="goal", vertices=((300, 20), (340, 20), (340, 60), (300, 60)))
    text = describe_problems(check(clean_field(obstacles=(twin,))))
    assert text.startswith("error:")


def test_describe_problems_is_empty_for_a_clean_field():
    assert describe_problems(check(clean_field())) == ""
