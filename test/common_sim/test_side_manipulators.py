"""Per-side intake/scoring manipulators: a robot only scores through the
side that actually carries the mechanism for that piece type, and only
while that side is genuinely in position.

The zones here are deliberately *small* -- shallower than the robot's own
half-length, so the chassis center can never be inside one. Only the
mechanism side's edge ever reaches them, which is the case that
distinguishes a real side-aware check from a center-point check that
happens to work when zones are big enough to swallow the whole robot.
"""
import math

from common_sim.field.field_config import FieldConfig, Obstacle, ScoringRegion, point_in_polygon
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics, SideManipulators

WIDGET = "widget"
OTHER = "doodad"

HALF = 14.0          # robot is 28x28
CY = 100.0           # field-center y, clear of the perimeter walls
FACE_X = 200.0       # where the "structure" face sits
ZONE_DEPTH = 12.0    # shallower than HALF on purpose -- see module docstring

# Extends *outward* from the face back toward an approaching robot, the way
# a REEF face's or PROCESSOR's zone does.
ZONE = ScoringRegion(
    name="face",
    vertices=((FACE_X, CY - 15), (FACE_X, CY + 15), (FACE_X - ZONE_DEPTH, CY + 15), (FACE_X - ZONE_DEPTH, CY - 15)),
    actions=frozenset({"score"}),
    piece_types=frozenset({WIDGET}),
)

RULES = TableScoringRules({("score", "auto"): 3.0, ("score", "teleop"): 1.0})


def make_match(*, solid_structure: bool = False, regions=(ZONE,)) -> Match:
    obstacles = ()
    if solid_structure:
        # A solid body filling the space behind the face, so a robot driving
        # at the zone physically collides instead of driving through it.
        obstacles = (Obstacle(name="structure", vertices=((FACE_X, CY - 40), (FACE_X + 60, CY - 40),
                                                          (FACE_X + 60, CY + 40), (FACE_X, CY + 40))),)
    field = FieldConfig(width=400, height=200, obstacles=obstacles, scoring_regions=regions)
    return Match(field, RULES, MatchConfig(auto_duration=1000, teleop_duration=1000))


def front_scoring_characteristics(**overrides) -> RobotCharacteristics:
    """Scores WIDGET out the front only -- the layout every orientation
    test below rotates around."""
    defaults = dict(
        max_speed=150.0, max_accel=400.0, width=28.0, length=28.0,
        piece_capacity=1, intake_range=6.0, deposit_time=0.2,
        accepted_piece_types=frozenset({WIDGET}), starting_piece_count=1,
        side_manipulators={
            "front": SideManipulators(score_piece_types=frozenset({WIDGET})),
            "back": SideManipulators(intake_piece_types=frozenset({WIDGET})),
            "left": SideManipulators(),
            "right": SideManipulators(),
        },
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def add_robot_at(match: Match, x: float, y: float, heading: float, characteristics=None):
    robot = match.add_robot(characteristics or front_scoring_characteristics(), Pose2d(x, y, heading))
    if robot.held_pieces:
        # The preloaded piece's type is pulled from an unordered frozenset;
        # pin it so these tests don't depend on set iteration order.
        robot.held_pieces[0].piece_type = WIDGET
    return robot


def run_deposit(match: Match, robot, action="score", ticks=200, drive_vx=0.0):
    piece = robot.held_pieces[0]
    robot.set_deposit_active(True, action=action)
    ever_ready = False
    for _ in range(ticks):
        if drive_vx:
            robot.drive_robot_relative(1.0 / 60.0, drive_vx, 0.0, 0.0)
        if match.deposit_region_for(robot) is not None:
            ever_ready = True
        match.step(1.0 / 60.0)
        if piece.scored:
            break
    return ever_ready, piece.scored


# -- the zone really is edge-only -------------------------------------------


def test_zone_is_too_shallow_for_the_chassis_center_to_ever_enter():
    """Guards the premise of every test below: a robot whose front bumper
    is anywhere in this zone has its center outside it, so a center-point
    check could never score here."""
    match = make_match()
    robot = add_robot_at(match, FACE_X - HALF, CY, 0.0)
    assert point_in_polygon(robot.side_bumper_point("front"), ZONE.vertices) or \
        robot.side_engages_polygon("front", ZONE.vertices)
    pose = robot.pose
    assert not point_in_polygon((pose.x, pose.y), ZONE.vertices)


# -- scoring through the correct side ----------------------------------------


def test_scores_with_the_manipulator_side_in_a_zone_the_center_cannot_reach():
    match = make_match()
    robot = add_robot_at(match, FACE_X - HALF - 2.0, CY, 0.0)
    ready, scored = run_deposit(match, robot)
    assert ready and scored
    assert match.scores["blue"] == 3.0


def test_scores_while_pressed_flush_against_a_solid_structure():
    """Two solid bodies settle with a little overlap, so a robot driving
    hard into the structure ends up with its bumper edge fractionally
    *past* the zone's inner boundary. Scoring must still work there --
    having to back off the very structure being scored on is exactly the
    failure this guards."""
    match = make_match(solid_structure=True)
    robot = add_robot_at(match, FACE_X - HALF - 6.0, CY, 0.0)
    ready, scored = run_deposit(match, robot, drive_vx=60.0)
    assert ready and scored


def test_does_not_score_with_the_wrong_side_presented():
    for heading, name in [(math.pi, "back"), (math.pi / 2, "left"), (-math.pi / 2, "right")]:
        match = make_match()
        robot = add_robot_at(match, FACE_X - HALF - 2.0, CY, heading)
        ready, scored = run_deposit(match, robot)
        assert not ready, f"{name} side should not report ready to score"
        assert not scored, f"{name} side should not score"


def test_does_not_score_from_a_side_that_only_intakes():
    """Back has an intake but no scoring manipulator, so backing into the
    zone must not score even though a side *is* in position."""
    match = make_match()
    robot = add_robot_at(match, FACE_X - HALF - 2.0, CY, math.pi)
    ready, scored = run_deposit(match, robot)
    assert not ready and not scored


def test_scoring_side_follows_the_held_piece_type():
    """Two piece types scoring out of different sides -- the side that
    matters is the one configured for whatever is actually held."""
    characteristics = front_scoring_characteristics(
        accepted_piece_types=frozenset({OTHER}),
        side_manipulators={
            "front": SideManipulators(score_piece_types=frozenset({WIDGET})),
            "right": SideManipulators(score_piece_types=frozenset({OTHER})),
            "back": SideManipulators(),
            "left": SideManipulators(),
        },
    )
    match = make_match()
    robot = add_robot_at(match, FACE_X - HALF - 2.0, CY, 0.0, characteristics)
    robot.held_pieces[0].piece_type = OTHER
    assert robot.scoring_side() == "right"
    # Front is in the zone, but OTHER scores out the right side, so no deposit.
    assert match.deposit_region_for(robot) is None


# -- corner between two adjacent zones ---------------------------------------


def test_does_not_register_against_a_neighbouring_zone_at_a_corner():
    """Two zones meeting like adjacent REEF faces at a hex vertex. A robot
    centered on the seam is squarely in neither, and must not pick up
    either one just by being near both."""
    upper = ScoringRegion(
        name="upper", vertices=((FACE_X, CY + 10), (FACE_X, CY + 30),
                                (FACE_X - ZONE_DEPTH, CY + 30), (FACE_X - ZONE_DEPTH, CY + 10)),
        actions=frozenset({"score"}), piece_types=frozenset({WIDGET}),
    )
    lower = ScoringRegion(
        name="lower", vertices=((FACE_X, CY - 30), (FACE_X, CY - 10),
                                (FACE_X - ZONE_DEPTH, CY - 10), (FACE_X - ZONE_DEPTH, CY - 30)),
        actions=frozenset({"score"}), piece_types=frozenset({WIDGET}),
    )
    match = make_match(regions=(upper, lower))
    robot = add_robot_at(match, FACE_X - HALF - 2.0, CY, 0.0)
    ready, scored = run_deposit(match, robot)
    assert not ready and not scored


# -- readiness and the deposit timer stay in lockstep -------------------------


def test_deposit_timer_advances_regardless_of_readiness_but_only_scores_when_ready():
    """The deposit timer runs purely off commanded+held+action -- a robot
    can hold the deposit button anywhere and the mechanism still completes
    and releases the piece (see Robot.update_manipulator). Whether that
    release counts as a score is decided separately, from
    Match.deposit_region_for at the moment of release; a robot's
    orientation here is fixed for the whole loop, so readiness is
    constant and must match the eventual scored outcome exactly."""
    checked = 0
    for degrees in range(0, 360, 15):
        match = make_match()
        robot = add_robot_at(match, FACE_X - HALF - 2.0, CY, math.radians(degrees))
        robot.set_deposit_active(True, action="score")
        piece = robot.held_pieces[0]
        ready = match.deposit_region_for(robot) is not None
        for _ in range(120):
            before = robot.manipulator.progress
            match.step(1.0 / 60.0)
            if piece.scored or piece.held_by is None:
                break
            assert robot.manipulator.progress > before, (
                f"deposit timer should keep advancing regardless of region at {degrees} degrees"
            )
            checked += 1
        assert piece.scored == ready, (
            f"piece scored={piece.scored} but region was ready={ready} at {degrees} degrees"
        )
    assert checked > 0


def test_deposit_releases_the_piece_outside_any_zone_without_scoring():
    """A robot can still complete a deposit away from any scoring region --
    the piece drops onto the field unscored rather than the deposit
    refusing to start at all."""
    match = make_match()
    robot = add_robot_at(match, FACE_X - 120.0, CY, 0.0)
    piece = robot.held_pieces[0]
    ready, scored = run_deposit(match, robot)
    assert not ready and not scored
    assert piece.held_by is None, "deposit should still release the piece even though it never scores"


# -- intake side gating ------------------------------------------------------


def test_intake_only_captures_through_a_side_configured_for_that_type():
    match = make_match()
    characteristics = front_scoring_characteristics(starting_piece_count=0)
    robot = add_robot_at(match, 100.0, CY, 0.0, characteristics)
    # Back intakes WIDGET; front only scores it. Put a piece off the front.
    piece = match.spawn_piece(WIDGET, (100.0 + HALF + 4.0, CY))
    robot.set_intake_active(True)
    for _ in range(120):
        match.step(1.0 / 60.0)
    assert piece.held_by is None, "front has no intake, so it must not capture"

    # Same piece geometry, but presented to the back side instead.
    match2 = make_match()
    robot2 = add_robot_at(match2, 100.0, CY, math.pi, front_scoring_characteristics(starting_piece_count=0))
    piece2 = match2.spawn_piece(WIDGET, (100.0 + HALF + 4.0, CY))
    robot2.set_intake_active(True)
    for _ in range(120):
        match2.step(1.0 / 60.0)
    assert piece2.held_by is robot2, "back has the intake, so it must capture"
