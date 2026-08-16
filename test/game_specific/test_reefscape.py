"""
Reefscape dry run: proves the two-day-after-reveal claim in
ARCHITECTURE.md by building the actual 2025 field/pieces/scoring on top
of common_sim with zero changes to common_sim itself, and running a
full pickup -> deposit -> score cycle against the real point values
from the Game Manual (V13) Table 6-2.
"""
import math

from common_sim.control.behavior import BehaviorContext, DriveToPose, RunIntake, RunManipulator, Sequence
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.robot.characteristics import RobotCharacteristics
from game_specific.reefscape.field import (
    FIELD_LENGTH,
    FIELD_WIDTH,
    REEF_HEX_APOTHEM,
    build_field,
    processor_position,
    reef_center,
)
from game_specific.reefscape.game_pieces import (
    ALGAE_RADIUS,
    ALGAE_TYPE,
    CORAL_RADIUS,
    CORAL_TYPE,
    spawn_algae,
    spawn_coral,
)
from game_specific.reefscape.scoring import REEFSCAPE_SCORING_RULES


def test_field_dimensions_match_manual():
    field = build_field()
    assert math.isclose(field.width, 690.875, abs_tol=1e-6)   # 57 ft 6-7/8 in
    assert math.isclose(field.height, 317.0, abs_tol=1e-6)    # 26 ft 5 in


def test_field_has_two_reef_obstacles_and_coral_stations():
    field = build_field()
    obstacle_names = {o.name for o in field.obstacles}
    assert obstacle_names == {"blue_reef", "red_reef"}
    station_names = {r.name for r in field.intake_locations}
    assert station_names == {
        "blue_coral_station_0", "blue_coral_station_1",
        "red_coral_station_0", "red_coral_station_1",
    }
    assert all(r.piece_type == CORAL_TYPE for r in field.intake_locations)
    assert all(r.starting_pieces == 30 for r in field.intake_locations)


def test_reef_offers_six_faces_with_all_four_levels():
    field = build_field()
    blue_faces = [r for r in field.scoring_regions if r.name.startswith("blue_reef_face")]
    assert len(blue_faces) == 6
    for face in blue_faces:
        assert face.actions == frozenset({"l1", "l2", "l3", "l4"})
        assert face.piece_types == frozenset({CORAL_TYPE})


def test_processor_and_net_regions_accept_only_algae():
    field = build_field()
    by_name = {r.name: r for r in field.scoring_regions}
    for name in ("blue_processor", "red_processor", "blue_net", "red_net"):
        assert by_name[name].piece_types == frozenset({ALGAE_TYPE})


def test_scoring_table_matches_manual_table_6_2():
    r = REEFSCAPE_SCORING_RULES
    assert r.points_for("l1", "auto") == 3.0 and r.points_for("l1", "teleop") == 2.0
    assert r.points_for("l2", "auto") == 4.0 and r.points_for("l2", "teleop") == 3.0
    assert r.points_for("l3", "auto") == 6.0 and r.points_for("l3", "teleop") == 4.0
    assert r.points_for("l4", "auto") == 7.0 and r.points_for("l4", "teleop") == 5.0
    assert r.points_for("processor", "auto") == 6.0 and r.points_for("processor", "teleop") == 6.0
    assert r.points_for("net", "auto") == 4.0 and r.points_for("net", "teleop") == 4.0


def _reefscape_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        max_speed=150.0, max_accel=400.0, max_angular_speed=6.0, max_angular_accel=20.0,
        width=28.0, length=28.0, piece_capacity=1, intake_time=0.3, intake_range=6.0,
        deposit_time=0.4,
        deposit_time_by_action={"l1": 0.3, "l2": 0.6, "l3": 1.0, "l4": 1.8},
        accepted_piece_types=frozenset({CORAL_TYPE, ALGAE_TYPE}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def test_full_coral_l4_scoring_cycle_awards_manual_point_value():
    """Pick up a CORAL out in the open, then drive to and deposit it on
    the blue REEF's -x face targeting L4 -- an autonomous-routine-shaped
    exercise of the exact field this dry run is meant to validate."""
    field = build_field()
    match = Match(field, REEFSCAPE_SCORING_RULES, MatchConfig(auto_duration=1000, teleop_duration=1000))

    center = reef_center("blue")
    face_center = (center[0] - REEF_HEX_APOTHEM, center[1])  # the reef face nearest -x
    robot_half_l = 14.0
    intake_reach = 6.0
    pickup_approach_offset = CORAL_RADIUS + robot_half_l + intake_reach - 2.0

    coral_pos = (face_center[0] - 40, face_center[1])  # out in the open, well clear of the reef and the field's own edge
    coral = spawn_coral(match, coral_pos)

    start_pose = Pose2d(coral_pos[0] - pickup_approach_offset - 20, coral_pos[1], 0)
    robot = match.add_robot(_reefscape_characteristics(), start_pose, alliance="blue")

    # Score pose: robot stops with a few inches of standoff clearance
    # from the solid REEF (bumper not touching it) -- its center still
    # lands inside the face's scoring zone from there.
    standoff = 3.0
    score_pose = Pose2d(face_center[0] - robot_half_l - standoff, face_center[1], 0)

    routine = Sequence([
        DriveToPose(Pose2d(coral_pos[0] - pickup_approach_offset, coral_pos[1], 0), position_tolerance=0.5),
        RunIntake(timeout=5.0),
        DriveToPose(score_pose, position_tolerance=1.0),
        RunManipulator("l4", timeout=5.0),
    ])

    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0)
    for _ in range(4000):
        routine.tick(ctx)
        match.step(1.0 / 60.0)
        ctx.elapsed += 1.0 / 60.0
        if coral.scored:
            break

    assert coral.scored
    assert coral.target_action == "l4"
    assert match.scores["blue"] == 7.0  # manual Table 6-2: L4 in AUTO


def test_algae_processor_scoring_cycle():
    """Pick up an ALGAE out in the open, drive to the PROCESSOR and score
    it there. The robot has to bring the side carrying its manipulator to
    the PROCESSOR opening -- a deposit only runs while that side is in
    the zone, so the drive-to-the-scoring-location leg is load-bearing
    here, not decoration."""
    field = build_field()
    match = Match(field, REEFSCAPE_SCORING_RULES, MatchConfig(auto_duration=0.0, teleop_duration=1000))

    proc_pos = processor_position("blue")
    robot_half_l = 14.0
    intake_reach = 6.0
    pickup_approach_offset = ALGAE_RADIUS + robot_half_l + intake_reach - 2.0

    # Out in the open, well clear of the PROCESSOR wall and the reef.
    algae_pos = (proc_pos[0] - 44.0, proc_pos[1] + 71.0)
    algae = spawn_algae(match, algae_pos)

    start_pose = Pose2d(algae_pos[0] - pickup_approach_offset - 20, algae_pos[1], 0)
    robot = match.add_robot(_reefscape_characteristics(deposit_time_by_action={"processor": 0.3}), start_pose)

    # Face the PROCESSOR opening (which is set into the y=0 wall) head on,
    # close enough that the front bumper -- where this robot's manipulator
    # is -- lands inside the zone, while the chassis stays clear of the wall.
    score_pose = Pose2d(proc_pos[0], proc_pos[1] + 21.0, -math.pi / 2)

    routine = Sequence([
        DriveToPose(Pose2d(algae_pos[0] - pickup_approach_offset, algae_pos[1], 0), position_tolerance=0.5),
        RunIntake(timeout=5.0),
        DriveToPose(score_pose, position_tolerance=1.0, heading_tolerance=0.05),
        RunManipulator("processor", timeout=5.0),
    ])
    ctx = BehaviorContext(robot=robot, dt=1.0 / 60.0)
    for _ in range(3000):
        routine.tick(ctx)
        match.step(1.0 / 60.0)
        ctx.elapsed += 1.0 / 60.0
        if algae.scored:
            break

    assert algae.scored
    assert match.scores["blue"] == 6.0  # manual Table 6-2: PROCESSOR in TELEOP
