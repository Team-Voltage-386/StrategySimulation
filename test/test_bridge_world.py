"""Tests for the bridge's world-state reader and its transcribed arena.

Two different kinds of check live here, and they fail for different reasons.

The **decoder** tests are ordinary: WPILib's struct encoding is a fixed
layout and either it is read correctly or it is not. These exist because
`bridge.robot_state` decodes poses by hand rather than through wpimath,
to keep pyntcore installable next to pymunk, and hand-rolled binary
parsing is exactly the thing that works on the happy path and truncates
silently on the other one.

The **arena** tests are unusual, and worth explaining. `bridge/arena.py`
is a transcription of maple-sim's own field definition, so its numbers
cannot be verified here -- only against the running simulator, which
`world_state.check_geometry` does on every connection. What these tests
guard is the *shape* of the transcription: that the field is the size it
should be, that the parts fit together, that the deliberate reproduction
of a maple-sim bug is still deliberate, and that the whole thing passes
sparky-sim's own field validator. A field the validator rejects produces
failures that belong to the field rather than to the robot code, which is
the most expensive kind of false positive an overnight campaign can have.

No JVM, no robot project, and no pyntcore: both modules import without
NetworkTables for this reason.
"""
from __future__ import annotations

import math
import struct

import pytest

from bridge import arena
from bridge import world_state as ws
from bridge.robot_state import Pose2d, Pose3d
from common_sim.field.validation import validate_field

# maple-sim's DriveTrainSimulationConfig in SimContainer.java.
ROBOT_SIZE_IN = 30.0


def _pose3d_bytes(x: float, y: float, z: float) -> bytes:
    """A Pose3d as WPILib writes it: Translation3d then a unit quaternion."""
    return struct.pack("<ddddddd", x, y, z, 1.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# struct decoding
# ---------------------------------------------------------------------------


def test_pose3d_reads_translation_and_drops_rotation():
    pose = Pose3d.decode(_pose3d_bytes(1.5, -2.25, 0.075))
    assert (pose.x, pose.y, pose.z) == (1.5, -2.25, 0.075)


def test_pose3d_array_is_elements_back_to_back_with_no_header():
    raw = _pose3d_bytes(1.0, 2.0, 3.0) + _pose3d_bytes(4.0, 5.0, 6.0)
    poses = Pose3d.decode_array(raw)
    assert [(p.x, p.y, p.z) for p in poses] == [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]


def test_an_empty_pose3d_array_is_no_pieces_not_an_error():
    """Legitimate once every piece has been collected."""
    assert Pose3d.decode_array(b"") == []


def test_a_partial_pose3d_array_raises_rather_than_truncating():
    """A short read would come back as a plausible list one element shy.

    Silently dropping the tail is the failure mode worth ruling out: the
    caller counts pieces, and a count that is quietly wrong is worse than
    one that is missing.
    """
    with pytest.raises(ValueError, match="whole number"):
        Pose3d.decode_array(_pose3d_bytes(1.0, 2.0, 3.0) + b"\x00" * 8)


def test_pose3d_rejects_a_pose2d_sized_payload():
    """56 bytes, not 24 -- the two topics are otherwise easy to swap."""
    with pytest.raises(ValueError, match="56 bytes"):
        Pose3d.decode(struct.pack("<ddd", 1.0, 2.0, 3.0))


# ---------------------------------------------------------------------------
# the arena, as transcribed
# ---------------------------------------------------------------------------


def test_the_field_is_the_2026_field_and_not_reefscape():
    """651 x 317 inches. REEFSCAPE was 690.875 long, so a season-old
    constant copied by accident shows up here rather than as a navigator
    that plans through a wall."""
    assert arena.FIELD_LENGTH == pytest.approx(651.22, abs=0.01)
    assert arena.FIELD_WIDTH == pytest.approx(317.01, abs=0.01)


def test_both_hubs_open_toward_midfield():
    """A robot approaching from behind is approaching a wall, so which
    face carries the goal decides every shooting approach on the field."""
    blue, red = arena.hub_centre("blue"), arena.hub_centre("red")
    assert arena.goal_face_x("blue") > blue[0]
    assert arena.goal_face_x("red") < red[0]
    assert blue[0] < arena.FIELD_LENGTH / 2 < red[0]


def test_the_hubs_leave_one_gap_at_each_end_and_a_robot_fits():
    """The single most consequential fact about navigating this field.

    `SimulatedArena.getInstance()` builds the arena with ramp colliders,
    making each HUB a 217-inch wall across a 317-inch field. The two gaps
    it leaves are the only ways past, and they are about 50 inches wide
    for a robot that needs 42.4 to turn in. That is passable and tight,
    which is the combination that produced a campaign's worth of wedged
    matches before anyone had read this geometry.
    """
    hub = next(o for o in arena.build_obstacles() if o.name == "blue HUB")
    low = min(v[1] for v in hub.vertices)
    high = max(v[1] for v in hub.vertices)
    diagonal = ROBOT_SIZE_IN * 2 ** 0.5

    assert low == pytest.approx(50.3, abs=0.1)
    assert arena.FIELD_WIDTH - high == pytest.approx(49.7, abs=0.1)
    for gap in (low, arena.FIELD_WIDTH - high):
        assert diagonal < gap < 2 * diagonal, "the gap should be tight but passable"


def test_the_arena_reproduces_maple_sims_missing_fourth_trench_wall():
    """maple-sim 0.4.0-beta places three trench walls, not four.

    `RebuiltFieldObstaclesMap` repeats the (-x, -y) corner verbatim in its
    fourth `addRectangularObstacle` call, so the (+x, +y) corner has no
    wall. That is a bug upstream, and reproducing it is the point: this
    file describes the arena the robot is really driving in, and steering
    around an obstacle the physics does not contain is wrong in the
    direction that is hardest to notice.

    If this test starts failing after a maple-sim upgrade, the bug was
    fixed -- flip `faithful_trenches` rather than deleting the check.
    """
    faithful = arena.trench_wall_centres()
    intended = arena.trench_wall_centres(faithful=False)

    assert len(faithful) == 3 and len(intended) == 4
    missing = set(intended) - set(faithful)
    assert len(missing) == 1
    corner = missing.pop()
    assert corner[0] > arena.FIELD_LENGTH / 2 and corner[1] > arena.FIELD_WIDTH / 2


def test_the_trench_walls_end_exactly_where_the_hub_ramps_do():
    """The trench offset and the HUB half-length are the same 108.5 inches.

    Not a coincidence -- the ramps run out to the trench walls -- and it
    is the cross-check that both transcriptions are right, since the two
    numbers are built from unrelated arithmetic in maple-sim's source
    (73 + 47/2 + 6 on one side, 217/2 on the other).

    It also means that with ramps on, which is what
    `SimulatedArena.getInstance()` gives, every trench wall sits within a
    fiftieth of an inch of a HUB's y extent and adds nothing a navigator
    would notice. Which is why the missing fourth wall changes almost
    nothing in practice, and why reproducing it is safe.
    """
    assert arena.TRENCH_OFFSET_Y_IN + arena.TRENCH_SIZE_IN[1] / 2 == arena.HUB_SIZE_IN[1] / 2

    obstacles = {o.name: o for o in arena.build_obstacles()}
    hub = obstacles["blue HUB"]
    hub_low, hub_high = min(v[1] for v in hub.vertices), max(v[1] for v in hub.vertices)

    for name, obstacle in obstacles.items():
        if not name.startswith("TRENCH"):
            continue
        low = min(v[1] for v in obstacle.vertices)
        high = max(v[1] for v in obstacle.vertices)
        assert low == pytest.approx(hub_low, abs=0.05) or high == pytest.approx(hub_high, abs=0.05)


def test_a_scoring_region_does_not_share_a_name_with_a_structure():
    """`deposit_region_for` and every bench resolve features by name, so a
    duplicate silently resolves to whichever was declared first. The
    field validator catches this; the test is here so it is caught before
    a JVM has been started."""
    field = arena.build_arena()
    names = [o.name for o in field.obstacles] + [r.name for r in field.scoring_regions]
    assert len(names) == len(set(names)), sorted(names)


def test_a_robot_only_scores_from_inside_its_own_alliance_zone():
    """`RobotContainer.isInAllianceArea`, transcribed. Blue is
    `x < 4.625594`, red is `x > 11.915394`."""
    bound = arena._in(4.625594)
    assert arena.in_alliance_zone(bound - 1, "blue")
    assert not arena.in_alliance_zone(bound + 1, "blue")

    bound = arena._in(11.915394)
    assert arena.in_alliance_zone(bound + 1, "red")
    assert not arena.in_alliance_zone(bound - 1, "red")


def test_the_alliance_boundary_is_the_trench_wall_line():
    """The two constants come from unrelated arithmetic in two different
    repos -- `8.27 - (120 + 47/2) inches` in maple-sim's obstacle map,
    and a bare `4.625594` in the robot code -- so their agreeing to a
    millimetre is a real cross-check that both were read off the same
    field."""
    for alliance, sign in (("blue", -1), ("red", +1)):
        trench = arena._in(arena.TRENCH_CENTRE_M[0]) + sign * arena.TRENCH_OFFSET_X_IN
        assert trench == pytest.approx(arena._in(arena.ALLIANCE_ZONE_BOUND_M[alliance]), abs=0.05)


def test_the_goal_regions_are_inside_the_alliance_zone_they_belong_to():
    """The rule that makes shooting *score* rather than pass.

    `Turret.setTarget` aims at the HUB only while the robot is in its own
    zone; outside it, the turret retargets a corner of that zone and
    throws the fuel back instead. The shot happens either way, the ball
    lands on the field either way, and nothing distinguishes the two from
    outside -- so a scoring region placed at the goal mouth (which faces
    *midfield*, outside the zone) produces a robot that shoots
    beautifully and scores nothing. Sixty-five shots went that way.
    """
    for region in arena.build_arena().scoring_regions:
        alliance = region.alliance
        for x, _ in region.vertices:
            assert arena.in_alliance_zone(x, alliance), (
                f"{region.name} reaches x={x:.1f} in, outside the {alliance} zone"
            )


def test_the_goal_regions_sit_behind_the_hub_not_at_its_mouth():
    """The counter-intuitive consequence: the mouths open toward
    midfield, and a robot shoots from the other side, over the
    structure. The goal is 1.57 m up and the turret has a pitch."""
    for alliance in ("blue", "red"):
        assert arena.shooting_face_x(alliance) != arena.goal_face_x(alliance)
        region = next(
            r for r in arena.build_arena().scoring_regions if r.name == f"{alliance} GOAL"
        )
        hub_x = arena.hub_centre(alliance)[0]
        midfield = arena.FIELD_LENGTH / 2
        for x, _ in region.vertices:
            assert abs(x - midfield) > abs(hub_x - midfield), "the region is on the far side"


def test_the_goal_regions_stand_off_the_structure_they_shoot_at():
    """A scoring region is tested against the robot's *centre*, so one
    reaching the goal face describes robot positions half inside the HUB."""
    field = arena.build_arena()
    for alliance in ("blue", "red"):
        region = next(r for r in field.scoring_regions if r.name == f"{alliance} GOAL")
        hub = next(o for o in field.obstacles if o.name == f"{alliance} HUB")
        xs = [v[0] for v in region.vertices]
        hub_xs = [v[0] for v in hub.vertices]
        assert min(xs) >= max(hub_xs) or max(xs) <= min(hub_xs)


def test_the_goal_regions_are_not_sized_to_the_goal_mouth():
    """The region was 47 inches tall once, because that is the width of
    the goal mouth. The mouth's width says nothing whatever about where a
    robot stands: the turret rotates, so being off the HUB's axis in y
    costs nothing, and a region a fraction of the legal area is a robot
    that drives past good scoring positions to reach a nominated one.

    Not the whole field width either, which was the overcorrection. The
    range term binds, and the y freedom this wins is the arc it allows at
    a scoring distance -- about three times the mouth, not seven."""
    mouth = 2 * arena._in(arena.GOAL_RADIUS_M)
    for region in arena.build_arena().scoring_regions:
        ys = [v[1] for v in region.vertices]
        assert max(ys) - min(ys) > 3 * mouth, region.name


def test_the_scoring_rule_is_a_radius_and_the_declared_range_is_not_it():
    """Two ranges live in this file and they are not the same number.

    `SHOT_MIN/MAX_DISTANCE_M` is what `Scoring.java` declares, and
    `ShotCalculation` computes `isValid` from it and then nothing reads
    the answer -- so the robot shoots outside it and the interpolating
    maps clamp. It is transcribed for the record.

    `SCORING_RANGE_M` is where a shot has been *measured* to arrive, and
    it is far narrower. Three forward models said the declared range all
    scored; live runs said 22-of-24 at 2.04 m and about one in ten at
    2.5 m. The rule follows the measurement.
    """
    near, far = arena.SCORING_RANGE_M
    assert far < arena.SHOT_MAX_DISTANCE_M, (
        "the measured band has grown past the declared one, which would mean the shot "
        "table is now the binding constraint -- re-read both before believing it"
    )
    assert near < arena.SHOT_MIN_DISTANCE_M, "the near edge is the robot's bumpers, not the table"

    # 2.04 m scored 22 of 24; 2.53 m scored 2 of 42. The rule has to
    # separate them or it is not carrying the measurement.
    hub_x, hub_y = arena.hub_centre("blue")
    assert arena.can_score_from(hub_x - arena._in(2.04), hub_y, "blue")
    assert not arena.can_score_from(hub_x - arena._in(2.53), hub_y, "blue")


def test_the_transcribed_arena_passes_the_field_validator():
    """Notes are allowed -- the tower poles really are wall furniture --
    but nothing error-level. A campaign run against a field the validator
    rejects reports failures that belong to the field, not the code."""
    problems = validate_field(
        arena.build_arena(), robot_width=ROBOT_SIZE_IN, robot_length=ROBOT_SIZE_IN
    )
    errors = [p for p in problems if p.severity == "error"]
    assert not errors, [str(p) for p in errors]


def test_fuel_is_registered_so_the_validator_can_size_it():
    """A 15 cm ball, which is why the spawn grid steps by 5.99 inches."""
    from common_sim.field.game_piece import has_piece_spec, piece_spec

    assert has_piece_spec(arena.PIECE_TYPE)
    assert piece_spec(arena.PIECE_TYPE).radius == pytest.approx(2.95, abs=0.01)


# ---------------------------------------------------------------------------
# reconciling the transcription with the live arena
# ---------------------------------------------------------------------------


def test_a_matching_pose_passes_the_geometry_check():
    check = ws._compare("blue HUB", (4.5974, 4.034536), Pose3d(4.5974, 4.034536, 1.5748))
    assert check.ok and check.error == pytest.approx(0.0)


def test_a_pose_off_by_more_than_a_millimetre_fails():
    """Tight on purpose. The question is not "is this good enough to
    navigate" -- a centimetre would be -- it is "did I transcribe the same
    field", and a loose tolerance passes a constant copied from the wrong
    season."""
    check = ws._compare("blue HUB", (4.5974, 4.034536), Pose3d(4.5974, 4.0400, 1.5748))
    assert not check.ok
    assert "MISMATCH" in str(check)


def test_an_unpublished_pose_is_not_silently_a_pass():
    """None means the check could not run, which is not the same as
    running and agreeing -- the distinction the oracles app already had
    to learn once."""
    check = ws._compare("blue HUB", (4.5974, 4.034536), None)
    assert not check.ok
    assert check.error is None
    assert "cannot check" in str(check)


# ---------------------------------------------------------------------------
# the snapshot
# ---------------------------------------------------------------------------


def _world(**overrides) -> ws.WorldState:
    defaults = dict(
        robot=Pose2d(8.0, 4.0, 0.0),
        fuel=(),
        held=0,
        match_clock=0.0,
        phase_clock=25.0,
        hub_active={"blue": True, "red": False},
        score={"blue": 0.0, "red": 0.0},
    )
    return ws.WorldState(**{**defaults, **overrides})


def test_nearest_fuel_measures_from_the_robot():
    world = _world(fuel=(Pose3d(9.0, 4.0, 0.075), Pose3d(8.2, 4.0, 0.075)))
    assert world.nearest_fuel().x == pytest.approx(8.2)


def test_nearest_fuel_is_none_on_an_empty_field_rather_than_raising():
    assert _world().nearest_fuel() is None


def test_nearest_fuel_is_none_when_the_robot_has_no_pose():
    assert _world(robot=None, fuel=(Pose3d(1.0, 1.0, 0.0),)).nearest_fuel() is None


def test_exactly_one_hub_is_addressed_per_alliance():
    """Both alliances' active flags come from separate tables whose
    capitalisation does not match ("Red Alliance", "blue Alliance"). That
    is maple-sim's, not a typo here, and normalising it breaks the read."""
    assert set(ws.HUB_ACTIVE) == set(ws.ALLIANCE_TABLE) == {"blue", "red"}
    assert ws.ALLIANCE_TABLE["blue"].endswith("blue Alliance")
    assert ws.ALLIANCE_TABLE["red"].endswith("Red Alliance")


def test_our_hub_active_reads_the_side_it_was_asked_about():
    world = _world(hub_active={"blue": True, "red": False})
    assert world.our_hub_active("blue") and not world.our_hub_active("red")
