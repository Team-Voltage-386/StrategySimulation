"""
What the sim does with an alliance whose robots are not interchangeable.

The reference robot (`build_characteristics`) has one front side that
intakes and scores everything, so nothing in it can tell a side apart
from a source. These tests drive the two lopsided archetypes beside it --
`build_algae_sweeper_characteristics` and
`build_station_cycler_characteristics` -- because the interesting
REEFSCAPE questions only appear once a robot's two jobs point in
different directions, or once "can pick up ALGAE" stops implying "can
clear a REEF face".

The sharp one is that last distinction. REEF ALGAE is staged as an
IntakeLocation, not a loose GamePiece, so a robot restricted to
`intake_source="field"` can sit in the staging zone all match and never
unblock anything -- it looks like an ALGAE specialist and cannot do the
one ALGAE job that gates CORAL points.
"""
import math
import random

from common_sim.control.world_view import scoring_slots_for_type
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig
from common_sim.robot.characteristics import SIDES
from common_sim.robot.robot import SIDE_OUTWARD
from game_specific.reefscape.field import (
    REEF_HEX_APOTHEM, build_field, coral_station_positions, reef_algae_blocked_level,
    reef_algae_staging_positions, reef_center,
)
from game_specific.reefscape.game_pieces import ALGAE_TYPE, CORAL_TYPE, spawn_algae
from game_specific.reefscape.robot import (
    build_algae_sweeper_characteristics, build_characteristics, build_station_cycler_characteristics,
)
from game_specific.reefscape.scoring import REEFSCAPE_SCORING_RULES


def _match() -> Match:
    # Long phases: these tests park a robot and wait on an intake timer,
    # and a match that ends underneath them would look like a capability
    # failure rather than the clock running out.
    return Match(build_field(), REEFSCAPE_SCORING_RULES,
                 MatchConfig(auto_duration=999.0, teleop_duration=999.0))


def _pose_presenting(side: str, target, approach, standoff: float = 20.0) -> Pose2d:
    """A pose standing `standoff` inches from `target`, rotated so that
    `side` faces it, having approached along the unit vector `approach`
    (which points from the robot toward the target).

    Worth spelling out rather than hand-placing per test: getting this
    wrong puts the robot inside the REEF and the resulting empty gripper
    looks exactly like a capability the archetype does not have."""
    side_angle = math.atan2(*reversed(SIDE_OUTWARD[side]))
    heading = math.atan2(approach[1], approach[0]) - side_angle
    return Pose2d(target[0] - approach[0] * standoff, target[1] - approach[1] * standoff, heading)


def _run(match: Match, seconds: float = 4.0) -> None:
    for _ in range(int(seconds * 60)):
        match.step(1.0 / 60.0)


def test_the_three_archetypes_differ_in_side_and_source():
    """One table, so a reader can see what separates them without
    reverse-engineering three constructors."""
    def profile(characteristics, piece_type):
        return (
            tuple(s for s in SIDES if characteristics.side_intake_accepts(s, piece_type, source="field")),
            tuple(s for s in SIDES if characteristics.side_intake_accepts(s, piece_type, source="station")),
            tuple(s for s in SIDES if characteristics.side_score_accepts(s, piece_type)),
        )

    reference = build_characteristics()
    sweeper = build_algae_sweeper_characteristics()
    cycler = build_station_cycler_characteristics()

    # (from floor, from station, scores)
    assert profile(reference, CORAL_TYPE) == (("front",), ("front",), ("front",))
    assert profile(reference, ALGAE_TYPE) == (("front",), ("front",), ("front",))

    assert profile(sweeper, CORAL_TYPE) == ((), (), ())
    assert profile(sweeper, ALGAE_TYPE) == (("right",), (), ("right",))

    # The cycler's two CORAL jobs point opposite ways: in through the
    # back, out through the front.
    assert profile(cycler, CORAL_TYPE) == ((), ("back",), ("front",))
    assert profile(cycler, ALGAE_TYPE) == (("front",), ("front",), ("front",))


def test_sweeper_takes_a_loose_algae_only_off_its_right_side():
    for side in SIDES:
        match = _match()
        robot = match.add_robot(build_algae_sweeper_characteristics(), Pose2d(300, 150, 0.0), alliance="blue")
        outward = SIDE_OUTWARD[side]
        spawn_algae(match, (300 + outward[0] * 18.0, 150 + outward[1] * 18.0))
        robot.set_intake_active(True)
        _run(match)
        held = [p.piece_type for p in robot.held_pieces]
        assert held == ([ALGAE_TYPE] if side == "right" else []), f"side {side!r} held {held}"


def test_sweeper_cannot_clear_a_reef_gate_however_long_it_waits():
    """"Floor pickup only" is not a flavour note in this game: the staged
    REEF ALGAE is an IntakeLocation, so a field-only intake never sees
    it, and the CORAL levels behind it stay blocked all match."""
    match = _match()
    face = next(r for r in match.field.scoring_regions if r.name == "blue_reef_face_0")
    algae_zone = next(s for s in match.field.intake_locations if s.name == "blue_reef_algae_0")
    blocked_level = reef_algae_blocked_level(0)

    target = reef_algae_staging_positions("blue")[0]
    robot = match.add_robot(
        build_algae_sweeper_characteristics(),
        # Face 0's normal is +x, so the zone is approached from +x inward.
        _pose_presenting("right", target, approach=(-1.0, 0.0)),
        alliance="blue",
    )
    robot.set_intake_active(True)
    _run(match, seconds=10.0)

    assert robot.nearby_station() is None, "a field-only intake must not register the staging zone"
    assert robot.held_pieces == []
    assert match.station_supply[algae_zone] == 1
    assert match.region_blocked(face, blocked_level)


def test_cycler_clears_a_reef_gate_with_its_front():
    """The same zone, the same approach, a robot whose front is wired for
    ALGAE from either source -- and the CORAL level behind it opens."""
    match = _match()
    face = next(r for r in match.field.scoring_regions if r.name == "blue_reef_face_0")
    algae_zone = next(s for s in match.field.intake_locations if s.name == "blue_reef_algae_0")
    blocked_level = reef_algae_blocked_level(0)
    assert match.region_blocked(face, blocked_level)

    target = reef_algae_staging_positions("blue")[0]
    robot = match.add_robot(
        build_station_cycler_characteristics(),
        _pose_presenting("front", target, approach=(-1.0, 0.0)),
        alliance="blue",
    )
    robot.set_intake_active(True)
    _run(match, seconds=10.0)

    assert [p.piece_type for p in robot.held_pieces] == [ALGAE_TYPE]
    assert match.station_supply[algae_zone] == 0
    assert not match.region_blocked(face, blocked_level)
    # L4 was never gated, and clearing the ALGAE must not have touched it.
    assert not match.region_blocked(face, "l4")


def test_cycler_loads_coral_only_through_its_back():
    """Its intake side and its scoring side are opposite ends of the
    robot, so a CORAL cycle costs it a turnaround the reference robot
    never pays. Same pose, same station -- only the wiring differs."""
    station = coral_station_positions("blue")[0]
    approach = (-1.0, 0.0)

    def load(build, side):
        match = _match()
        robot = match.add_robot(build(), _pose_presenting(side, station, approach, standoff=22.0), alliance="blue")
        robot.set_intake_active(True)
        _run(match)
        return [p.piece_type for p in robot.held_pieces]

    assert load(build_station_cycler_characteristics, "back") == [CORAL_TYPE]
    assert load(build_station_cycler_characteristics, "front") == []
    # Not the geometry -- the reference robot loads from the same spot.
    assert load(build_characteristics, "front") == [CORAL_TYPE]


def test_only_robots_wired_to_score_coral_are_offered_reef_slots():
    """world_view is where a strategy finds out what it may attempt, so
    the sweeper's inability to score CORAL has to show up there rather
    than as a wasted trip to a REEF face."""
    match = _match()
    pose = Pose2d(300, 150, 0.0)
    sweeper = match.add_robot(build_algae_sweeper_characteristics(), pose, alliance="blue")
    cycler = match.add_robot(build_station_cycler_characteristics(), Pose2d(300, 200, 0.0), alliance="blue")

    assert scoring_slots_for_type(match, sweeper, CORAL_TYPE) == []
    assert {a for _, a in scoring_slots_for_type(match, sweeper, ALGAE_TYPE)} == {"processor", "net"}
    assert {a for _, a in scoring_slots_for_type(match, cycler, CORAL_TYPE)} == {"l1", "l2", "l3", "l4"}


def test_one_alliance_of_each_archetype_scores_a_level_no_one_robot_could_reach():
    """The whole point of the gate, on one alliance, in order.

    All three archetypes are on the field together at the blue REEF's -x
    face, which Figure 6-3 stages at L3. The CORAL scorer cannot open the
    face and the ALGAE sweeper cannot either -- for different reasons --
    so L3 there is worth nothing until the cycler does a chore whose
    payoff it does not collect. The robot that banks the points never
    touches an ALGAE.

    Friendly collisions are off: three robots crowding one REEF face is a
    traffic question, and this test is about which robot is *allowed* to
    do what. Phases are driven by hand rather than through a strategy so
    the ordering is the assertion, not an emergent outcome.

    The scorer's deposit is made deterministic rather than left on the
    reference robot's real numbers. `build_characteristics` puts L3 at
    0.90 reliability, and Match's default RNG is unseeded -- so the
    honest version of this test fails one run in ten on a dice roll that
    has nothing to do with the gate it is checking. Reliability 1.0
    short-circuits before drawing at all (see _roll_scoring_success), so
    this also keeps the test from consuming an RNG draw."""
    match = Match(
        build_field(), REEFSCAPE_SCORING_RULES,
        MatchConfig(auto_duration=0.0, teleop_duration=1000.0, disable_friendly_collisions=True),
        rng=random.Random(0),
    )
    face_index = 3  # normal (-1, 0): the -x face, approached from open floor
    face = next(r for r in match.field.scoring_regions if r.name == f"blue_reef_face_{face_index}")
    algae_zone = next(s for s in match.field.intake_locations if s.name == f"blue_reef_algae_{face_index}")
    gated_level = reef_algae_blocked_level(face_index)
    assert gated_level == "l3"

    target = reef_algae_staging_positions("blue")[face_index]
    approach = (1.0, 0.0)  # this face is reached from -x, driving +x

    # The scorer: parked at the face holding a preloaded CORAL, intake
    # off, so it can never clear anything for itself.
    face_center = (reef_center("blue")[0] - REEF_HEX_APOTHEM, reef_center("blue")[1])
    scorer = match.add_robot(
        build_characteristics(
            starting_piece_count=1, preload_piece_type=CORAL_TYPE,
            scoring_reliability_by_action={},  # see the docstring: the gate is under test, not the dice
        ),
        Pose2d(face_center[0] - 17.0, face_center[1], 0.0), alliance="blue",
    )
    scorer.set_intake_active(False)

    sweeper = match.add_robot(
        build_algae_sweeper_characteristics(),
        _pose_presenting("right", target, approach), alliance="blue",
    )
    cycler = match.add_robot(
        build_station_cycler_characteristics(),
        _pose_presenting("front", target, approach), alliance="blue",
    )

    def offered_levels():
        return {a for r, a in scoring_slots_for_type(match, scorer, CORAL_TYPE) if r is face}

    # 1. Nobody has cleared anything: the scorer is not offered L3 here.
    assert match.region_blocked(face, gated_level)
    assert gated_level not in offered_levels()

    # 2. The sweeper tries and cannot -- the ALGAE is staged, not loose.
    sweeper.set_intake_active(True)
    _run(match, seconds=6.0)
    sweeper.set_intake_active(False)
    assert sweeper.held_pieces == []
    assert match.station_supply[algae_zone] == 1
    assert match.region_blocked(face, gated_level)

    # 3. The cycler does the chore.
    cycler.set_intake_active(True)
    _run(match, seconds=6.0)
    cycler.set_intake_active(False)
    assert [p.piece_type for p in cycler.held_pieces] == [ALGAE_TYPE]
    assert match.station_supply[algae_zone] == 0
    assert not match.region_blocked(face, gated_level)

    # 4. The payoff lands on a robot that never touched an ALGAE.
    assert gated_level in offered_levels()
    coral = scorer.held_pieces[0]
    scorer.set_deposit_active(True, action=gated_level)
    _run(match, seconds=4.0)

    assert coral.scored
    assert coral.target_action == gated_level
    assert match.scores["blue"] == 4.0  # Table 6-2: L3 in TELEOP
    assert scorer.held_pieces == []


def test_a_gated_face_offers_only_its_ungated_levels():
    """The union across six faces hides the gate (every level is open
    somewhere), so the per-face view is the one that shows it."""
    match = _match()
    robot = match.add_robot(build_station_cycler_characteristics(), Pose2d(300, 150, 0.0), alliance="blue")

    def levels_at(face_index):
        name = f"blue_reef_face_{face_index}"
        return {a for r, a in scoring_slots_for_type(match, robot, CORAL_TYPE) if r.name == name}

    for face_index in range(6):
        blocked = reef_algae_blocked_level(face_index)
        assert levels_at(face_index) == {"l1", "l2", "l3", "l4"} - {blocked}

    for location in list(match.station_supply):
        if location.name.startswith("blue_reef_algae"):
            match.station_supply[location] = 0
    for face_index in range(6):
        assert levels_at(face_index) == {"l1", "l2", "l3", "l4"}
