"""
End-to-end exercise of field/robot/match wiring using a trivial made-up
game (not any real FRC game) -- this is the "synthetic game" checkpoint
from ARCHITECTURE.md's build sequencing, proving common_sim's generic
pipeline (drive -> intake -> deposit -> score) works before any
game_specific code exists.
"""
import math
import random

import pytest

from common_sim.control.world_view import scoring_slots_for_type
from common_sim.field.field_config import FieldConfig, IntakeLocation, ScoringRegion, SecondaryAward
from common_sim.geometry import Pose2d
from common_sim.match.match import Match, MatchConfig, Phase
from common_sim.match.scoring import TableScoringRules
from common_sim.robot.characteristics import RobotCharacteristics

WIDGET = "widget"


def make_field() -> FieldConfig:
    # Scoring region is a big zone downfield of the robot's start/capture
    # path -- generous on purpose so the test is about the pipeline wiring,
    # not precision navigation.
    region = ScoringRegion(
        name="goal",
        vertices=((80, -60), (250, -60), (250, 160), (80, 160)),
        actions=frozenset({"score_widget"}),
        piece_types=frozenset({WIDGET}),
    )
    return FieldConfig(width=300, height=200, scoring_regions=(region,))


def make_scoring_rules() -> TableScoringRules:
    return TableScoringRules({
        ("score_widget", "auto"): 3.0,
        ("score_widget", "teleop"): 1.0,
    })


def make_characteristics(**overrides) -> RobotCharacteristics:
    defaults = dict(
        name="test-bot",
        max_speed=150.0,
        max_accel=400.0,
        max_angular_speed=6.0,
        max_angular_accel=20.0,
        width=28.0,
        length=28.0,
        piece_capacity=1,
        intake_time=0.1,
        deposit_time=0.1,
        intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    defaults.update(overrides)
    return RobotCharacteristics(**defaults)


def run_ticks(match: Match, n: int, dt: float = 1.0 / 60.0):
    for _ in range(n):
        match.step(dt)


def test_full_pickup_and_score_pipeline():
    field = make_field()
    match = Match(field, make_scoring_rules(), MatchConfig(auto_duration=1000, teleop_duration=1000))
    # Started off the y=0 wall: a robot spawned half-inside the perimeter
    # can no longer bulldoze its way out now that the drivetrain is
    # traction-limited rather than an unconditional velocity source.
    robot = match.add_robot(make_characteristics(), Pose2d(20, 100, 0), alliance="blue")
    piece = match.spawn_piece(WIDGET, (60, 100))

    robot.set_intake_active(True)
    # Drive forward until the piece is captured (or give up after a while).
    for _ in range(600):
        robot.drive_field_relative(1.0 / 60.0, 120.0, 0.0, 0.0)
        match.step(1.0 / 60.0)
        if robot.held_pieces:
            break
    assert robot.held_pieces, "robot never captured the piece"
    assert robot.held_pieces[0] is piece
    assert piece.held_by is robot

    # Piece should be pinned to the chassis while held. Checked at rest:
    # the pin is applied once per control tick, so a robot still moving
    # carries the piece a step's worth of travel behind it.
    robot.set_intake_active(False)
    for _ in range(60):
        robot.drive_field_relative(1.0 / 60.0, 0.0, 0.0, 0.0)
        match.step(1.0 / 60.0)
    assert math.isclose(piece.position.x, robot.chassis.body.position.x, abs_tol=1e-6)

    # Drive into the scoring zone first -- a deposit only spends its
    # timer once the robot is already positioned to score (see
    # Robot.update_manipulator's scoring_ready gate); commanding it
    # before arriving now just drops the piece instantly instead of
    # scoring it, so the action is set (for deposit_region_for to check
    # against) without yet activating the deposit.
    robot.set_deposit_active(False, action="score_widget")
    for _ in range(600):
        robot.drive_field_relative(1.0 / 60.0, 120.0, 0.0, 0.0)
        match.step(1.0 / 60.0)
        if match.deposit_region_for(robot) is not None:
            break
    assert match.deposit_region_for(robot) is not None, "robot never reached the scoring zone"

    robot.drive_field_relative(1.0 / 60.0, 0.0, 0.0, 0.0)
    robot.set_deposit_active(True)
    for _ in range(600):
        match.step(1.0 / 60.0)
        if piece.scored:
            break

    assert piece.scored
    assert match.scores.get("blue", 0.0) > 0
    assert piece.last_holder_alliance == "blue"
    assert piece not in match.active_pieces

    score_events = match.events.of_kind("score")
    assert len(score_events) == 1
    assert score_events[0].data["alliance"] == "blue"
    assert score_events[0].data["action"] == "score_widget"

    intake_events = match.events.of_kind("intake")
    deposit_events = match.events.of_kind("deposit")
    assert len(intake_events) == 1
    assert len(deposit_events) == 1


def test_phase_transitions_from_auto_to_teleop():
    field = make_field()
    match = Match(field, make_scoring_rules(), MatchConfig(auto_duration=0.5, teleop_duration=2.0))
    assert match.phase == Phase.AUTO
    run_ticks(match, 20)  # 20/60s < 0.5s, still auto
    assert match.phase == Phase.AUTO
    run_ticks(match, 20)  # crosses the 0.5s boundary
    assert match.phase == Phase.TELEOP
    phase_events = match.events.of_kind("phase_change")
    assert len(phase_events) == 1
    assert phase_events[0].data["phase"] == "teleop"


def test_match_ends_after_total_duration():
    field = make_field()
    match = Match(field, make_scoring_rules(), MatchConfig(auto_duration=0.1, teleop_duration=0.1))
    run_ticks(match, 20)  # 20/60s > 0.2s total
    assert match.ended
    elapsed_before = match.elapsed
    match.step(1.0 / 60.0)
    assert match.elapsed == elapsed_before, "step() should be a no-op once the match has ended"


def test_scoring_points_differ_between_auto_and_teleop():
    rules = make_scoring_rules()
    assert rules.points_for("score_widget", "auto") == 3.0
    assert rules.points_for("score_widget", "teleop") == 1.0
    assert rules.points_for("unknown_action", "auto") == 0.0


def test_piece_capacity_prevents_second_pickup():
    field = make_field()
    match = Match(field, make_scoring_rules(), MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot = match.add_robot(make_characteristics(piece_capacity=1), Pose2d(20, 100, 0))
    first = match.spawn_piece(WIDGET, (60, 100))
    second = match.spawn_piece(WIDGET, (60, 110))

    robot.set_intake_active(True)
    for _ in range(600):
        robot.drive_field_relative(1.0 / 60.0, 120.0, 0.0, 0.0)
        match.step(1.0 / 60.0)
        if robot.held_pieces:
            break

    # Which of the two pieces gets there first depends on exact physics
    # timing, not something the test should pin down -- what matters is
    # capacity=1 is respected: exactly one is held, the other free.
    assert len(robot.held_pieces) == 1
    assert {first.held_by, second.held_by} == {robot, None}


def test_robot_cannot_drive_through_field_perimeter():
    field = make_field()
    match = Match(field, make_scoring_rules(), MatchConfig(auto_duration=1000, teleop_duration=1000))
    robot = match.add_robot(make_characteristics(), Pose2d(field.width - 20, field.height / 2, 0))

    for _ in range(300):
        robot.drive_field_relative(1.0 / 60.0, 300.0, 0.0, 0.0)
        match.step(1.0 / 60.0)

    assert robot.chassis.pose.x <= field.width + 1.0


def test_scoring_region_supports_multiple_actions_with_different_time_and_points():
    """One physical zone (e.g. a reef face) offering several scoring
    levels, each with its own robot deposit time and point value -- the
    generic mechanism game_specific/reefscape's L1-L4 will build on."""
    region = ScoringRegion(
        name="reef_face",
        vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"l1", "l4"}),
        piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(region,))
    rules = TableScoringRules({
        ("l1", "auto"): 2.0, ("l1", "teleop"): 2.0,
        ("l4", "auto"): 7.0, ("l4", "teleop"): 7.0,
    })
    characteristics = make_characteristics(
        deposit_time=999.0,  # fallback should never be used -- every action below is listed explicitly
        deposit_time_by_action={"l1": 0.1, "l4": 2.0},
    )

    def score_at(action: str, expected_points: float, expected_min_ticks: int):
        match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
        robot = match.add_robot(characteristics, Pose2d(150, 0, 0))
        piece = match.spawn_piece(WIDGET, (165, 0))  # within the stationary robot's intake wedge
        robot.set_intake_active(True)
        for _ in range(60):
            match.step(1.0 / 60.0)
            if robot.held_pieces:
                break
        assert robot.held_pieces, f"setup failure for action={action}"

        robot.set_deposit_active(True, action=action)
        ticks = 0
        for _ in range(300):
            match.step(1.0 / 60.0)
            ticks += 1
            if piece.scored:
                break

        assert piece.scored, f"never scored for action={action}"
        assert piece.target_action == action
        assert match.scores["blue"] == expected_points
        assert ticks >= expected_min_ticks, "deposit completed faster than its configured duration allows"

    score_at("l1", 2.0, expected_min_ticks=int(0.1 * 60) - 1)
    score_at("l4", 7.0, expected_min_ticks=int(2.0 * 60) - 1)


def test_deposit_action_must_match_region_to_score():
    """A piece released while targeting an action the region it lands in
    doesn't offer is a miss, not a crash or a silent wrong score."""
    region = ScoringRegion(
        name="reef_face",
        vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"l1"}),
        piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(region,))
    rules = TableScoringRules({("l1", "auto"): 2.0, ("l4", "auto"): 7.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    characteristics = make_characteristics(deposit_time_by_action={"l1": 0.1, "l4": 0.1})
    robot = match.add_robot(characteristics, Pose2d(150, 0, 0))
    piece = match.spawn_piece(WIDGET, (165, 0))  # within the stationary robot's intake wedge

    robot.set_intake_active(True)
    for _ in range(60):
        match.step(1.0 / 60.0)
        if robot.held_pieces:
            break

    robot.set_deposit_active(True, action="l4")  # region only offers "l1"
    run_ticks(match, 30)

    assert not piece.scored
    assert match.scores.get("blue", 0.0) == 0.0


def test_region_capacity_by_action_caps_scoring():
    region = ScoringRegion(
        name="goal",
        vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"score_widget"}),
        piece_types=frozenset({WIDGET}),
        capacity_by_action={"score_widget": 1},
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(region,))
    rules = make_scoring_rules()
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    characteristics = make_characteristics()

    def score_one(position):
        robot = match.add_robot(characteristics, Pose2d(150, 0, 0))
        piece = match.spawn_piece(WIDGET, position)
        robot.set_intake_active(True)
        for _ in range(60):
            match.step(1.0 / 60.0)
            if robot.held_pieces:
                break
        robot.set_deposit_active(True, action="score_widget")
        for _ in range(60):
            match.step(1.0 / 60.0)
            if piece.scored:
                break
        return piece

    first = score_one((165, 0))
    assert first.scored
    assert match.region_full(region, "score_widget")

    second = score_one((165, 10))
    assert not second.scored, "region should already be at capacity"
    assert match.scores["blue"] == 3.0


def test_region_full_is_false_without_capacity_configured():
    field = make_field()
    match = Match(field, make_scoring_rules())
    region = field.scoring_regions[0]
    assert match.region_full(region, "score_widget") is False


def _gated_field(starting_pieces: int | None = 1) -> FieldConfig:
    """A goal whose "score_widget" action is blocked until the depot next
    to it has been emptied -- the synthetic-game shape of REEFSCAPE's
    ALGAE sitting on a REEF branch."""
    region = ScoringRegion(
        name="goal",
        vertices=((80, -60), (250, -60), (250, 160), (80, 160)),
        actions=frozenset({"score_widget"}),
        piece_types=frozenset({WIDGET}),
        blocked_until_collected={"score_widget": "depot"},
    )
    depot = IntakeLocation(
        name="depot", vertices=((0, -60), (60, -60), (60, 60), (0, 60)),
        piece_type=WIDGET, starting_pieces=starting_pieces,
    )
    return FieldConfig(width=300, height=200, scoring_regions=(region,), intake_locations=(depot,))


def test_region_blocked_until_its_location_runs_dry():
    field = _gated_field()
    match = Match(field, make_scoring_rules())
    region, depot = field.scoring_regions[0], field.intake_locations[0]

    assert match.region_blocked(region, "score_widget")
    # Not the same failure as being full -- the two are reported apart on
    # purpose, so a postmortem can tell a banked score from an unfinished chore.
    assert match.region_full(region, "score_widget") is False

    match.station_supply[depot] = 0
    assert match.region_blocked(region, "score_widget") is False


def test_blocked_region_refuses_to_score():
    """Two runs of one identical deposit, differing only in whether the
    gate has opened -- separate matches so the second isn't reading points
    off the first one's abandoned piece."""
    def deposit_one(open_the_gate: bool):
        field = _gated_field()
        match = Match(field, make_scoring_rules(), MatchConfig(auto_duration=1000, teleop_duration=1000))
        if open_the_gate:
            match.station_supply[field.intake_locations[0]] = 0
        robot = match.add_robot(make_characteristics(), Pose2d(150, 0, 0))
        piece = match.spawn_piece(WIDGET, (165, 0))
        robot.set_intake_active(True)
        for _ in range(60):
            match.step(1.0 / 60.0)
            if robot.held_pieces:
                break
        robot.set_intake_active(False)
        robot.set_deposit_active(True, action="score_widget")
        run_ticks(match, 30)
        return piece, match

    blocked, blocked_match = deposit_one(open_the_gate=False)
    assert not blocked.scored, "gate should suppress the score while the depot is stocked"
    assert blocked_match.scores.get("blue", 0.0) == 0.0

    opened, opened_match = deposit_one(open_the_gate=True)
    assert opened.scored, "the same deposit should score once the gate opens"
    assert opened_match.scores["blue"] > 0.0


def test_blocked_action_is_hidden_from_planning():
    """The gate has to reach the planner, not just the scoring check --
    otherwise a robot happily drives to a slot it cannot use and the
    failure shows up as a mystery miss instead of a re-plan."""
    field = _gated_field()
    match = Match(field, make_scoring_rules())
    robot = match.add_robot(make_characteristics(), Pose2d(150, 0, 0))

    assert scoring_slots_for_type(match, robot, WIDGET) == []

    match.station_supply[field.intake_locations[0]] = 0
    assert [action for _, action in scoring_slots_for_type(match, robot, WIDGET)] == ["score_widget"]


def test_ungated_region_is_never_blocked():
    field = make_field()
    match = Match(field, make_scoring_rules())
    assert match.region_blocked(field.scoring_regions[0], "score_widget") is False


def test_gate_naming_an_unknown_location_fails_at_construction():
    """Loud at construction rather than silently never-blocked -- a typo
    here would otherwise read as a scoring action nobody chose to use."""
    region = ScoringRegion(
        name="goal", vertices=((80, -60), (250, -60), (250, 160), (80, 160)),
        actions=frozenset({"score_widget"}), piece_types=frozenset({WIDGET}),
        blocked_until_collected={"score_widget": "nonexistent"},
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(region,))
    with pytest.raises(KeyError):
        Match(field, make_scoring_rules())


def test_secondary_award_pays_opponent_after_delay():
    """SecondaryAward's motivating case is REEFSCAPE's PROCESSOR gift: a
    piece scored on one action creates a delayed, probabilistic follow-on
    award for the *other* alliance -- modeling a human player, not a
    robot, doing the converting. Exercised directly against `_try_score` /
    `step`, the same way test_utility.py drives `_roll_scoring_success`
    directly, since driving a robot through a second scoring cycle just to
    trigger this would test navigation, not the award."""
    region = ScoringRegion(
        name="giver", vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"give"}), piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(
        width=300, height=200, scoring_regions=(region,),
        secondary_awards=(
            SecondaryAward(action="give", alliance_of="opponent", award_action="take", probability=1.0, delay=1.0),
        ),
    )
    rules = TableScoringRules({("give", "auto"): 6.0, ("take", "auto"): 4.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.target_action = "give"
    piece.last_holder_alliance = "blue"

    match._try_score(piece, region, explicit=True)
    assert match.scores["blue"] == 6.0
    assert match.scores.get("red", 0.0) == 0.0, "the award hasn't resolved yet"

    run_ticks(match, 59)  # just under the 1s delay
    assert match.scores.get("red", 0.0) == 0.0

    run_ticks(match, 2)  # crosses the 1s delay
    assert match.scores["red"] == 4.0
    assert match.scores["blue"] == 6.0, "the scoring alliance's own points are untouched"

    award_events = match.events.of_kind("secondary_award")
    assert len(award_events) == 1
    assert award_events[0].data == {"alliance": "red", "action": "take", "points": 4.0}


def test_secondary_award_alliance_of_scoring_pays_the_scoring_alliance():
    """The other half of alliance_of: a mechanic like 2018's EXCHANGE ->
    VAULT, where the human player payout belongs to whoever triggered it,
    not the opponent."""
    region = ScoringRegion(
        name="exchange", vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"exchange"}), piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(
        width=300, height=200, scoring_regions=(region,),
        secondary_awards=(
            SecondaryAward(action="exchange", alliance_of="scoring", award_action="vault", probability=1.0, delay=0.0),
        ),
    )
    rules = TableScoringRules({("exchange", "auto"): 0.0, ("vault", "auto"): 5.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.target_action = "exchange"
    piece.last_holder_alliance = "blue"

    match._try_score(piece, region, explicit=True)
    run_ticks(match, 1)  # 0.0 delay still needs a step() to resolve

    assert match.scores["blue"] == 5.0
    assert match.scores.get("red", 0.0) == 0.0


def test_secondary_award_probability_gates_the_award():
    """probability=0.0 means the human player never converts it -- no
    award ever gets scheduled, checked against a fixed RNG so the test
    isn't flaky."""
    region = ScoringRegion(
        name="giver", vertices=((80, -60), (250, -60), (250, 60), (80, 60)),
        actions=frozenset({"give"}), piece_types=frozenset({WIDGET}),
    )
    field = FieldConfig(
        width=300, height=200, scoring_regions=(region,),
        secondary_awards=(
            SecondaryAward(action="give", alliance_of="opponent", award_action="take", probability=0.0, delay=0.0),
        ),
    )
    rules = TableScoringRules({("give", "auto"): 6.0, ("take", "auto"): 4.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000), rng=random.Random(0))
    piece = match.spawn_piece(WIDGET, (0, 0))
    piece.target_action = "give"
    piece.last_holder_alliance = "blue"

    match._try_score(piece, region, explicit=True)
    run_ticks(match, 5)

    assert match.scores["blue"] == 6.0
    assert match.scores.get("red", 0.0) == 0.0
    assert match._pending_secondary_awards == []
    assert match.events.of_kind("secondary_award") == []


def test_starting_piece_count_preloads_robot():
    field = make_field()
    match = Match(field, make_scoring_rules())
    robot = match.add_robot(make_characteristics(starting_piece_count=1), Pose2d(0, 0, 0), alliance="red")
    assert len(robot.held_pieces) == 1
    assert robot.held_pieces[0].held_by is robot
    assert robot.held_pieces[0].last_holder_alliance == "red"
