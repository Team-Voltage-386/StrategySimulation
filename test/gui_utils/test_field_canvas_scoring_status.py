"""FieldCanvas's scoring-validity indicator.

The indicator answers "will this deposit score", so it has to disagree
with `Match.deposit_region_for` in exactly the cases where that method
still returns a region but `Match._try_score` will refuse the piece: a
region already at capacity, and one still blocked by an uncollected
piece. Both used to light up green and then drop the piece on the floor.
"""
from __future__ import annotations

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from pyqtgraph.Qt import QtWidgets  # noqa: E402

from common_sim.field.field_config import FieldConfig, IntakeLocation, ScoringRegion  # noqa: E402
from common_sim.geometry import Pose2d  # noqa: E402
from common_sim.match.match import Match, MatchConfig  # noqa: E402
from common_sim.match.scoring import TableScoringRules  # noqa: E402
from common_sim.robot.characteristics import RobotCharacteristics  # noqa: E402
from gui_utils.field_canvas import FieldCanvas  # noqa: E402

import pytest  # noqa: E402

WIDGET = "status-widget"


@pytest.fixture(scope="module")
def app():
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def _build(*, capacity=None, gated=False):
    region = ScoringRegion(
        name="goal", vertices=((80, -60), (250, -60), (250, 160), (80, 160)),
        actions=frozenset({"stow"}), piece_types=frozenset({WIDGET}),
        capacity_by_action=capacity,
        blocked_until_collected={"stow": "depot"} if gated else None,
    )
    depot = IntakeLocation(
        name="depot", vertices=((0, -60), (60, -60), (60, 60), (0, 60)),
        piece_type=WIDGET, starting_pieces=1,
    )
    field = FieldConfig(width=300, height=200, scoring_regions=(region,), intake_locations=(depot,))
    rules = TableScoringRules({("stow", "auto"): 3.0, ("stow", "teleop"): 1.0})
    match = Match(field, rules, MatchConfig(auto_duration=1000, teleop_duration=1000))
    characteristics = RobotCharacteristics(
        name="status-bot", max_speed=150.0, max_accel=400.0, max_angular_speed=6.0,
        max_angular_accel=20.0, width=28.0, length=28.0, piece_capacity=1,
        intake_time=0.1, deposit_time=100.0, intake_range=6.0,
        accepted_piece_types=frozenset({WIDGET}),
    )
    robot = match.add_robot(characteristics, Pose2d(150, 0, 0))

    # A long deposit_time keeps the robot mid-deposit -- holding the piece,
    # engaged with the region -- for the whole test, which is the state the
    # indicator is drawn for.
    match.spawn_piece(WIDGET, (165, 0))
    robot.set_intake_active(True)
    for _ in range(60):
        match.step(1.0 / 60.0)
        if robot.held_pieces:
            break
    robot.set_intake_active(False)
    robot.set_deposit_active(True, action="stow")
    match.step(1.0 / 60.0)
    return match, robot, region, depot


def test_indicator_is_valid_for_an_ordinary_reachable_region(app):
    match, robot, region, _ = _build()
    canvas = FieldCanvas(match)
    assert canvas._scoring_status_for(robot, region) == "valid"


def test_indicator_reads_invalid_while_the_region_is_blocked(app):
    match, robot, region, depot = _build(gated=True)
    canvas = FieldCanvas(match)
    assert match.deposit_region_for(robot, robot.held_pieces[0]) is region, (
        "deposit_region_for still resolves a blocked region -- the indicator, not "
        "the deposit gate, is what has to be stricter"
    )
    assert canvas._scoring_status_for(robot, region) == "invalid"

    match.station_supply[depot] = 0
    assert canvas._scoring_status_for(robot, region) == "valid"


def test_indicator_reads_invalid_once_the_region_is_full(app):
    """The same dishonesty predates the ALGAE gate: capacity was never
    consulted here either."""
    match, robot, region, _ = _build(capacity={"stow": 1})
    canvas = FieldCanvas(match)
    assert canvas._scoring_status_for(robot, region) == "valid"

    match.region_scores.setdefault(region.name, {})["stow"] = 1
    assert match.region_full(region, "stow")
    assert canvas._scoring_status_for(robot, region) == "invalid"
