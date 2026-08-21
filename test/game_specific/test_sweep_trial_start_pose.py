"""
sweep_trial.start_pose is the one function apps/run_reefscape.py's MatchView
and SWEEP both call to place robots -- see its docstring. These pin the
"line up under your own NET, facing your own ALLIANCE WALL" formation.
"""
from __future__ import annotations

import math

from game_specific.reefscape import sweep_trial
from game_specific.reefscape.field import FIELD_WIDTH, net_center
from game_specific.reefscape.sweep_trial import PRIMARY_LATERAL_OFFSET


def test_primary_sits_under_its_alliance_net_x():
    for alliance in ("blue", "red"):
        pose = sweep_trial.start_pose(alliance, -1)
        net_x, _ = net_center(alliance)
        assert pose.x == net_x


def test_primary_defaults_to_its_own_driver_left_not_center():
    net_x, net_y = net_center("blue")
    pose = sweep_trial.start_pose("blue", -1)
    assert pose.y != net_y
    assert pose.y == net_y - PRIMARY_LATERAL_OFFSET
    assert 0.0 <= pose.y <= FIELD_WIDTH


def test_primary_left_offset_mirrors_between_alliances():
    blue_net_x, blue_net_y = net_center("blue")
    red_net_x, red_net_y = net_center("red")
    blue = sweep_trial.start_pose("blue", -1)
    red = sweep_trial.start_pose("red", -1)
    assert blue.y == blue_net_y - PRIMARY_LATERAL_OFFSET
    assert red.y == red_net_y + PRIMARY_LATERAL_OFFSET


def test_primary_faces_its_own_alliance_wall():
    blue_pose = sweep_trial.start_pose("blue", -1)
    red_pose = sweep_trial.start_pose("red", -1)
    assert blue_pose.heading == math.pi
    assert red_pose.heading == 0.0


def test_roster_robots_stagger_around_the_net_center_y():
    net_x, net_y = net_center("blue")
    pose0 = sweep_trial.start_pose("blue", 0)
    pose1 = sweep_trial.start_pose("blue", 1)
    assert pose0.x == net_x and pose1.x == net_x
    assert pose0.y != net_y and pose1.y != net_y
    # Even indices offset one way, odd the other, both within the field.
    assert pose0.y > net_y
    assert pose1.y < net_y
    assert 0.0 <= pose0.y <= FIELD_WIDTH
    assert 0.0 <= pose1.y <= FIELD_WIDTH


def test_blue_and_red_are_mirror_images():
    blue = sweep_trial.start_pose("blue", 0)
    red = sweep_trial.start_pose("red", 0)
    blue_net_x, _ = net_center("blue")
    red_net_x, _ = net_center("red")
    assert blue.x == blue_net_x
    assert red.x == red_net_x
    assert blue.y == red.y
    assert blue.heading == math.pi
    assert red.heading == 0.0
