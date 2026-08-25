"""The seeded operator: does it steer where it thinks it steers?

The sign convention here is the kind of thing that fails silently. If
`_drive_axes` is wrong, the fuzzer still runs all night, still reports clean,
and has spent eight hours driving the robot in a direction nobody intended --
including into the wall it is supposed to be avoiding. It was derived from
observation rather than from reading the Java, so it gets pinned down here.
"""
from __future__ import annotations

import math

from bridge import operator as op
from bridge import scenario
from bridge.scenario import Action, ScenarioGenerator, _drive_axes, near_wall


def _velocity(axes: dict[int, float]) -> tuple[float, float]:
    """Field velocity the robot will command, per DriveCommands.joystickDrive.

    `xSupplier = () -> -kDriveController.getLeftY()`
    `ySupplier = () -> -kDriveController.getLeftX()`
    """
    return -axes.get(op.AXIS_LEFT_Y, 0.0), -axes.get(op.AXIS_LEFT_X, 0.0)


def test_drive_axes_point_where_they_claim():
    for direction in [(1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-2, 3)]:
        axes = _drive_axes(direction, 0.8)
        vx, vy = _velocity(axes)
        norm = math.hypot(*direction)
        assert math.isclose(vx, direction[0] / norm * 0.8, abs_tol=1e-9)
        assert math.isclose(vy, direction[1] / norm * 0.8, abs_tol=1e-9)


def test_pushing_left_x_positive_moves_toward_negative_y():
    """The observed behaviour the wall-pin provocation depends on."""
    _, vy = _velocity({op.AXIS_LEFT_X: 0.7})
    assert vy < 0


def test_recovery_actually_heads_for_the_middle():
    gen = ScenarioGenerator(1)
    # Bottom-left corner: recovery must command +x and +y.
    action = gen.recover_toward_centre(0.5, 0.5)
    vx, vy = _velocity(action.axes)
    assert vx > 0 and vy > 0

    # Top-right corner: the other way.
    action = gen.recover_toward_centre(scenario.FIELD_LENGTH - 0.5, scenario.FIELD_WIDTH - 0.5)
    vx, vy = _velocity(action.axes)
    assert vx < 0 and vy < 0


def test_zero_direction_does_not_divide_by_zero():
    """Recovery from dead centre is degenerate but must not explode."""
    axes = _drive_axes((0.0, 0.0), 0.5)
    assert all(math.isfinite(v) for v in axes.values())


def test_near_wall_covers_all_four_edges():
    cx, cy = scenario.FIELD_CENTRE
    assert not near_wall(cx, cy)
    assert near_wall(0.3, cy)
    assert near_wall(scenario.FIELD_LENGTH - 0.3, cy)
    assert near_wall(cx, 0.3)
    assert near_wall(cx, scenario.FIELD_WIDTH - 0.3)


def test_the_robot_start_pose_counts_as_near_a_wall():
    """SimContainer starts the robot at y=0.815, about half a metre off it.

    If this were not caught, the very first action of every match would be a
    free push into the lower wall.
    """
    assert near_wall(8.790, 0.815)


def test_a_seed_reproduces_its_script():
    a = [ScenarioGenerator(4711).next_action() for _ in range(30)]
    b = [ScenarioGenerator(4711).next_action() for _ in range(30)]
    assert [x.label for x in a] == [x.label for x in b]
    assert [x.seconds for x in a] == [x.seconds for x in b]

    other = [ScenarioGenerator(4712).next_action() for _ in range(30)]
    assert [x.label for x in a] != [x.label for x in other]


def test_generated_actions_are_physically_sendable():
    gen = ScenarioGenerator(99)
    for _ in range(400):
        action = gen.next_action()
        assert action.seconds > 0
        for axis, value in action.axes.items():
            assert -1.0 <= value <= 1.0, f"{action.label} axis {axis} = {value}"
            if axis in (op.AXIS_LEFT_TRIGGER, op.AXIS_RIGHT_TRIGGER):
                assert 0.0 <= value <= 1.0, "triggers only travel one way"
        for button in action.buttons | action.manip_buttons:
            assert 1 <= button <= op.BUTTON_COUNT


def test_back_off_reverses_the_move_that_got_stuck():
    gen = ScenarioGenerator(3)
    stuck_on = Action("drive", 1.0, axes=_drive_axes((1.0, 0.0), 0.6))
    out = gen.back_off(stuck_on)

    vx_before, _ = _velocity(stuck_on.axes)
    vx_after, _ = _velocity(out.axes)
    assert vx_before > 0 and vx_after < 0
    assert out.axes[op.AXIS_RIGHT_X] != 0, "turning off a corner needs rotation too"


def test_later_back_offs_try_sideways_instead_of_straight_back():
    """Straight back failed twice; a person slides along the obstacle next."""
    gen = ScenarioGenerator(3)
    stuck_on = Action("drive", 1.0, axes=_drive_axes((1.0, 0.0), 0.6))

    early_vx, early_vy = _velocity(gen.back_off(stuck_on, attempt=0).axes)
    late_vx, late_vy = _velocity(gen.back_off(stuck_on, attempt=2).axes)

    assert abs(early_vy) < abs(early_vx), "the first attempt is a straight reverse"
    assert abs(late_vy) > abs(late_vx), "a later attempt pushes perpendicular"


def test_back_offs_escalate_but_stay_sendable():
    gen = ScenarioGenerator(4)
    stuck_on = Action("drive", 1.0, axes=_drive_axes((1.0, 0.5), 0.4))
    strengths = []
    for attempt in range(5):
        out = gen.back_off(stuck_on, attempt)
        for value in out.axes.values():
            assert -1.0 <= value <= 1.0
        strengths.append(abs(out.axes[op.AXIS_RIGHT_X]))
    assert strengths == sorted(strengths), "each attempt should be at least as forceful"


def test_every_back_off_is_shorter_than_the_frozen_robot_window():
    """Ordinary contact must never reach the detector, however many tries it takes.

    What survives is the finding worth having: commanded, tried repeatedly to
    reverse, still not moving.
    """
    from bridge.harness import MatchRunner
    from bridge.oracles import LivenessThresholds

    gen = ScenarioGenerator(5)
    window = LivenessThresholds().frozen_seconds
    for attempt in range(MatchRunner.MAX_CONSECUTIVE_BACKOFFS):
        for _ in range(100):
            out = gen.back_off(Action("drive", 1.0, axes={op.AXIS_LEFT_Y: -0.5}), attempt)
            assert out.seconds < window, f"attempt {attempt} lasts {out.seconds:.2f}s"


def test_back_off_carries_no_stale_buttons():
    gen = ScenarioGenerator(11)
    stuck_on = Action("intake", 1.0,
                      axes={op.AXIS_LEFT_Y: -0.5, op.AXIS_RIGHT_TRIGGER: 1.0},
                      buttons=frozenset({op.BTN_LEFT_BUMPER}))
    out = gen.back_off(stuck_on)

    assert out.buttons == frozenset()
    assert op.AXIS_RIGHT_TRIGGER not in out.axes


def test_commands_drive_ignores_actions_that_do_not_translate():
    assert not Action("idle", 1.0).commands_drive
    assert not Action("spin-up", 1.0, buttons=frozenset({op.BTN_LEFT_BUMPER})).commands_drive
    assert not Action("turn", 1.0, axes={op.AXIS_RIGHT_X: 0.8}).commands_drive
    assert Action("drive", 1.0, axes={op.AXIS_LEFT_Y: -0.5}).commands_drive


def test_truncation_shortens_without_reaching_zero():
    action = Action("shoot", 2.0, axes={op.AXIS_LEFT_TRIGGER: 1.0},
                    buttons=frozenset({op.BTN_LEFT_BUMPER}))
    cut = action.truncated(0.3)

    assert cut.seconds < action.seconds
    assert cut.seconds >= 0.15, "a zero-length press is no press at all"
    assert cut.buttons == action.buttons, "the intent is unchanged; only the follow-through"
    assert "cut" in cut.label


def test_truncation_of_a_tiny_action_still_produces_an_edge():
    cut = Action("tap", 0.2).truncated(0.01)
    assert cut.seconds >= 0.15


def test_the_awkward_moves_are_seasoning_not_the_meal():
    """An operator that is mostly fumbling never reaches a state to fumble in."""
    gen = ScenarioGenerator(7)
    labels = [gen.next_action().label for _ in range(2000)]
    awkward = sum(1 for l in labels if l.startswith(("contradict", "mash")))
    assert awkward / len(labels) < 0.10


def test_interruptions_happen_often_enough_to_matter():
    gen = ScenarioGenerator(7)
    labels = [gen.next_action().label for _ in range(2000)]
    cut = sum(1 for l in labels if l.endswith("(cut)"))
    assert 0.12 < cut / len(labels) < 0.35
