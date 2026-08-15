import math

from common_sim.geometry import Pose2d, wrap_angle


def test_wrap_angle_normalizes_into_range():
    assert math.isclose(wrap_angle(0.0), 0.0)
    assert math.isclose(wrap_angle(3 * math.pi), -math.pi, abs_tol=1e-9) or math.isclose(
        wrap_angle(3 * math.pi), math.pi, abs_tol=1e-9
    )
    assert math.isclose(wrap_angle(-3 * math.pi), math.pi, abs_tol=1e-9) or math.isclose(
        wrap_angle(-3 * math.pi), -math.pi, abs_tol=1e-9
    )


def test_distance_to():
    a = Pose2d(0, 0, 0)
    b = Pose2d(3, 4, 0)
    assert math.isclose(a.distance_to(b), 5.0)


def test_heading_to():
    a = Pose2d(0, 0, 0)
    b = Pose2d(0, 10, 0)
    assert math.isclose(a.heading_to(b), math.pi / 2)


def test_relative_to_same_pose_is_origin():
    p = Pose2d(12, -7, 0.4)
    rel = p.relative_to(p)
    assert math.isclose(rel.x, 0.0, abs_tol=1e-9)
    assert math.isclose(rel.y, 0.0, abs_tol=1e-9)
    assert math.isclose(rel.heading, 0.0, abs_tol=1e-9)


def test_relative_to_translation_only():
    origin = Pose2d(10, 10, 0)
    p = Pose2d(15, 10, 0)
    rel = p.relative_to(origin)
    assert math.isclose(rel.x, 5.0, abs_tol=1e-9)
    assert math.isclose(rel.y, 0.0, abs_tol=1e-9)


def test_relative_to_accounts_for_origin_heading():
    origin = Pose2d(0, 0, math.pi / 2)
    p = Pose2d(0, 5, math.pi / 2)
    rel = p.relative_to(origin)
    # origin faces +y; p is 5 "forward" of origin in origin's own frame
    assert math.isclose(rel.x, 5.0, abs_tol=1e-9)
    assert math.isclose(rel.y, 0.0, abs_tol=1e-9)
    assert math.isclose(rel.heading, 0.0, abs_tol=1e-9)
