import math
import random

from common_sim.control.vision import (
    AprilTag,
    NoisyAprilTagEstimator,
    PerfectPoseEstimator,
    visible_pieces,
)
from common_sim.field.game_piece import GamePiece
from common_sim.geometry import Pose2d
from common_sim.physics.engine import SimEngine


def test_perfect_pose_estimator_returns_ground_truth():
    est = PerfectPoseEstimator()
    pose = Pose2d(12.0, -5.0, 0.7)
    assert est.estimate(pose) == pose


def test_noisy_estimator_close_tag_has_small_error_on_average():
    tag = AprilTag(id=1, pose=Pose2d(100, 0, math.pi))
    rng = random.Random(42)
    est = NoisyAprilTagEstimator([tag], noise_per_inch=0.01, heading_noise_per_inch=0.001, rng=rng)
    true_pose = Pose2d(90, 0, 0)  # 10in from the tag

    errors = []
    for _ in range(500):
        estimate = est.estimate(true_pose)
        errors.append(true_pose.distance_to(estimate))
    mean_error = sum(errors) / len(errors)
    # distance=10, noise_per_inch=0.01 -> position std ~0.1in per axis;
    # generous bound just proves noise scales small at close range.
    assert mean_error < 0.5


def test_noisy_estimator_far_tag_has_larger_error_than_close_tag():
    close_tag = AprilTag(id=1, pose=Pose2d(20, 0, math.pi))
    far_tag = AprilTag(id=2, pose=Pose2d(220, 0, math.pi))
    rng = random.Random(7)

    close_est = NoisyAprilTagEstimator([close_tag], noise_per_inch=0.02, rng=rng)
    far_est = NoisyAprilTagEstimator([far_tag], noise_per_inch=0.02, rng=random.Random(7))

    true_pose = Pose2d(0, 0, 0)
    close_errors = [true_pose.distance_to(close_est.estimate(true_pose)) for _ in range(300)]
    far_errors = [true_pose.distance_to(far_est.estimate(true_pose)) for _ in range(300)]

    assert sum(far_errors) / len(far_errors) > sum(close_errors) / len(close_errors)


def test_noisy_estimator_out_of_range_holds_last_estimate():
    tag = AprilTag(id=1, pose=Pose2d(20, 0, math.pi))
    est = NoisyAprilTagEstimator([tag], max_range=50.0, rng=random.Random(1))

    near_pose = Pose2d(0, 0, 0)
    near_estimate = est.estimate(near_pose)  # tag in range, updates _last_estimate

    far_pose = Pose2d(-500, 0, 0)  # tag now far out of max_range
    far_estimate = est.estimate(far_pose)

    assert far_estimate == near_estimate  # held last known estimate, not snapped to far_pose


def test_noisy_estimator_falls_back_to_true_pose_before_any_tag_seen():
    tag = AprilTag(id=1, pose=Pose2d(20, 0, math.pi))
    est = NoisyAprilTagEstimator([tag], max_range=10.0, rng=random.Random(1))
    far_pose = Pose2d(-500, 0, 0)  # tag out of range from the very first call
    assert est.estimate(far_pose) == far_pose


def test_noisy_estimator_respects_field_of_view():
    tag_behind = AprilTag(id=1, pose=Pose2d(-50, 0, 0))  # directly behind a robot facing +x
    est = NoisyAprilTagEstimator([tag_behind], fov=math.radians(90), rng=random.Random(1))
    pose = Pose2d(0, 0, 0)
    assert est.nearest_visible_tag_distance(pose) is None
    assert est.estimate(pose) == pose  # no tag seen yet -> falls back to true pose


def test_noisy_estimator_picks_nearest_of_multiple_visible_tags():
    near_tag = AprilTag(id=1, pose=Pose2d(30, 0, math.pi))
    far_tag = AprilTag(id=2, pose=Pose2d(100, 0, math.pi))
    est = NoisyAprilTagEstimator([near_tag, far_tag], rng=random.Random(1))
    pose = Pose2d(0, 0, 0)
    assert est.nearest_visible_tag_distance(pose) == 30.0


def _make_piece(piece_type="widget", position=(0, 0), held_by=None):
    engine = SimEngine()
    piece = GamePiece(engine.space, piece_type, position)
    piece.held_by = held_by
    return piece


def test_visible_pieces_filters_by_range_and_fov_and_sorts_nearest_first():
    observer = Pose2d(0, 0, 0)
    near = _make_piece(position=(20, 0))
    far_but_visible = _make_piece(position=(80, 0))
    out_of_range = _make_piece(position=(500, 0))
    behind = _make_piece(position=(-20, 0))

    result = visible_pieces(observer, [far_but_visible, near, out_of_range, behind], max_range=120.0)
    assert result == [near, far_but_visible]


def test_visible_pieces_excludes_held_pieces():
    observer = Pose2d(0, 0, 0)
    held = _make_piece(position=(20, 0), held_by=object())
    free = _make_piece(position=(20, 10))
    result = visible_pieces(observer, [held, free], max_range=120.0)
    assert result == [free]
