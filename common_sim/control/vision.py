"""
Simulated field-position awareness via AprilTags -- not a camera model.
The sim only needs "how well can this robot estimate its own field
pose," tunable per robot design (a design trade: a robot with more/
better-placed cameras should localize better), not actual image
processing. Also offers coarse game-piece detection (nearest visible
free piece within FOV/range) for behaviors that want to find pieces
without already knowing where they are.
"""
from __future__ import annotations

import math
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from common_sim.field.game_piece import GamePiece
from common_sim.geometry import Pose2d, Vec2d, wrap_angle

DEFAULT_FOV = math.radians(90)


@dataclass(frozen=True)
class AprilTag:
    id: int
    pose: Pose2d  # field position + the direction the tag faces (unused for visibility, kept for future use)


def _visible_range(observer_pose: Pose2d, target: Vec2d, fov: float, max_range: float) -> Optional[float]:
    """Distance to `target` if it's within `observer_pose`'s FOV and
    max_range, else None."""
    delta = target - observer_pose.translation
    distance = delta.length
    if distance < 1e-6 or distance > max_range:
        return None
    bearing = math.atan2(delta.y, delta.x)
    relative_bearing = wrap_angle(bearing - observer_pose.heading)
    return distance if abs(relative_bearing) <= fov / 2.0 else None


class PoseEstimator(ABC):
    @abstractmethod
    def estimate(self, true_pose: Pose2d) -> Pose2d:
        raise NotImplementedError


class PerfectPoseEstimator(PoseEstimator):
    """Ground truth, unmodified -- the default; a robot only gets worse
    than this by construction (NoisyAprilTagEstimator), never better."""

    def estimate(self, true_pose: Pose2d) -> Pose2d:
        return true_pose


class NoisyAprilTagEstimator(PoseEstimator):
    """
    Gaussian position/heading noise scaled by distance to the nearest
    visible tag (further = noisier, matching real AprilTag pose
    estimation degrading with range). If no tag is currently visible
    (outside every tag's FOV or max_range), returns the last estimate
    unchanged rather than snapping to true_pose or going stale to zero --
    modeling "holds last known position" rather than simulating dead-
    reckoning odometry between updates, which is out of scope here.
    Before any tag has ever been seen, falls back to true_pose.
    """

    def __init__(
        self,
        tags: list[AprilTag],
        *,
        fov: float = DEFAULT_FOV,
        max_range: float = 240.0,
        noise_per_inch: float = 0.01,
        heading_noise_per_inch: float = 0.0005,
        rng: Optional[random.Random] = None,
    ):
        self.tags = tags
        self.fov = fov
        self.max_range = max_range
        self.noise_per_inch = noise_per_inch
        self.heading_noise_per_inch = heading_noise_per_inch
        self.rng = rng or random.Random()
        self._last_estimate: Optional[Pose2d] = None

    def nearest_visible_tag_distance(self, true_pose: Pose2d) -> Optional[float]:
        distances = [
            d for tag in self.tags
            if (d := _visible_range(true_pose, tag.pose.translation, self.fov, self.max_range)) is not None
        ]
        return min(distances) if distances else None

    def estimate(self, true_pose: Pose2d) -> Pose2d:
        distance = self.nearest_visible_tag_distance(true_pose)
        if distance is None:
            return self._last_estimate if self._last_estimate is not None else true_pose

        pos_std = distance * self.noise_per_inch
        heading_std = distance * self.heading_noise_per_inch
        noisy = Pose2d(
            true_pose.x + self.rng.gauss(0.0, pos_std),
            true_pose.y + self.rng.gauss(0.0, pos_std),
            wrap_angle(true_pose.heading + self.rng.gauss(0.0, heading_std)),
        )
        self._last_estimate = noisy
        return noisy


def visible_pieces(
    observer_pose: Pose2d,
    pieces: list[GamePiece],
    *,
    fov: float = DEFAULT_FOV,
    max_range: float = 120.0,
) -> list[GamePiece]:
    """Free (un-held) pieces within FOV/range, nearest first."""
    found = []
    for piece in pieces:
        if piece.held_by is not None:
            continue
        distance = _visible_range(observer_pose, piece.position, fov, max_range)
        if distance is not None:
            found.append((distance, piece))
    found.sort(key=lambda pair: pair[0])
    return [piece for _, piece in found]
