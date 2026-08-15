"""
Shared 2D geometry types for common_sim.

Units: inches, seconds, radians -- matches how FRC field drawings and
game manuals specify dimensions, and keeps common_sim decoupled from
whatever unit convention a team's actual robot code uses (WPILib itself
favors meters; this sim has no reason to match).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import pymunk

Vec2d = pymunk.Vec2d


def wrap_angle(angle: float) -> float:
    """Wrap an angle in radians to (-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


@dataclass(frozen=True)
class Pose2d:
    x: float
    y: float
    heading: float  # radians, CCW from +x axis

    @property
    def translation(self) -> Vec2d:
        return Vec2d(self.x, self.y)

    def with_translation(self, x: float, y: float) -> "Pose2d":
        return Pose2d(x, y, self.heading)

    def with_heading(self, heading: float) -> "Pose2d":
        return Pose2d(self.x, self.y, wrap_angle(heading))

    def distance_to(self, other: "Pose2d") -> float:
        return self.translation.get_distance(other.translation)

    def heading_to(self, other: "Pose2d") -> float:
        """Field-relative bearing from this pose's translation to other's."""
        delta = other.translation - self.translation
        return math.atan2(delta.y, delta.x)

    def relative_to(self, origin: "Pose2d") -> "Pose2d":
        """This pose expressed in `origin`'s reference frame."""
        delta = self.translation - origin.translation
        rotated = delta.rotated(-origin.heading)
        return Pose2d(rotated.x, rotated.y, wrap_angle(self.heading - origin.heading))

    def as_tuple(self) -> tuple:
        return (self.x, self.y, self.heading)
