"""
Perspective camera for a driver-station view of the field -- projects
field-space (inches, +x toward the red wall, +y across, z up) points to
viewport pixels, so gui_utils/field_canvas.py can render the same match
state it already draws top-down from a tilted, alliance-relative eye
instead. Pure geometry: no Qt import, so it's testable headless.

The eye always sits behind its own alliance's wall looking down-field, so
every point on the field is in front of the camera (positive camera-space
depth) for any sane preset -- there is no near-plane clipping problem to
solve here, which is normally what makes a first perspective camera hard.

"Right" is fixed per alliance rather than built from a general yaw
rotation -- there are exactly two driver-station orientations (face the
far wall, or face the near wall), so hardcoding each one's forward/right
axes sidesteps any handedness ambiguity a general yaw formula would
introduce. It also means red's view is a true mirror of blue's, not an
independent formula that happens to agree.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

ALLIANCE_FORWARD: dict[str, tuple[float, float]] = {"blue": (1.0, 0.0), "red": (-1.0, 0.0)}
ALLIANCE_RIGHT: dict[str, tuple[float, float]] = {"blue": (0.0, 1.0), "red": (0.0, -1.0)}

# Camera-space depth is clamped to this minimum before dividing -- guards a
# degenerate preset (eye_setback=0 with a point exactly at the wall) rather
# than anything real field geometry produces.
_MIN_DEPTH = 1.0


@dataclass(frozen=True)
class ViewPreset:
    """One named eye placement, in inches, relative to the field. Applied
    to whichever alliance wall the caller selects -- `lateral` shifts
    along the wall (FRC has three driver stations across it), positive
    toward that alliance's "right" as FieldCamera defines it."""
    name: str
    eye_height: float = 110.0    # inches above the carpet
    eye_setback: float = 90.0    # inches behind the alliance wall
    lateral: float = 0.0         # inches off the field's lateral center
    fov_deg: float = 70.0        # vertical field of view
    pitch_deg: float = -22.0     # negative tilts the view down


# A true ~65" driver eye pressed right up against the glass makes the far
# half of a 690"-long field a few pixels tall on a laptop screen --
# authentic, but close to unusable for practice. DRIVER is a usable
# compromise (raised and pulled back); ELEVATED reads more like a
# coach/scouting view from the stands. Both are starting points a UI can
# offer a slider around rather than the only two options.
DRIVER = ViewPreset("DRIVER", eye_height=100.0, eye_setback=65.0, lateral=0.0, fov_deg=70.0, pitch_deg=-22.0)
ELEVATED = ViewPreset("ELEVATED", eye_height=220.0, eye_setback=160.0, lateral=0.0, fov_deg=60.0, pitch_deg=-35.0)


def _normalize(v: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if length < 1e-9:
        return (0.0, 0.0, 1.0)
    return (v[0] / length, v[1] / length, v[2] / length)


def _cross(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class FieldCamera:
    """Projects field-space (x, y, z) inches to (screen_x, screen_y, depth)
    pixels for one alliance's driver station, sized to a `viewport_w` x
    `viewport_h` widget. Cheap to build (a handful of trig calls), so
    FieldCanvas builds one fresh every paint rather than keeping it live
    across a resize -- there is then never a stale-camera-vs-widget-size
    bug to chase."""

    def __init__(
        self, field_width: float, field_height: float, alliance: str,
        viewport_w: float, viewport_h: float, preset: ViewPreset,
    ):
        if alliance not in ALLIANCE_FORWARD:
            raise ValueError(f"unknown alliance {alliance!r}, expected 'blue' or 'red'")
        fx, fy = ALLIANCE_FORWARD[alliance]
        rx, ry = ALLIANCE_RIGHT[alliance]

        wall_x = 0.0 if alliance == "blue" else field_width
        eye_x = wall_x - fx * preset.eye_setback
        eye_y = field_height / 2.0 + preset.lateral
        self.eye = (eye_x, eye_y, preset.eye_height)

        pitch = math.radians(preset.pitch_deg)
        self.right = (rx, ry, 0.0)
        self.forward = _normalize((fx * math.cos(pitch), fy * math.cos(pitch), math.sin(pitch)))
        self.up = _normalize(_cross(self.forward, self.right))

        self.viewport_w = max(viewport_w, 1)
        self.viewport_h = max(viewport_h, 1)
        fov = math.radians(preset.fov_deg)
        self.focal = (self.viewport_h / 2.0) / math.tan(fov / 2.0)

    def project(self, x: float, y: float, z: float = 0.0) -> tuple[float, float, float]:
        """(screen_x, screen_y, depth). depth is camera-space distance
        along the view direction (not straight-line range) -- what
        `scale_at` and depth-sorting both key off."""
        dx, dy, dz = x - self.eye[0], y - self.eye[1], z - self.eye[2]
        cx = dx * self.right[0] + dy * self.right[1] + dz * self.right[2]
        cy = dx * self.up[0] + dy * self.up[1] + dz * self.up[2]
        cz = dx * self.forward[0] + dy * self.forward[1] + dz * self.forward[2]
        cz = max(cz, _MIN_DEPTH)
        px = self.viewport_w / 2.0 + self.focal * cx / cz
        py = self.viewport_h / 2.0 - self.focal * cy / cz
        return px, py, cz

    def scale_at(self, x: float, y: float, z: float = 0.0) -> float:
        """Pixels-per-inch at this field point -- what a top-down view's
        constant `_field_scale()` becomes once distance from the eye
        matters: near things draw bigger than far ones."""
        _, _, cz = self.project(x, y, z)
        return self.focal / cz


def orient_drive(alliance: str, up: float, right: float) -> tuple[float, float]:
    """Maps a driver-relative stick reading -- `up` (away from the
    driver, into the field) and `right` (the driver's own right hand) --
    to field-absolute (vx, vy) for `alliance`'s driver-station
    orientation. Uses the exact same ALLIANCE_FORWARD/ALLIANCE_RIGHT
    vectors FieldCamera projects with, so "stick right" and "the robot
    visibly moves right on screen" agree by construction rather than by
    two formulas that happen to match."""
    fx, fy = ALLIANCE_FORWARD[alliance]
    rx, ry = ALLIANCE_RIGHT[alliance]
    return right * rx + up * fx, right * ry + up * fy
