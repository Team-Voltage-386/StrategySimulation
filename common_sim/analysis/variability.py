"""
Seeded config perturbation -- the sweep's only source of randomness.
No physics/control/engine changes: this only jitters the *inputs* a
trial is built from (characteristics, start pose, piece scatter) before
the deterministic sim takes over.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass


def substream(seed: int, stream: str) -> random.Random:
    """A named sub-stream of `seed`. A str seed goes through sha512
    inside CPython's random.Random, stable across machines and
    unaffected by PYTHONHASHSEED (unlike hash()). Named substreams
    ("chars:PRIMARY", "pose:BLUE 0", "pieces") mean adding a new
    perturbation later does not shift the draws an existing one makes."""
    return random.Random(f"{seed}:{stream}")


@dataclass(frozen=True)
class VariabilityModel:
    enabled: bool = False
    intake_time_pct: float = 0.0     # sigma of a multiplier on intake times
    deposit_time_pct: float = 0.0
    max_speed_pct: float = 0.0
    max_accel_pct: float = 0.0
    start_pose_xy_in: float = 0.0    # sigma, inches
    start_pose_heading_deg: float = 0.0
    piece_scatter_in: float = 0.0    # sigma, inches, per spawned piece


def _clamped_multiplier(rng: random.Random, pct: float) -> float:
    """1.0 + gauss(0, pct), clamped to [0.25, 4.0] so a fat tail can
    never produce a zero or negative time/speed. Draws nothing when
    pct <= 0, so a disabled perturbation channel doesn't shift the draw
    order of the ones after it."""
    if pct <= 0.0:
        return 1.0
    return max(0.25, min(4.0, 1.0 + rng.gauss(0.0, pct)))


def perturb_characteristics(char_spec: dict, model: VariabilityModel, rng: random.Random) -> dict:
    """One shared multiplier per family (all intake times move together,
    all deposit times move together) -- a slow robot is slow at
    everything; per-key independent noise would average out and
    understate real variance. Dict keys are sorted before iterating so
    draw order does not depend on how the GUI built the spec."""
    if not model.enabled:
        return char_spec

    result = dict(char_spec)
    intake_mult = _clamped_multiplier(rng, model.intake_time_pct)
    deposit_mult = _clamped_multiplier(rng, model.deposit_time_pct)
    speed_mult = _clamped_multiplier(rng, model.max_speed_pct)
    accel_mult = _clamped_multiplier(rng, model.max_accel_pct)

    if "intake_time" in result:
        result["intake_time"] = result["intake_time"] * intake_mult
    if result.get("intake_time_by_type"):
        result["intake_time_by_type"] = {
            k: result["intake_time_by_type"][k] * intake_mult for k in sorted(result["intake_time_by_type"])
        }
    if "station_intake_time" in result:
        result["station_intake_time"] = result["station_intake_time"] * intake_mult

    if "deposit_time" in result:
        result["deposit_time"] = result["deposit_time"] * deposit_mult
    if result.get("deposit_time_by_action"):
        result["deposit_time_by_action"] = {
            k: result["deposit_time_by_action"][k] * deposit_mult for k in sorted(result["deposit_time_by_action"])
        }

    if "max_speed" in result:
        result["max_speed"] = result["max_speed"] * speed_mult
    if "max_accel" in result:
        result["max_accel"] = result["max_accel"] * accel_mult

    return result


def perturb_pose(x: float, y: float, heading_rad: float, model: VariabilityModel, rng: random.Random):
    if not model.enabled:
        return x, y, heading_rad
    dx = rng.gauss(0.0, model.start_pose_xy_in) if model.start_pose_xy_in > 0.0 else 0.0
    dy = rng.gauss(0.0, model.start_pose_xy_in) if model.start_pose_xy_in > 0.0 else 0.0
    dheading_deg = rng.gauss(0.0, model.start_pose_heading_deg) if model.start_pose_heading_deg > 0.0 else 0.0
    return x + dx, y + dy, heading_rad + math.radians(dheading_deg)


def scatter_offset(model: VariabilityModel, rng: random.Random):
    if not model.enabled or model.piece_scatter_in <= 0.0:
        return 0.0, 0.0
    return rng.gauss(0.0, model.piece_scatter_in), rng.gauss(0.0, model.piece_scatter_in)
