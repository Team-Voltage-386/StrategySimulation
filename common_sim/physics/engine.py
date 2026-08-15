"""
Generic fixed-timestep pymunk wrapper. Game-agnostic: knows nothing
about robots, pieces, or scoring -- it just advances a pymunk.Space in
deterministic substeps so a headless Monte Carlo trial and the 60Hz
interactive viewer produce identical physics for the same commands,
regardless of caller frame rate.
"""
from __future__ import annotations

import pymunk

# Fine enough to keep thin bumper/sensor shapes from tunneling through
# each other at robot top speed; coarser substeps let fast-moving pieces
# skip past thin sensor shapes between steps.
DEFAULT_SUBSTEP = 1.0 / 240.0


class SimEngine:
    """Owns the pymunk.Space and the fixed-substep clock. Game-specific
    and common_sim code should add bodies/shapes to `engine.space`
    directly; this class only owns time advancement."""

    def __init__(self, substep: float = DEFAULT_SUBSTEP):
        self.space = pymunk.Space()
        self.space.gravity = (0, 0)
        self.substep = substep
        self.elapsed = 0.0

    def step(self, dt: float) -> None:
        """Advance simulated time by dt, broken into fixed-size substeps
        (plus a final partial substep if dt isn't an exact multiple)."""
        remaining = dt
        while remaining > 1e-9:
            step_dt = min(self.substep, remaining)
            self.space.step(step_dt)
            self.elapsed += step_dt
            remaining -= step_dt

    def add(self, *objs) -> None:
        self.space.add(*objs)

    def remove(self, *objs) -> None:
        self.space.remove(*objs)
