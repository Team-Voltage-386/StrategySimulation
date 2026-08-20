"""
CMA-ES (Covariance Matrix Adaptation Evolution Strategy) as a plain
ask/tell loop over a box-bounded real vector. No Qt, no game_specific,
no knowledge of what a match is -- the caller evaluates whatever it
likes and hands the numbers back.

Why this and not a grid or a genetic algorithm. The strategy fitness
landscape is *piecewise-constant* in the discrete fields (nudging a
priority does nothing until it flips a rule ordering, then everything
changes at once) but genuinely continuous-ish in the timing and range
fields, which is the half searched here. On that half there is no
gradient to follow -- the evaluation is a noisy 150-second simulation,
not a formula -- so the method has to be derivative-free, has to cope
with an evaluation whose repeat-to-repeat noise is a few points, and has
to learn the *shape* of the landscape rather than assume the axes are
independent (a longer `min_duration` and a longer `cooldown` interact:
both keep a rule on the robot). CMA-ES is the standard answer to exactly
that description, and at the dimensionality here (a handful of fields)
it converges in tens of generations rather than thousands.

Written out rather than taking a dependency on `cma`: this is ~120 lines
of the published reference algorithm, the team already has to install
numpy, and a search whose behaviour we may need to explain to a
high-school student is better read than pip-installed.

Two deliberate simplifications versus a production implementation:

* The eigendecomposition is redone on every `tell` rather than amortized
  over O(N) generations. At N of a dozen that costs microseconds against
  an evaluation costing minutes -- the lazy update is an optimization for
  problems this is not.
* Out-of-bounds samples are repaired by *clipping*, and the clipped point
  is what gets evaluated and what feeds the distribution update. This
  biases the distribution toward an active bound (a known property of
  clip-repair) which is acceptable here because the bounds are physical
  rather than arbitrary -- a `cooldown` wants to be able to sit at 0.

Minimization, by convention. A caller maximizing a score negates.
"""
from __future__ import annotations

import math

import numpy as np


class CMAES:
    """Ask/tell CMA-ES over a box.

    `x0` seeds the distribution mean -- for a strategy search that is the
    hand-written strategy being improved, so generation 0 is a
    neighbourhood of something already known to work rather than a random
    draw. `sigma0` is the initial step size *in the units of x*, so a
    caller working in a normalized [0, 1] box should pass something like
    0.25 (a quarter of each axis's range) and one working in raw units
    has to scale per-axis itself -- which is why `param_search` normalizes
    first.
    """

    def __init__(
        self, x0, sigma0: float, *, lower=None, upper=None,
        population_size: int | None = None, seed: int = 0,
    ):
        self.xmean = np.asarray(x0, dtype=float).copy()
        if self.xmean.ndim != 1 or self.xmean.size == 0:
            raise ValueError("x0 must be a non-empty 1-D vector")
        if not (sigma0 > 0.0):
            raise ValueError(f"sigma0 must be positive, got {sigma0!r}")

        n = self.n = int(self.xmean.size)
        self.sigma = float(sigma0)
        self.lower = None if lower is None else np.broadcast_to(np.asarray(lower, float), (n,)).copy()
        self.upper = None if upper is None else np.broadcast_to(np.asarray(upper, float), (n,)).copy()

        # Population and recombination weights: Hansen's defaults. The
        # log-spaced weights mean the best sample counts for a few times
        # what the mu-th does, which is what makes the mean move usefully
        # on a noisy objective instead of averaging the signal away.
        self.population_size = int(population_size or 4 + int(3 * math.log(n)))
        if self.population_size < 2:
            raise ValueError("population_size must be at least 2")
        self.mu = self.population_size // 2
        weights = math.log(self.mu + 0.5) - np.log(np.arange(1, self.mu + 1))
        self.weights = weights / weights.sum()
        self.mueff = 1.0 / np.sum(self.weights ** 2)

        # Adaptation rates.
        self.cc = (4 + self.mueff / n) / (n + 4 + 2 * self.mueff / n)
        self.cs = (self.mueff + 2) / (n + self.mueff + 5)
        self.c1 = 2 / ((n + 1.3) ** 2 + self.mueff)
        self.cmu = min(1 - self.c1, 2 * (self.mueff - 2 + 1 / self.mueff) / ((n + 2) ** 2 + self.mueff))
        self.damps = 1 + 2 * max(0.0, math.sqrt((self.mueff - 1) / (n + 1)) - 1) + self.cs
        self.chi_n = math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n ** 2))

        self.pc = np.zeros(n)
        self.ps = np.zeros(n)
        self.C = np.eye(n)
        self._B = np.eye(n)
        self._D = np.ones(n)
        self._inv_sqrt_C = np.eye(n)

        self.rng = np.random.default_rng(seed)
        self.generation = 0
        self.evaluations = 0
        self.best_x = self.xmean.copy()
        self.best_fitness = float("inf")

    # -- the loop ---------------------------------------------------------

    def ask(self) -> list:
        """One generation of candidate vectors, already clipped into the
        box. Returned as a list of arrays; `tell` wants these same
        (clipped) vectors back alongside their fitnesses."""
        z = self.rng.standard_normal((self.population_size, self.n))
        samples = self.xmean + self.sigma * (z * self._D) @ self._B.T
        return [self._clip(row) for row in samples]

    def tell(self, solutions, fitnesses) -> None:
        """Update the distribution from one generation's results.
        `fitnesses` is minimized."""
        xs = np.asarray(solutions, dtype=float)
        f = np.asarray(fitnesses, dtype=float)
        if xs.shape != (len(f), self.n):
            raise ValueError(f"expected {self.n}-vectors, got array of shape {xs.shape}")
        if len(f) < self.mu:
            raise ValueError(f"need at least mu={self.mu} evaluated solutions, got {len(f)}")

        # A failed evaluation arrives as +inf and must not poison the mean:
        # argsort puts them last, and they are only ever in the top mu if
        # the whole generation failed -- which the caller should have
        # stopped for.
        order = np.argsort(f, kind="stable")
        xs, f = xs[order], f[order]
        self.evaluations += len(f)
        self.generation += 1

        if f[0] < self.best_fitness:
            self.best_fitness = float(f[0])
            self.best_x = xs[0].copy()

        x_old = self.xmean
        self.xmean = self.weights @ xs[: self.mu]

        y = (self.xmean - x_old) / self.sigma
        self.ps = (1 - self.cs) * self.ps + math.sqrt(self.cs * (2 - self.cs) * self.mueff) * (self._inv_sqrt_C @ y)
        ps_norm = float(np.linalg.norm(self.ps))
        # Stall the rank-one update when ||ps|| says the step size is
        # about to be raised sharply, so a single lucky generation cannot
        # stretch C along a direction that was noise.
        decay = 1 - (1 - self.cs) ** (2 * self.evaluations / self.population_size)
        h_sigma = ps_norm / math.sqrt(max(decay, 1e-12)) / self.chi_n < 1.4 + 2 / (self.n + 1)
        self.pc = (1 - self.cc) * self.pc + h_sigma * math.sqrt(self.cc * (2 - self.cc) * self.mueff) * y

        steps = (xs[: self.mu] - x_old) / self.sigma
        self.C = (
            (1 - self.c1 - self.cmu) * self.C
            + self.c1 * (np.outer(self.pc, self.pc) + (0.0 if h_sigma else self.cc * (2 - self.cc)) * self.C)
            + self.cmu * (steps.T @ (self.weights[:, None] * steps))
        )
        self.sigma *= math.exp(min(1.0, (self.cs / self.damps) * (ps_norm / self.chi_n - 1)))
        self._update_eigensystem()

    def should_stop(self, *, tol_sigma: float = 1e-6) -> str | None:
        """A human-readable reason to stop early, or None. Only the
        cheap, unambiguous conditions: a collapsed step size means the
        search has converged, a non-finite one means it has diverged and
        every further generation is wasted wall time."""
        if not np.isfinite(self.sigma) or not np.all(np.isfinite(self.C)):
            return "distribution diverged (non-finite sigma or covariance)"
        if self.sigma * float(np.max(self._D)) < tol_sigma:
            return f"converged (step size below {tol_sigma:g})"
        return None

    # -- internals --------------------------------------------------------

    def _clip(self, x):
        if self.lower is not None:
            x = np.maximum(x, self.lower)
        if self.upper is not None:
            x = np.minimum(x, self.upper)
        return x

    def _update_eigensystem(self) -> None:
        self.C = np.triu(self.C) + np.triu(self.C, 1).T  # re-symmetrize against float drift
        values, vectors = np.linalg.eigh(self.C)
        # Numerically negative eigenvalues appear on a nearly-singular C
        # (every sample agreeing along one axis, which happens as soon as
        # a bound is active). Flooring them keeps sqrt real and the search
        # alive rather than raising in the middle of an overnight run.
        values = np.maximum(values, 1e-20)
        self._D = np.sqrt(values)
        self._B = vectors
        self._inv_sqrt_C = (vectors / self._D) @ vectors.T
