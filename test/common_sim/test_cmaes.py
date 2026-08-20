"""CMA-ES is written out rather than pip-installed, so these pin the
algorithm's contract: it finds a minimum of a cheap analytic function, it
respects its box, and it survives the degenerate inputs a strategy search
will actually hand it (one dimension, an optimum sitting on a bound, a
generation full of failures)."""
from __future__ import annotations

import math

import numpy as np
import pytest

from common_sim.analysis.cmaes import CMAES


def _minimize(fn, x0, sigma=0.3, generations=60, **kw):
    optimizer = CMAES(x0, sigma, **kw)
    for _ in range(generations):
        candidates = optimizer.ask()
        optimizer.tell(candidates, [fn(x) for x in candidates])
        if optimizer.should_stop() is not None:
            break
    return optimizer


def test_finds_the_minimum_of_a_separable_quadratic():
    target = np.array([0.7, 0.2, 0.45])
    optimizer = _minimize(lambda x: float(np.sum((x - target) ** 2)), [0.5, 0.5, 0.5], seed=1)
    assert optimizer.best_x == pytest.approx(target, abs=1e-3)
    assert optimizer.best_fitness < 1e-6


def test_finds_the_minimum_of_a_rotated_ill_conditioned_quadratic():
    """The reason for CMA-ES over coordinate search: the axes interact.
    A `min_duration` and a `cooldown` both keep a rule on the robot, so
    the real landscape is not axis-aligned either."""
    angle = 0.6
    rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    scale = np.array([1.0, 100.0])

    def fn(x):
        y = rotation @ (np.asarray(x) - np.array([0.3, 0.8]))
        return float(np.sum(scale * y ** 2))

    optimizer = _minimize(fn, [0.5, 0.5], generations=200, seed=2)
    assert optimizer.best_x == pytest.approx([0.3, 0.8], abs=5e-3)


def test_never_proposes_a_point_outside_the_box():
    optimizer = CMAES([0.5, 0.5], 0.9, lower=[0.0, 0.25], upper=[1.0, 0.75], seed=3)
    for _ in range(10):
        candidates = optimizer.ask()
        for x in candidates:
            assert 0.0 <= x[0] <= 1.0
            assert 0.25 <= x[1] <= 0.75
        optimizer.tell(candidates, [float(np.sum(x ** 2)) for x in candidates])


def test_converges_onto_a_bound_when_the_optimum_is_outside_it():
    """A `cooldown` whose best value is "as small as possible" is the
    common case, and clip-repair has to land on the bound rather than
    stalling next to it or blowing up the covariance."""
    optimizer = _minimize(lambda x: float(np.sum(x ** 2)), [0.6], sigma=0.2,
                          lower=0.0, upper=1.0, generations=80, seed=4)
    assert optimizer.best_x[0] == pytest.approx(0.0, abs=1e-3)


def test_one_dimensional_search_is_not_a_special_case():
    optimizer = _minimize(lambda x: float((x[0] - 0.25) ** 2), [0.8], sigma=0.2, seed=5)
    assert optimizer.best_x[0] == pytest.approx(0.25, abs=1e-3)


def test_infinite_fitnesses_sort_last_and_do_not_poison_the_mean():
    """A failed trial arrives as +inf. The mean must stay finite and must
    move toward the candidates that actually ran."""
    optimizer = CMAES([0.5, 0.5], 0.2, seed=6)
    for _ in range(5):
        candidates = optimizer.ask()
        fitnesses = [float("inf")] * len(candidates)
        fitnesses[0] = 1.0
        fitnesses[1] = 2.0
        optimizer.tell(candidates, fitnesses)
        assert np.all(np.isfinite(optimizer.xmean))
    assert optimizer.best_fitness == 1.0


def test_reports_convergence_rather_than_running_forever():
    optimizer = _minimize(lambda x: float(np.sum(x ** 2)), [0.1, 0.1], sigma=0.05,
                          generations=500, seed=7)
    assert optimizer.should_stop() is not None
    assert "converged" in optimizer.should_stop()


def test_same_seed_gives_the_same_search():
    """A search that cannot be re-run is a search whose result cannot be
    checked."""
    fn = lambda x: float(np.sum((x - 0.3) ** 2))  # noqa: E731
    a = _minimize(fn, [0.5, 0.5], generations=15, seed=11)
    b = _minimize(fn, [0.5, 0.5], generations=15, seed=11)
    assert a.best_x == pytest.approx(b.best_x)
    assert a.best_fitness == pytest.approx(b.best_fitness)


@pytest.mark.parametrize("kwargs", [
    {"x0": [], "sigma0": 0.1},
    {"x0": [0.5], "sigma0": 0.0},
    {"x0": [0.5], "sigma0": -1.0},
    {"x0": [0.5], "sigma0": 0.1, "population_size": 1},
])
def test_rejects_nonsense_configuration(kwargs):
    with pytest.raises(ValueError):
        CMAES(kwargs.pop("x0"), kwargs.pop("sigma0"), **kwargs)


def test_tell_rejects_a_mismatched_generation():
    optimizer = CMAES([0.5, 0.5], 0.2, seed=8)
    candidates = optimizer.ask()
    with pytest.raises(ValueError):
        optimizer.tell(candidates, [0.0] * (len(candidates) - 1))
    with pytest.raises(ValueError):
        optimizer.tell([[0.0, 0.0]], [0.0])
