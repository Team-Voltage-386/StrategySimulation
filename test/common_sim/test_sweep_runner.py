from __future__ import annotations

import pytest

from common_sim.analysis.runner import CancelToken, iter_results, run_all


def _square(x: int) -> int:
    return x * x


def _boom(x: int) -> int:
    if x == 3:
        raise ValueError("boom")
    return x


def test_iter_results_yields_every_item_once_sequential():
    results = dict(iter_results(_square, range(10), parallel=False))
    assert results == {i: i * i for i in range(10)}


def test_iter_results_yields_every_item_once_parallel():
    results = dict(iter_results(_square, range(10), parallel=True, max_workers=2))
    assert results == {i: i * i for i in range(10)}


def test_run_all_preserves_order_sequential():
    assert run_all(_square, range(6), parallel=False) == [i * i for i in range(6)]


def test_run_all_preserves_order_parallel():
    assert run_all(_square, range(6), parallel=True, max_workers=3) == [i * i for i in range(6)]


def test_precancelled_token_returns_early_sequential():
    token = CancelToken()
    token.cancel()
    results = list(iter_results(_square, range(100), parallel=False, cancel=token))
    assert len(results) < 100


def test_precancelled_token_returns_early_parallel():
    token = CancelToken()
    token.cancel()
    results = list(iter_results(_square, range(100), parallel=True, max_workers=2, cancel=token))
    assert len(results) < 100


def test_raising_fn_propagates_sequential():
    with pytest.raises(ValueError):
        list(iter_results(_boom, range(6), parallel=False))


def test_raising_fn_propagates_parallel():
    with pytest.raises(ValueError):
        list(iter_results(_boom, range(6), parallel=True, max_workers=2))
