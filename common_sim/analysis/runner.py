"""
Streaming/cancellable parallel runner, generic over `fn(item) -> result`
-- no Qt, no game_specific. Used by both the SWEEP tab (via a QThread
worker) and headless scripts.

Submission is bounded to `max_workers * pending_per_worker` outstanding
futures rather than submitting everything up front: on a large sweep
that keeps abort near-instant (no giant queue to drain) and keeps peak
memory flat.
"""
from __future__ import annotations

import os
import threading
from concurrent.futures import FIRST_COMPLETED, ProcessPoolExecutor, wait
from typing import Any, Callable, Iterable, Iterator, Optional


class CancelToken:
    """threading.Event wrapper, set from the Qt main thread (or any
    caller) to ask a running iter_results/run_all to stop early."""

    def __init__(self):
        self._event = threading.Event()

    def cancel(self) -> None:
        self._event.set()

    @property
    def cancelled(self) -> bool:
        return self._event.is_set()


def default_worker_count() -> int:
    """One core held back from the worker count so the Qt event loop
    stays responsive."""
    return max(1, (os.cpu_count() or 2) - 1)


def iter_results(
    fn: Callable[[Any], Any], items: Iterable[Any], *,
    parallel: bool = True, max_workers: Optional[int] = None,
    cancel: Optional[CancelToken] = None, pending_per_worker: int = 2,
) -> Iterator[tuple]:
    """Yield (index, result) as each item finishes -- index is the
    position of the item in `items`, NOT completion order, so a
    consumer can re-sort. Stops early (yielding fewer than len(items)
    results) if `cancel` is set before all items finish; already-
    executing futures cannot be interrupted -- worst case the caller
    waits for one more item to finish."""
    items = list(items)

    if not parallel:
        for index, item in enumerate(items):
            if cancel is not None and cancel.cancelled:
                return
            yield index, fn(item)
        return

    workers = max_workers or default_worker_count()
    max_pending = max(1, workers * pending_per_worker)

    with ProcessPoolExecutor(max_workers=workers) as executor:
        pending: dict = {}
        next_index = 0

        def _submit_more():
            nonlocal next_index
            while next_index < len(items) and len(pending) < max_pending:
                if cancel is not None and cancel.cancelled:
                    break
                future = executor.submit(fn, items[next_index])
                pending[future] = next_index
                next_index += 1

        _submit_more()
        while pending:
            if cancel is not None and cancel.cancelled:
                for future in list(pending):
                    future.cancel()
                executor.shutdown(wait=False, cancel_futures=True)
                return
            done, _ = wait(pending.keys(), return_when=FIRST_COMPLETED)
            for future in done:
                index = pending.pop(future)
                yield index, future.result()
            _submit_more()


def run_all(fn: Callable[[Any], Any], items: Iterable[Any], **kw) -> list:
    """iter_results collected, re-sorted by item index."""
    results = list(iter_results(fn, items, **kw))
    results.sort(key=lambda pair: pair[0])
    return [result for _, result in results]
