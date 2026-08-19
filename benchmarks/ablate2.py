"""Split the controller tick: triggers vs tactic, and inside the tactic,
navigation vs target selection."""
import sys
import time

sys.argv = ["x"]
from bench import build_jobs  # noqa: E402
from common_sim.control import navigation, tactics  # noqa: E402
from common_sim.control.strategy import StrategyController  # noqa: E402
from game_specific.reefscape.sweep_trial import build_match_for_job  # noqa: E402

B = {}


def bump(k, v):
    B[k] = B.get(k, 0.0) + v


# --- triggers vs tactic ------------------------------------------------
_eval_all = StrategyController._evaluate_all
_orig_tick = StrategyController.tick


def evaluate_all(self, ctx):
    t = time.perf_counter()
    try:
        return _eval_all(self, ctx)
    finally:
        bump("triggers", time.perf_counter() - t)


def tick(self, ctx):
    t = time.perf_counter()
    try:
        return _orig_tick(self, ctx)
    finally:
        bump("controller_total", time.perf_counter() - t)


StrategyController._evaluate_all = evaluate_all
StrategyController.tick = tick

# --- NavigateTo.tick (drive + replan) ----------------------------------
_nav_tick = navigation.NavigateTo.tick


def nav_tick(self, ctx):
    t = time.perf_counter()
    try:
        return _nav_tick(self, ctx)
    finally:
        bump("navigate_to", time.perf_counter() - t)


navigation.NavigateTo.tick = nav_tick

# --- per-tactic _provide_target (the target-choosing logic) ------------
for cls_name in ("Collect", "Score", "Defend"):
    cls = getattr(tactics, cls_name)
    if not hasattr(cls, "_provide_target"):
        continue
    orig = cls._provide_target

    def make(orig=orig, name=cls_name):
        def wrapper(self, ctx):
            t = time.perf_counter()
            try:
                return orig(self, ctx)
            finally:
                bump(f"provide_target.{name}", time.perf_counter() - t)
        return wrapper

    cls._provide_target = make()

# --- clear_standoff ----------------------------------------------------
_clear = navigation.clear_standoff


def clear_standoff(*a, **k):
    t = time.perf_counter()
    try:
        return _clear(*a, **k)
    finally:
        bump("clear_standoff", time.perf_counter() - t)


navigation.clear_standoff = clear_standoff
tactics.clear_standoff = clear_standoff if hasattr(tactics, "clear_standoff") else None
if tactics.clear_standoff is None:
    del tactics.clear_standoff

for target in ("cycle_v_cycle", "evasive_v_def"):
    B.clear()
    job = next(job for name, job in build_jobs() if name == target)
    match, _, _ = build_match_for_job(job)
    t0 = time.perf_counter()
    while not match.ended:
        match.step(job.dt)
    wall = time.perf_counter() - t0
    ctrl = B.get("controller_total", 0.0)
    print(f"\n{target}: {wall:.2f}s wall, controller {ctrl:.2f}s ({ctrl / wall:.1%})")
    for k, v in sorted(B.items(), key=lambda kv: -kv[1]):
        if k == "controller_total":
            continue
        print(f"  {k:<28} {v:6.2f}s  {v / wall:5.1%} of wall   {v / ctrl:5.1%} of controller")
