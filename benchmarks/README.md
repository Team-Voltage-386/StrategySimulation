# benchmarks

Measurement harness for performance and fidelity work. Nothing here is
imported by the app; these are scripts you run when you need to answer
"did that change make it faster" or "is a cheaper setting still honest".

Run from the repo root with the root on `PYTHONPATH`:

```
set PYTHONPATH=%CD%
python benchmarks\bench.py baseline
```

## Three lessons that cost a day to learn

**1. A fingerprint, not a score, is what proves a change is safe.**
`bench.py` hashes the whole event stream plus final scores and poses. Any
optimisation that claims to be behaviour-preserving must leave the
combined fingerprint byte-identical across all six matches. A matching
final score proves nothing -- two different simulations reach 230 points
all the time.

**2. cProfile's ranking is not the wall clock's.** A profile run made
`plan_path` look like ~35% of runtime. Replacing `estimate_travel_time`'s
full A* with a straight-line estimate -- deleting 75% of all `plan_path`
calls -- bought 1.01x. Deterministic profilers inflate call-heavy Python
by roughly 3.5x here while leaving the C physics step alone. Use
`ablate.py` / `ablate2.py` (coarse `perf_counter` buckets) to decide
where time actually goes; use cProfile only to find *call counts*.

**3. Run-to-run variance on a laptop is about 4%.** Two runs of identical
code measured 52.4s and 54.7s. Any claimed speedup below ~1.1x from a
single pair of runs is noise. `ab.py` interleaves the two arms in one
process so thermal drift hits both equally, and reports medians.

## The scripts

| script | what it answers |
|---|---|
| `bench.py` | Throughput + determinism fingerprint over six mixed matches. The gate for any perf change. |
| `ab.py` | Interleaved A/B of the navigation caches on vs off (via `__wrapped__`). The honest way to measure a small speedup. |
| `ablate.py` | Undistorted wall-clock split of `Match.step`: controllers vs physics vs protection/pins vs robot I/O. |
| `ablate2.py` | Splits the controller tick further: triggers vs tactic, `NavigateTo.tick`, per-tactic `_provide_target`, `clear_standoff`. |
| `dt_study.py` | Timestep fidelity, design grid (strategy x speed, no defender). Rank preservation vs the seed noise floor. |
| `dt_defense_study.py` | Timestep fidelity, defensive grid. Persists raw rows to `dt_defense_rows.json`. |
| `pin_probe.py` | Why the pin rule never fires: per-condition counters for `is_pinning` plus the peak pin clock. |
| `dt_defense_rows.json` | 648 raw match rows from the defensive study, so follow-ups need no re-run. |

## What these measured (Aug 2026, i7-10710U 6c/12t)

- Memoizing `_inflate`, `_clearance_for_goal`, `polygon_distance`: **1.067x**,
  fingerprint identical. Larger cache sizes measured *slower*.
- Timestep: **dt=1/30 recommended for search** (1.52x on cycle matches,
  1.36x with defenders). Across 1,800 matches every config pair separated
  by >8 standard errors ranked identically at every timestep tested.
  Score bias is -0.5 to -1.2 points, monotonic, ~5x under the noise floor.
- Do **not** change `SWEEP_DT` from 1/60 -- `TelemetryRecorder` assumes
  60Hz and MATCH-tab replay would stop matching its results row. A search
  needs its own timestep constant, which is now
  `sweep_trial.SEARCH_DT` (1/30) and is what `apps/run_param_search.py`
  runs at by default. Winners get re-run at 1/60 for inspection.
- Parameter search (`apps/run_param_search.py`) **must** be read off its
  held-out confirmation, not its own best-of-N. First real run: 6
  generations x 6 candidates over **4 seeds** reported +9.5 points and
  kept **+0.2** on 12 fresh seeds. Seeds per candidate, not generations,
  is the knob that buys signal here. The SEARCH tab exists so this is
  hard to get wrong: it defaults to 16 matches per candidate, warns below
  8, confirms by default, and shows the held-out figure as the headline
  with the search's own best-of-N greyed out beneath it. That run is also
  what the screenshots in `docs/param_search_guide.html` show.
- Pin rule: live and unit-tested, but unreachable in match play -- peak
  clock 0.250s against a 3.0s limit, so `Defend._respect_pin_limit` (which
  releases at 2.1s) never engages either. Consequence: the sim can express
  *denial* defense but not *impedance* defense.

Use `python -m apps.run_calibration` to find out what your own machine can
do before committing to a long run.
