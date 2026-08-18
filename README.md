# sparky-sim

Early-season robot design-trade simulator for FRC. Not a substitute for
WPILib's own sim tools — this is for comparing *robot concepts* (cycle
time, capacity, speed, intake/deposit timing) before code or CAD
exists, at faster-than-real-time, across many randomized trials. See
[ARCHITECTURE.md](ARCHITECTURE.md) for the package layout and design
rationale.

## Setup

```
pip install -r requirements.txt
```

A bare `python`/`py` on this machine can resolve to the Microsoft
Store stub, which exits immediately without the packages this app
needs. Use a real interpreter (e.g. an Anaconda install) explicitly if
`python -m apps...` exits with no output.

## The apps

Three entry points under `apps/`, in the order you'd actually reach
for them:

### `run_reefscape.py` — the interactive REEFSCAPE viewer

```
run.bat
```

This is the real tool: the actual 2025 REEFSCAPE field, pieces, and
scoring, driven live. Launch it via `run.bat`, not `python
apps/run_reefscape.py` directly — the batch file sets `PYTHONPATH` to
the repo root first, which plain `python apps/run_reefscape.py` won't
do on its own (Python puts the *script's* directory on `sys.path`, not
the current directory, so `common_sim`/`game_specific`/`gui_utils`
fail to import). If you want to run it another way, `python -m
apps.run_reefscape` from the repo root works too.

Three tabs:

- **MATCH** — roster/config panel on the left (add extra AI-driven
  robots per alliance, tune a robot's speed/capacity/intake timing),
  the live field in the center with a play/pause/reset transport bar,
  and telemetry/scoring/controls-reference on the right. Drive the
  primary robot yourself (WASD + Space/F, or an Xbox controller if one
  is connected) or check "AI drives primary robot" to hand it to a
  strategy too. **Pause** (the transport bar's ⏸ button, or a
  gamepad's Start button) to drag the transport bar's slider and scrub
  back through the match — every robot's recorded
  position/heading/velocity is replayed at whatever point you scrub
  to. Hit ▶ to resume live from exactly where you paused (not from
  wherever you last scrubbed to).
- **STRATEGY** — edit a robot's `Rule[]` list (trigger → tactic,
  priority, dwell/cooldown) and fallback tactic, with a live
  state-machine graph showing the arbiter's actual transitions as the
  match runs. Changes apply on the next RESET.
- **SWEEP** — configure a roster the same way MATCH does (its own,
  independent roster — editing MATCH mid-sweep can't change it), then
  tick which `RobotCharacteristics` fields (or `strategy`) to vary and
  how to sample them: a MIN/MAX/POINTS range, an explicit comma-
  separated list, or a checkable list of choices for `strategy`. The
  TOTAL RUNS readout updates live as you add variables/repetitions;
  hit EXECUTE to run the grid across worker processes, watch the
  progress bar (ABORT stops it within about one in-flight match per
  worker), and read the RESULTS table or the PLOTS tab (auto-picks a
  line/heatmap/faceted-heatmap depending on how many variables you
  swept). Double-click any results row — or right-click → "REPLAY IN
  MATCH TAB" — to re-run that exact trial and watch/scrub it on the
  MATCH tab.

### `run_match.py` — minimal placeholder viewer

```
python -m apps.run_match
```

A single keyboard-driven robot over a small placeholder field (one
made-up scoring region, no real game rules). This predates
`game_specific/reefscape` — it's what proved out the
`common_sim` + `gui_utils.FieldCanvas` pipeline end-to-end before any
real game existed to plug in, per
[ARCHITECTURE.md](ARCHITECTURE.md)'s build sequencing. There's no
reason to reach for this over `run_reefscape.py` for anything
REEFSCAPE-related now; it's mainly useful as a minimal reference if
you're touching the GUI/input plumbing itself and want to isolate it
from REEFSCAPE-specific code.

### `run_strategy_sweep.py` — headless Monte Carlo sweep

```
python -m apps.run_strategy_sweep
```

No GUI. Runs a batch of matches across a swept parameter grid
(strategy × max_speed, by default) via
`common_sim/analysis/monte_carlo.py`, multiprocessed, and prints a
results table + per-strategy score summary. This is the pattern to
copy for any other "run N trials varying some robot/strategy
parameter and compare scores" question — `strategy` here is just
another named param in the sweep, no special-casing needed.

## Tests

```
pytest
```

## Distributing to teammates (no Python required)

```
packaging\build_release.bat
```

Builds a standalone `dist\SparkySim\SparkySim.exe` via PyInstaller (from
an isolated `.build_venv`, not your everyday environment, so unrelated
packages don't bloat the build) and zips it to `dist\SparkySim_<timestamp>.zip`.
Send that zip to a teammate; they unzip it anywhere and run
`SparkySim.exe` -- no Python install needed. It only packages the
`run_reefscape.py` viewer. See [packaging/README.md](packaging/README.md).
