# sparky-sim

Early-season robot design-trade simulator for FRC. Not a substitute for
WPILib's own sim tools — this is for comparing *robot concepts* (cycle
time, capacity, speed, intake/deposit timing) before code or CAD
exists, at faster-than-real-time, across many randomized trials. See
[ARCHITECTURE.md](ARCHITECTURE.md) for the package layout and design
rationale.

## Setup (new students start here)

You only need to install Python. Anaconda isn't needed, and setup.bat
installs everything else.

1. **Install Python 3.13** from
   [python.org/downloads](https://www.python.org/downloads/): pick the
   "Windows installer (64-bit)" for the latest 3.13.x. On the installer's
   first screen, tick **"Add python.exe to PATH"**, then click
   *Install Now*.
2. **Get the code**: clone this repo (GitHub Desktop is fine) or
   download and unzip it.
3. **Double-click `setup.bat`** in the sparky-sim folder. It finds your
   Python, creates a private environment in `.venv\`, and installs the
   pinned packages from `requirements.txt` (about 150 MB the first
   time). It finishes with a check that prints `preflight: ready`.
4. **Double-click `run.bat`** to start the REEFSCAPE viewer.

Re-run `setup.bat` whenever `requirements.txt` changes after a pull. It's
safe to run as often as you like, and it rebuilds `.venv` if it breaks
(for example, after you upgrade Python). To start completely fresh, delete
the `.venv` folder and run it again.

To run the tests:

```
.venv\Scripts\python -m pytest -q
```

**If something goes wrong:**
- *Typing `python` opens the Microsoft Store.* That's a Windows
  placeholder, not a real Python. Install from python.org as in step 1.
  You can also turn the placeholder off: Settings → Apps → Advanced app
  settings → App execution aliases → switch off both `python` entries.
- *pip says "building wheel" or "Microsoft Visual C++ is required".*
  Your Python is newer than the pinned packages support. Install 3.13 and
  re-run `setup.bat`.
- *Downloads fail.* Some school networks block pypi.org. Run setup from
  home or on a phone hotspot.

In your editor (VS Code, PyCharm), set the interpreter to
`.venv\Scripts\python.exe` so it sees the same packages as the launchers.

**How the launchers pick Python:** they use `.venv` if it's there.
Otherwise they try the common Anaconda/python.org install locations,
then `py`, then `python`, and take the first one that both starts *and*
has the packages. Nothing needs to be on PATH and no virtualenv needs
activating. An existing Anaconda setup also works: `pip install -r
requirements.txt` into it and skip `setup.bat`.

**Bumping a dependency:** change its pin in `requirements.txt`, re-run
`setup.bat`, run the tests, and commit. CI runs on Python 3.13 too, so
it tests the same versions the students have.

To check a machine without launching anything:

```
python packaging\preflight.py
```

It prints which interpreter it is, what's missing, and the exact `pip`
line to fix it. Worth knowing why it exists: a bare `python` on Windows
very often resolves to the Microsoft Store placeholder, which exits
immediately, prints nothing, and closes the console with it -- so a
double-clicked launcher used to look like nothing happened at all. The
launchers now skip that stub, and say so if it's the only Python
present.

## The apps

Entry points under `apps/`, in the order you'd actually reach for
them:

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

### `run_salvage.py` — drive the dry-run game

```
run_salvage.bat
```

SALVAGE 2027 is an **invented** game, written to test the claim that a
new season is a new `game_specific/` package and nothing else — see
[DRY_RUN_LOG.md](DRY_RUN_LOG.md) for what that exercise found, which
included two framework bugs that had been latent in REEFSCAPE the whole
time. This is a small MATCH-only window for playing it: the field, one
robot you drive, three AI, and four panels.

The panels are the point. SALVAGE's rules are mostly about *timing*,
which no field drawing shows: the deep hold pays 8 in TELEOP and 3 in
AUTO while the wall hold goes the other way, the REACTOR pays 10 then 7
but there are only ten CELLs on the field all match, and the BEACON's
six slots are shared between the alliances so filling it scores and
denies at once. SCORING lists every action's value *as of right now*,
WHAT IS LEFT counts down everything finite, and WHERE YOU ARE names the
zone under your robot and says whether a deposit would actually land.

| Keyboard | Xbox controller | |
|---|---|---|
| `W A S D` | Left stick | Drive (field-relative) |
| `←` / `→` | Right stick X, or LB / RB | Rotate — the bumpers turn at a fixed rate, which is easier for squaring up |
| `Space` | A | Intake — hold it while sitting on a depot or bay |
| `F` | RT (half-press is enough) | Deposit — hold it while sitting in a scoring zone |
| `P` | Start | Pause / resume |
| `R` | Back | Restart the match |

Both devices are live at the same time; there is no mode to pick. (This
differs from `run_reefscape.py`, which picks the gamepad whenever one is
*detected* — meaning a controller merely plugged into the machine
silently disables `W A S D` there.) `run_salvage.bat --check` reports
whether the field is valid and whether a controller is being seen,
without opening a window.

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
