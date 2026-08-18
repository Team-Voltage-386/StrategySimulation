# Packaging

Builds `apps/run_reefscape.py` into a standalone Windows folder via
[PyInstaller](https://pyinstaller.org/), so teammates can run the
viewer with no Python install.

## Build

```
packaging\build_release.bat
```

First run creates `.build_venv\` (repo-root, gitignored) and installs
`requirements.txt` + `pyinstaller` into it -- deliberately *not* your
everyday Anaconda env, since PyInstaller sweeps in everything
importable it can find, and a full conda base env drags in
Jupyter/boto3/numba/bokeh and similar unrelated packages (turned an
~88MB zip into 625MB in testing). Re-run the script any time; it
reuses the venv and only reinstalls if `pyinstaller` is missing from
it.

Output:
- `dist\SparkySim\` -- the runnable folder (`SparkySim.exe` +
  `_internal\`)
- `dist\SparkySim_<timestamp>.zip` -- what to hand to a teammate

Both `build\` and `dist\` are gitignored; rebuild locally rather than
committing artifacts.

## Running the built exe

Unzip anywhere and double-click `SparkySim.exe`. It opens with a
console window (same as `run.bat`) so a crash's traceback is visible
instead of silently vanishing.

## What's bundled

`packaging/sparky_sim.spec` bundles `game_specific/reefscape/strategies/*.json`
(loaded at runtime) alongside the code. `gui_utils/resources/icons/`
is *not* bundled -- nothing in the codebase currently reads from it.
If that changes, add it to the spec's `datas` list.

## Only `run_reefscape.py` is packaged

`run_match.py` and `run_strategy_sweep.py` (see the main
[README](../README.md)) aren't included -- they're dev/reference
entry points, not something to hand teammates. Add another
`Analysis`/`EXE`/`COLLECT` block to the spec if that changes.
