# sparky-sim Architecture

Early-season robot design-trade simulator for FRC. Not a substitute for
WPILib's own sim tools — this is for comparing *robot concepts*
(cycle time, capacity, speed, intake/deposit timing) before code or CAD
exists, at faster-than-real-time, across many randomized trials.

## Guiding constraint

Everything under `common_sim/` and `gui_utils/` must be completely
game-agnostic. A new game (post-reveal) should only require adding a
new `game_specific/<game>/` package — a field layout, game pieces, and
a scoring table — with **zero** changes to physics, GUI, input, or
Monte Carlo code. That contract is the two-day goal; it's a discipline
enforced by import direction, not a suggestion:

```
game_specific/*  --> imports from --> common_sim/*, gui_utils/*
common_sim/*     --> never imports from --> game_specific/*
gui_utils/*      --> never imports from --> game_specific/*
```

If code in `common_sim` ever needs to branch on "which game is this,"
that's a sign the abstraction is wrong — it belongs in a game-specific
subclass instead.

## Tech stack decision

| Concern | Choice | Rejected alternative & why |
|---|---|---|
| Physics | **pymunk** | Matches the reference spikes; a 2D rigid-body + sensor-shape engine is the right fidelity level for this tool. |
| GUI shell | **PyQt** via `pyqtgraph.Qt` | Reuses `gui_utils/theme.py` (sci-fi QSS) and `overlay_panel.py` as-is from your other projects. Streamlit is dropped entirely — its rerun-the-whole-page-per-interaction model fights a stateful physics/game loop and made the `streamlit_demo.py` spike need a `time.sleep`+`st.rerun()` polling hack just to animate. |
| Field rendering | **Custom `QWidget`/QPainter canvas**, not an embedded pygame window | Embedding an SDL surface inside a Qt widget (via `winId()`) is fragile across Windows/Mac and buys nothing pygame's debug-draw gave the spikes — a QPainter canvas composes cleanly with `theme.py`'s QSS and `OverlayPanel`'s floating controls, and can be styled to match the sci-fi theme directly. |
| Charts | **pyqtgraph** for live in-match plots (already a `gui_utils` dependency); **matplotlib** acceptable for static post-hoc Monte Carlo report plots/exports | — |
| Keyboard/mouse input | Native Qt key/mouse events on the field canvas | — |
| Xbox controller input | **`pygame.joystick`, initialized headless** (`pygame.init()` without opening a display) | Full pygame is kept *only* for its joystick subsystem, not rendering — sidesteps ever needing a second window. |
| Monte Carlo batch runner | Plain Python + `multiprocessing.Pool` + `pandas` | No GUI import anywhere on this path, so it can run headless on a CI box. |

This drops `streamlit` from `requirements.txt` and keeps `pygame` only
for joystick polling. Flag if you'd rather keep the pygame-rendering
path as a lightweight fallback — I'm treating that as settled unless
you push back.

## Package layout

```
common_sim/
  geometry.py          Pose2d, Vector2 helpers on top of pymunk's vector type
  physics/
    engine.py           SimEngine: wraps pymunk.Space, fixed-timestep step(), headless-capable
    swerve.py            SwerveDriveModel: (vx, vy, omega) -> module states -> forces on chassis body
    tank_drive.py         (optional second drivetrain model, same interface as swerve)
  field/
    field_config.py      FieldConfig dataclass: dims, obstacles, ScoringRegion list, PieceSpawnRegion list
    game_piece.py         GamePiece base: pymunk body/shape + type tag
  robot/
    characteristics.py    RobotCharacteristics dataclass: speed, accel, size, capacity, intake/deposit time...
    robot.py               Robot base: chassis + mechanisms + pose + alliance + held-piece slots
    mechanisms.py          Intake / Manipulator base classes, parameterized by characteristics
  control/
    input_sources.py      InputSource ABC; KeyboardInput, GamepadInput -> normalized DriveCommand
    behavior.py             Behavior-tree-lite: Sequence/Parallel/DriveToPose/RunIntake/Wait/Branch nodes
    vision.py                 Simulated AprilTag pose estimation + piece detection, tunable noise/FOV/dropout
  match/
    match.py                Match orchestrator: clock, phase (auto/teleop/endgame), collision routing
    scoring.py                ScoringRules ABC: point table keyed by (action, phase)
    events.py                  Timestamped match event log, feeds telemetry + metrics
  analysis/
    monte_carlo.py          Batch trial runner: DOE sweep over RobotCharacteristics, multiprocessing
    metrics.py                 Per-match metric extraction (score, cycles, utilization, ...)
    results.py                   Aggregation into pandas DataFrame, summary stats

gui_utils/               (existing — extended, not replaced)
  theme.py, overlay_panel.py, telemetry_store.py, ...   reused as-is
  field_canvas.py         NEW: QPainter canvas rendering FieldConfig + live Robot/GamePiece state
  analysis_panels.py       NEW: Monte Carlo result dashboards (pyqtgraph/matplotlib)

game_specific/
  reefscape/                (existing stub, or whatever game is current)
    field.py                 concrete FieldConfig instance
    game_pieces.py             concrete GamePiece subclasses
    scoring.py                    concrete ScoringRules
    behaviors.py                    example autonomous / scripted-opponent routines

apps/
  run_match.py            Interactive single/multi-robot viewer (keyboard+gamepad, scripted alliance/opponents)
  run_monte_carlo.py       Headless batch CLI -> results file, optional dashboard launch

test/                     (existing) unit tests per package
```

## Core runtime model

**Fixed-timestep physics, decoupled render rate.** `SimEngine.step(dt)`
advances pymunk at a fixed substep (e.g. 1/120s) regardless of who's
calling it. The interactive app drives it from a `QTimer` at ~60Hz for
display; the Monte Carlo runner drives it in a tight loop with no
timer/vsync at all, which is what makes "faster than real time"
trivial rather than a separate code path.

**Collision routing goes through `Match`, not global handlers.** The
reference spikes registered `pymunk` collision handlers with module-
level globals (`held_piece`, `score`). `Match` owns that instead:
`common_sim` assigns collision-type IDs generically (piece / intake /
scoring-region), and `Match` dispatches begin/separate callbacks to
`Robot.try_intake(piece)` / `ScoringRegion.try_score(piece, phase)`.
Game-specific scoring point values never leak into physics code.

**Behavior scripting is one small framework, reused three ways.** A
lightweight composable node set (`Sequence`, `Parallel`, `DriveToPose`,
`RunIntake`, `RunManipulator`, `Wait`, `Branch`) drives: (1) autonomous
routines for the robot under test, (2) scripted alliance-partner
robots, and (3) scripted opponent robots. No separate "AI opponent"
system — it's the same `Behavior` tree attached to a different `Robot`
instance. This also gives a natural slot for "replay recorded human
input" later without a new abstraction.

**Vision is a swappable pose-estimation strategy, not a camera sim.**
Each `Robot` holds a `PoseEstimator`. Default `PerfectPoseEstimator`
returns ground truth. `NoisyAprilTagEstimator` adds Gaussian error
scaled by distance-to-nearest-visible-tag plus configurable FOV
dropout — enough to study "does better localization measurably shrink
cycle time" without simulating actual camera optics.

**Monte Carlo is a parameter sweep over `RobotCharacteristics` +
`Behavior` config**, running headless `Match` instances (optionally
via `multiprocessing.Pool`), collecting `Metrics` per trial, and
aggregating into a `pandas.DataFrame`. Day-one DOE support is a
full-factorial grid over 2–3 parameters; random/Latin-hypercube
sampling is a drop-in extension of the same trial-generator interface,
not a rewrite.

## What ships in `common_sim` vs. what a new game writes

| Reusable now (`common_sim`) | Written per-game (`game_specific/<game>/`) |
|---|---|
| Physics engine, swerve/tank kinematics | Field dimensions & obstacle polygons |
| Robot/mechanism/characteristics model | Scoring region locations + point table (auto vs. teleop) |
| Behavior-tree node types | Concrete `GamePiece` subclasses |
| Vision noise model | Example autonomous routines / opponent scripts |
| Monte Carlo runner, metrics, DOE | Game-specific metrics, if any beyond score/cycle-time |
| GUI shell, theme, overlay chrome, field canvas | — |

## Sequencing (not committing to code yet)

1. `common_sim/geometry.py`, `physics/engine.py`, `physics/swerve.py` — physics core, unit-testable headless.
2. `common_sim/field/`, `robot/`, `match/` — the generic game-loop skeleton, exercised by a trivial synthetic game in `test/`.
3. `gui_utils/field_canvas.py` + `apps/run_match.py` — first visible, drivable robot, keyboard only.
4. `control/input_sources.py` gamepad support; `control/behavior.py`; `control/vision.py`.
5. `analysis/monte_carlo.py` + `gui_utils/analysis_panels.py` + `apps/run_monte_carlo.py`.
6. `game_specific/reefscape/` as the first real game — this is the two-day-after-reveal dry run for the whole architecture.

Open items I'm treating as later/non-blocking per your notes: video
export via ffmpeg, NetworkTables/AdvantageScope publishing — both slot
in as additional consumers of `match/events.py`'s telemetry stream
without touching the core.
