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
| Charts | **pyqtgraph** for live in-match plots (already a `gui_utils` dependency); **matplotlib**, embedded in Qt via `FigureCanvasQTAgg`, for the SWEEP tab's line/heatmap plots | — |
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
    field_config.py      FieldConfig dataclass: dims, obstacles, ScoringRegion / PieceSpawnRegion /
                          IntakeLocation / EmitterRegion / ProtectedZone lists
    game_piece.py         GamePiece base: pymunk body/shape + type tag
  robot/
    characteristics.py    RobotCharacteristics dataclass: speed, accel, size, capacity, intake/deposit time...
    robot.py               Robot base: chassis + mechanisms + pose + alliance + held-piece slots
    mechanisms.py          Intake / Manipulator base classes, parameterized by characteristics
  control/
    input_sources.py      InputSource ABC; KeyboardInput, GamepadInput -> normalized DriveCommand
    behavior.py             Behavior-tree-lite: Sequence/Parallel/DriveToPose/RunIntake/Wait/Branch nodes
    vision.py                 Simulated AprilTag pose estimation + piece detection, tunable noise/FOV/dropout
    param.py                 Param dataclass: shared PARAM_SCHEMA element for Trigger/Tactic GUI forms
    world_view.py             Read-only queries over Match: collectable pieces, scoring/station options, opponents, defenders, denial targets
    navigation.py             plan_path (visibility-graph A*), NavigateTo behavior, estimate_travel_time
    planning.py                ScoringOption/ScorePlanner: GreedyRatePlanner (default), LookaheadPlanner stub
    triggers.py                 Declarative Trigger dataclasses (PiecesAvailable, MatchTime, BeingDefended, AllOf/AnyOf/Not, ...)
    tactics.py                   Collect/Score/Defend/RunScript/Idle -- Behaviors that replan their own target
    strategy.py                   Rule/Strategy/StrategyController: priority arbiter over Trigger->Tactic rules
    strategy_io.py                 Strategy <-> JSON (REGISTRY-driven, round-trips through the GUI editor)
  match/
    match.py                Match orchestrator: clock, phase (auto/teleop/endgame), collision routing
    scoring.py                ScoringRules ABC: point table keyed by (action, phase)
    events.py                  Timestamped match event log, feeds telemetry + metrics
  analysis/
    monte_carlo.py          Batch trial runner: DOE sweep over RobotCharacteristics, expressed on top of runner.run_all
    metrics.py                 Per-match metric extraction (score, cycles, utilization, ...)
    results.py                   Aggregation into pandas DataFrame, summary stats
    runner.py                     Qt-free CancelToken + bounded-submission iter_results/run_all over a ProcessPoolExecutor
    sweep_spec.py                  Picklable FieldDescriptor/SweepVariable/RobotSpec/TrialJob/TrialOutcome + expand_jobs
    variability.py                   Seeded config-perturbation model (VariabilityModel) -- the sweep's only randomness

gui_utils/               (existing — extended, not replaced)
  theme.py, overlay_panel.py, telemetry_store.py, ...   reused as-is
  field_canvas.py         QPainter canvas rendering FieldConfig + live Robot/GamePiece state, plus an
                            optional per-robot AI-intent overlay (target region/piece, active tactic label)
  strategy_editor.py       Schema-driven STRATEGY tab: rule list + trigger/tactic property inspector,
                              built entirely from PARAM_SCHEMA -- adding a Trigger/Tactic to common_sim
                              needs zero edits here. One in-memory Strategy per robot; Load/Save/Apply.
  strategy_graph.py         QGraphicsView state-machine diagram of a Strategy: priority-banded nodes,
                               live-highlighted active rule + flashing transition edges + history strip.
  sweep_panel.py             Game-agnostic SWEEP-tab widgets (VariableTable, SweepControlPanel,
                                 SweepResultsModel/Table, SweepPlotPanel) + the QThread/ProcessPoolExecutor
                                 plumbing (SweepWorker, SweepRunController) -- built from FieldDescriptors
                                 handed in, never imports game_specific.
  sweep_plots.py              matplotlib rendering onto a caller-supplied Figure (line/heatmap/faceted
                                 heatmaps), testable headless under Agg with no Qt or widget involved.

game_specific/
  reefscape/                concrete REEFSCAPE field/pieces/scoring (the one game currently plugged in)
    field.py                 concrete FieldConfig instance (alliance-owned scoring regions/stations)
    game_pieces.py             concrete GamePiece subclasses (CORAL, ALGAE)
    scoring.py                    concrete ScoringRules
    strategies/                     example Strategy JSON files (also strategy_io round-trip fixtures):
                                       cycle_coral, algae_processor, endgame_defense, auto_then_cycle
    sweep_trial.py                    Qt-free worker entry point (run_trial/replay_trial) + build_match_for_job,
                                         the match builder MATCH-tab replay shares with the SWEEP tab

apps/
  run_reefscape.py        Interactive REEFSCAPE viewer: MATCH tab (keyboard+gamepad+AI roster) +
                             STRATEGY tab (strategy_editor.py + strategy_graph.py side by side) +
                             SWEEP tab (sweep_panel.py + sweep_tab.py)
  reefscape_widgets.py      Shared robot-config Qt widgets (RobotConfigTab, RosterPanel,
                               RobotRosterConfigPanel, ...) extracted from run_reefscape.py so MATCH and
                               SWEEP reuse the same classes and RobotRosterConfigPanel.robot_specs()
  sweep_tab.py               SWEEP tab wiring: owns its own roster (independent of MATCH), builds
                                TrialJobs, drives SweepRunController, wires replay back into MATCH
  run_strategy_sweep.py    Headless Monte Carlo sweep with "strategy" as just another swept param
  run_defense_bench.py      Head-to-head (red plan x blue plan) grid measuring what a full-time defender
                               costs the alliance it defends -- the one thing a one-sided strategy sweep
                               structurally cannot show, since a defender scores nothing itself

test/                     (existing) unit tests per package
```

## Core runtime model

**Fixed-timestep physics, decoupled render rate.** `SimEngine.step(dt)`
advances pymunk at a fixed substep (e.g. 1/120s) regardless of who's
calling it. The interactive app drives it from a `QTimer` at ~60Hz for
display; the Monte Carlo runner drives it in a tight loop with no
timer/vsync at all, which is what makes "faster than real time"
trivial rather than a separate code path.

**The drivetrain is a traction-limited force, not a velocity write.**
`drive_field_relative` latches a target; `SwerveChassis._integrate_velocity`
(a pymunk `velocity_func`) ramps toward it under `max_accel` on *every
physics substep*, before the contact solver runs. In free space this is
indistinguishable from the old once-per-control-tick slew -- the same
`max_accel * tick` gets applied either way, and the undefended benchmark
is unchanged to the point. In contact it is a different simulation.
Slewing per control tick let a robot re-assert its command only every
20ms and do nothing in the 4-5 substeps between, so contact impulses had
uncontested authority over a robot trying to hold still, and conversely a
robot could bulldoze through anything by rewriting its velocity from
scratch each tick.

Per substep, each chassis can add or remove at most `mass * max_accel` of
momentum, and the solver arbitrates the rest. Three consequences worth
naming: commanding zero velocity is *braking*, not passivity, and spends
the robot's whole traction budget holding position; two equally powered
robots pressed square against each other therefore produce zero net force
on the pair and neither can move the other; and a robot that spawns
overlapping a wall can no longer shove its way out, so a start pose half
outside the perimeter is now a real error rather than a slow start.

Momentum from the *impact* is not dissipated -- there is no floor
friction model -- so a pair that collides while closing at `v` slides on
together at about `v/2` until someone disengages. That is correct rigid-
body behavior for a frictionless floor and it is bounded by approach
speed (bumper-to-bumper from rest, a braced robot gives up ~6in in 5s),
but it does mean a defender who *rams* moves its victim further than one
who arrives and leans.

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

**Strategy is data (`Rule[]`), not hand-assembled code.** A `Tactic`
(`Collect`/`Score`/`Defend`/`RunScript`/`Idle`) *is* a `Behavior`
subclass that decides its own target each tick via `world_view`/
`planning`, so it composes with `Sequence`/`Parallel`/`Repeat`
unchanged. A `Strategy` is a list of `Rule(trigger, tactic, priority,
min_duration, cooldown, once)`; `StrategyController` evaluates every
rule's `Trigger` each tick, arbitrates by priority (ties broken by list
order), and ticks the winner — switches get logged to `match.events` as
`"behavior_change"`, which is what drives `strategy_graph.py`'s live
highlighting for free. **`Match.step` ticks each robot's
`robot.controller` (if any) before mechanism updates and physics** —
the loop owner is `Match`, not the app — so a strategy runs identically
whether it's driven by the interactive GUI, a headless
`run_match_to_completion` call, or a Monte Carlo trial; human-driven
robots simply pass `controller=None`. `strategy_io.py` serializes the
whole tree to/from JSON (`REGISTRY`-driven, so adding a Trigger/Tactic
needs no serialization code), which is what makes a strategy a GUI-
editable, file-saveable, Monte-Carlo-sweepable *thing* rather than a
one-off Python script.

**Intent is a published channel, and defense declares itself on it.**
Each `StrategyController` republishes an `Intent` every tick (active
tactic, target piece/region) and every robot on the field — either
alliance — may read it. That is what stops two teammates converging on
one piece, and it is also the whole defense/counter-defense mechanism:
a `Defend` tactic additionally sets `Intent.defending` and
`Intent.marking`, so a robot can distinguish *"an opponent is standing
where I want to be"* (ordinary traffic; wait) from *"an opponent has
declared it is here to take this away from me"* (won't resolve itself;
do something else). `world_view.defenders`/`defenders_against`/
`region_denied_by` are the query surface over that, and the
`BeingDefended` trigger is the strategy-level hook onto it, so a
response is authored as data in a strategy file rather than hardcoded
in a tactic.

Note that the counterplay a defender's declaration enables is *not*
mostly about re-choosing a target — `world_view.region_occupants`
already treats a declared claim as occupancy, so picking somewhere else
happens without any defense-specific code. What actually matters is
that a robot notices it is *failing*: `Score` re-opens its choice of
what to score and where once an attempt runs past the time it should
have taken (`_STALL_PATIENCE_*`), which is what keeps a denied — or
merely unlucky — robot from committing to one impossible target for the
rest of the match. Measured on a 2v2 with one full-time defender, that
alone is the difference between blue scoring 54 and 151 points, at no
cost when undefended. `apps/run_defense_bench.py` is the harness those
numbers come from; a plain strategy sweep cannot produce them, because
a defender scores nothing itself and its effect is only visible in the
*other* alliance's per-alliance metrics.

**Defense denies a *cycle*, not a scoring region.** A robot's job is a
loop — go get a piece, go put it somewhere — and either half can be
taken away, so `Defend` resolves its target through
`world_view.denial_target_by_name` / `likely_denial_target`, which treat
a `ScoringRegion` and an `IntakeLocation` as the same kind of thing: a
polygon somebody has to reach. Everything downstream (block segment,
centroid, occupancy claim) was already duck-typed on `.name`/
`.vertices`, so the lookup was the only thing deciding which half could
be attacked at all. The `deny` param picks which half, and *that* is
where the game shows through: a game with no-contact zones around its
scoring locations and none around its feeders has made supply the softer
target by construction (see `ProtectedZone`): on REEFSCAPE, blocking a
REEF face fouls away everything it denies (blue 234.6 against 234.5
undefended, at 15.4 fouls a match), while blocking a CORAL STATION costs
blue 27 points at 0.8 fouls.

The counterpart on offense is that **a declared claim is not
possession**. `Collect` yields a station only to a claimant that would
actually reach it first (`_station_has_room_for`, the same ETA race two
teammates use to break a deadlock over one feeder) rather than to
anyone who merely names it -- otherwise a single defender takes both
feeders off the list by announcing them, from anywhere on the field.
Worth 234.5 against 221.8 undefended and 207.0 against 187.2 under
supply denial.

**`for_duration` belongs to the rule's outermost trigger.** Hysteresis
is bookkept by `StrategyController`, not by the `Trigger` (which stays a
pure, serializable function of state). A composite (`AllOf`/`AnyOf`/
`Not`) evaluates its children directly, so a `for_duration` nested
inside one is never consulted. `StrategyController.__init__` walks the
trigger tree and refuses to build rather than run a strategy whose text
says "for two seconds" and whose behavior says "immediately".

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

**A sweep trial is a picklable spec, not a live object.** Windows uses
the `spawn` multiprocessing start method, so a worker process re-imports
whatever module `trial_fn` lives in from scratch — it never inherits
live Python state from the parent. `common_sim/analysis/sweep_spec.py`
therefore describes a trial entirely as frozen dataclasses of
primitives (`TrialJob` -> `RobotSpec`s -> a plain `dict` copy of
`RobotCharacteristics`, never the dataclass or a `Strategy` object), and
the actual worker entry point (`game_specific/reefscape/sweep_trial.
run_trial`) lives in `game_specific`, imports **no Qt**, and is enforced
Qt-free by a subprocess `sys.modules` check in
`test/game_specific/test_reefscape_sweep.py`. `expand_jobs`'s
full-factorial grid expansion reuses `monte_carlo.ParameterSweep`
keyed by `SweepVariable.column`; swapping in Latin-hypercube (or any
other `.configs()`-shaped sampler) later is a one-call change inside
`expand_jobs`, not a rewrite of the sweep engine.

**Determinism contract:** `run_trial(TrialJob)` is exact for a fixed
`(TrialJob, seed)` — every draw of randomness (config perturbation via
`variability.py`, piece scatter) comes from a named `random.Random`
substream seeded off `job.seed`, and the timestep is pinned
(`sweep_trial.SWEEP_DT == 1/60`, matching `TelemetryRecorder`/
`MatchView`'s tick rate) so `run_trial` and `replay_trial` agree
bit-for-bit *on one machine* — not guaranteed bit-identical across
machines/pymunk builds.

## What ships in `common_sim` vs. what a new game writes

| Reusable now (`common_sim`) | Written per-game (`game_specific/<game>/`) |
|---|---|
| Physics engine, swerve/tank kinematics | Field dimensions & obstacle polygons |
| Robot/mechanism/characteristics model | Scoring region locations + point table (auto vs. teleop) |
| Behavior-tree node types | Concrete `GamePiece` subclasses |
| Vision noise model | Example autonomous routines / opponent scripts |
| Monte Carlo runner, metrics, DOE | Game-specific metrics, if any beyond score/cycle-time |
| GUI shell, theme, overlay chrome, field canvas | — |
| Strategy/tactic/trigger layer, arbiter, JSON I/O, GUI editor+graph | Example `Strategy` JSON files; which region/action/piece-type strings exist |
| No-contact safe zones + foul accounting (`ProtectedZone`) | Where this year's safe zones are and what a violation costs |

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
