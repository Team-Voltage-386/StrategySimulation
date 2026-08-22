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

`test/test_import_contract.py` enforces the three lines above on every
run, two ways: a static scan of every import statement in `common_sim/`
and `gui_utils/` (which is how `gui_utils` gets checked without needing
Qt on the box), and a subprocess that imports all of `common_sim` for
real and inspects `sys.modules` — the only way to catch an *indirect*
leak, where `common_sim` imports something else that imports a game.
Until that test existed the contract held only because everyone
remembered it, which is the kind of guarantee that expires on the
afternoon of game reveal.

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
    validation.py          Static checks on a built FieldConfig -- unreachable scoring regions,
                            unregistered piece types, actions worth nothing, mis-linked emitters,
                            gaps narrower than a robot. Every one of them is a mistake that
                            otherwise produces a plausible-looking *wrong match* rather than an
                            error. Imports control.navigation on purpose: the question "can a
                            robot park somewhere that scores here" has to be asked with the same
                            geometry the robot will use
  robot/
    characteristics.py    RobotCharacteristics dataclass: speed, accel, size, capacity, intake/deposit time,
                            scoring reliability per piece type and per action (a REEF branch is harder to hit
                            than a trough; the two multiply)...
    robot.py               Robot base: chassis + mechanisms + pose + alliance + held-piece slots
    mechanisms.py          Intake / Manipulator base classes, parameterized by characteristics
  control/
    input_sources.py      InputSource ABC; KeyboardInput, GamepadInput -> normalized DriveCommand
    behavior.py             Behavior-tree-lite: Sequence/Parallel/DriveToPose/RunIntake/Wait/Branch nodes
    vision.py                 Simulated AprilTag pose estimation + piece detection, tunable noise/FOV/dropout
    param.py                 Param dataclass: shared PARAM_SCHEMA element for Trigger/Tactic GUI forms
    world_view.py             Read-only queries over Match: collectable pieces, scoring/station options,
                                station supply, opponents, defenders, denial targets, region contention
    navigation.py             plan_path (visibility-graph A*), NavigateTo behavior, estimate_travel_time
    utility.py                 Outcome: any candidate action priced in points/second, deposits and pickups
                                 alike (a pickup's value is `enables`, the deposit it sets up). Ranking
                                 currency is `expected_rate` -- points discounted by how often this robot
                                 lands the attempt; `value_rate` is the same number gross. Generates
                                 and prices only -- it does not choose
    planning.py                ScoringOption/ScorePlanner: GreedyRatePlanner (default, ranks on
                                 expected_rate, so a flaky high-value target loses to a reliable cheaper
                                 one at the travel times where that is true), LookaheadPlanner stub
    triggers.py                 Declarative Trigger dataclasses (PiecesAvailable, MatchTime, BeingDefended, AllOf/AnyOf/Not, ...)
    tactics.py                   Collect/Score/Pursue/Defend/RunScript/Idle -- Behaviors that replan their own target
                                   Pursue arbitrates fetch-vs-score on utility.py's rate, then runs Collect or
                                   Score to do it -- the tradeoff a Rule's integer `priority` cannot express.
                                   Modulates that rate by context the raw Outcome cannot see: whether the job
                                   still fits in the match (time_fit_slack), how often the deposit lands
                                   (reliability_weight), and who is already on the field feature it needs
                                   (contest_penalty/claim_penalty). All flat float Params, so a search tunes them
    strategy.py                   Rule/Strategy/StrategyController: priority arbiter over Trigger->Tactic rules
    strategy_io.py                 Strategy <-> JSON (REGISTRY-driven, round-trips through the GUI editor)
    strategy_params.py              The continuous half of a Rule[] as a flat vector (ParamRef/to_vector/
                                       with_vector) -- structure (types, priority, counts, unset optionals)
                                       is deliberately not exposed, so a search cannot change what a
                                       strategy *is*, only its numbers
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
    calibration.py                    Times the machine in front of the user and reports what batch sizes are
                                         realistic on it (game-agnostic: caller supplies worker + reference jobs)
    cmaes.py                           CMA-ES as a plain ask/tell loop over a box-bounded vector, written out
                                          rather than pip-installed; minimizes, knows nothing about matches
    param_search.py                     Parameter search over a fixed rule structure: search_parameters (the
                                           loop, in a normalized box) + AllianceScoreEvaluator (candidates ->
                                           TrialJobs via expand_jobs/run_all, common random numbers across
                                           candidates, refuses a disabled VariabilityModel with seeds > 1)

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
  search_panel.py              Game-agnostic SEARCH-tab widgets (SearchSetupPanel, ParameterPanel,
                                  ProgressPanel, VerdictPanel) + the QThread plumbing (SearchWorker,
                                  SearchRunController). VerdictPanel reports the held-out number as the
                                  result and the search's own best-of-N as a dim aside -- deliberately.
  doc_tags.py                   One explanation per widget, used three ways: the tooltip in the app, the
                                   numbered callout on the user guide's screenshots, and the reference
                                   entry beside them (apps/build_guide.py reads them back).

game_specific/
  reefscape/                concrete REEFSCAPE field/pieces/scoring (the real game)
    field.py                 concrete FieldConfig instance (alliance-owned scoring regions/stations)
    game_pieces.py             concrete GamePiece subclasses (CORAL, ALGAE)
    scoring.py                    concrete ScoringRules + per-action scoring reliability
    strategies/                     example Strategy JSON files (also strategy_io round-trip fixtures):
                                       cycle_coral, algae_processor, endgame_defense, auto_then_cycle,
                                       pursue (one Always -> Pursue rule; the arbitration is the strategy)
    sweep_trial.py                    Qt-free worker entry point (run_trial/replay_trial) + build_match_for_job,
                                         the match builder MATCH-tab replay shares with the SWEEP tab.
                                         SWEEP_DT (1/60, pinned by MATCH-tab replay) and SEARCH_DT (1/30,
                                         for search, which never needs scrubbing) live here
  salvage/                  SALVAGE 2027 -- an *invented* game, written as a dry run for the
                               two-day new-game turnaround this whole layout exists to support.
                               Deliberately awkward where REEFSCAPE is smooth: 7 obstacles rather
                               than 2, 3 piece types sourced 3 different ways, a finite contested
                               neutral depot, shared scoring capacity, scoring value that moves
                               (and reorders) at the phase boundary, and holds whose value,
                               reliability and travel are all coupled. Same file set as
                               reefscape/, plus robot.py -- the reference robot, which on
                               REEFSCAPE lives in apps/ and is duplicated there.
                               Findings: DRY_RUN_LOG.md

apps/
  run_reefscape.py        Interactive REEFSCAPE viewer: MATCH tab (keyboard+gamepad+AI roster) +
                             STRATEGY tab (strategy_editor.py + strategy_graph.py side by side) +
                             SWEEP tab (sweep_panel.py + sweep_tab.py) +
                             SEARCH tab (search_panel.py + search_tab.py)
  reefscape_widgets.py      Shared robot-config Qt widgets (RobotConfigTab, RosterPanel,
                               RobotRosterConfigPanel, ...) extracted from run_reefscape.py so MATCH and
                               SWEEP reuse the same classes and RobotRosterConfigPanel.robot_specs()
  sweep_tab.py               SWEEP tab wiring: owns its own roster (independent of MATCH), builds
                                TrialJobs, drives SweepRunController, wires replay back into MATCH
  search_tab.py               SEARCH tab wiring: the GUI front end for param_search. Reuses
                                 run_param_search's roster and variability so a search started from the
                                 tab and one started from the CLI measure the same quantity
  build_guide.py               Builds all four tab guides (MATCH/STRATEGY/SWEEP/SEARCH): screenshots
                                  the live tab, draws callouts from its doc_tags, generates the control
                                  reference from the same tags, and cross-links the four via a shared
                                  nav. Prose is docs/{match,strategy,sweep,guide}_template.html; shared
                                  CSS is docs/guide_style.css. `--guide <name>` builds just one
  run_strategy_sweep.py    Headless Monte Carlo sweep with "strategy" as just another swept param
  run_defense_bench.py      Head-to-head (red plan x blue plan) grid measuring what a full-time defender
                               costs the alliance it defends -- the one thing a one-sided strategy sweep
                               structurally cannot show, since a defender scores nothing itself
  run_salvage_bench.py       The same idea on the dry-run game, but *paired*: every blue plan is run on
                                the same seeds as the baseline and reported as a per-seed difference, which
                                resolves an order of magnitude better than differencing two means. A second
                                copy rather than a second arm because run_defense_bench and run_stall_audit
                                both import game_specific.reefscape at module scope -- see DRY_RUN_LOG.md, F6
  run_stall_audit.py         Frozen-robot detector over the same grid: longest motionless span per robot per
                                match, with what the robot was *asking* for while frozen (translation and
                                rotation separately -- a robot can be held rotationally while commanding no
                                translation at all) and how much match was left when it moved again
  run_calibration.py         Prints what batch sizes this particular machine can finish
  run_param_search.py         Best-response-per-design: CMA-ES over a strategy's continuous fields, once per
                                 design point, so a design is judged by its own tuned strategy rather than by
                                 six hand-written ones. --estimate-only sizes the run before committing to it

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

**A pin is about motion, not access.** `field_config.PinRule` is the
generic form of "you may not prevent an opponent from moving for more
than N seconds"; `Match._step_pins` keeps a per-(offender, victim) clock
alongside the protected-zone one. Three conditions have to hold at once,
and each excludes a defense that is legal. The victim must be in
*contact*; it must be commanding motion it is not achieving, which is
only a meaningful question now that the drivetrain is traction-limited;
and it must be trapped against something on the far side of the push --
a wall, an obstacle, a third robot. That last one is the official shape
of the rule ("impeding the movement of an opponent ROBOT against a FIELD
element or another ROBOT") and it is load-bearing: without it every
mutual shove in open space charges *both* alliances with pinning each
other, since by construction both robots are stopped and both are asking
to move.

The consequence worth knowing is that camping a feeder mouth or a
scoring spot is never a pin, however long it lasts. The victim can drive
anywhere it likes; it just cannot get to the one place it wanted, and
its own commanded speed says so. Denial of access is unlimited, denial
of motion is not, and the two are separated by a question the sim can
actually answer.

`Defend._respect_pin_limit` backs off at `_PIN_RELEASE_FRACTION` of the
limit, using the same retreat as the protected-zone release. Whether to
release at all is a real trade rather than a free win -- see the
constant's comment for the swept numbers.

**Every commitment needs an expiry, and the expiry is time spent
failing.** This is the same lesson in three places now, so it is worth
stating as a rule rather than rediscovering it a fourth time. `Score` has
`_STALL_PATIENCE_*`, `Collect` has `_STATION_PATIENCE_*` for a committed
station and `_PIECE_PATIENCE_*` for a committed loose piece, and all
three exist because the instantaneous tests a tactic can run -- is this
region full, is that feeder occupied, would somebody beat me there -- are
all blind to a defender that denies by *standing in the approach*. Such a
defender is not in the region, not on the feed, and not racing us
anywhere, so nothing it does makes the target read as unavailable. Worse,
`estimate_travel_time` routes around field geometry only and never models
robots, so a target one foot behind a parked opponent prices as one foot
away: being close and stopped reads as success. Only the fact that we
have been failing for a while does.

Four things generalise from getting this wrong repeatedly. The budget
must be priced off what the attempt *should* cost and floored well above
ordinary variance, or it fires on the everyday case (a teammate ahead of
you in a queue) instead of the case it is for. And the cooldown on the
target you abandoned has to outlast the trip that giving up sends you
on, or the robot alternates between two targets on exactly the period of
the patience clock -- measured once at 96 of 150 seconds spent
oscillating between two feeders.

The third is that "elapsed time" is the wrong clock for a target whose
honest cost varies widely. A station trip is short and stereotyped, so
overrunning its budget really does mean denial; a loose piece is wherever
it came to rest, and reaching one legitimately takes anywhere from half a
second to most of a cycle. Timed on plain elapsed time the piece escape
threw away good trips -- visibly so, because it moved the *undefended*
control, where by definition nothing was being denied. What it should
spend the budget on is time making no progress, ratcheted on the closest
approach so far (`_PIECE_PROGRESS_EPSILON`). A trip that is still closing
then never expires however long it takes, and a robot held at arm's
length from a piece spends the whole budget standing there. Prefer that
formulation wherever an escape's target is not a fixed, uniform trip.

The fourth is that the give-up sometimes has to leave the tactic. A
tactic can only re-pick within its own scope, and scope is exactly what
can run out: `Collect(piece_type="algae")` whose last reachable ALGAE is
behind a defender has nowhere to go, and no way to know its strategy also
has a CORAL rule one priority below. So a piece is *not* re-offered when
it is the last one (unlike a station, where going back and waiting is
usually right, because a station is the only source of its type and
regenerates -- but see the sixth case below, where "usually" was doing
more work than it could carry).
`Collect` reports `FAILURE` instead, and `_FAILED_RULE_SUPPRESSION` in the
arbiter is what makes that mean something: a rule whose tactic failed
yields briefly so a lower-priority rule can have the robot. Before that,
`_best_candidate` never consulted status, so a rule that had just declared
it could not do its job was handed the robot straight back and kept it
for as long as its trigger held -- which for an availability trigger is
the rest of the match. `Collect` is still the only tactic that returns
`FAILURE`, so that edge exists for it alone today.

The honest caveat, recorded so nobody re-derives it as a win: on this
game these escapes are worth almost nothing on the mean. What they move
is the spread, by removing the tail where a robot spends fifteen seconds
achieving nothing. That is worth having in a baseline meant to survive a
rules set nobody has seen yet, but it is not a scoring improvement and
should not be quoted as one. The piece escape is the same shape: over a
2v2 mixed-blue defense grid it is +4.5 points across 14 plan pairings and
bit-identical on the undefended control, while on the lineup it was found
on it took blue 228.4 +-9.9 to 230.6 +-6.6 and cut the longest single
commitment from 29.8s to 10.7s.

A note on measuring these, since it caught out the piece escape: the
defense bench's two blue plans both cycle CORAL only, so a whole class of
Collect behavior -- anything to do with loose pieces, or with a robot
whose supply can run out -- is invisible to it. The piece stall showed up
only on a *mixed* alliance (one ALGAE cycler, one CORAL cycler). A grid
that varies the defense while holding one offensive shape fixed measures
that shape, not the tactic.

A fifth case is the one that needs *no* expiry, and getting that wrong
cost more than any of the four above. A patience clock is for a trip that
is going badly; it is the wrong instrument entirely for a target that has
stopped existing. `Collect` committed to a REEFSCAPE REEF ALGAE position,
which holds exactly one, and arrived to find it already taken -- and then
sat on it for 126 of 150 seconds. Every release declined, each for a
locally sensible reason: `_station_stalled` stops its clock while the
robot is on the feed, because an intake under way should be allowed to
finish (none was under way); `_better_station_exists` wants the committed
station to be *full*, which an empty one conspicuously is not; and the
escape needs somewhere to give up to, which a field with no ALGAE left
does not have. The sim knew the whole time -- `Match.step` gates
`Robot.update_station_intake` on the same supply counter and simply
declines to dispense -- but the control layer had no way to ask. So the
release is unconditional and mirrors the loose-piece one (`held_by is not
None or scored`): no clock, no cadence, no requirement that anywhere
better exist, and no cooldown either, since an empty station is already
excluded by `station_options` for exactly as long as it is empty. The
general rule: distinguish *this attempt is failing* from *this target is
gone*, and only the first is a matter of patience.

The sixth case is a third thing again -- *this target is real, available,
and unreachable* -- and it is where the fourth case's reasoning had a
hole. Requiring somewhere to give up *to* is right when the wait is a
queue, and the station escape asks for that alternative to be of the same
piece type. At the last feeder of a type the alternative can never
exist, so the condition is not merely unmet, it is unfalsifiable: the
robot waits out the match by construction. Measured on `shadow/supply`
seed 1004, both blue robots queued at the one REEF ALGAE position a red
defender was physically parked on, 78 seconds past an 8-second patience,
holding nothing, while two CORAL STATIONS sat free and offered the whole
time. The match ended 51-234 and 10 pieces; with the escape it ends
221 and 54 pieces.

What distinguishes the case is *who* is in the way, and that distinction
already existed (`_held_by_opponent`, engagement rather than a declared
claim). A teammate on a feed is a queue that is moving -- it loads and
leaves. An opponent on a feed is not a queue at all, because standing
there is the entire reason it came. So an opponent physically on the
feed is its own reason to leave, with no alternative required.

The fix needed a second half to be a fix at all, which is the part worth
remembering: `_best_station` falls back to the cooled-down list when
filtering leaves nothing, so releasing the station and re-picking it on
the next tick would have made the cooldown a clock reset and nothing
else. The fallback has to decline to resurrect an opponent-held station
too. Then `_best_station` returns nothing, `_pick_target` weighs loose
pieces, and failing that `Collect` reports `FAILURE` and the choice goes
up to whoever picked the piece type -- Pursue re-arbitrating, or a rule
strategy falling through. A release the next tick undoes is not a
release.

It was found by the expected-points ranking rather than by defense work,
which is the other lesson: that ranking discounts CORAL more than ALGAE
(L4 lands 0.82 of the time, the PROCESSOR 0.95), so it tilted `Pursue`
toward the piece type whose entire supply is a few one-piece staging
positions -- and a latent deadlock needing a scarce contested feeder went
from never firing to one seed in twelve. Changing what a robot prefers is
a way of finding out which of your escapes were load-bearing. On the
grid it is worth +21.3 on the one row it fires (`pursue` /
`shadow/supply`) and leaves every other cell bit-identical.

**A rate can be modulated, but only by something the robot can act
on.** `Pursue` prices fetching against scoring in points per second
(`utility.Outcome`), and the obvious next move is to bend that rate by
context the raw number cannot see. Four such terms exist
(`_time_fit`, `_reliability`, `_pressure`'s contest and claim halves),
and measuring them one at a time on the 2v2 defense bench was worth
more than any of them.

*Time-fit* is not tuning, it is arithmetic: a fetch is priced on the
deposit it enables, and a deposit after the buzzer scores nothing, so
without it a robot sets off at t=147 for a cycle that returns zero. It
measures as neutral (157.8 against 158.2 with it disabled) precisely
because it only ever binds in the last seconds -- which is the argument
for keeping it, not against.

*Contest pressure* pays where it was designed to and nowhere else: +23.0
points on `block/supply` and +8.5 on `shadow/supply`, the two rows where
a defender camps the feeder, against roughly a wash elsewhere. That is
the shape to expect from an honest term, and the reason to measure per
row rather than on a grand mean.

*Reliability* took three tries and is the one worth reading. Multiplying
`success_probability` in is textbook expected value, but as a weight on
`Pursue` alone it lost 48.9 points summed across the grid. The reason is
that `Pursue` chooses *which job*, while reliability differs mostly
between *actions within* a job (L4 misses far more often than L1).
Handing the priced action down to `Score` -- the obvious fix, the exact
mirror of handing the piece type down to `Collect` -- cost 51 points a
match under `block/scoring`: piece type is a genuine either/or, but which
action to score is not, because the robot puts down everything it carries
eventually. That makes it a *sequencing* question.

Sequencing is what the planner is for, so that is where the discount
belongs, and putting it there is the third try. `Outcome.expected_rate`
(points times probability, over seconds) is now the ranking currency
everywhere a deposit is chosen -- `GreedyRatePlanner`, `Score`'s re-pick,
and `best_score_for_type`, the payoff half of a pickup. Reliability then
reaches the branch choice with nobody pinning anything, and it reaches
every strategy rather than just `Pursue`. `value_rate` survives beside it
as the gross number for reporting.

Two things fall out of that. `Pursue`'s weight defaults to 1.0 now, not
because it wins -- it is worth about 5 points summed, inside a sum's
noise -- but because at anything less the tactic prices scoring at the
richest target's gross value while its own `Score` child performs a
cheaper, likelier one, and the fetch-versus-score comparison is made in a
currency nobody spends. The older -48.9 was mostly that mismatch. And the
discount is *asymmetric across piece types* -- CORAL's best action lands
0.82 of the time against the PROCESSOR's 0.95 -- so it quietly re-weights
which piece a robot goes for, which is how it surfaced the deadlock in
"Every commitment needs an expiry" above. A term is only worth its tick
if some robot can act on it at the layer that computes it; and a term
that changes what a robot prefers will find out which of your escapes
were load-bearing.

None of this clears the bar `Pursue` was aiming at, and expected-points
ranking did not move it closer. Summed over the seven red plans the whole
package is -20.0 for `pursue` and +6.6 for `cycle_coral`, both inside the
noise of a seven-row sum, with one real win (`block/supply` +21.0) and a
scatter of small losses; the strategy sweep is flat to within a point or
two on all four entries. `Pursue` now loses every row to `cycle_coral`,
undefended included (244.5 against 249.1), where it used to win that one.
The utility layer is a better *representation* -- every weight is a flat
float `Param`, so a search tunes a strategy's reasoning with no new
search code -- and it is now internally consistent, in that the currency
a job is priced in is the currency its target is chosen in. On this bench
it is still not a better *player*.

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

**The planner is the sim's clock, and A\* is why.** A 3v3 defended match
spends over half its wall time in `plan_path`, which is asked ~27,000
questions per match: half from `estimate_travel_time` (tactics pricing
candidate targets, every tick) and half from `NavigateTo._replan`. The
inner loop is `_visible`, the segment-vs-obstacle test. Four things were
measured there, and the split between what worked and what didn't is the
useful part:

- **Don't build the graph you won't search.** A visibility graph is
  O(N²) in vertices, but A\* expands only the handful of nodes on or near
  the route — 5.8 of 24.1 per plan, measured. Generating a node's edges
  when A\* first pops it, and caching per unordered pair, cut `_visible`
  calls by 68% and `plan_path` by 2.1x. Same graph, same edges, same
  answer; just not computed until asked for. This was worth more than
  every arithmetic-level change put together.
- **Exact beats sampled, and is also faster.** `_visible` used to test
  each obstacle edge for a strict straddle and then sample seven points
  along the segment for containment — the samples not as insurance but
  because the straddle test has a real blind spot at vertices, which
  `_octagon` puts straight along +x. Every obstacle here is convex, so
  clipping the segment's parameter interval against the edge half-planes
  (Cyrus-Beck) answers the same question exactly, in one pass, with no
  ray casts. Zero disagreements against the sampling version over 400,000
  queries captured from a real match.
- **Cull once over the list, not once per pair.** `clear_standoff` tried
  up to 24 rotations against every obstacle; every chassis it builds sits
  within one disc of the target, so obstacles outside it are dropped
  before the loop. Conversely, a bounding-box pre-reject *inside*
  `convex_overlap` measured **slower** and was removed: a separating-axis
  test already answers "far apart" on its first axis after two
  projections, which is cheaper than the box test that would have
  replaced it.
- **A cache's hit rate is not its value.** `plan_path` is a pure function
  of its arguments and 17% of calls repeat one verbatim, so memoising it
  is exactly behavior-preserving — and bought 1% of wall time, because
  the repeated calls are overwhelmingly the ones that take the cheap
  direct-visible exit (0.02ms against a 0.32ms mean). Not kept. Measure
  the time a cache saves, not the calls it serves.

Together: 24.2s -> 11.9s per 3v3 match, **bit-identical** results on the
defense bench (42 matches at 2v2, 8 at 3v3) and 312 tests unchanged.
Bit-identity is the acceptance test that makes this kind of work safe to
do at all — an optimization that changes trajectories is a behavior
change wearing a speed change's clothing, and has to be measured as one
(see `_within_corridor`, which is exactly that and is documented as a
bound rather than a proof).

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
