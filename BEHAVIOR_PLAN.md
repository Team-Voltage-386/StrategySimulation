# Strategy & Behavior Layer — Implementation Plan

Goal: make *strategy* the thing this sim is fast at iterating on. Today
`control/behavior.py` gives run-to-completion primitives (Sequence,
DriveToPose, RunIntake, ...) that a caller must hand-assemble in Python
and tick itself. That's a scripting layer, not a strategy layer. What's
missing:

- **High-level tactics** (Collect / Score / Defend) that *decide their
  own target* each tick instead of being handed a `Pose2d`.
- **Declarative triggers** and a **priority arbiter** so tactics can
  preempt each other, rather than a hand-wired `Branch` tree.
- **A serializable spec** so a strategy is data (editable in a GUI,
  saved to a file, swept in Monte Carlo) rather than code.
- **A world-query surface** — none of the above can be written without
  "what pieces are collectable", "what's the best scoring option",
  "what is that opponent going after".

Everything below stays under the ARCHITECTURE.md contract: `common_sim`
and `gui_utils` never import `game_specific`. Regions/actions/piece
types are referred to by the **string names** the game already assigns.

---

## 1. Layering

```
Strategy (data)  ──  Rule[] : (Trigger, Tactic, priority, dwell/cooldown)
      │
StrategyController  — evaluates triggers, arbitrates, ticks winner, logs switches
      │
Tactic  (Collect / Score / Defend / RunScript)  — a Behavior; replans its own target
      │
behavior.py primitives (Sequence, RunIntake, RunManipulator) + navigation.NavigateTo
      │
Robot API (drive_field_relative, set_intake_active, set_deposit_active)
```

A Tactic **is** a `Behavior` subclass, so it composes with the existing
`Sequence`/`Parallel`/`Repeat` and existing tests/routines keep working
unchanged. Nothing in `behavior.py` is modified or deleted.

## 2. New modules

### `common_sim/control/world_view.py` — read-only queries over Match
Pure functions, duck-typed on `match` (no import of `match.py`; use
`TYPE_CHECKING` only), so triggers/tactics stay decoupled and unit-
testable against a stub.

- `collectable_pieces(match, *, piece_type=None, alliance=None, exclude_held=True) -> list[GamePiece]`
  — un-held, un-scored, optionally filtered by type.
- `piece_clusters(match, pieces, radius) -> list[Cluster]` — greedy
  radius clustering; `Cluster(centroid, pieces, count)`. Backs "highest
  density" collection.
- `station_options(match, robot) -> list[IntakeLocation]` — stations
  with remaining supply (`match.station_supply`) whose `piece_type` the
  robot has a side configured to intake and capacity for.
- `scoring_options(match, robot) -> list[ScoringOption]` — every
  (region, action) pair legal for a piece the robot holds: region
  accepts the type, action is in `region.actions`, region-action not
  full (see §5), robot has a scoring side for that type.
- `opponents(match, alliance)`, `partners(match, alliance)`.
- `region_by_name(match, name)`.

### `common_sim/control/planning.py` — the "what's worth the most" model
```python
@dataclass(frozen=True)
class ScoringOption:
    region: ScoringRegion
    action: str
    piece: GamePiece
    points: float          # scoring_rules.points_for(action, phase)
    deposit_time: float    # characteristics.deposit_duration(action)
    travel_time: float     # navigation.estimate_travel_time(...)
    @property
    def value_rate(self): return self.points / max(1e-6, self.travel_time + self.deposit_time)
```
- `ScorePlanner` ABC → `plan(match, robot) -> list[ScoringOption]`
  (ordered; the tactic executes the head and replans after each
  deposit).
- `GreedyRatePlanner` (default): sorts by `value_rate`; for a robot
  holding several pieces it chains greedily — pick best first option,
  advance a virtual pose/clock to that region, re-pick for the next held
  piece. That naturally satisfies "account for how many pieces I carry,
  their value, and how long to score them all."
- `LookaheadPlanner` left as a stub subclass with the same interface —
  this is the seam for real multi-step planning later, so adding it
  never touches the tactic.
- Deliberately excluded from day one, with a comment saying so: route
  blockage/congestion. It enters as an extra term in `travel_time` once
  `navigation` can report a blocked path.

### `common_sim/control/navigation.py` — obstacle-aware driving
**This is the highest-risk item and must land before tactics.**
`DriveToPose` is a pure P-controller with zero obstacle awareness; on
the REEFSCAPE field it will pin robots against the REEF hex on roughly
half of all approaches and poison every strategy comparison.

- `plan_path(field, start, goal, robot_radius) -> list[Vec2]` — visibility
  graph over `field.obstacles` inflated by `robot_radius`, A* over it.
  The field has 2–4 convex polygons, so this is cheap, deterministic,
  and re-plannable every ~0.25 s. Use LocalADStarAK.java as reference
- `NavigateTo(target_provider, *, heading_mode, standoff, replan_period)`
  — a `Behavior` that calls `plan_path`, follows waypoints by delegating
  to the same P-control math `DriveToPose` uses, and re-plans on a timer.
  `target_provider` is a callable so a tactic can retarget without
  rebuilding the node. `heading_mode`: `"face_travel"`, `"face_target"`,
  or a fixed angle — Score needs the correct *side* presented to the
  region, so it computes the heading that points the scoring side at the
  region (`SIDE_OUTWARD` + `characteristics.score_side_for`).
- `estimate_travel_time(field, start, goal, characteristics)` — path
  length under a trapezoidal-profile approximation from `max_speed` /
  `max_accel`. Used by the planner; no simulation required.

### `common_sim/control/triggers.py`
`Trigger` ABC: `evaluate(ctx) -> bool`, plus `describe() -> str` (the
GUI edge label) and dataclass fields only (serializable).

| Trigger | Params |
|---|---|
| `PiecesAvailable` | `piece_type=None`, `min_count=None`, `max_count=None`, `within=None` (in) |
| `MatchTime` | `after=None`, `before=None`, `phase=None` — seconds elapsed; `remaining_under=` convenience |
| `PiecesHeld` | `piece_type=None`, `min_count=None`, `max_count=None` |
| `AtCapacity` | `piece_type=None` — held == `capacity_for(type)`; sugar over PiecesHeld that survives capacity edits |
| `ScoringAvailable` | `min_value=None`, `region=None` — any legal option exists / worth ≥ N |
| `OpponentNear` | `region=None`, `within=` — feeds Defend |
| `AllOf` / `AnyOf` / `Not` / `Always` | combinators |

Every trigger supports an optional `for_duration` (must hold N seconds
before firing) — the controller tracks the timer, not the trigger, so
triggers stay pure functions of state.

### `common_sim/control/tactics.py`
Each takes a small dataclass of params, exposes `PARAM_SCHEMA` (see §4),
and internally owns a target + a `NavigateTo` + primitive children,
re-evaluated on a `replan_period`.

- **`Collect(piece_type=None, mode="nearest"|"densest", cluster_radius, prefer_station=False, max_range=None)`**
  Picks a target from `collectable_pieces` / `piece_clusters` /
  `station_options`; navigates so an *intaking* side faces it; holds
  intake. SUCCESS when held count increases or capacity is reached;
  FAILURE when nothing is collectable. Re-targets if its piece is taken
  by someone else — checked every tick, not just on replan.
- **`Score(planner=GreedyRatePlanner(), region=None, action=None)`**
  `region`/`action` `None` = "let the planner choose"; naming either
  pins it (this is how a saved strategy says "always L4 on the far
  face"). Navigates to the option's region with the scoring side
  presented, calls `set_deposit_active(True, action)`, and — critically
  — gates the deposit on `match.deposit_region_for(robot) is not None`
  so the sim's own readiness check is the single source of truth (the
  GUI's indicator and the tactic can never disagree). Loops until
  empty-handed.
- **`Defend(target="opponent_intent"|<region name>, standoff, engage_range)`**
  Resolves the region to deny: an explicit name, or — when the chosen
  opponent has a `StrategyController` — that controller's published
  `intent.target_region` (see §3). Picks the opponent nearest the
  defended region and holds the blocking pose on the segment between it
  and the region centroid, at `standoff` from the region, facing the
  opponent. Never terminates on its own.
- **`RunScript(children=[...])`** — wraps a plain `Sequence` of existing
  primitives so hand-scripted autos are first-class Rules alongside
  reactive ones.
- **`Idle()`** — explicit do-nothing, so "no rule fired" is a visible
  state in the graph rather than a silent stall.

### `common_sim/control/strategy.py` — the arbiter
```python
@dataclass
class Rule:
    name: str
    trigger: Trigger
    tactic: Behavior
    priority: int = 0
    min_duration: float = 0.0   # hysteresis: can't be preempted by equal/lower priority before this
    cooldown: float = 0.0       # can't re-fire for N s after ending
    once: bool = False

@dataclass
class Strategy:
    name: str
    rules: list[Rule]
    fallback: Behavior = Idle()
```
`StrategyController(strategy, robot)`:
- Each tick: evaluate every rule's trigger (with `for_duration` /
  `cooldown` / `once` bookkeeping); candidate = highest-priority
  satisfied rule, ties broken by list order (so the GUI's list order is
  meaningful and stable).
- Switch when: candidate priority **>** active priority; or active
  tactic returned a terminal Status; or the active rule's own trigger
  went false. `min_duration` blocks equal-or-lower-priority preemption
  only — a strictly higher priority always wins immediately (that's what
  priority means, and it's what makes "defend overrides collect" work).
- On switch: `reset()` the outgoing tactic, log a `behavior_change`
  event to `match.events` (`{robot, from, to, trigger}`) so it shows up
  in the existing console panel and the event timeline for free.
- Publishes `controller.intent` — `Intent(tactic_name, target_region,
  target_piece, target_pose)` — read by `Defend` on opposing robots and
  by the GUI to draw the active target on the field canvas.

### `common_sim/control/strategy_io.py`
- `REGISTRY`: type-name → class, for every Trigger and Tactic.
- `to_dict(strategy)` / `from_dict(d, field)` — recursive, driven by
  dataclass fields; region/action/piece-type params serialize as plain
  strings so a strategy file is human-readable and game-portable.
- `load_strategy(path)` / `save_strategy(strategy, path)` — JSON.
- Round-trip test is the guard that keeps the GUI, the file format, and
  Monte Carlo from drifting apart.

## 3. Changes to existing code (small, listed exhaustively)

1. **`Match.add_robot(..., controller=None)`** and `Match.step` ticks
   each robot's controller (building a `BehaviorContext` with
   `match=self`) *before* mechanism updates and physics.
   Rationale: today the app owns behavior ticking, which means Monte
   Carlo, tests, and the GUI each re-implement the loop and can order it
   differently. Centralizing it makes "run a strategy headless to
   completion" already work via `run_match_to_completion`, and
   guarantees every consumer sees identical ordering. Human-driven
   robots pass `controller=None` and are unaffected.
   *(ARCHITECTURE.md's "the loop owner ticks behaviors" note gets
   updated to match.)*
2. **`ScoringRegion.capacity_by_action: Mapping[str,int] | None = None`**
   plus a `Match` check in `_try_score` and a `region_full(region,
   action)` helper. Without it, `Score` picks L4 forever and stuffs
   unlimited coral into one REEF face — every strategy comparison
   becomes meaningless. Default `None` = unlimited, so nothing existing
   changes; `game_specific/reefscape/field.py` then declares 1 per
   branch action per face (L1 gets the trough count).
3. **`Robot.controller`** attribute + `Robot.intent` passthrough (thin).
4. **`gui_utils/field_canvas.py`**: optional overlay drawing each
   AI robot's `intent` — target region highlighted, a line to the
   target piece/pose, and the tactic name over the robot. This is what
   makes a strategy debuggable at a glance.
5. **`apps/run_reefscape.py`**: extract the current central widget into
   `MatchView(QWidget)` and put it in a `QTabWidget` alongside the new
   strategy tab. Mostly mechanical (move, don't rewrite).

## 4. GUI — the STRATEGY tab

Two new game-agnostic widgets in `gui_utils/`:

**`strategy_editor.py`** — schema-driven, so adding a tactic or trigger
in `common_sim` makes it appear in the GUI with **zero GUI edits**.
Each Trigger/Tactic class declares:
```python
PARAM_SCHEMA = (
    Param("piece_type", kind="piece_type", default=None, optional=True),
    Param("mode", kind="choice", choices=("nearest","densest")),
    Param("standoff", kind="float", min=0, max=120, suffix=" in"),
    Param("region", kind="region_name", optional=True),
)
```
`kind="region_name"` / `"piece_type"` / `"action"` populate their combo
boxes from the live `match.field` / scoring regions — which is exactly
why regions were given names earlier. Layout: rule list on the left
(drag to reorder = priority, checkbox to enable, +/- to add/remove),
property inspector on the right (trigger builder tree + tactic params),
Load / Save / **Apply on Reset** at the bottom, plus a robot selector
("My robot", "Partner 1", "Opponent 1", ...) so each robot gets its own
strategy.

**`strategy_graph.py`** — `QGraphicsView` state-machine diagram:
- One rounded node per rule, laid out in priority bands (highest at
  top), colored by tactic type; the `fallback`/Idle node at the bottom.
- Edges = possible transitions, labeled with `trigger.describe()`.
  Drawn from each node to every strictly-higher-priority node whose
  trigger can preempt it, plus completion edges back to the fallback.
- **Live mode**: while a match runs, the active rule's node pulses in
  the alliance color and the traversed edge flashes on each
  `behavior_change` event. Clicking a node selects it in the editor.
  A small transition-history strip under the graph doubles as a
  strategy timeline.
- Styled with `gui_utils/theme.py` constants — no new palette.

## 5. Example strategies (`game_specific/reefscape/strategies/`)

JSON files that double as the format's documentation and as test
fixtures:
- `cycle_coral.json` — Collect(coral, prefer_station) ⇄ Score(auto),
  driven by `AtCapacity` / `PiecesHeld(max=0)`.
- `algae_processor.json` — mixed-type cycling, exercises multi-piece
  planning.
- `endgame_defense.json` — adds a `MatchTime(remaining_under=25)`
  high-priority `Defend(target="opponent_intent")` rule over a normal
  cycle, i.e. the canonical preemption demo.
- `auto_then_cycle.json` — `RunScript` auto at priority 100 gated on
  `MatchTime(phase="auto")`, cycling below it.

## 6. Monte Carlo integration

`ParameterSweep` already takes arbitrary named params, so a
strategy-comparison sweep is `{"strategy": ["cycle_coral.json",
"algae_processor.json"], "max_speed": [...]}` with a module-level
`trial_fn` that loads the strategy and attaches a controller. Add
`apps/run_strategy_sweep.py` demonstrating exactly that, plus a
`strategy` column in the results DataFrame. No changes to
`monte_carlo.py` itself.

## 7. Build order (each step independently testable, headless)

1. `world_view.py` + tests against a synthetic field. *(no deps)*
2. `navigation.py` (`plan_path`, `NavigateTo`, `estimate_travel_time`)
   + tests that a robot routes around an obstacle instead of into it.
   **Do this before tactics** — everything downstream depends on robots
   actually arriving.
3. `ScoringRegion.capacity_by_action` + `Match` enforcement + REEFSCAPE
   capacities.
4. `triggers.py` + tests (pure, fast).
5. `planning.py` + tests (option ordering vs. hand-computed rates).
6. `tactics.py` + per-tactic tests on the synthetic game.
7. `strategy.py` arbiter + tests: priority preemption, `min_duration`
   hysteresis, cooldown, `once`, fallback, event logging.
8. `Match` controller ticking + `Robot.controller`.
9. `strategy_io.py` + round-trip test + the four example JSON files.
10. `apps/run_reefscape.py` tab refactor (`MatchView` extraction).
11. `gui_utils/strategy_editor.py`.
12. `gui_utils/strategy_graph.py` + live highlighting + canvas intent
    overlay.
13. `apps/run_strategy_sweep.py`; update `ARCHITECTURE.md`.

Steps 1–9 are headless and carry the real risk; 10–13 are GUI work that
can't start until the spec format (9) is frozen.

## 8. Open questions

- **Alliance partners / opponents on the field.** Defend is meaningless
  without at least one opposing robot, and `run_reefscape.py` currently
  spawns exactly one. Proposal: a roster panel to add N robots per
  alliance, each with its own strategy file and characteristics, plus an
  "AI drives my robot" toggle so a strategy can play a whole match while
  you watch. This is a prerequisite for Defend, not an extra.
- **Contact/pushing model for Defend.** Blocking currently only works
  through pymunk chassis collisions; there's no pushing-power or
  contact-penalty model. Day one = pure positional denial (get in the
  way), which is enough to study whether denial is worth the cycles.
- **Endgame/climb** still isn't modeled (noted in `field.py`), so a
  `MatchTime`-triggered climb tactic has nothing to do yet.
