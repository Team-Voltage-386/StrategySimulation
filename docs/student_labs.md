# sparky-sim student labs

The detailed, screenshot-based version is generated as
[`student_labs_guide.html`](student_labs_guide.html) by:

```
python -m apps.build_guide --guide labs
```

Its source is [`student_labs_template.html`](student_labs_template.html). This
Markdown page remains a compact outline for readers who prefer plain text.

These short exercises use the simulator as a way to learn how our robot
decisions are represented, tested, and challenged.  They are not tutorials to
memorize: write a prediction first, then use the evidence the simulator gives
you to revise it.

## Before starting

Open the REEFSCAPE app with `run.bat`.  The generated tab guides explain every
control; rebuild them with `python -m apps.build_guide` if the UI has changed.
For headless work, run commands from the repository root with the same Python
that runs the app.

Keep a one-page lab note with four headings: **prediction**, **setup**,
**result**, and **what I would test next**.  Record seeds, strategy names, and
every value you changed.  A result that cannot be recreated is not useful for
a design decision.

## Lab 1 — Trace one CORAL cycle

Goal: connect the code a robot runs to what appears on the field.

1. In the MATCH tab, set PRIMARY to AI-driven and select `cycle_coral`.
   Reset, run for roughly 20 seconds, pause, and scrub through a pickup and a
   deposit.
2. Find `game_specific/reefscape/strategies/cycle_coral.json`.  For the rule
   that was active during each moment, trace the named trigger and tactic into
   `common_sim/control/`.
3. Continue into the code that produces the visible effect: navigation,
   manipulator timing, then `Match` scoring.  Use the event/telemetry panels to
   identify when your explanation agrees with the simulation.
4. Explain one reason a robot can be in a scoring region yet fail to score.

Deliverable: a one-page flow diagram from strategy JSON to score event, with
the filenames/functions you visited and one timestamped observation.

## Lab 2 — Make and test a cycle-time claim

Goal: distinguish an intuition from a measured conclusion.

1. In SWEEP, vary PRIMARY's `max_speed` across a modest range, such as
   130/150/170, with at least five repetitions.  Keep all other choices fixed.
2. Before executing, predict the ordering of mean score and explain what could
   prevent the fastest robot from winning.
3. Read the results and plots.  Compare score, pieces scored, and cycle time;
   do not rely on one lucky row.
4. Double-click one result and replay it in MATCH.  Write down its seed and a
   concrete field event that helps explain the result.

Deliverable: one chart or table, the original prediction, and a conclusion
that states both the observed effect and its uncertainty.

## Lab 3 — Strategy change, held-out check

Goal: practice avoiding overfitting.

1. Copy an existing strategy JSON under a new descriptive name.  Make one
   small, explainable change: priority, dwell/cooldown, a target action, or a
   fallback tactic.
2. Use the STRATEGY graph and a MATCH replay to verify that the intended
   transition actually occurs.
3. Compare the old and new strategy in SWEEP using a first set of seeds.
4. Repeat the comparison with different seeds before claiming an improvement.
   If SEARCH is used, treat its confirmation result—not its best generation—as
   the decision evidence.

Deliverable: a short change note, two seed sets, and a decision: keep, reject,
or investigate further.  Include an explanation when the numbers disagree
with the replay.

## Lab 4 — Read a release regression

Goal: learn what we protect when refactoring the simulator.

Run:

```
python -m apps.run_regression_scenarios
```

Then read `game_specific/reefscape/regression_scenarios.py` and its test.
The two scenarios are intentionally short and fixed-seed: one verifies normal
cycling; the other verifies that a cyclist and defender can share the real
field.  They are not strategy rankings.

For each scenario, identify the field/strategy/match components it crosses.
Choose one metric that should remain exactly fixed and one metric that would
need a broader multi-seed study before you called it a regression.  If a
golden number changes, replay and explain the behavioral cause before editing
the expected value.

Deliverable: a brief review comment describing whether a hypothetical score
change is an intentional behavior change, a bug, or still unknown.
