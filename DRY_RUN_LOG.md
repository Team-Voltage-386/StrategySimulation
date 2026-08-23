# New-game dry run: SALVAGE 2027

A timestamped log of what it actually cost to stand up a second game in
`game_specific/`, kept the way `benchmarks/README.md` keeps its lessons
— because the whole justification for this project's architecture is a
two-day turnaround on game-reveal day, and until this ran, every claim
about that turnaround was a guess.

The game is invented. `game_specific/salvage/field.py` explains what it
is and, more importantly, *why each of its features was chosen*: it is
deliberately awkward in the places REEFSCAPE is smooth, so that the
framework gets asked questions it has never been asked.

**Session: 2026-08-22, one evening.** Written from scratch, in order:
`game_pieces.py`, `scoring.py`, `field.py`, `robot.py`, `sweep_trial.py`,
six strategy files. About 600 lines, matching the estimate. The first
2v2 match ran within the hour and **was wrong**, which is the finding
this whole exercise existed to produce.

---

## The headline

A stock 2v2, both alliances running the same strategy on a
rotationally-symmetric field, scored **red 243 – blue 73**.

Two framework bugs, both game-agnostic, both invisible in REEFSCAPE:

| # | Bug | Effect on the dry run |
|---|---|---|
| F1 | `navigation.clear_standoff` never checked the field perimeter | parking poses outside the field near any wall-adjacent target |
| F2 | `Score` re-chose the same unreachable target forever | one robot frozen 110s of a 150s match |

Fixing F2 alone took blue from **76 to 218**. The final line is
**red 236 – blue 218** on the same seed — a real match, and one where
the residual gap is now small enough to be strategy rather than defect.

Neither bug is about SALVAGE. Both are latent in REEFSCAPE and one of
them (F2) is the fifth instance of the failure family
`apps/run_stall_audit.py` was written to hunt.

### And they cost REEFSCAPE nothing

Both are behavior changes, so the REEFSCAPE fingerprint moves and the
defense grid is the only honest way to price them. Run paired -- the
same 40 seeds through a clean `HEAD` worktree and through the fixed
tree, differencing within each seed rather than across two means:

    row                              before                after            paired diff        W/L
    none/cycle_coral        245.6 sd 9.5 min224   244.1 sd 8.5 min229   -1.45 +/-1.81 (-0.8)  19/20
    none/pursue_tuned       251.5 sd 6.0 min237   252.2 sd 4.7 min240   +0.70 +/-1.20 (+0.6)  19/16
    block/scoring/cycle     226.7 sd11.8 min206   224.1 sd13.8 min194   -2.60 +/-2.46 (-1.1)  14/24
    block/scoring/pursue    221.9 sd11.6 min195   223.6 sd14.5 min201   +1.70 +/-2.77 (+0.6)  22/17
    block/supply/cycle      189.7 sd14.9 min167   189.6 sd17.0 min152   -0.10 +/-2.67 (-0.0)  16/23
    block/supply/pursue     194.3 sd12.2 min165   191.8 sd 9.6 min174   -2.50 +/-2.28 (-1.1)  13/26
    block/any/cycle         207.4 sd16.5 min163   205.9 sd17.0 min172   -1.48 +/-3.26 (-0.5)  16/23
    block/any/pursue        206.1 sd24.7 min108   203.2 sd24.5 min 83   -2.90 +/-4.84 (-0.6)  20/20
    shadow/scoring/cycle    192.9 sd15.8 min170   194.8 sd15.4 min163   +1.95 +/-2.92 (+0.7)  22/17
    shadow/scoring/pursue   192.9 sd14.7 min157   195.3 sd15.0 min168   +2.38 +/-3.04 (+0.8)  21/17
    shadow/supply/cycle     229.9 sd29.1 min162   222.5 sd15.3 min178   -7.38 +/-4.59 (-1.6)  14/25
    shadow/supply/pursue    219.9 sd12.8 min182   220.5 sd14.2 min173   +0.62 +/-2.68 (+0.2)  24/16
    shadow/any/cycle        198.8 sd14.4 min176   202.2 sd15.9 min170   +3.40 +/-2.77 (+1.2)  22/17
    shadow/any/pursue       194.6 sd14.4 min167   199.4 sd11.8 min172   +4.78 +/-2.72 (+1.8)  24/16

    average over 14 rows: -0.21

**Nothing moves.** No row reaches 2 SEM in either direction, and across
fourteen comparisons you would expect one to by chance anyway. That is
the right result for a bug fix: the bugs never fired on this field, so
removing them buys nothing here and costs nothing here.

The one thing worth noticing is `shadow/supply/cycle_coral`, whose
standard deviation falls 29.1 -> 15.3 and whose worst match rises 162 ->
178 while its mean drops. That is the same signature the station-escape
work left: a failure mode removed from the tail, not points added to the
middle.

---

## Findings

### F1 — `clear_standoff` treated the field perimeter as empty space
*Status: **fixed**, `common_sim/control/navigation.py`.*

`clear_standoff` rotates a parking pose around a target until the
chassis clears `field.obstacles`. The field's own walls are not in
`field.obstacles` — they never have been, and `plan_path` already
carries a comment about exactly this, having been bitten by it once
before (`_in_bounds`, and commit `bca0fcc` for the wedge test).
`clear_standoff` had not been.

Measured directly: aiming at SALVAGE's `blue_hold_high` from the north
returned a pose at (220, 309) on a 320-inch-tall field — three inches of
chassis through the guardrail. The robot drives at it, the contact
solver holds it, and it never "arrives" at a target it is standing on.

REEFSCAPE hides this because its scoring regions sit in open field. Its
CORAL STATIONs are in the corners, which is the one place it could have
bitten, and the repeated corner-stall bugs in that area
(`bca0fcc`, `7e545e8`) are suggestive but not proof.

The fix tests the *ungrown* chassis against the perimeter — a robot may
put its bumper on the wall, it may just not be outside it — and only
does so when the target is within reach of a wall at all, so an
open-field aim keeps the original first-bearing fast path untouched.

### F2 — `Score` had no ratchet on re-choosing the same target
*Status: **fixed**, `common_sim/control/tactics.py`.*

The trace that found it, one robot, five-second samples:

```
t=  42.0 ( 182, 300) v 0.0 ask 0.1  want=blue_hold_high/hold_high  aim=(196,300) park=(182,300,-0.00) eng=False om=-4.43 w=-0.00
t=  44.0 ( 182, 300) v 0.0 ask 0.1  want=blue_hold_high/hold_high  aim=(196,300) park=(182,300,-0.00) eng=False om=-4.43 w=-0.00
...unchanged for 110 seconds...
```

Read that line carefully, because every individual number in it looks
fine. The robot is **at** its computed parking pose. The pose is
correct — it is the right standoff from a legal aim point inside the
region it chose. It is commanding essentially **no translation**. It is
re-picking its target on schedule and getting the same answer.

What is wrong is `om=-4.43 w=-0.00`: it is asking for 4.4 rad/s of
**rotation** and getting none. A corner of its chassis is against a
structure, it cannot turn into the heading the deposit needs, and so
`side_engages_polygon` is never satisfied.

`Score._reconsider_target` already re-opens the choice when an attempt
overruns its patience budget, and already cools down a target it walks
away from. But it deliberately did *not* cool down a re-pick that landed
on the same place — "the robot still trying, not giving up". With
`_pick_option` deterministic in the field state, that closes a loop: the
same target is still best-valued, `_commit` restarts the clock, nothing
goes on cooldown, and the robot tries the identical impossible thing
until the buzzer.

The fix is a two-strike ratchet (`_SAME_TARGET_OVERRUNS`): one re-pick
onto the same target is still trying, two consecutive ones cools it down
anyway and forces a different choice. Full suite green (524 tests).

### F3 — the stall audit's `asking` column is translation-only
*Status: **fixed**, `apps/run_stall_audit.py` + a new
`Robot.commanded_angular_speed`.*

This one matters more than its size. The audit reports `asking`, the
mean commanded speed over a frozen window, and the documented reading is
that "near zero means the robot **chose** to wait, a large number means
it is being **held**." That reading was used to dismiss eleven of
thirteen surviving stalls as benign.

`Robot.commanded_speed` is `chassis.commanded_velocity.length` —
translation only. The F2 robot was being held as hard as a robot can be
held, and reported `asking` ≈ 0.1.

So a rotationally pinned robot is currently indistinguishable from a
robot standing still on purpose, and the audit's own conclusions about
the REEFSCAPE grid were drawn with that blind spot in place.

### F4 — game nouns in game-agnostic code
*Status: **open**, cosmetic but exactly the kind of thing reveal day
does not need.*

`MatchConfig.emit_coral_to_field` is the master switch for
`Match._step_emitters`. SALVAGE has no coral; enabling its SCRAP drops
means writing `emit_coral_to_field=True` in a file that has never heard
of coral. The field's own docstring already anticipates this ("a future
game with a differently-themed emitter would still reuse this same
flag") — the anticipation was right and the name is still wrong.

### F5 — the reference robot has no home
*Status: **fixed**, `game_specific/reefscape/robot.py`.*

"What does this game's benchmark robot look like" used to be answered in
REEFSCAPE by `apps/reefscape_widgets.py` **and again**, duplicated
verbatim (and already drifted -- the sweep copy was missing the preloaded
CORAL and the explicit reliability dict), by `apps/run_strategy_sweep.py`.
Neither was where a second game would look. REEFSCAPE's copy now lives
in `game_specific/reefscape/robot.py`, matching the SALVAGE pattern:
`apps/reefscape_widgets.py`'s `build_demo_characteristics` and
`apps/run_strategy_sweep.py`'s `build_characteristics` both delegate to
it. The GUI's extra preloaded-CORAL behavior is passed as explicit
overrides rather than folded into the shared defaults, so no existing
caller's numbers moved.

### F6 — the diagnostic tooling does not port
*Status: **fixed**, `common_sim/analysis/game_bench.py`.*

`apps/run_defense_bench.py` and `apps/run_stall_audit.py` were the two
most valuable measurement tools in the repo and both imported
`game_specific.reefscape.sweep_trial` at module scope; the audit also
reached into the bench for `build_job`. Every hour of debugging SALVAGE
that night was spent in hand-written scratch scripts re-deriving what
those two already did.

The fix was exactly the shape it looked like: a `BenchGame` (match
builder, strategies directory, dt, reference robot) plus the job-building
and trial logic, extracted into `common_sim/analysis/game_bench.py` --
game-agnostic, no Qt, no `game_specific`, enforced by
`test_import_contract.py` the same as everything else under
`common_sim`. `apps/run_defense_bench.py` and `apps/run_salvage_bench.py`
are now the thin, game-specific remainder: a plan table plus a `GAME`.
`apps/run_stall_audit.py` takes `--game reefscape|salvage`, importing
each game's plan table and match builder lazily so picking one never
pulls in the other's strategy files -- the stall audit that found F2 now
runs against both games from the same command.

### F7 — nothing validates a `FieldConfig`
*Status: **fixed**, `common_sim/field/validation.py`.*

Both F1 and F2 presented as "a strategy is losing", and it took a trace
to find that a region was effectively unreachable. Nothing in the
framework will tell a game author that a scoring region has no legal
approach, that a gap between two obstacles is narrower than a robot,
that an action has no entry in the scoring table, or that a piece type
has no registered spec. All of those are static properties of a
`FieldConfig` plus a robot size, checkable in milliseconds, and all of
them produce a silently plausible-looking wrong match instead of an
error.

`validate_field(field, robot_width=, robot_length=, scoring_rules=)`
now answers all of them, at three severities: `ERROR` for something that
makes a match meaningless, `WARNING` for a probable mistake, `NOTE` for
geometry worth a second look. It deliberately imports
`control.navigation`, because a checker with its own private idea of
standoffs and inflation is a second opinion that can disagree with the
simulator.

On the two fields that exist it reports:

* **SALVAGE: no errors, two notes** -- and the two notes are the
  38-inch gap between a pylon and the north wall, against a 39.6-inch
  robot diagonal. That is the exact pocket F1 and F2 wedged a robot
  into. The checker would have found in milliseconds what took a
  tick-by-tick trace.
* **REEFSCAPE: no errors, four warnings** -- each corner CORAL STATION
  has three of its four corners outside the field. True, longstanding,
  harmless (a robot only has to reach the part that is in), and worth
  knowing.

Two things it does *not* do, on purpose. It is never called
automatically: a game package calls it from its own tests. And it takes
no view on whether a `NOTE` is a mistake, because a pinch point is
legal geometry and a game is entitled to one.

### F8 — value that changes mid-match can only change at the auto boundary
*Status: **open**, design note.*

`Phase` is `AUTO | TELEOP`, and `ScoringRules.points_for(action, phase)`
is the only time-dependence in scoring. SALVAGE gets its time-varying
values by loading them onto that one boundary, which was enough. A game
with an endgame multiplier, a scoring window, or a target that opens
partway through cannot currently be expressed without a custom
`ScoringRules` that closes over the `Match` — which nothing supports.
`ARCHITECTURE.md` already describes the phase as "auto/teleop/endgame",
so the intent was there.

### F9 — `plan_path` can route through an obstacle when two are close together
*Status: **fixed**.*

Found by pointing the new checker at a synthetic test field. A
polygon's own boundary edges are added to the visibility graph
*unconditionally*, bypassing the visibility test — correctly, because an
edge's midpoint sits exactly on its own polygon's boundary, where
`point_in_polygon`'s ray cast is a coin flip. But the bypass skips every
*other* polygon too. When two obstacles are closer together than twice
the robot radius their inflated outlines overlap, and one polygon's own
boundary edge can then run clean through the other.

The repro is exact:

```
obstacles at x 160-190, 210-240 (y 70-150) and y 70-100, 120-150 (x 160-240)
plan_path((333, 110) -> (54, 110), robot_radius=19.8)
  -> [(333.3, 110.0), (259.8, 100.2), (140.2, 100.2), (54.0, 110.0)]
```

The middle leg is a straight line at y=100.2 from x=259.8 to x=140.2,
which passes through both of the x-range obstacles. Those two endpoints
are consecutive vertices of the *third* obstacle's inflated outline, so
the edge was never tested.

The fix was exactly that: a boundary edge is now checked with `_visible`
against every *other* polygon (own polygon excluded, so the coin-flip
case `boundary_edges` exists for still doesn't apply) before it's
trusted, rather than unconditionally. The synthetic repro above no
longer cuts through either obstacle -- the middle leg now routes around
at y=50.2 instead of straight across at y=100.2 -- and
`test_field_validation.py`'s `test_a_region_walled_in_on_every_side_is_an_error`
(the field this bug was found on) now reports only the region that is
genuinely unworkable, not the feeder that F9 made falsely look
unroutable. Full suite (569 tests) is green.

Neither current field's own *structures* trigger the bug (REEFSCAPE's
two hexes are far apart, SALVAGE's closest pair is 43in against a 39.6in
diagonal), but the fix still touches every plan: `NavigateTo._replan`
feeds other robots in as `extra_obstacles`, and robot outlines overlap
each other and the structures constantly, so boundary edges that used to
bypass the check now sometimes don't. That was worth a REEFSCAPE
defense-bench pass before calling it behavior-neutral there, and it is
not neutral: 24 seeds, `apps/run_defense_bench.py`, blue points, fix vs.
pre-fix (same seeds, `git stash` on just the one file) --

    red plan        blue points (post-fix)   blue points (pre-fix)   red fouls
    none            244-256, +-2                same, +-2               0 both
    block/*         167-215, -1 to -18          same range              down 1-3
    shadow/*        171-213, -13 to -32         196-227                 down 5-8

`none` (no defender) is unaffected, as expected -- nothing is close
enough to overlap. `block/*` (defender holds a fixed position) moves a
little. `shadow/*` (defender stays glued to its mark) moves a lot in
one direction, 13 to 32 points down across every blue lineup, red fouls
down 5-8 with it. That is the exact mechanism: "shadow" is the one mode
that keeps the defender's inflated footprint overlapping blue's
constantly, which is precisely the condition F9 needs, and the pre-fix
numbers were blue routing *through* the overlap -- extra contact (hence
the higher foul count) but also, incoherently, a *better* score for it.
Post-fix blue routes around, contact drops, and so does blue's
production. This is a real balance shift for anyone tuning `shadow` as
a defensive tactic, not measurement noise -- it moves every one of the
15 shadow/* x blue-plan cells the same direction.

---

## What the dry run was actually for

Finding two framework bugs was the unplanned half. The planned half was
the question REEFSCAPE cannot answer: **does the arbitration layer
transfer to a game it was not developed on?**

REEFSCAPE under-tests `Pursue` by construction. Its decision is nearly a
constant -- cycle CORAL -- so a correct static policy is near-optimal and
an arbiter can at best tie it. Measured paired at 80 seeds, `Pursue` is
level with `cycle_coral` under defense (+0.01 average) and wins the
undefended row by +8.45. Two of its mechanisms never run at all: the
`enables` lookahead only ever goes one level deep, and the scarcity term
sits at weight 0 because a CORAL STATION never empties.

SALVAGE is built so a constant policy is *wrong*. The valuable target
changes at the end of AUTO, the depot runs dry, and the BEACON fills.
`apps/run_salvage_bench.py`, 2v2, 24 paired seeds, 480 matches, blue
points against `cycle_crates` on the same seeds:

    red plan        blue plan        mean    sd  min       vs cycle_crates   W/L
    none            cycle_crates    216.0  14.1  159                   --
    none            rush_reactor    239.7  24.4  186   +23.67 (+5.2 SEM)  20/4
    none            pursue          268.7  40.0  146   +52.62 (+7.9 SEM)  21/3
    none            pursue_tuned    162.3  54.7   98   -53.75 (-5.0 SEM)   5/19
    none            pursue_scarce   277.3  20.8  249   +61.25 (+12.5 SEM) 24/0

    block/scoring   cycle_crates    171.1  25.1   78                   --
    block/scoring   rush_reactor    213.7  17.1  186   +42.58 (+7.1 SEM)  24/0
    block/scoring   pursue          198.7  17.8  159   +27.62 (+4.8 SEM)  21/3
    block/scoring   pursue_tuned    146.6  32.8   82   -24.50 (-2.8 SEM)   5/19
    block/scoring   pursue_scarce   202.8  21.2  165   +31.71 (+4.6 SEM)  21/3

    block/supply    cycle_crates    164.8  14.5  138                   --
    block/supply    rush_reactor    190.0  21.0  155   +25.25 (+5.1 SEM)  20/4
    block/supply    pursue          218.7  31.4  118   +53.96 (+9.4 SEM)  23/1
    block/supply    pursue_tuned    160.2  42.7   81    -4.54 (-0.4 SEM)  12/12
    block/supply    pursue_scarce   222.1  22.7  157   +57.33 (+12.0 SEM) 24/0

    shadow/any      cycle_crates    169.2  25.9  137                   --
    shadow/any      rush_reactor    109.5  70.9   20   -59.71 (-3.8 SEM)   8/16
    shadow/any      pursue          180.7  25.0  114   +11.54 (+1.8 SEM)  17/7
    shadow/any      pursue_tuned    139.4  49.1   42   -29.79 (-2.4 SEM)  10/14
    shadow/any      pursue_scarce   177.7  23.6  114    +8.54 (+1.4 SEM)  18/5

Three results, and they are much larger than anything the REEFSCAPE
grid can resolve.

**1. The arbiter transfers, and here it is decisive.** `Pursue` beats
the static plan on all four rows, by +52.6 undefended at 7.9 SEM and
+54.0 under supply denial at 9.4 SEM. That is the *opposite shape* to
REEFSCAPE, where it is level under defense and wins modestly undefended.
Read together, the two games say the arbiter's value is proportional to
how much there is to decide -- which is the claim it was always making
and which one game could not test.

**2. The scarcity term works, and had never been shown to.**
`pursue_scarce` is plain `Pursue` with `scarcity_weight` 0.6 and
`scarcity_floor` 4.0, and it is the best plan on the board: +61.3 at
12.5 SEM undefended, 24 seeds to 0, and +57.3 under supply denial, again
24-0. The term was written for REEFSCAPE, defaulted to 0.0 because it
could not be validated there, and has sat inert ever since. A game with
a finite contested supply says it was right.

**3. The tuned parameters transfer *negatively*, and badly.**
`pursue_tuned` carries the CMA-ES numbers fitted on the REEFSCAPE
defense grid. It is **worse than untuned defaults on every row** --
-53.8 undefended -- and worse than the static plan it was tuned to beat.
Its standard deviation is roughly double plain `Pursue`'s on three of
four rows (54.7 vs 40.0, 42.7 vs 31.4, 49.1 vs 25.0), so it is not
losing evenly; it is collapsing on some seeds.

That third result is the operational one for reveal day. The instinct
on January 2 will be to start from the parameters that won last season.
This says: **start from the defaults, and re-fit against the new game or
not at all.** A fitted vector is a statement about a game, not about a
robot.

### Which number does not transfer, and why

Worth knowing exactly, because "don't transfer tuning" is advice and
"don't transfer *this*" is a fix. One parameter at a time, from the
defaults, 24 paired seeds on the undefended row:

    variant                mean    sd  min          vs defaults    W/L
    defaults              268.7  40.0  146                   --
    all (pursue_tuned)    162.3  54.7   98  -106.38 ( -7.9 SEM)   1/23
    lookahead_weight      133.5  41.1   88  -135.17 (-12.4 SEM)   0/24
    switch_margin         277.0  22.6  227    +8.38 ( +1.0 SEM)  13/9
    reliability_weight    274.1  25.9  202    +5.42 ( +0.7 SEM)  10/13
    claim_penalty         271.5  27.6  205    +2.83 ( +0.4 SEM)  13/11
    min_commit            269.8  37.4  146    +1.12 ( +0.3 SEM)   6/5
    time_fit_slack        268.6  40.0  146    -0.08 ( -1.0 SEM)   0/1
    contest_penalty       268.7  40.0  146            identical   0/0
    rule min_duration/cooldown   268.7  40.0  146     identical   0/0

**It is one number.** `lookahead_weight`, alone, is -135 points at 12.4
SEM and loses all 24 seeds. Every other tuned value is inside noise, and
two are bit-identical no-ops. The full tuned vector is *less* bad than
that one parameter on its own (-106 against -135), so the rest of the
search's work was partially compensating for it.

Note also that the rule-level `min_duration` and `cooldown` are again
bit-identical no-ops on a one-rule strategy -- the same result as on
REEFSCAPE, now confirmed on a second game.

The mechanism is not subtle once you look. Same seed, same everything
else:

    lookahead_weight=1.000  blue=303   intaked 104 -> scored 97
      intakes {crate: 80, cell: 12, scrap: 12}
      scores  {hold_low: 70, airlock: 11, reactor: 10, hold_high: 6}

    lookahead_weight=1.346  blue= 94   intaked  19 -> scored 13
      intakes {cell: 13, crate: 5, scrap: 1}
      scores  {reactor: 10, hold_low: 2, hold_high: 1}

`lookahead_weight` multiplies the *collect* branch's rate in
`Pursue._cycle_rate`, so above 1.0 it systematically prefers fetching to
scoring. The high-weight robot takes 19 pieces in 150 seconds instead of
104, and thirteen of the nineteen are CELLs -- it fixates on the fetch
with the best payoff (the REACTOR pays 10 in AUTO) and keeps driving to
the depot instead of scoring what it is already holding.

**REEFSCAPE structurally cannot punish that.** Every capacity there is
1, and the piece you fetch is the piece you must score, so "prefer
fetching" is not a choice the robot can act on -- there is nothing else
to do while empty and nothing to fetch while full. SALVAGE has a
two-CRATE capacity and three piece types, so preferring the fetch is a
live option, and it is the wrong one.

That is a much sharper statement than "the search overfit". The search
found a parameter that is *unconstrained* on REEFSCAPE -- free to drift
anywhere without cost -- and drifted it. The lesson for a search, rather
than for a transfer, is that a parameter the game cannot punish is not
tuned, it is uncontrolled, and its final value is an artifact.

---

## What was actually game-specific

Useful for costing the real thing. `game_specific/salvage/` is:

| file | lines | genuinely game content? |
|---|---|---|
| `field.py` | ~300 | yes — all of it |
| `game_pieces.py` | ~55 | yes |
| `scoring.py` | ~45 | yes |
| `robot.py` | ~60 | yes |
| `sweep_trial.py` | ~135 | **no — only `_stage_pieces`, `start_pose`, and 3 imports** |
| `strategies/*.json` | 6 files | yes |

`sweep_trial.py`'s `_resolve_strategy`, `build_match_for_job`,
`run_trial` and `replay_trial` are identical to REEFSCAPE's modulo the
imports. It was copied deliberately rather than factored first — the
point of a dry run is to find out what is shared by observing it, not by
predicting it. Now that both exist, the shared four functions want to
move to `common_sim/analysis/` with the game supplied as a small
protocol (build field, build scoring rules, stage pieces, start pose).

## What the framework got right

Worth recording, because a log of only complaints is misleading:

* The import contract held. `test/test_import_contract.py` stayed green
  the whole way; nothing in `common_sim` needed to learn what a CELL is.
* Every primitive SALVAGE wanted already existed and was already
  general: neutral (`alliance=None`) scoring regions and intake
  locations both worked first time, `capacity_by_action` expressed a
  shared six-slot BEACON, `starting_pieces` expressed a finite contested
  depot, `passive_scoring` expressed a lob, and `EmitterRegion`
  expressed pieces arriving on the clock.
* Three piece types, per-type capacities of 2/1/1, and per-type intake
  times all worked with no changes — even though every REEFSCAPE
  capacity is 1, so the multi-piece paths had never run in a match.
* `PinRule` and `ProtectedZone` took different numbers without
  complaint.
