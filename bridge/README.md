# The maple-sim bridge

Drives the team's **real robot code**, running under maple-sim in WPILib's
simulator, from Python — so a competitive scenario can be generated on a night
when the robot is on the mechanical team's cart and the field doesn't exist yet.

Background and rationale: [The maple-sim Bridge](https://claude.ai/code/artifact/648dfe02-ea7d-4b2f-b0ee-44094eb28407).

**Status: steps 1–4 of 7 done.** The loop closes, the oracles fire, the harness
runs unattended and leaves a morning report, the live REBUILT world is
readable, and sparky-sim's strategy layer drives the real robot code from it.
What remains widens the variety of situations it reaches.

## Run it

```
pip install -r bridge/requirements.txt
python apps/run_bridge_smoke.py      # step 1: does the loop close?
python apps/run_bridge_oracles.py    # step 2: can it tell a failure from a run?
python apps/run_bridge_overnight.py --matches 40    # step 3: the product
python apps/run_bridge_world.py      # step 4a: can it see the field?
python apps/run_bridge_strategy.py   # step 4b: can it play?
```

Each takes ~90 s: gradle compiles, the JVM boots, the checks run, the sim is killed.

```
python apps/run_bridge_smoke.py --dump-topics    # everything the robot publishes
python apps/run_bridge_smoke.py --echo           # stream the robot console
python apps/run_bridge_smoke.py --attach         # against a sim you started yourself
python apps/run_bridge_oracles.py --no-provoke   # exercise only (leaves oracle 02 unverified)
```

`pytest test/test_bridge_oracles.py` covers oracle 01's rules with no JVM
involved, so they run in CI.

No JAVA_HOME needed — `sim_process.find_java_home` locates the WPILib JDK.

## Pointing it at the robot project

Check the robot repo out **beside** this one and the bridge finds it:

```
somewhere/
  sparky-sim/        <- this repo, on feature/maple-bridge
  TyRapXXVI/         <- the robot repo, on feature/maple-bridge
```

Otherwise, either per run or once:

```
python apps/run_bridge_smoke.py --repo /path/to/robot-project
export SPARKY_ROBOT_REPO=/path/to/robot-project
```

Discovery looks for a directory with both `build.gradle` and
`vendordeps/maple-sim.json`, **and** the `-Pbridge` profile inside that
`build.gradle`. The last condition is the one that earns its keep: "is a robot
project" and "is a robot project this can talk to" are different questions, and
two checkouts of the same repo side by side — one on the bridge branch, one on
`main` — is not a hypothetical layout. Picking the wrong one launches a sim
that never opens its WebSocket, and that surfaces as a connection timeout three
minutes into a gradle build.

It refuses to guess when more than one candidate qualifies, and says so. If
*none* does, the error names the missing profile rather than the missing
directory — the usual cause is that the robot-side branch has not been checked
out, and the two branches have to move together.

## Shape

| Module | Direction | Transport |
|---|---|---|
| `operator.py` | Python → robot | HALSim WebSocket, `ws://127.0.0.1:3300/wpilibws` |
| `robot_state.py` | robot → Python | NetworkTables 4, via `pyntcore` |
| `sim_process.py` | — | launches and reaps `gradlew simulateJava -Pbridge` |
| `oracles.py` | — | decides whether a run failed |
| `scenario.py` | — | the seeded virtual operator |
| `harness.py` | — | campaign lifecycle, retention, and the report |
| `world_state.py` | robot → Python | the live REBUILT field, over NT |
| `arena.py` | — | REBUILT's static geometry, transcribed from maple-sim |
| `drive_model.py` | Python → robot | velocities ↔ joystick axes, and its calibration |
| `match_view.py` | both | the live world in the shape tactics already read |

Input is injected **at the joystick layer**, not at the command layer. The
binding layer and its interlocks are a substantial part of what is being
tested, so bypassing them with NT-driven `Trigger`s would test less for no
less work.

The only robot-repo change is a gated block in `build.gradle`: `-Pbridge` swaps
the Sim GUI and the real-DriverStation extension for `halsim_ws_server`. Without
the flag, `simulateJava` behaves exactly as it always has.

## What the smoke test proves

Four checks, ordered so the first failure names the layer at fault.

1. **LINK** — WebSocket open, NT publishing. Transport only.
2. **ECHO** — with the robot still *disabled*, a distinctive stick pattern is
   pushed and read back off AdvantageKit's own DriverStation log. This is the
   important one: it separates "the wire works" from "the robot code reacted".
3. **AXIS** — hold the left stick; maple-sim's ground-truth pose moves ~1.25 m
   in 2 s and stops on release.
4. **BUTTON** — hold the left bumper; `flywheel.shootCommand` runs, the setpoint
   goes to 2200 and simulated wheel speed follows to ~230. A non-drive
   subsystem, so it isn't the drive path twice.

## The oracles

**01 — hard faults** (`FaultOracle`). Reads the captured console after the run,
plus AdvantageKit's `Alerts` arrays live. Reports stack traces (one finding
carrying its frames, not one per frame), `DriverStation.reportError/Warning`,
hard stops, and sustained loop overruns.

The muffle list is the part that decides whether anyone reads the morning report
twice. Every entry carries a written reason, so the next person can re-litigate
it instead of guessing what it was hiding. It currently muffles gradle output,
javac warnings, the JNI boilerplate line that contains the word "errors" and
reports nothing, our own NT teardown, and unpopulated-joystick warnings.

**02 — liveness** (`LivenessMonitor`). Samples NT at 20 Hz while the run
proceeds. Six detectors, each debounced by *duration* — a single sample of "not
moving" is scheduling jitter, two seconds of it is a bug — and each latched to
fire once per episode, re-arming only after the condition clears.

| kind | severity | fires when |
|---|---|---|
| `robot-code-stalled` | error | AdvantageKit's timestamp stops advancing while NT is still up |
| `input-ignored` | error | stick pushed, drive commands nothing — the binding layer dropped it |
| `frozen-robot` | error | drive commanded, not moving, **motors drawing nothing** |
| `robot-pinned` | warning | drive commanded, not moving, **motors straining** |
| `mechanism-not-following` | error | flywheel setpoint commanded, speed never gets near it |
| `loop-overrun-sustained` | warning | cycle time above threshold continuously |
| `brownout` | error | the robot says so |

As with the Python-side stall audit, these are **detectors, not watchdogs**.
Nothing intervenes or nudges the robot back into motion; a detector that fixes
what it detects destroys its own evidence.

### An oracle that has never fired is not known to work

`run_bridge_oracles.py` runs two phases and requires both to come out right: an
**exercise** phase of ordinary operation that must be clean, and a **provoke**
phase that pins the robot against the field wall while still commanding it
forward, which must produce `frozen-robot`. Without the second, "0 findings"
from a broken detector is indistinguishable from "0 findings" from a clean run,
and the first silent regression turns every subsequent night into a rubber stamp.

Oracle 01 has no runtime equivalent — deliberately crashing robot code to prove
a grep works is a poor trade — so `test/test_bridge_oracles.py` stands in for it,
which is why `bridge.oracles` imports without pyntcore.

The report also tracks whether each phase *ran*. "No findings" and "never ran"
are the same empty list, and the first version of this cheerfully reported a run
that died during gradle configuration as "exercise phase clean".

## The overnight harness

```
python apps/run_bridge_overnight.py --matches 200 --max-hours 8
python apps/run_bridge_overnight.py --matches 200 --driver strategy
```

### Two drivers, and neither subsumes the other

`--driver scripted` (the default) is step 3's seeded operator: a weighted random
walk of button presses that never asks where anything is. `--driver strategy` is
step 4's — sparky-sim's own `StrategyController` reading the live field and
playing a real cycle.

They find different things, and that is the point. A scripted operator reaches
**strange** states cheaply: it will mash two contradictory buttons, retract an
intake mid-collection, and spin in place against a wall, none of which a
strategy would ever do on purpose. A strategy reaches **plausible** states: it
will drive a full collect-and-score cycle, thread a HUB gap under time pressure,
and hold a shot while its HUB clock runs out, none of which a random walk will
ever reach by accident.

Scripted stays the default deliberately — it is what the false-positive work
below was tuned against, so changing what an unqualified run does would silently
re-baseline every number on this page.

The drive model is measured once, before the first strategy match's autonomous,
and reused: it is a property of the robot code rather than of a match, and
re-measuring would spend four seconds per match confirming a constant. Before
autonomous, because the calibration probes drive the robot and would otherwise
be fighting whatever auto is doing.

A seed means something different under each. Scripted, it replays the same
button sequence. Strategy, it replays the same *starting conditions* against a
field that no longer unfolds identically — which is the honest reading anyway,
since the physics does not repeat bit-for-bit across processes. The kept WPILOG
is the record either way.

Each match gets a fresh JVM (isolation beats throughput when the deliverable is
a crash), 15 s of autonomous, a transition into teleop *while something is
running*, then teleop to the buzzer — and the buzzer disables mid-action rather
than tidying up first, because that is what a real match end does.

Measured on this machine: **~11 s of boot per match, 15% overhead** at 60 s
matches and ~7% at 150 s. That puts a 150 s campaign at roughly 22 matches/hour,
close to the brief's ~190 per night.

Statuses are `pass`, `fail`, and **`error`** — the last meaning the harness
could not run the match at all. Folding that into either of the others is how a
night of nothing gets reported as a night of clean runs. Three consecutive
`error`s abort the campaign: something is wrong with the setup, not the robot,
and the rest of the night would just be a longer report of the same thing.

The campaign is **preflighted**: one deliberate wall pin, which must be detected
*and* classified `robot-pinned` off a live drive-current reading, before
committing eight hours to a detector that might not detect. The current reading
is checked explicitly because `robot-pinned` is also what the classifier returns
when the signal is *missing* — so the kind alone cannot distinguish a working
classifier from a blind one, and a blind one silently retires the `frozen-robot`
error path for the whole night. The preflight line records the amperage it saw:

```
preflight  : ok -- wall pin detected as robot-pinned at t=3.8s, 58 A while pinned
```

Results are flushed to `campaign.jsonl` as they happen, so the report survives
the machine going down at 3am.

Failing matches keep their console, findings, and WPILOG. Passing matches keep
one JSONL line and everything else is deleted — that is the ~8.5 GB question,
and an unreaped log is a slow leak that fills the disk somewhere around 3am.

A finding present in *every* match is flagged environmental rather than treated
as a regression, so the known `Vision camera 0 is disconnected` warning does not
drown the report each morning. That flag needs at least three matches to mean
anything: in a one-match campaign it is true of everything, including the single
failure the reader came to look at.

### False positives: find a signal, or fix the operator — don't move the threshold

The first real campaign failed a third of its matches with `frozen-robot`, all
of them genuine "commanded but not moving", and all of them the robot leaning on
the hub. (`SimulatedArena.getInstance()` gives the real 2026 field, so there is
substantial geometry mid-field that a perimeter-only wall margin never sees.)

Raising the detector's time threshold would have "fixed" it by hiding real
wedges. Two better answers, applied in order:

**1. Find a signal that separates the cases.** Pinned-on-geometry and
drive-not-working look identical in pose, and identical in wheel speed —
maple-sim stalls a blocked drivetrain rather than letting it slip, so the
encoders read zero either way. Drive-motor *current* separates them cleanly:

| regime | commanded | wheel | mean drive current |
|---|---|---|---|
| enabled, idle | 0.00 m/s | 0.0 rad/s | **0.0 A** |
| driving freely | 1.47 m/s | 21.3 rad/s | 9.4 A |
| pinned, gentle command | 0.50 m/s | 0.0 rad/s | **13.9 A** |
| pinned, hard command | 2.06 m/s | 0.0 rad/s | **58.2 A** |

So `frozen-robot` (error) now means *commanded, not moving, and the motors are
drawing nothing* — the command never reached them. `robot-pinned` (warning)
means the motors are working and something is holding the robot back. Still
reported, because a match spent wedged is a strategy problem; just not a fault.

The threshold is a **floor at 5 A, not a high-water mark**. Stall current scales
with applied voltage, so how hard a pinned robot pulls depends on how hard it
was told to go. A threshold picked from the 58 A case calls the 14 A case a
fault — which it did, on the first attempt.

**2. Make the operator behave like a person.** Independently of the above:

* it steers away from the field perimeter, and
* it **reverses out of anything it has run into**, escalating — straight back,
  then back with more rotation, then sliding sideways along the obstacle — and
  then **gives up** after four tries.

Giving up is the part that keeps the detector sharp. Being *more* patient
increases discrimination rather than reducing it: a robot held by geometry comes
free eventually, and one whose code has wedged never does, however many times it
is asked. Every back-off is shorter than the detector's window, so ordinary
contact never reaches it; what survives is the finding worth having.

**Measured effect of the two changes together: 1/3 → 1/8 → 0/8 matches failing.**
The remaining pins are reported as warnings and counted, which is what they are.

## Reading the world — and why REBUILT is not implemented here

```
python apps/run_bridge_world.py
```

The strategy layer needs to know what is on the field before it can decide
anything. The obvious way to give it that is to implement REBUILT in
`game_specific/`, the way REEFSCAPE and SALVAGE are — and it is the wrong way.
**maple-sim already implements REBUILT, including its scoring, and publishes
the result.** A second implementation on this side would not be a shortcut
skipped; it would be a model to keep in agreement with the one that actually
decides what happens, and a bridge whose two halves disagree about the score
reports failures that are really disagreements.

So the split is: everything that *changes* is read live, and only the static
geometry — which nobody publishes — is written down.

| | source |
|---|---|
| fuel positions | `/AdvantageKit/RealOutputs/FieldSimulation/Fuel` (`Pose3d[]`) |
| possession | `/AdvantageKit/RealOutputs/Intake/BallCount` |
| which HUB is live | `…/MapleSim/MatchData/Breakdown/{blue,Red} Alliance/… is active` |
| the 25 s HUB clock | `…/Breakdown/Time left in current phase` |
| the score | `…/{blue,Red} Alliance/TotalScore`, `TotalFuelInHub`, `WastedFuel` |
| field geometry | `bridge/arena.py`, transcribed from `Arena2026Rebuilt.java` |

Everything in the `MapleSim/` tree is published by the arena itself, not through
AdvantageKit, so it is there whatever the robot code does. Note the
capitalisation: `Red Alliance` but `blue Alliance`. That is maple-sim's, and
normalising it breaks the read.

`Intake/BallCount` is worth calling out because an earlier note in this project
said intake state was unobservable, and that was half wrong. The
`IntakeIOInputs` — deployed, position, intaking state — really are invisible,
because `IntakeSubsystem` holds a bare inputs object and never calls
`Logger.processInputs`. But `BallCount` is published separately by a
`recordOutput` inside the same `updateInputs`, which `periodic` runs every
cycle. Possession is readable, and it is the one signal the strategy layer
cannot work without.

### The transcription checks itself

`arena.py` is copied by hand out of maple-sim's source, which is exactly the
kind of thing that is right today and wrong after an upgrade. But maple-sim
publishes the HUB and OUTPOST poses it is really using, so the copy can be
weighed against the running arena on every connection rather than trusted:

```
blue HUB centre: transcribed (4.5974, 4.0345) vs published (4.5974, 4.0345)  0.0 mm  [ok]
```

Tolerance is a millimetre — far tighter than the navigator would notice. The
question is not "is this good enough to drive with", it is "did I transcribe
the same field", and a loose tolerance passes a constant copied from the wrong
season. What it cannot check is the obstacle *sizes*, which nothing publishes;
those stay trusted, and the tests in `test/test_bridge_world.py` guard their
shape instead.

### What the geometry says about the wedging

The transcription explains, outright, a false-positive rate that took a whole
campaign to characterise empirically. `SimulatedArena.getInstance()` builds the
arena with ramp colliders, which makes each HUB a **47 × 217 inch wall**. Two of
them, on a 317-inch-wide field, leave about **50 inches at each end** — and
those two gaps are the only ways past. A 30-inch robot needs 42.4 inches to
turn. Passable and tight is exactly the combination that wedges.

`validate_field` agrees the gaps are passable and flags nothing error-level, so
the field is sound; the tightness is a fact about REBUILT, not a bug.

Two smaller findings, both reproduced deliberately rather than corrected:

* maple-sim 0.4.0-beta places **three** trench walls, not four —
  `RebuiltFieldObstaclesMap` repeats the (−x, −y) corner verbatim in its fourth
  call, so the (+x, +y) corner has no wall. Steering around an obstacle the
  physics does not contain is wrong in the direction hardest to notice. It
  matters little in practice: the trench walls end within a fiftieth of an inch
  of the HUB ramps, so with ramps on they add nothing.
* The tower poles leave 41 inches to the wall, under a robot diagonal. They are
  wall furniture, and the validator says so.

### Proving the reader reads

Same discipline as the oracles app, one layer up. A reader that returns a
plausible constant forever is indistinguishable from a working one, so the two
live quantities the strategy layer cannot function without are not checked by
reading them — they are checked by making the field change and requiring the
reader to notice:

* **POSSESSION** — the robot drives through the fuel grid and the ball count
  must move. Measured: `held 8 → 32`, `loose 192 → 168`. The two agree, which
  is the check that both topics are being read and not just one.
* **CLOCK** — the active HUB must flip within the 25-second phase. Measured:
  `blue → red after 14.5 s`.

Getting the first of those to fire took two wrong attempts, and the reason is
worth writing down. `IntakeSimulation.OverTheBumperIntake(..., FRONT, ...)`
puts the collecting fixture 0.37–0.68 m ahead of the robot centre **along its
heading**, while `joystickDrive` is field-relative — so the robot can translate
in any direction while still facing +x, and collects only what crosses that
forward band. Strafing north through the grid slides fuel down the robot's
flank; reversing west drags the intake over ground the bumper already cleared.
Both look exactly like "the intake is broken". The instrumented probe showed
the nearest fuel pinned at 0.53 m from the intake for eight seconds, which is
what said the problem was approach geometry rather than the intake or the
reader. **Standing in a pile is not intaking from it.**

## Letting the strategy layer drive

```
python apps/run_bridge_strategy.py --seconds 45
python apps/run_bridge_strategy.py --seconds 60 --gui     # watch it
```

A real `StrategyController`, running the same `Collect` and `Score` tactics
the strategy sim uses, reading `MapleMatchView` and pressing the buttons
itself. **Nothing in `common_sim/control` was changed to make this work** — the
tactics do not know they are driving a JVM.

`MapleMatchView` duck-types the `match` contract rather than subclassing
`Match`, for the same reason REBUILT is not implemented here: a real `Match`
brings its own pymunk world, its own scoring and its own piece bookkeeping,
all of which would drift from the simulation that actually decides what
happens. `MapleRobot` *does* subclass `Robot`, for the opposite reason — its
geometry helpers are pure functions of a pose and a set of characteristics,
and a reimplementation is a copy that disagrees at the edges. Its pymunk body
is never stepped; it is somewhere to write the pose that arrives over the wire.

### The three writes

| tactic calls | bridge presses |
|---|---|
| `drive_field_relative` | left stick + right stick X, through the inverse joystick model |
| `set_intake_active` | manip Y (`DeployIntake`) / manip B (`RetractIntake`), drive right trigger |
| `set_deposit_active` | drive left bumper (flywheel) + manip right trigger (feeder) |

Manip Y and B are bound `onTrue`, so they need *edges*, not held buttons. The
mapping is "hold Y while intaking, hold B while not" — one rising edge per
transition, none while held, and no sleeping inside a control loop. Nothing is
pressed before the first call, because a robot that has never been told either
way should not be issuing retract commands.

### The drive model is an inverse, not a scale factor

`DriveCommands.joystickDrive` deadbands the stick, **squares** it, and **halves
the whole drivetrain** while the feeder is running (`spindexer::isFeederOn` is
its speed multiplier). Miss any of the three and the robot drives — just not
where it was told, which is the hardest kind of wrong to see in a report.

Translation is inverted as a magnitude and a direction rather than per-axis,
because the squaring applies to the magnitude: undoing it component-wise
rotates the commanded direction. A request beyond the drivetrain is clipped to
full stick *in the requested direction* and flagged `saturated` — a robot that
cannot go as fast as asked still goes the right way, whereas one whose heading
quietly bends looks like a navigation bug.

### Calibrate, don't transcribe

The maxima are **measured off the running robot**, not read out of
`DriveConstants` — which sets them inside an `if (isReefscape)` branch resolved
at class-init, where which values win is not obvious from reading the file.
That decision paid immediately:

```
measured    : 4.45 m/s, 11.09 rad/s (DriveConstants nominal 5.3 m/s)
```

Transcribing the 5.3 would have made every commanded velocity 19% low — and
nothing would have failed. The robot would just have arrived somewhere the
navigator did not plan for.

The probes then check the model at *intermediate* stick, not only at full,
because at magnitude 1 the deadband rescale and the squaring are both
identities: a model fitted at full stick and checked at full stick confirms a
scale factor and misses the squaring entirely.

Speed and direction get separate error budgets. `joystickDrive` builds
field-relative speeds and immediately converts them to robot-relative using the
robot's heading; the check converts back using a pose read a moment later. Any
heading change between those two instants shows up as a direction error scaled
by speed — a couple of degrees at full stick, nothing at half. That is sampling
skew, not a wrong model, and one lumped tolerance would have to be either loose
enough to hide a real scaling error or tight enough to fail on it every run.

### A robot outside its own zone does not miss — it passes

The single most consequential rule after the HUB collider, and the one that
cost the most to learn. `RobotContainer.isInAllianceArea`:

```java
blue: pose.x < 4.625594      red: pose.x > 11.915394
```

`Turret.setTarget` branches on exactly that. Inside the zone it aims at the
HUB; outside it, `isScoring = false` and the turret retargets a **corner of its
own zone** and throws the fuel back. The shot happens either way, the ball
leaves the robot either way, the ball lands on the field either way — and
nothing distinguishes the two from outside.

Which is why the first working runs shot 65 pieces of fuel from midfield with
auto-aim confirmed on and scored nothing, and why that read as "the robot's aim
is broken". It was doing precisely what it was told. **The scoring regions were
in the wrong place.**

The trap is that the goal mouths face *midfield* — `RebuiltHub` puts the shoot
poses at `+GoalRadius` in x from the blue hub centre — so the obvious place to
put a scoring region is the one place a robot cannot score from. The regions
now sit **behind** each HUB, inside the zone, and the fuel goes over the
structure: the goal is 1.57 m up and the turret has a pitch. `build_scoring_regions`
asserts every vertex is inside the zone, so this cannot regress quietly.

Getting them on the right *side* was only half of it; see
[the region was also the wrong size](#the-region-was-also-the-wrong-size-and-that-was-the-expensive-half)
for the half that took another two campaigns to notice.

Two things fall out of it worth keeping:

* The zone boundaries agree with the trench-wall line to within a millimetre,
  and the two constants come from unrelated arithmetic in two different repos.
  That is a real cross-check that both transcriptions read the same field.
* `Turret.isScoring` is not logged, but `isInAllianceArea` is a pure function
  of the pose — so `arena.in_alliance_zone` evaluates the same rule exactly
  rather than observing its consequences. The run report carries a `Z` flag
  per line and splits the shot count by it, so "passed" and "missed" are never
  reported as the same number again.

### The 25-second clock, as a strategy input

`region_blocked` returns True for a HUB that is not currently accepting. That
routes maple-sim's alternating-HUB rule into the place the strategy layer
already looks: `world_view.scoring_slots_for_type` drops a blocked slot, so a
robot holding fuel with no live HUB stops presenting a scoring option and falls
through to collecting or repositioning instead of shooting fuel into a HUB that
returns it as `WastedFuel`.

`region_blocked` and not `region_full`, deliberately — "full" says the same
thing and means something permanent.

`deposit_region_for` is also overridden rather than inherited, twice over.
`Match`'s version asks whether a bumper *side* engages the region, which is
right for a robot reaching into a structure and wrong for one with a turret:
this robot shoots from wherever it is standing, so position is the whole test.
And the position test is `arena.can_score_from` — the alliance-zone rule
itself — rather than a point-in-polygon on the region, for the reasons in the
next section.

### Three ways an object identity went wrong

Every one of these produced a robot that looked like it was working, and none
of them looked like an adapter bug from the outside. They are the same mistake
in three places: treating a *position in a list* as an identity.

**1. The fuel array's order is not stable.** `SimulatedArena.gamePieces` is a
`HashSet`, and `getGamePiecesPosesByType` walks it directly, so the order can
change whenever a piece is added or removed. Handing out pooled `GamePiece`
objects by array index gives `Collect` a target whose coordinates jump to some
other piece somewhere else on the field between ticks. The robot chases a ghost
at full stick and never arrives. `PieceTracker` matches by position through a
coarse spatial hash instead.

**2. Recycling a collected piece reintroduces the same bug.** The obvious
optimisation — keep freed `GamePiece` objects in a pool — means a tactic
holding the piece it just collected finds that object silently reissued for a
*new* piece elsewhere. Retired instead, and marked `scored` so a stale
reference reads as "not an option any more" rather than as a live target.
Retirement costs one allocation per piece that leaves the field, a few hundred
over a match rather than a few hundred per tick.

**3. A hopper is a queue, and the order is load-bearing.**
`behavior.RunManipulator` names `held_pieces[0]` as the piece it is depositing
and returns SUCCESS when that object is no longer held. Truncating the held
list from the tail — the obvious implementation — means the named piece never
leaves, so `Score` runs to its timeout every time instead of finishing.
Collected fuel goes on the back; shot fuel leaves from the front.

### Edge-triggered buttons over a 50 Hz link need a controller, not a translator

Manip Y and B are bound `onTrue`, so the mapping depends on the robot *seeing*
a transition. Two things break that, and both did:

* **The command chatters.** `behavior.RunIntake` turns the intake off the
  instant a piece is captured and on again on the next tick, so a robot
  collecting a stream of fuel toggles it once per ball. The arm takes about a
  tenth of a second each way, so obeying every toggle leaves it halfway
  whenever it matters — and a driver does not stow the intake between pieces
  anyway. A 0.5 s debounce on *stopping* an intake that is already running.
* **Edges get swallowed.** The operator link transmits at 50 Hz; a press and
  release inside one 20 ms window never reaches the wire. The result is a
  command that reads as active while the mechanism sits stowed.

So the adapter drives the intake from the arm's *observed* angle rather than
from the last thing it sent, re-issuing an edge when the two disagree for more
than 0.6 s. `intake_reasserts` counts those and the run reports it — a rising
count means the transport is losing edges faster than expected, which is worth
knowing rather than silently compensating for.

The feeder needed the same debounce for the same reason — `Score` runs one
`RunManipulator` per piece and cycles the deposit command between them, and a
burst that emptied twenty balls then sat with the deposit commanded and nothing
leaving for fourteen seconds.

It also needed the same reconciliation, and for a while it could not have one:
nothing published the feeder's state, so a lost edge there could be avoided but
not detected. `SpindexerSubsystem.periodic` now logs `Spindexer/FeederOn`, and
`feeder_reasserts` counts alongside `intake_reasserts`.

**The quieter half of that is the drivetrain, not the feeder.**
`joystickDrive` takes `spindexer::isFeederOn` as its speed multiplier, so the
bridge has to know whether the feeder is running before it can invert the drive
model at all — and it was inferring that from the last button it had sent. A
lost edge did not just stall the shot; it left every commanded velocity out by
a factor of two, in the direction that reads as a navigation bug rather than a
transport one. That inference is now an observation.

A run that reports `feeder 0` is only meaningful because the unit tests prove
the counter fires when the observation reads off — which also makes the zero
the evidence that the topic is live, since a missing topic reads as `false` and
would re-issue continuously.

Auto-aim is the same shape. Manip Start is a *toggle*
(`turret.toggleAutoAimCommand`), so pressing it blind is as likely to turn
auto-aim off as on — and without auto-aim the turret points wherever it was
left and every shot lands on the floor. That is exactly what the first working
runs did: fuel left the robot, the loose count rose by the same amount, and the
score stayed at zero. The robot publishes `autoAimEnabled`, so the press is
conditioned on it.

### The shot was 14% too fast, and the fix was one number

The first runs to reach a legal shooting position scored **1 in 29** and then
**0 in 55**, from inside the zone with auto-aim confirmed on. That reads as a
mis-aimed turret or a badly chosen `SHOOTING_STANDOFF_IN`, and it was neither.

`TurretIOSim` converts the flywheel's commanded RPM into a launch speed:

```java
calculatedVelocity = (flywheel.getFlywheelVelocity() - randomOffsetVelocity(true))
    * TurretConstants.turretRPMToMetersPerSecond
    * 0.58;                        // <- this
```

The comment two lines below it reads "assumed to be 16 meters/second at 6000
RPM". That statement and that literal are not the same number: 6000 RPM is
31.4 m/s of wheel surface speed, so 16 m/s is a factor of **0.509**. The code
had drifted to 0.58 and taken the shot with it.

Fourteen percent high on the speed is not a near miss. Worked against the shot
table in `Scoring.java` and maple-sim's `GRAVITY = 11`, it puts the fuel
**0.61 m over the goal at 1.6 m and 1.25 m over it at 5.5 m**, against a goal
radius of 0.597 m — a miss at every range in the table, marginal at the near
end and hopeless past about 4 m. That is the 1-in-29.

The number is recoverable without measuring anything, because the shot table is
itself a claim about the physics: for each distance, solve for the speed that
puts the trajectory through the HUB centre and divide by the wheel speed the
table asks for at that distance.

| distance | hood | RPM | required factor |
|---|---|---|---|
| 1.602 m | 62° | 2262 | 0.478 |
| 2.602 m | 57° | 2400 | 0.516 |
| 3.602 m | 53° | 2650 | 0.521 |
| 4.602 m | 51° | 2975 | 0.510 |
| 4.830 m | 50° | 3102 | 0.498 |
| 5.540 m | 48° | 3270 | 0.500 |

**The spread is ±4% across the whole table, and the code's own comment lands in
the middle of it.** That agreement is the actual result: it says the table and
the sim's physics were always consistent and one scalar was wrong, rather than
the table being untrustworthy — which would have been a much longer job. The
value it works out to is also what a single backed wheel does physically, since
the contact point matches the wheel's surface speed and the ball's centre
leaves at roughly half of it.

So the constant is now *derived* from the documented intent rather than written
as a literal, which is the only reason to think it will not drift again:

```java
private static final double shotSpeedAt6000RpmMPS = 16.0;
private static final double flywheelSurfaceSpeedToShotSpeed =
    shotSpeedAt6000RpmMPS / (6000.0 * TurretConstants.turretRPMToMetersPerSecond);
```

Measured over two live runs afterwards: **27 of 28**, then **53 of 59**.

And the standoff was never the suspect it looked like. With the factor right,
the table's whole declared range scores from anywhere a robot can stand, which
is why the standoff constant does not exist any more — see the next section.

**One caveat worth keeping.** The sim shooter is now *very* forgiving: with only
±2° of yaw, ±3° of pitch and ±50 RPM of noise, and no air drag, it scores from
anywhere in 0.8–7.0 m. A real shooter does not. That is fine for fuzzing
navigation and possession, and it is a bad basis for any conclusion about how
good the shot itself is — including, when it arrives, oracle 04's differential
scoring.

### The region was also the wrong size, and that was the expensive half

Putting the scoring regions behind the HUB instead of at its mouth fixed *where*
they were. It left them the wrong **shape**, and that took two more campaigns to
see because nothing about it looks like a bug: the robot drives somewhere legal
and scores.

The original region was an 82 × 47 inch pocket, and both of its dimensions were
guesses dressed as measurements:

* **47 inches tall** because that is the goal mouth's width. The mouth's width
  has nothing to do with where a robot stands — *the turret rotates*. Being off
  the HUB's axis in y costs exactly nothing.
* **82 inches deep** because of `SHOOTING_STANDOFF_IN = 100`, a guess at
  flywheel range. Range was never the binding constraint, as the shot
  calibration above eventually showed.

So the region was about a ninth of the floor a robot can actually score from,
and `deposit_region_for` — a point-in-polygon test on it — inherited the error.

The cost is not that the robot scores from the wrong place. It is that `Score`
stops the moment `deposit_region_for` says the pose is legal, and drives at the
region until then. A region a ninth of the legal area is a robot that drives
**past** perfectly good scoring positions to reach a nominated one. On this
field that drive goes through the 50-inch pinch between the HUB ramp and the
field wall, which is where the campaign's recurring `robot-pinned` finding lives:
every long match reported it, at (3.720, 7.518) m, which is inside the blue
alliance zone and outside the old blue GOAL polygon. That pose is the one the
regression test uses, so the gap it stands for cannot silently reopen.

The fix separates two jobs that had been sharing one polygon.

**The rule is `arena.can_score_from`, and it has no polygon in it.** One term:
are you in your own alliance zone. That is what `Turret.setTarget` branches on
and it is the whole of what decides score-versus-pass.

**The polygon is a navigation aid** — where `Score` and `Stage` aim, and what
`region_occupants` shares between robots. It is now the alliance zone inset by
half a robot from the walls and the HUB, which makes it deliberately *smaller*
than the rule. That is safe now and was not before: a robot in the 18-inch band
beside the HUB is outside the polygon and still, correctly, ready to score.

The range term the rule leaves out is checked rather than argued.
`build_scoring_regions` asserts that every corner of an alliance zone is inside
the shot table's declared 6.7 m reach — the worst is 6.12 m — so the day the
geometry changes, the assertion says so instead of a comment going quietly
stale. The table's *lower* bound, 1.5 m, can be crossed by a robot with its
bumpers on the HUB's back face; nothing in the robot code enforces it
(`ShotCalculation/isValid` is computed, logged, and never read), and solving the
maple-sim ballistics puts the ≥95% hit band at 0.8–7.0 m. The declared minimum
is conservative against what the simulated shot actually does.

**This is also what unblocks `Pass`.** Its guard is "I could not score from
here" — so while readiness was point-in-polygon, a `Pass` rule would have fired
across most of the alliance zone, where the turret is aimed at the HUB and the
throw is not a pass but a deliberately bad shot. With the rule asked directly,
the guard means what it says: `Pass` refuses inside the zone and lets go outside
it, which is exactly where `Turret.setTarget` retargets a corner and lobs the
fuel back. Both directions are pinned in `test/test_bridge_match_view.py`.

Whether the demo strategy *should* pass is a separate question, and a
measurement rather than a blocker: `cycle_fuel` stays minimal on purpose,
because what the campaign is testing is the adapter and not the strategy.

**What it did to the wedge: moved it, did not remove it.** Two matches
re-run on their original seeds, before and after:

| seed | before | after |
|---|---|---|
| 4375 | t=44.5s at (3.720, 7.518) | t=58.4s at (3.789, 0.532) |
| 4376 | t=50.7s at (3.761, 7.519) | t=111.7s at (3.821, 0.529) |

Same x to within 10 cm, and y mirrored across the field: the north gap
became the south one. That is the identical pinch, and it says the
remaining traversals are on the **collect** leg — going back out to
midfield for fuel, which no scoring rule touches. The first pin does
arrive later in both, which is consistent with fewer traversals but is
two matches and should not be read as more than a hint.

### The cycle this creates, and where it breaks

Putting the scoring regions where they belong changes the shape of the whole
match. The fuel is at midfield; the only place to score it is behind your own
HUB; and the HUB is a 217-inch wall with two ~50-inch gaps. **Every scoring
cycle threads one of those gaps twice.**

That is the navigation problem this tool exists to fuzz, and the first run to
score also found it — 25 of 75 seconds spent asking to move and not moving,
wedged in the upper gap at (3.76, 7.52). The run still passed every check, so
the report now carries a longest-stall line and raises it as a finding: a PASS
that hides a third of the match spent stuck is the same mistake as a report
path that only runs on bad news.

### What 75 seconds of it already showed

The demo strategy is two rules — collect fuel, shoot it when there is enough
and a HUB will take it — and the first clean run surfaced a real strategic gap
in REBUILT that has nothing to do with the bridge:

```
17.5  Collect   ( 7.42, 3.89)  ( 7.80, 4.65)   27  125  red    38  I- arm  0
19.5  Idle      ( 7.66, 4.75)  -               40  112  red     0  -- arm 90
   ...  twenty seconds of nothing ...
41.9  Score     ( 7.65, 4.77)  blue GOAL       40  112  blue   98  -- arm 90
```

The intake fills to its 40-ball capacity in a single sweep of the centre grid,
and then there is nothing to do: it cannot collect more and its HUB is not
accepting. A real strategy needs a third behaviour, and now that the alliance-zone rule is
modelled there are two obvious candidates: **drive to the goal and wait there**,
or **pass the fuel deliberately** toward your own corner, which is what the
turret does anyway when you are out of position. sparky-sim has no vocabulary
for either — no positioning tactic, and no notion of a shot that is not a
scoring attempt. That is the kind of thing this was built to find, and it found
it in the first minute rather than in a match.

### The intake reach is load-bearing

`INTAKE_REACH_IN = 11.6` is not cosmetic. `Collect._piece_aim` parks the robot
so the target sits *mid-wedge*, at `half_length + intake_range/2` from the
piece — so getting the reach wrong parks the robot somewhere the intake cannot
reach, and the symptom is a robot that drives beautifully to a piece and never
picks it up. It comes from maple-sim's own
`IntakeSimulation.getIntakeRectangle`: with 30-inch bumpers and a 12-inch
extension the collecting fixture spans 14.6–26.6 inches from the chassis
centre, i.e. 11.6 past the bumper. That the strategy layer then handles the
approach geometry correctly, with no changes, is the payoff for adapting rather
than reimplementing.

### The two tactics the bridge asked for

`Stage` and `Pass`, both in `common_sim/control/tactics.py` and both
game-agnostic. They exist because a run found a hole, not because a tactic
list looked short: a robot that is **both** full and unable to score matches
no rule at all and drops to `Idle`, which does nothing wherever the last
tactic happened to stop. Measured at eighteen to twenty seconds of a
sixty-second run, parked at midfield on the wrong side of a wall with two
gaps in it.

They are the two answers to that one situation. Wait nearer to where the way
will open, or throw the pieces that way and go back to work. Which is right
is a question about the game, which is why both are tactics and neither is a
rule.

**`Stage` parks where `Score` will want to be**, through the same
`scoring_approach_pose` Score uses — staging somewhere Score then wants to
leave would spend the wait making the robot late. It ends `SUCCESS` the
moment a slot opens, so the wait is a state the strategy leaves rather than
a rule it has to be outranked out of. Ranked *below* `collect_fuel` on
purpose: fuel on the floor is always worth collecting, and a staging rule
that outranked collecting would answer a dead HUB by parking a half-full
robot.

With it wired in, `Idle` disappears from the run:

```
23.8  Stage   (7.16, 5.78)   40 held, red HUB active -- full, nowhere to score
25.8  Stage   (3.26, 6.96)   crossing the gap
27.8  Stage   (2.60, 4.39)   arrived
 ...  seven seconds of waiting, in the right place ...
42.0  Score   (2.61, 4.42)   the HUB flips and it deposits from where it stands
```

Total distance travelled is unchanged (8.72 m against 8.55 m). The driving
is not extra; it has moved out of the live HUB window and into the dead one.
Whether that is worth points is a paired-campaign question, not a
one-run-each-way question — the numbers swing by more than the effect across
single runs, and `--driver strategy` is what settles it.

**`Pass` does not drive.** A pass that navigated to its destination would be
a slow `Score`, and the reason to pass at all is that going there is not
worth it. It turns on the spot until the throwing side points at the
destination and lets go. It fails rather than throwing from a pose it could
score from, because that is a strictly worse `Score` and quietly scoring
instead would hide the mistake.

Two things fell out of building it. The framework already had the primitive —
`Robot.update_manipulator` with nothing to score against releases the piece
onto the field — so no new write was needed, exactly as predicted. And
`Manipulator` is **edge-triggered**: one press is one piece, and the next
release is withheld until the command drops and is re-raised. A tactic that
held the deposit down would throw once and stand there holding the rest of
the hopper. `Pass`'s cooldown re-arms it *and* prices it at
`deposit_duration`, since a release with nothing to score against otherwise
completes on the tick it is commanded.

**`Pass` is not yet wired into REBUILT, deliberately.** Its guard asks
`match.deposit_region_for`, which here is point-in-polygon on the GOAL
pocket — but the rule that actually decides whether a REBUILT shot scores or
passes is `isInAllianceArea`, and the zone is much larger than the pocket.
So a `Pass` rule would fire from inside the zone, where the turret aims at
the HUB and the throw is really a bad shot. Fixing that means making the
bridge's deposit test the alliance-zone rule rather than the pocket, which
is a change to what `Score` does too, and worth doing on its own.

### One strategy, defined once

Adding the staging rule surfaced a defect worth recording: `cycle_fuel`
existed in both `bridge/harness.py` and `apps/run_bridge_strategy.py`. The
rule went into the harness copy, the app kept playing the old two-rule
strategy, and the run showed eighteen unbroken seconds of `Idle` — so the
new tactic looked broken when what was broken was having two definitions of
one strategy. It now lives in `bridge.harness` and the app imports it. The
overnight campaign and the single-run app have to play the same game, or
neither tells you anything about the other.

## Two things found on the way

**Odometry is not an independent channel here.** `SimContainer.simulationPeriodic`
feeds maple-sim's ground-truth pose to `addVisionMeasurement` with standard
deviations of `(0, 0, 0)` — infinite confidence — so the pose estimator is pinned
to truth and re-snaps every cycle. `Odometry/Robot` and
`FieldSimulation/RobotPosition` are the same number, and anything that writes
odometry (`drive.setPose`, bound to the Y button) is undone within one loop. An
odometry-vs-truth divergence oracle would read zero forever until those std devs
are made realistic.

**`--no-daemon` is load-bearing.** With the gradle daemon, `gradlew` is a thin
client and the robot JVM is a child of the *daemon*; killing the client orphans a
robot still holding port 3300, and the next run fails to bind with no visible
cause. Without it the robot is a descendant of the process we started, so a
tree-kill ends it.

**Log volume.** `-Pbridge` turns on AdvantageKit's `WPILOGWriter` (it was
commented out in SIM), because a detected failure with nothing to replay is half
a deliverable. Logs land in the robot repo's gitignored `logs/bridge/` at roughly
**45 MB per 150 s match** — about 8.5 GB across an overnight run. The harness
will have to keep the failures and delete the rest.

## Reproducing a reported failure

```
python apps/run_bridge_overnight.py --matches 1 --first-seed 8106          # re-run it
python apps/run_bridge_overnight.py --matches 1 --first-seed 8106 --gui    # and watch
```

A seed reproduces the **script**, not the run: the same moves in the same order,
but the back-off is closed-loop and the physics does not repeat bit-for-bit
across processes, so the robot will not land on the same coordinates. The kept
WPILOG is the authoritative record, and it replays through the robot code
deterministically in AdvantageScope. That is the same division the feasibility
brief settled on — reproducibility comes from the recorded input trace, not from
deterministic stepping.

## Next

Step 4 is done: the strategy layer reads the live field and drives the robot.
What remains widens the variety of situations reached.

* **Intent→button mapping**, broadened past the canned tactic. `Stage` and
  `Pass` needed none of it, which was the prediction: a positioning tactic only
  drives, and a pass presses exactly what a score presses.
* **A `Pass` rule, if it earns one.** `Pass` can now fire honestly — the
  alliance-zone rule above is what unblocked it — so what is left is not a
  blocker but a measurement: does throwing fuel back toward your own end beat
  carrying it, on a field where every cycle threads a 50-inch gap? A paired
  campaign answers that; `cycle_fuel` stays minimal until one does.
* **AI opponents** — the other five robots driven by sparky-sim.
* **Oracles 03–05** — invariants, differential scoring, JaCoCo coverage.
