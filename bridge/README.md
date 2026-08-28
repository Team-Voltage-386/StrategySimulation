# The maple-sim bridge

Drives the team's **real robot code**, running under maple-sim in WPILib's
simulator, from Python — so a competitive scenario can be generated on a night
when the robot is on the mechanical team's cart and the field doesn't exist yet.

Background and rationale: [The maple-sim Bridge](https://claude.ai/code/artifact/648dfe02-ea7d-4b2f-b0ee-44094eb28407).

**Status: steps 1–4 and 6 of 7 done, and oracle 03 with them.** The loop closes,
the oracles fire, the harness runs unattended and leaves a morning report, the
live REBUILT world is readable, sparky-sim's strategy layer drives the real
robot code from it, and the other five robots on the field are driven the same
way. What remains is oracle breadth: differential scoring and coverage.

## Run it

```
pip install -r bridge/requirements.txt
python apps/run_bridge_smoke.py      # step 1: does the loop close?
python apps/run_bridge_oracles.py    # step 2: can it tell a failure from a run?
python apps/run_bridge_overnight.py --matches 40    # step 3: the product
python apps/run_bridge_world.py      # step 4a: can it see the field?
python apps/run_bridge_strategy.py   # step 4b: can it play?
python apps/run_bridge_opponents.py  # step 6: can it play against somebody?
```

Each takes ~90 s: gradle compiles, the JVM boots, the checks run, the sim is killed.

```
python apps/run_bridge_smoke.py --dump-topics    # everything the robot publishes
python apps/run_bridge_smoke.py --echo           # stream the robot console
python apps/run_bridge_smoke.py --attach         # against a sim you started yourself
python apps/run_bridge_oracles.py --no-provoke   # exercise only (leaves oracle 02 unverified)
```

```
python apps/run_bridge_overnight.py --driver strategy --opponents 3 --partners 2
```

is the contested campaign: six robots, all of them deciding, for as many
matches as you leave it running. Without those flags it runs the solo field it
always ran, unchanged.

`pytest test/test_bridge_oracles.py test/test_bridge_invariants.py` covers
oracles 01 and 03's rules with no JVM involved, so they run in CI.

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
| `opponents.py` | both | the other five robots: roster, wire, and their brains |

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

**03 — invariants** (`InvariantMonitor`). Samples NT at 20 Hz, like oracle 02,
and asks the opposite question. 02 is liveness: something that should happen
and has not. 03 is safety: something that should never happen and has.

| kind | severity | fires when |
|---|---|---|
| `not-a-number` | error | any published pose, speed, setpoint or voltage is NaN or infinite |
| `off-the-field` | error | a robot's centre is outside the field border |
| `teleport` | error | a robot moved further between two samples than any drivetrain could |
| `driven-while-disabled` | error | the motors draw current with the DriverStation disabled |
| `command-out-of-range` | warning | the chassis is commanded past its own measured maximum |
| `possession-impossible` | error | the robot believes it holds a negative number of pieces, or more than fit |

Keeping these apart from 02 is not tidiness. Almost every design decision
differs: 02 debounces by *duration*, because one sample of "not moving" is
jitter — but a NaN is never jitter and is frequently gone by the next read, so
half of these fire on a single sample. 02 is only meaningful while enabled;
`driven-while-disabled` is meaningful only while it is not. 02's evidence is a
stretch of time, an invariant's is one instant and the value that broke it.

**Every robot on the field, not just ours.** `off-the-field` and `teleport`
are held per robot against `FieldSimulation/BridgeRobots` as well as our own
truth pose, which is most of what step 6 bought the oracles: the interesting
version of both is one of the extras being squeezed out through a wall by a
contact the physics resolved badly, and a solo run cannot produce it at all.

Which raises the same question one level down — an invariant applied to an
empty list is not an invariant that held. A contested campaign whose monitor
only ever saw one robot checked one robot and reported five robots' worth of
silence as a clean field. So the monitor records the most robots it ever judged
at once, and the harness compares that against the number of extras it
deployed; a shortfall goes into the same **INVARIANTS NOT CHECKED** block as
any other detector that stood down.

**Nothing here knows what a FUEL is.** No HUB, no alliance, no scoring. These
are properties of any robot on any field, which is the point of writing them
against the game this bridge was only ever proved against — `bridge.oracles`
does not even import `bridge.arena`, and the field dimensions are a threshold
with an FRC-standard default rather than a constant read from the game.

**What it does not check, and why.** `possession-impossible` needs a hopper
capacity and `command-out-of-range` needs measured drive limits, and neither is
guessable. Without them those invariants stand down — and say so, through
`InvariantMonitor.inactive`, which the campaign report prints under **INVARIANTS
NOT CHECKED** and the app prints as `NOT CHECKED`. A detector standing down for
a good reason is fine; one standing down silently is how a campaign spends eight
hours checking less than the report claims. The one deliberate omission is
odometry-versus-truth divergence, which would read zero forever here for the
reason in *Two things found on the way*.

### An oracle that has never fired is not known to work

`run_bridge_oracles.py` runs three phases and requires all of them to come out
right: an **exercise** phase of ordinary operation that must be clean, a
**provoke** phase that pins the robot against the field wall while still
commanding it forward, which must produce `frozen-robot`, and an **inject**
phase that pushes a deliberate violation of each of oracle 03's six invariants
through detectors built with the thresholds this run is actually using, all six
of which must fire. Without the last two, "0 findings" from a broken detector is
indistinguishable from "0 findings" from a clean run, and the first silent
regression turns every subsequent night into a rubber stamp.

**PROVOKE and INJECT are not equally strong**, and it is worth saying which is
which rather than counting them together. A wall pin is a real robot genuinely
wedged and genuinely commanded forward — the exact signature the detector is
for, in a situation a real match produces constantly. INJECT is synthetic
snapshots pushed through the detectors by hand, because making real robot code
publish a NaN or drive its motors while disabled would mean breaking it on
purpose. That is oracle 01's trade, and it comes out the same way: deliberately
crashing robot code to prove a grep works is a poor trade, so
`test/test_bridge_oracles.py` and `test/test_bridge_invariants.py` stand in for
it — which is why `bridge.oracles` imports without pyntcore, and why
`InvariantMonitor`'s *constructor* has no NetworkTables requirement even though
its `sample()` does.

What INJECT buys over those tests, which prove the same six detectors in CI, is
narrow and real: it runs the **shipped** thresholds and the drive limits
measured minutes earlier on this machine. A threshold edited to something
unreachable passes the unit tests, which supply their own, and fails here.

The report also tracks whether each phase *ran*, and how many samples each
monitor took. "No findings", "never ran", and "ran but never sampled" are the
same empty list, and the first version of this cheerfully reported a run that
died during gradle configuration as "exercise phase clean".

One ordering detail that is a bug waiting to happen: the drive-limit
measurement runs **after** the live phases, not before. Calibration drives —
four full-stick probes whose reversal does not perfectly undo them — and
PROVOKE depends on the robot still being near the wall it was placed by. That
cost a whole run during step 6 in a different guise, and the rule it left is
that a check which rearranges the field has changed the experiment it was meant
to validate. The price is that `command-out-of-range` is inactive during the
live phases, which the run prints rather than papers over.

### What oracle 03 actually did

First live run, and every run since:

```
-- ORACLE 03 -- invariants, live phases ----------------------
   no findings
-- phase INJECT (expect all six invariants to fire) ----------
   ok    not-a-number             fired
   ok    off-the-field            fired
   ok    teleport                 fired
   ok    driven-while-disabled    fired
   ok    command-out-of-range     fired
   ok    possession-impossible    fired
```

The interesting line is the quiet one. That same run produced oracle 02's
`robot-pinned` — a robot wedged against the border at y=0.378 m, drawing 58 A,
commanded 2.06 m/s and moving 0.000 m — and oracle 03 said nothing about it.
A robot pressed flat against a wall is the closest thing ordinary operation
produces to "off the field", and an invariant that could not tell the two apart
would fail every match that touched anything. Getting that boundary right is
most of the work in a safety oracle; the detection is the easy half.

Contested, one match, three opponents and two partners: **zero** oracle-03
findings with all six robots' poses under `off-the-field` and `teleport`, and
`robot-pinned` from oracle 02 for the defender doing its job. The report grew no
**INVARIANTS NOT CHECKED** block, which is how the run says the monitor really
saw six robots rather than one.

No finding from a real robot yet, on either field. That is the expected result
and not a disappointment: five of these six are things well-written robot code
simply does not do, and the value of writing them now is that the campaign that
finds one is a campaign that already knew what to call it.

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
error path for the whole night. It then injects a violation of each of oracle
03's invariants, all of which must fire. The preflight line records what it
saw:

```
preflight  : ok -- wall pin detected as robot-pinned at t=4.3s, 58 A while pinned;
             oracle 03 fired on 6/6 injected invariants; drive measured at 4.45 m/s
```

The drive measurement is the last thing the preflight does, and it is handed to
the campaign rather than thrown away. It has to be last for the same reason the
oracle app's does — calibration drives, and the wall pin above depends on the
robot being where the field put it. Handing it forward buys two things: the
first match does not spend four seconds re-measuring a constant, and oracle 03's
command-range invariant is active from that match's first sample instead of from
wherever it happened to calibrate. Which is visible, because the campaign report
says so:

```
------------------------------------------------------------------------
  INVARIANTS NOT CHECKED
------------------------------------------------------------------------
     1/1 matches  command-out-of-range: no calibrated drive limits were supplied
```

That block is what a campaign looked like before the limits were passed
forward. `InvariantMonitor` accumulates every reason an invariant stood down
while it was sampling, rather than reporting what is switched off at the end —
a monitor asked afterwards reports full coverage for a match that spent its
first four seconds checking five invariants out of six, which is exactly the
flattering answer this whole mechanism exists to refuse.

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
[the region was the wrong shape](#the-region-was-the-wrong-shape-and-the-fix-was-half-wrong-too)
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
mis-aimed turret or a badly chosen the standoff, and it was neither.

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

**This section's arithmetic is against the wrong target, and the conclusion
still holds.** Every number above was solved against `RebuiltHub.checkCollision`,
a 0.597 m sphere centred on the hub -- which the next section shows is dead code
that maple-sim never calls. The *ratio* the calibration turns on is unaffected:
it comes from the shot table's own internal consistency, and 0.58 was 14% high
against any target. The live result stands too -- 1-in-29 before, 27-of-28
after. What does **not** survive is the claim that followed it, that the shooter
is so forgiving it scores from anywhere in 0.8-7.0 m. Measured against the real
scoring volume it scores from a narrow band, and finding that out cost a
tenfold scoring regression.

### The region was the wrong shape, and the fix was half wrong too

Putting the scoring regions behind the HUB instead of at its mouth fixed *where*
they were. It left them the wrong **shape**, and correcting that took a wrong
turn worth writing down, because the wrong turn is the more useful half.

The original region was an 82 × 47 inch pocket, and both of its dimensions were
guesses dressed as measurements:

* **47 inches tall** because that is the goal mouth's width. The mouth's width
  has nothing to do with where a robot stands — *the turret rotates*. Being off
  the HUB's axis in y costs exactly nothing.
* **82 inches deep** from `SHOOTING_STANDOFF_IN = 100`, a guess at flywheel
  range.

The first of those is simply wrong and is now gone. The second turned out to be
right for a reason nobody had established.

#### The wrong turn

`deposit_region_for` was a point-in-polygon test on that pocket, and the rule it
was standing in for — `RobotContainer.isInAllianceArea` — covers nine times the
ground. That is a real inconsistency, and `Score` stops the moment
`deposit_region_for` says the pose is legal, so an undersized region is a robot
driving *past* good scoring positions to reach a nominated one.

So the region became the whole alliance zone. Two 60-second runs on either side
of that change:

| | shooting distance | shots | scored |
|---|---|---|---|
| pocket | 2.04 m | 24 | **22** |
| whole zone | 2.53 m | 42 | **2** |

**Scoring fell by a factor of ten.** The region's centroid is the pose `Score`
and `Stage` drive to, so widening the region moved the shot half a metre
further out — and half a metre was the difference between working and not.

Three separate forward models had said that could not happen. All three were
wrong, and they were wrong for the same reason.

#### What the scoring geometry actually is

Two independent things happen to a FUEL projectile near the HUB, and the one
that reads like the scoring rule is not the one that scores.

**`Goal.simulationSubTick` scores it** — via `checkValidity` → `positionChecker`,
which the `Goal` constructor set to `box(xyBox, position.getZ(),
position.getZ() + height)`. `RebuiltHub` passes 47 × 47 inches and a height of
10 inches, so the target is a flat pad: ±0.597 m in x and y, and **z from
1.5748 to 1.8288** — a 254 mm window sitting *on top of* the hub height.
`RebuiltHub.checkCollision`, the 3D sphere test that reads exactly like the
scoring rule and that every model in this repo was built on, is overridden and
**never called by anything**.

**`GamePieceProjectile.launch()` deletes it.** It walks the arc on a 0.02 s grid
and latches `calculatedHitTargetTime` at the first step inside the ±0.5 m
`withTargetTolerance` box. From then on `updateGamePieceProjectiles` removes the
piece and runs `hitTargetCallBack` — which `TurretIOSim` never sets. The ball
vanishes whether it scored or not, which is why the loose-fuel count barely
moved during the bad runs while twenty balls left the hopper.

So a shot has to be inside a 254 mm-tall pad during the 97 mm of travel between
entering the goal box and entering the delete box. That is thin enough that the
outcome turns on details a closed-form model does not have — which is why
`SCORING_RANGE_M` is a **measurement**, and is written down as one.

#### What the region is now

A circular segment: the arc at the far edge of the measured scoring range,
closed by the chord along the face of the HUB. Two terms, answering two
questions, and `arena.can_score_from` is the single place both are asked:

* **The alliance zone** decides score-versus-pass. This is the term `Pass`
  reads, and it is what unblocks it.
* **The range** decides whether a shot aimed at the HUB arrives. A *radius*, not
  a slab in x, because the turret rotates.

The polygon stays a navigation aid and is deliberately a different shape from
the rule — inset half a robot from the HUB face, so the band right beside the
structure is outside the polygon and still, correctly, a scoring pose.

Against the original pocket that is about three times the mouth's width in y and
a shorter reach in x. The y freedom is the part that was always wrong; the depth
is the part that measurement supports.

Live, on the same 60-second test the two rows above came from:

| region | shooting distance | shots | scored |
|---|---|---|---|
| original pocket | 2.04 m | 24 | 22 |
| whole alliance zone | 2.53 m | 42 | 2 |
| **zone ∩ measured range** | **1.80 m** | **48** | **42** |

Twice the points of the pocket it started from, and the run had no
`robot-pinned` finding at all — longest stall 0.7 s, drive current 0 A. The y
freedom is what removes the wedge and the range term is what keeps the shot;
neither alone was enough.

**And it closed the wedge at match length.** A 150-second campaign on seeds 4375
and 4376 — the two that had reported `robot-pinned` in every previous campaign,
4 matches out of 4 — came back with **none**. The preflight still detects a
deliberate wall pin at 58 A, so the oracle has not stopped working; the robot
has stopped wedging. Two matches, and the finding it replaces was 4-for-4, which
is worth more than the sample size alone suggests.

That is the same conclusion the [cycle section](#the-cycle-this-creates-and-where-it-breaks)
below reaches from the other direction: the pinch is only compulsory if the
place you have to shoot from is on the far side of it. Widen where you may shoot
from, in the axis that costs nothing, and most cycles never approach the gap.

**The honest limit: nobody has swept the distance, and nobody is going to.**
2.04 m works and 2.5 m does not; the cliff is somewhere between, and the far
edge is set short of the failure rather than at a measured boundary. That is
good enough on purpose. REBUILT is the game this bridge was built *against*,
not the game it is for — a precise scoring band for a hypothetical season buys
nothing that survives the 2027 reveal. What had to be true is that the strategy
layer can score at all, so everything downstream of it can be exercised, and
42 of 48 is that.

The finding underneath it is the part that transfers, and it is not about
REBUILT: **a simulator's scoring volume is not always the code that looks like
its scoring rule.** Read what actually gets called, and if the geometry is thin,
measure rather than model.

**What the zone-wide version did to the wedge: moved it, did not remove it.**
Two matches re-run on their original seeds, before and after. Measured on the
version that had no range term, so it says something about the wedge and nothing
about the region that shipped:

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

**"Every cycle threads a gap twice" turned out to be a claim about the scoring
region, not about the field.** It was true while the only legal place to shoot
from was a 47-inch pocket on the HUB's axis, which is behind the wall from
almost everywhere. Once the region became the arc the turret can actually reach
— same depth, three times the width — most cycles stop approaching the gap at
all, and `robot-pinned` went from 4 matches out of 4 to none. The gaps are still
there and still 50 inches; the robot has stopped needing them. Worth remembering
the next time a field constant looks like it forces a shape on the match: check
whether it is the field forcing it or your own model of where the job is.

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

## The other five robots

Everything up to here drove one robot on an empty field. That is enough to
prove a bridge and it is not enough to be worth running overnight: a solo cycle
only ever exercises the paths where nothing is in the way, and the failures
worth finding in robot code are the ones where something is.

maple-sim will hold as many drivetrains as it is given and has no opinion about
what any of them does — its own answer for opponents is a PathPlanner replay or
a second gamepad, neither of which decides anything. sparky-sim has years of
decision-making and no way to touch a real robot. So `BridgeRobots.java` owns
the bodies and `bridge/opponents.py` says where they go.

**The extras are not props.** Each is a real `Robot` running a real
`StrategyController` against the same `MapleMatchView` our robot reads. They
collect the same fuel, navigate around the same obstacles, and react to what
our robot is doing — which is the point, because contesting a piece means
changing your mind about one somebody else just took, and a recorded path
cannot do that.

### The wire, and the one thing on it that is not a command

Inbound to the JVM is plain NetworkTables under `/Bridge/Robots`: a `Roster`
of (x, y, θ) triples read exactly once, then per-robot `Speeds`, `Intake` and
`Release`. Outbound comes back as AdvantageKit outputs, so AdvantageScope and
the replay get the extra robots for free.

The one entry that is not a command is `Tick`, a counter incremented every time
Python drives. `BridgeRobots` counts cycles since it last changed and zeroes
every robot after half a second of silence. Without it, the last command sent
becomes a permanent instruction: a crashed harness leaves five robots driving
into a wall for the rest of the session, and the next thing anybody sees is a
field that has to be cleaned up before it can be used.

A counter and not a timestamp, because the two processes do not share a clock —
and the version of this that compares NT timestamps to the local one is the
kind of thing that works until the day it does not.

### Deposit means dump

`BridgeRobots` gives an extra robot a drivetrain and an intake and no shooter,
so pressing deposit puts its fuel back on the floor behind it. That is the one
place the extras are honestly less than the real robot, and it is worth saying
plainly rather than hiding behind the word "score".

Building an opponent shooter would mean transcribing this game's ballistics
into the opponents' half of the arena — REBUILT work, on the game this bridge
was only ever proved against. What the extras are *for* is bodies that move
with intent and pieces that get taken; a dump contests both, and it also keeps
fuel circulating instead of ending the match with three full hoppers parked in
a corner. The visible consequence is that maple-sim's score for the other
alliance stays at zero, and a differential scoring oracle must not read it.

### maple-sim has one battery for the whole arena

This is the finding, and it is the sort that only appears once there is more
than one robot.

`SimulatedBattery` is **static**. Every `SwerveModuleSimulation` constructor
registers its drive motor's supply current on it, every `MapleMotorSim`
registers a steer motor, and `simulationSubTick` sums the lot, drops the
voltage accordingly, and hands the result to `RoboRioSim.setVInVoltage` — which
is the rail voltage the robot code under test reads as its own.

So five extra drivetrains draw from the battery of the robot being tested. Six
robots driving hard is several hundred amps, the voltage clamps at the brownout
threshold, and the first contested campaign produced this in every match:

```
  ds-error   console:80   [MapleSim] BrownOut Detected, protecting battery voltage...
  ds-error   console:81   [MapleSim] BrownOut Detected, protecting battery voltage...
  ...                     (hundreds of lines, both matches, 2 FAIL)
```

Not cosmetic. `SimulatedBattery.clamp` then limits every motor's applied
voltage, so the robot under test is genuinely slowed down by the presence of
opponents. The same 60-second scenario, before and after:

| | red1 | red2 | red3 | blue2 | blue3 | worst stall | fuel left |
|---|---|---|---|---|---|---|---|
| shared battery | 23.3 m | 27.7 m | 17.8 m | 31.8 m | 26.9 m | 20.8 s | 62 |
| own batteries | 34.4 m | 61.5 m | 57.1 m | 67.7 m | 41.1 m | 5.5 s | 15 |

Every robot travels roughly twice as far and stalls a quarter as long. **The
stalls were mostly the battery, not the field.** Before the fix it read as a
robot repeatedly wedging in REBUILT's 50-inch gaps — an entirely plausible
story, given how much of this project's history is exactly that — and it was
wrong.

There is no unregister on that static list, so the fix registers a *negative*
appliance per extra robot: a supplier returning minus its own draw, which the
sum cancels exactly. Both terms read the same instantaneous voltage, so the
cancellation is exact rather than approximate. Physically this is the correct
model and not a workaround for one: every robot in a real match carries its own
battery, and an opponent accelerating has no effect at all on our rail voltage.
The shared battery is the artefact; this removes it.

What it does not do is let an extra robot sag under its own load — they all run
at whatever our rail is doing. That is the right trade: the point of the extras
is bodies that move with intent, and modelling their brownouts would mean
simulating five more electrical systems to make five opponents slightly slower.

`run_bridge_opponents.py` now fails the run if the rail drops below 7 V, so the
fix is checked rather than trusted. Same discipline as the oracles.

### The probe that parked a robot in a wall

`WIRE`, the third check, drives one extra robot by hand before the match — a
constant velocity, no tactic involved, because a robot that fails to move under
a strategy could be failing anywhere between the trigger and the module
controllers, while one that fails to move under a constant 1.5 m/s is failing
on the wire.

The first version drove along **x**, which on this field is straight into the
face of that robot's own HUB. Every check downstream passed. The robot then
began the match wedged, and spent all of it pinned — which reads as a defender
that would not defend, and not at all as a probe that parked it in a wall. The
match before the fix: 3.2 m travelled in 45 seconds, ten of them stuck.

Two changes, and both are the general lesson rather than the specific one. The
probe drives along **y**, where the starting lanes are clear from wall to wall.
And it drives the robot **back to its mark** afterwards, closed-loop off the
pose — reversing for the same duration overshot by 0.6 m, because the
acceleration ramp is not symmetric with the coast. A check that leaves the
field rearranged has changed the experiment it was meant to validate.

### Thirteen seconds of handshakes

`RobotStateLink` primes a new subscription by blocking until it carries a
value, up to two seconds each, and a first tick creates a dozen of them between
the world state and the cast readback. Left inside the loop, that cost thirteen
seconds of a sixty-second match — a fifth of the run spent in NT handshakes,
with `elapsed` running and every robot doing nothing.

It presented as a match that started late rather than as a match that was slow,
which is why it survived step 4: a solo run creates fewer subscriptions and the
delay was small enough to look like boot jitter. Both the app and the harness
now warm every subscription before the clock starts.

### What it changed

Two matches, same seeds, same length, with and without the other five:

| | robot-pinned | alert-warning |
|---|---|---|
| solo | 1/2 | 2/2 |
| 3 opponents, 2 partners | **2/2** | 0/2 |

`robot-pinned` on a contested field is the defender doing its job, which is why
it is a warning and not a failure. The point is not the number — two matches
decide nothing — but that the finding now has a cause that a solo field cannot
produce.

Defaults stay solo. A populated field is a different experiment, not a better
version of the same one: it reaches states a solo run cannot, and it spends
real time with somebody wedged against somebody else. Two nights are worth
comparing; one night silently swapped for the other is not. `--opponents` and
`--partners` with the scripted driver are refused rather than ignored, because
the extras have no decisions without a strategy layer and a solo campaign filed
under a report saying "3 opponents" would say nothing about it.

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

Steps 1–4 and 6 are done, and oracle 03 with them: the strategy layer reads the
live field, drives the robot, and drives the other five as well, and the run is
judged by three oracles rather than two.

**Judge the rest by whether it survives the 2027 reveal.** This bridge is
machinery for driving *whatever* robot code exists against *whatever* game
arrives; REBUILT is the thing it was proved against, and REBUILT-specific polish
is spent effort.

* **Oracle 05 — coverage.** Which robot code the campaign never entered is the
  single most actionable thing a fuzzing run can tell you, and it is entirely
  game-agnostic. There is one specific obstacle, found by looking rather than
  by trying: JaCoCo writes its execution data from a JVM **shutdown hook**, and
  `sim_process` ends the robot with `taskkill /F /T`, which runs no hooks. The
  hard kill is load-bearing — see `--no-daemon` below — so this needs the
  agent in `output=tcpserver` mode and a dump requested over TCP before the
  kill, not a `jacoco {}` block. Budget a session for the build change, the
  dump client, and merging exec files across matches.
* **Oracle 04 — differential scoring**, still last, and the reasons have got
  slightly stronger rather than weaker. It sits on a shot whose range band is
  deliberately approximate, the extras deliberately do not score, and the
  alliance score it would read is therefore a solo-field claim. Worth doing
  when there is a game whose scoring sparky-sim models exactly; REBUILT is not
  it.
* **Intent→button mapping**, broadened past the canned tactic. `Stage`, `Pass`
  and now `Defend` have needed none of it, which was the prediction: a
  positioning tactic only drives. Only `eject` and `stopWithX` are plausibly
  missing, and adding either before a tactic wants it is speculative.
* **A `Pass` rule** — no longer blocked, and no longer obviously worth it.
  `Pass` the tactic is machinery and is done; wiring it into `cycle_fuel` is
  REBUILT strategy tuning, which is the category this list deprioritises.
  Left here because the measurement is cheap if a reason appears.

Two things the extras could grow, both deliberately not built yet because
nothing has asked for them: a per-robot drivetrain in the roster (opponents are
currently given exactly ours, so a contested result is not a statement about
which robot is faster), and a shooter, which is REBUILT ballistics and belongs
to whatever game arrives instead.
