# The maple-sim bridge

Drives the team's **real robot code**, running under maple-sim in WPILib's
simulator, from Python — so a competitive scenario can be generated on a night
when the robot is on the mechanical team's cart and the field doesn't exist yet.

Background and rationale: [The maple-sim Bridge](https://claude.ai/code/artifact/648dfe02-ea7d-4b2f-b0ee-44094eb28407).

**Status: steps 1–3 of 7 done — the spine is complete.** The loop closes, the
oracles fire, and the harness runs unattended and leaves a morning report. What
remains widens the variety of situations it reaches.

## Run it

```
pip install -r bridge/requirements.txt
python apps/run_bridge_smoke.py      # step 1: does the loop close?
python apps/run_bridge_oracles.py    # step 2: can it tell a failure from a run?
python apps/run_bridge_overnight.py --matches 40    # step 3: the product
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

## Shape

| Module | Direction | Transport |
|---|---|---|
| `operator.py` | Python → robot | HALSim WebSocket, `ws://127.0.0.1:3300/wpilibws` |
| `robot_state.py` | robot → Python | NetworkTables 4, via `pyntcore` |
| `sim_process.py` | — | launches and reaps `gradlew simulateJava -Pbridge` |
| `oracles.py` | — | decides whether a run failed |
| `scenario.py` | — | the seeded virtual operator |
| `harness.py` | — | campaign lifecycle, retention, and the report |

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
```

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

The campaign is **preflighted**: one deliberate wall pin that must produce
`frozen-robot`, before committing eight hours to a detector that might not
detect. Results are flushed to `campaign.jsonl` as they happen, so the report
survives the machine going down at 3am.

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

The spine is done. What remains widens the variety of situations reached:

* **NT world-state reader + `MapleMatchView` adapter** — the strategy layer starts
  reacting to the live field instead of driving a seeded script. This is where
  the 2026-vs-REEFSCAPE mismatch finally starts to matter.
* **Intent→button mapping**, broadened past the canned tactic.
* **AI opponents** — the other five robots driven by sparky-sim.
* **Oracles 03–05** — invariants, differential scoring, JaCoCo coverage.
