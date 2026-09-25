# FRC game-manual trends (2017–2026)

Distilled from the manuals in this folder, for sanity-checking `common_sim`'s
game-agnostic abstractions (`common_sim/field/field_config.py`,
`common_sim/match/scoring.py`) against real cross-year variation before
building the next `game_specific/` package. See `ARCHITECTURE.md` for the
contract this is checking.

All nine manuals below were extracted and cross-checked in full against
their own rule numbers/section citations — no partial-source caveats remain
(an earlier pass had truncated downloads for 2022/2023; both have since been
re-sourced as complete manuals, and 2017/2018 were added new).

| Year | Game |
|---|---|
| 2017 | FIRST STEAMWORKS |
| 2018 | FIRST POWER UP |
| 2019 | DESTINATION: DEEP SPACE |
| 2020 | (no manual in this folder — COVID-cancelled season) |
| 2021 | INFINITE RECHARGE (2021 reissue, At-Home Challenges season) |
| 2022 | RAPID REACT |
| 2023 | CHARGED UP |
| 2024 | CRESCENDO |
| 2025 | REEFSCAPE |
| 2026 | REBUILT |

---

## 1. Match structure & timing

Every year follows the same skeleton: a short **AUTO** period with no driver
control, then a longer **TELEOP** period, with **endgame** either a distinct
labeled window or just "the last N/20/30 seconds of TELEOP."

| Year | AUTO | TELEOP | Endgame |
|---|---|---|---|
| 2017 | 15s | 135s | Not a separate phase — TOUCHPAD only arms in the last 30s |
| 2018 | 15s | 135s | Last 30s of TELEOP (not separately clocked) |
| 2019 | 15s (SANDSTORM) | 135s | Not a separate phase — HAB climb any time, contact rule tightens at T-20s |
| 2021 | 15s | 135s | Last 30s of TELEOP (not separately clocked) |
| 2022 | 15s | 135s | Not a separate phase — HANGAR contact protection keyed to last 30s |
| 2023 | 15s | 135s | Last 30s of TELEOP, FMS-cued ("Train Whistle") but not a separate clocked phase |
| 2024 | 15s | 135s | Not a separate phase — STAGE contact protections tighten in last 20s (G424-B) |
| 2025 | 15s | 135s | Not a separate phase — audio cue at 0:20 remaining, climb/park assessed at 0:00 |
| 2026 | **20s** | **140s**, subdivided into a 10s TRANSITION SHIFT + four 25s ALLIANCE SHIFTs + a **30s labeled END GAME** | First year with a clock-labeled, FMS-tracked END GAME period as its own phase |

**Pattern:** total match length has been a stable **2:30** for eight straight
years (2017–2025) and only changes in **2026** (**2:40** — the only year to
lengthen it, and the first to add a 20s AUTO instead of 15s). "Endgame" is
almost never a distinct engine phase — it's a contact-rule/scoring-window
change applied during the tail of TELEOP by wall-clock threshold, not a
`Match.phase` transition, in every single year except 2026, which is the
first to make it a first-class phase with its own name and FMS audio cue.

Near-transition scoring grace periods (pieces still settling get 3–10s to
finish counting, or "when all robots are at rest, whichever first") appear
in every year checked (2017 fuel/kPa counters, 2019, 2022, 2023, 2024, 2025,
2026) — this is a match-engine detail (`Match` assesses state N seconds
after a timer hits zero, or when all robots are at rest) rather than a
game-specific rule, and looks like a `common_sim` candidate if not already
handled generically.

**2026 novelty — phase-gated scoring by alliance:** REBUILT's HUB toggles
active/inactive per-alliance across the four ALLIANCE SHIFTs (which alliance
starts "off" is determined by who scored more FUEL in AUTO), and FUEL scored
into an *inactive* HUB is worth 0. This is new across all nine years: no
other game ties scoring eligibility to a rotating, outcome-dependent time
window per alliance. See "Implications" below.

## 2. Field & region archetypes

Every year's field decomposes into the same handful of region *roles*, even
though the concrete names change every year. Mapping onto today's
`field_config.py` primitives:

| Generic role | `field_config.py` primitive | Examples across years |
|---|---|---|
| Alliance-owned scoring target(s), possibly tiered/leveled | `ScoringRegion` (+ `capacity_by_action`, `alliance`) | 2017 BOILER (High/Low Efficiency); 2018 SCALE/SWITCH; 2019 ROCKET bays/CARGO SHIP; 2021 Power Port (3 tiers); 2023 GRID (3 rows x 3 grids); 2024 AMP/SPEAKER; 2025 REEF (4 levels)/PROCESSOR/NET; 2026 HUB |
| Human-player feeder with unlimited/replenishing supply | `IntakeLocation` | 2017 LOADING STATION; 2018 PORTAL; 2019 LOADING STATION; 2021 Loading Bay; 2022 TERMINAL; 2023 SUBSTATION; 2024 SOURCE; 2025 CORAL STATION; 2026 OUTPOST/CHUTE |
| Finite pre-staged piece pile | `PieceSpawnRegion` (non-station) | 2017 HOPPERS; 2018 POWER CUBE PILE; 2019 DEPOT; 2023 STAGING MARKS; 2025/2026 NEUTRAL-ZONE floor spawns, DEPOT |
| Robot starting zone | (not a scoring/contact primitive — just a spawn pose) | STARTING LINE / TARMAC / HAB PLATFORM / Initiation Line / COMMUNITY / ROBOT STARTING ZONE — present every year |
| No-contact/protected area | `ProtectedZone` | See dedicated section below — present in some form every year but the *shape* of protection varies a lot |
| Piece that returns to play after being emitted/scored | `EmitterRegion` (+ `linked_scoring_region`/`return_delay`) | 2018 EXCHANGE returns cubes to robots via a chute; 2022 HUB recirculates scored CARGO (~5-7s); 2025 NET/PROCESSOR outputs feed back to opposing human player |
| Endgame climb/hang/balance structure | `Obstacle` + custom scoring criteria | ROPE+TOUCHPAD / SCALE RUNGS / HAB (3 levels) / Generator Switch (hang+level) / HANGAR (4 rungs) / CHARGE STATION (balance seesaw) / STAGE (chain climb+harmony+trap) / BARGE+CAGE (shallow/deep) / TOWER (3 levels) |

**Confirms the abstraction set is sound**, with two things worth flagging:

- `ScoringRegion.capacity_by_action` (per-branch/bay/node caps) matches real
  rules in every tiered-scoring year (2019 one-per-bay, 2023 one-per-node
  with a "SUPERCHARGED" overflow exception once the grid is full, 2025
  one-per-branch).
- The 2018 EXCHANGE (cubes fed to a human player, returned to robots via a
  chute) and 2022 HUB's recirculation (scored CARGO re-enters the field
  after ~5-7s, not removed from play) are exactly what
  `EmitterRegion.linked_scoring_region` + `return_delay` was built for —
  good real-world validation of that field.

## 3. Protected / no-contact zones — the most game-variable mechanic

This is where the years genuinely diverge, and where `ProtectedZone`'s
design (protection attaches to the *robot*, not a blanket "no defense here"
rule; `foul_points` and `foul_period` configurable per zone) earns its
keep. Every year has *some* form of contact restriction, but the shape
differs along two axes: **geographic vs. temporal**, and **whole-match vs.
phase-gated**.

| Year | Protected zone(s) | Geographic or temporal? |
|---|---|---|
| 2017 | RETRIEVAL ZONE (always); "touching own rope" is a protected *state* (G07, always); no contact with opponent's rope in last 30s (G20); KEY is a restricted-dwell (not no-contact) zone, 5s max | Both — mostly geographic, plus a state-based protection independent of location |
| 2018 | NULL TERRITORY (always); PLATFORM ZONE (ENDGAME only, G18) | Both |
| 2019 | HAB ZONE (own robots only, always); no defense at all during SANDSTORM (G3); no contact w/ opponent ROCKET in last 20s (G16) | Both — a static zone (HAB) plus two purely time-gated rules |
| 2021 | Target Zone/Trench Run/Loading Zone (asymmetric — only the *intruder* is restricted, G10/G11); Rendezvous Point during Endgame only (G14) | Both, and notably **asymmetric** protection (G10 vs G11 protect different parties) |
| 2022 | Opponent's side of field during AUTO (G210); LAUNCH PAD while shooting (G207); MID/HIGH/TRAVERSAL rungs + HANGAR ZONE in last 30s (G208) | Both |
| 2023 | LOADING ZONE + COMMUNITY — but **inverted**: the *intruding* robot loses protection, not the home robot (G207 "right of way"); CHARGE STATION during ENDGAME only (G209) | Both, and same asymmetric shape as 2021 |
| 2024 | PODIUM (pre-last-20s only), SOURCE ZONE, AMP ZONE (always), STAGE ZONE (always, tightens further in last 20s) | Both |
| 2025 | REEF ZONE + BARGE ZONE (always, symmetric — matches `ProtectedZone`'s model exactly); CAGE-contact protection (last 20s only, G428); full no-contact once a robot crosses BARGE ZONE in AUTO (G403) | Both |
| 2026 | **No geographic protected zone at all.** Only a temporal rule: no contact with a robot touching its own TOWER in the final 30s (G420) | **Temporal only** — first year with no static safe zone |

**Implication for `common_sim`:** `ProtectedZone` today is inherently
*geographic* (a `vertices` polygon). Several years have rules that don't fit
that shape at all: 2017's "touching your own rope" and 2026's "touching your
own TOWER" are **state-based, not location-based** — the protection follows
a *condition on the robot*, independent of where it is on the field. 2019's
SANDSTORM no-defense rule is **purely temporal** — no contact anywhere, for
anyone, during a time window. **2026 has zero geographic protected zones for
the whole match**, which is the first game in this set where
`FieldConfig.protected_zones` would legitimately be empty. This is worth a
design note before building `game_specific/rebuilt/`: G420's "protection
follows the robot, gated by what it's touching and the clock" needs either
(a) a zero-size/degenerate zone hack, or (b) a small new primitive (a
time-windowed, contact-triggered protection rule keyed on robot *state*
rather than *position*) alongside `ProtectedZone`. Given this state-based
shape recurs in 2017 (rope), 2019 (SANDSTORM), 2021 (Endgame Rendezvous),
2022 (LAUNCH PAD/rungs), 2024 (last-20s STAGE tightening), 2025 (CAGE
protection), and now 2026 (TOWER, exclusively), it looks like a genuine
recurring pattern, not a one-off.

Also worth noting: 2021 and 2023 both have **asymmetric** protected zones,
but in opposite directions — 2021's Target Zone/Trench Run/Loading Zone
protects the zone's *owner* from an intruding opponent (G10/G11), while
2023's LOADING ZONE/COMMUNITY rule (G207 "right of way") does the reverse:
the *intruder* loses contact protection while inside the opponent's zone.
Both fit today's `ProtectedZone.alliance` field, but confirm the asymmetric
case is common enough (2 of 9 years) to keep testing against.

`PinRule` (3-5s pin limit, near-universal) is confirmed in **every single
year** in this set (2017 G11 5s, 2018 G14 5s, 2019 G18 5s, 2021 G21 5s, 2022
G202 5s, 2023 G202 5s, 2024 G420 5s, 2025 G425 3s, 2026 G418 3s) — the trend
toward a *shorter* pin count (5s → 3s, first seen in 2025/2026) is worth
noting if tuning defaults, though it held steady at 5s for eight straight
years before that.

**A recurring, structurally distinct penalty pattern: "victim gets credited,
not the offender fined."** Several years' protected-zone violations don't
work like an ordinary foul (points to the opponent) — they directly award
the *victim* the scoring credit it was denied: 2018 G18 (illegal PLATFORM
ZONE contact credits the victim as CLIMBED), 2022 G208 (illegal rung/HANGAR
contact credits the victim TRAVERSAL RUNG points), 2023 G209 (illegal
CHARGE STATION contact credits the victim DOCKED+ENGAGED), 2025 G428
(illegal CAGE contact awards the opponent alliance the BARGE ranking
point), 2026 G420 (illegal TOWER contact credits the victim LEVEL 3 TOWER
points outright). This is common enough (5 of 9 years, always tied to the
endgame climbing mechanic specifically) that it's a second penalty *shape*
worth having a name for, distinct from the flat point-credit foul model —
"denied action auto-succeeds" rather than "opponent pays a fine."

## 4. Scoring pattern shapes

- **Tiered/leveled scoring is the default, not the exception**: 2017
  (High/Low Efficiency BOILER), 2018 (bottom/middle/top NODE... actually see
  2023 for that — 2018's SCALE/SWITCH pay a flat rate but scale with time
  held), 2019 (HAB L1-3, ROCKET bays), 2021 (Bottom/Outer/Inner Port), 2022
  (LOWER/UPPER HUB), 2023 (bottom/middle/top GRID row), 2024 (AMP vs
  SPEAKER, amplified vs not), 2025 (REEF L1-L4), 2026 (TOWER L1-3) all price
  the same general action higher for a harder-to-reach target.
- **AUTO points consistently outweigh the equivalent TELEOP action** where
  both exist (2017's per-fuel rate is much better in auto — 1 point per 3
  fuel in Low/1 fuel in High during auto vs 1 per 9/3 in teleop; 2019's HAB
  climb bonus is auto-only; 2021 Bottom/Outer/Inner Port pay 2/4/6 in auto
  vs 1/2/3 in teleop; 2022 CARGO pays 2x in auto; 2023 GRID rows pay 3/4/6
  in auto vs 2/3/5 in teleop, and DOCKED/ENGAGED pay 8/12 in auto vs 6/10 in
  teleop; 2024 AMP/SPEAKER pay roughly 2x auto vs teleop; 2025 CORAL pays
  more in auto at every level; 2026 TOWER LEVEL 1 pays 15 in auto vs 10 in
  teleop) — a strong, consistent, nine-year-deep auto-is-worth-more design
  pattern worth reflecting in strategy defaults.
- **Ranking Points (RP), not raw score, decide qualification rank** every
  year checked — always Win/Tie RP plus 1-3 game-specific bonus RPs (a
  scoring-volume threshold RP + a "did a hard optional task" RP is the
  recurring shape: 2017 kPa/rotor bonuses; 2018 AUTO QUEST/FACE THE BOSS;
  2019 Complete Rocket/HAB Docking; 2021 Operational/Energized; 2022 CARGO
  BONUS/HANGAR BONUS; 2023 SUSTAINABILITY/ACTIVATION; 2024 MELODY/ENSEMBLE;
  2025 AUTO/CORAL/BARGE; 2026 ENERGIZED/SUPERCHARGED/TRAVERSAL). RP
  thresholds routinely step up at District Championship / FIRST
  Championship events (2024, 2025, 2026 all confirm this).
- **Win/Playoff bonus RP is interchangeable with a flat point bonus**: 2017
  is the clearest example — its two bonus RPs (kPa threshold, all-4-rotors)
  become flat 20/100-match-point bonuses instead in Playoffs, where RP
  doesn't apply. This "same achievement, different currency depending on
  tournament phase" pattern is worth confirming other years don't hide the
  same swap (most years in this set simply drop the RP-only bonuses
  entirely in Playoffs rather than converting them).
- **Coopertition-style bonuses** (both alliances must cooperate to unlock a
  bonus) appear in 2023 (both alliances place ≥3 pieces on their own CO-OP
  grid lowers the SUSTAINABILITY threshold from 6 to 5 links), 2024
  (NOTE-through-AMP within first 45s) and 2025 (both alliances score ≥2
  ALGAE in their own PROCESSOR) but were **not** found as an in-game
  mechanic in 2017, 2018, 2019, 2021, 2022, or 2026 (general FIRST
  "Coopertition" philosophy is always referenced, but a scored bonus isn't
  universal — roughly half of years have one).
- **Time-accrued scoring is a real, distinct pattern (not just discrete
  per-action points)**: 2018's SCALE/SWITCH OWNERSHIP scores a flat amount
  on establishing ownership *plus* points per second held (e.g. "2 + 2/sec"
  in auto), rather than a one-time award per piece scored. This is the only
  year in the set that scores *possession over time* instead of *discrete
  events* — worth confirming `ScoringRules`/`ScoringRegion` can express a
  per-tick accrual if a future game revives this shape (2018's own
  BOOST/FORCE power-ups additionally *multiply* or *override* this
  time-accrual mid-match, which is a second layer on top).

## 5. Game piece patterns

- **One or two piece types**, never more: 2017 (FUEL + GEARS, two types with
  a per-rotor quota relationship), 2018 (single type: POWER CUBE), 2019
  (CARGO + HATCH PANEL, two types with a dependency — CARGO needs a HATCH
  PANEL placed first), 2021/2022/2026 (single type: Power Cell/CARGO/FUEL),
  2023 (CONE + CUBE, independent, node-shape-dependent scoring), 2024 (NOTE,
  with a rare "HIGH NOTE" variant restricted to human-player scoring only),
  2025 (CORAL + ALGAE, independent, not dependent on each other).
- **Simultaneous-control capacity is usually tightly capped** — most years
  cap a robot at 1 piece (2017 GEARS specifically — no explicit FUEL cap
  during play beyond preload, 2018, 2019, 2023 outside the loading
  zone/community) or 1-per-type (2025: 1 CORAL + 1 ALGAE). 2021 and 2022 are
  outliers at 5 and 2 respectively. **2026 is the biggest outlier: no
  in-match capacity cap at all** ("a ROBOT may CONTROL any number of
  SCORING ELEMENTS") — only a start-of-match preload cap (8). This is a
  meaningful design swing worth confirming against `RobotCharacteristics`
  capacity modeling, which likely assumes "small integer cap" as the norm.
- **Preload counts vary widely** from 1 piece (2018, 2019, 2023, 2024) to 3
  (2021) to 10 FUEL + 1 GEAR (2017) to 8 (2026) per robot —
  `RobotCharacteristics`/setup code should treat this as a per-game
  parameter, not a constant.

## 6. Penalty/foul patterns

Remarkably stable shape across all nine years:

| Concept | 2017 | 2018 | 2019 | 2021 | 2022 | 2023 | 2024 | 2025 | 2026 |
|---|---|---|---|---|---|---|---|---|---|
| Minor foul name & value | FOUL, 5 | FOUL, 5 | FOUL, 3 | FOUL, 3 | FOUL, 4 | FOUL, 5 | FOUL, 2 | MINOR FOUL, 2 | MINOR FOUL, 5 |
| Major foul name & value | TECH FOUL, 25 | TECH FOUL, 25 | TECH FOUL, 10 | TECH FOUL, 15 | TECH FOUL, 8 | TECH FOUL, 12 | TECH FOUL, 5 | MAJOR FOUL, 6 | MAJOR FOUL, 15 |
| Card escalation | YELLOW→RED (2nd yellow) | same | same | same | same | same | same | same | same |
| DISABLED / DISQUALIFIED | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes | Yes |

Every single year: same two-tier foul system, same card-escalation rule (a
second YELLOW in the same tournament phase becomes a RED), same
DISABLED-robot and DISQUALIFIED-team mechanics. Point *values* drift year to
year (no stable trend up or down — 2017/2018 run notably higher than
2022-2024, then 2025/2026 climb again) but the *structure* — two foul tiers
+ a card system — is FRC-constant enough across all nine years that it's a
safe `common_sim` assumption, while the point values themselves clearly
belong in `game_specific` (already the case per `ScoringRules`).

Also consistent every year: "stay out of other robots" (no non-bumper
component contacting an opponent's frame), "not combat robotics" (no
deliberate damage), "no tipping/entangling," "10-second grace period for a
tipped robot righting itself" (explicit in 2017/2019, implied elsewhere),
and "no collusion to close off a major game element, but single-robot
defense is always legal" as named rule categories, present verbatim in
every year checked — these look like they could become named
`Trigger`/foul-category constants shared across `game_specific` packages
rather than being re-derived from scratch each year, though nothing in
`common_sim` currently models fouls as game logic (that's presumably left
to `ScoringRules`/manual judgment in this sim, which is reasonable —
refereeing isn't simulated).

## 7. Alliance / ranking-point patterns

- Win/Tie RP values have drifted upward: 2017/2018/2019/2021/2022/2023/2024
  pay 2 RP for a win, 2025/2026 pay **3 RP** for a win (tie stays 1 RP
  throughout all nine years).
- RP is Qualification-only in every year checked; Playoffs are straight
  win/loss (no RP, no bonus-RP mechanics like Coopertition buttons) — 2017's
  RP→flat-point-bonus conversion (see Section 4) is the one partial
  exception, converting rather than dropping its bonuses in Playoffs.
- Max per-match RP has crept up alongside the extra bonus-RP categories:
  2017/2018/2019/2021 cap at 4 RP/match (2 win + up to 2 bonus), 2022/2023
  also cap at 4, 2024 effectively 4 (2+1+1), 2025 up to 5 (3+1+1), 2026 up
  to 6 (3+1+1+1).

## 8. Per-year quick-reference table

| Year | Game | Pieces | Scoring region "kinds" | Endgame mechanic | Protected zone shape |
|---|---|---|---|---|---|
| 2017 | FIRST STEAMWORKS | FUEL, GEAR | BOILER (High/Low Efficiency goals), ROTORS (gear sets) | Rope climb + TOUCHPAD press | RETRIEVAL ZONE (static) + "touching own rope" (state-based) + no-rope-contact in last 30s (temporal) |
| 2018 | FIRST POWER UP | POWER CUBE | SCALE, SWITCH (time-accrued OWNERSHIP), VAULT | SCALE climb (PARK/CLIMB) | NULL TERRITORY (static) + PLATFORM ZONE (endgame-only) |
| 2019 | DESTINATION: DEEP SPACE | CARGO, HATCH PANEL | ROCKET (6 bays x2), CARGO SHIP (8 bays) | HAB climb, 3 levels | HAB ZONE (static) + no-defense-in-auto (temporal) |
| 2021 | INFINITE RECHARGE | Power Cell | Power Port (3 tiers) | Hang/Park/Level Generator Switch | Target/Trench/Loading Zone (asymmetric static) + Rendezvous (endgame-only) |
| 2022 | RAPID REACT | CARGO | HUB (Upper/Lower, recirculating) | HANGAR climb, 4 rungs | Opponent's side in auto (temporal) + LAUNCH PAD/rungs (state-based) |
| 2023 | CHARGED UP | CONE, CUBE | GRID (3 rows x 3 grids/alliance, incl. SUPERCHARGED overflow) | CHARGE STATION balance (DOCK/ENGAGE) | LOADING ZONE/COMMUNITY (asymmetric, intruder loses protection) + CHARGE STATION (endgame-only) |
| 2024 | CRESCENDO | NOTE (+HIGH NOTE) | AMP, SPEAKER (amplified/not) | STAGE climb (onstage/harmony/trap/spotlight) | PODIUM/SOURCE/AMP/STAGE zones (mixed static+temporal) |
| 2025 | REEFSCAPE | CORAL, ALGAE | REEF (4 levels), PROCESSOR, NET | BARGE CAGE climb (shallow/deep) | REEF ZONE + BARGE ZONE (static, symmetric) + CAGE (endgame-only) |
| 2026 | REBUILT | FUEL | HUB (alliance-toggling active/inactive) | TOWER climb, 3 levels | None static — TOWER protection is temporal/state-based only |

## 9. Implications for `common_sim`

What recurs across most/all years (validates an existing abstraction):

- `ScoringRegion` + `capacity_by_action` + `alliance` — tiered, per-slot-capped,
  alliance-owned scoring targets are the norm every year. ✅ matches, and
  2023's SUPERCHARGED overflow (extra points once the grid is full) is a
  good stress test of `capacity_by_action`'s edges.
- `IntakeLocation` — every year has an unlimited human-player feeder station.
  ✅ matches.
- `ProtectedZone` (asymmetric protection model, configurable `foul_points`) —
  matches 2025's REEF/BARGE and 2019's HAB directly; 2021's and 2023's
  *intruder-only* asymmetric zones (opposite polarity from each other) are a
  good stress test that the "protection is on the robot, not the zone"
  design already handles correctly.
- `PinRule` — present, with a shrinking time limit, in **every one** of the
  nine years checked.
- `EmitterRegion.linked_scoring_region`/`return_delay` — validated by 2018's
  EXCHANGE return chute, 2022's recirculating HUB, and 2025's NET/PROCESSOR
  output-to-opponent flow.
- Near-buzzer scoring-assessment grace windows (3-10s) — recurring enough
  (2017, 2019, 2022, 2023, 2024, 2025, 2026) that if `Match`/`match.py`
  doesn't already model "assess N seconds after phase end or when robots
  are at rest, whichever first" generically, it's worth promoting out of
  one-off game code.

What's genuinely one-off or year-specific (confirms it belongs in
`game_specific`, not `common_sim`):

- Exact foul-point values, RP thresholds, and piece-capacity numbers — these
  drift every year and are already correctly isolated behind `ScoringRules`
  and `RobotCharacteristics`.
- Piece dependency chains (2019's CARGO-needs-a-HATCH-PANEL-first, 2017's
  per-rotor GEAR quota) — a one-off relationship between two piece types,
  not a general primitive. Cleanly expressible as `game_specific` logic
  checking region/piece state.
- 2018's time-accrued OWNERSHIP scoring and its BOOST/FORCE power-up
  multipliers — a one-off scoring shape (only 1 of 9 years) that doesn't
  need a `common_sim` primitive unless it recurs; worth confirming
  `ScoringRules.points_for` could express a per-second accrual via
  `game_specific` code alone (e.g. calling it every tick with an
  ownership-duration-derived action) before assuming it needs new plumbing.

Three gaps worth a design decision before or during the next
`game_specific/` build:

1. **No temporal/state-based contact-protection primitive.** `ProtectedZone`
   is geometry-based; 2017's "touching your own rope," 2019's SANDSTORM
   no-defense rule, 2022's LAUNCH-PAD/rung protection, and — most
   pressingly — **2026's TOWER protection (G420), which is REBUILT's only
   contact restriction and has no geographic zone at all**, don't fit it.
   Building `game_specific/rebuilt/` will need either a new lightweight
   primitive (e.g. a rule keyed on "robot touching X" + a time window,
   independent of a `vertices` polygon) or a documented convention for
   faking it with a degenerate zone — worth deciding explicitly rather than
   improvising in `game_specific` code. This pattern showed up in the
   majority of years checked (6 of 9), so it's due for a real primitive,
   not another one-off workaround.
2. **No primitive for the "denied action auto-succeeds" penalty shape.**
   Five of nine years (2018, 2022, 2023, 2025, 2026) resolve an illegal
   endgame-zone contact by crediting the *victim* with the scoring action
   it was denied, rather than (or in addition to) fining the offender.
   Nothing in `common_sim`'s scoring/penalty path currently models "an
   illegal contact directly grants a `ScoringRegion` action to the
   contacted robot" — today `ProtectedZone.foul_points` only credits flat
   points, not a scoring *action*. Worth a design note if the sim ever
   wants to model this penalty shape rather than approximating it with a
   large flat point credit.
3. **No primitive for phase-gated, alliance-alternating scoring
   eligibility.** REBUILT's HUB toggles active/inactive per alliance across
   four TELEOP sub-shifts, with scoring into an inactive HUB worth 0.
   `ScoringRules.points_for(action, phase)` takes only a coarse
   auto/teleop `phase` string — REBUILT needs something closer to
   `points_for(action, phase, alliance, match_time)` or an equivalent way
   for a `ScoringRegion` to know whether it's "hot" for a given alliance
   right now. This is the one mechanic across all nine years that doesn't
   fit the current scoring-lookup signature at all.
