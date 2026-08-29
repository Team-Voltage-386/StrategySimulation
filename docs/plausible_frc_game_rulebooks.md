# Three hypothetical FRC game rulebooks

For a visual, web-page version with custom field-layout diagrams, open
[plausible_frc_game_rulebooks.html](plausible_frc_game_rulebooks.html).

> **Design-study material, not an official FIRST game manual.** These three
> concepts use the common FRC match rhythm and manual conventions as a guide,
> but all names, field elements, values, and rules below are invented. They
> are deliberately specified far enough to become separate
> `game_specific/<game>/` packages in StrategySim.

## Common competition conventions

These conventions apply to all three games unless a game explicitly changes
one of them.

- An alliance has three robots. A qualification MATCH is 15 seconds of
  **AUTO**, followed by 135 seconds of **TELEOP**. The final 30 seconds are
  the **ENDGAME** window. A game piece that is in the process of scoring when
  the MATCH ends may settle for up to 5 seconds; it scores if it comes to
  rest meeting its target's criteria.
- Field coordinates are inches. The origin `(0, 0)` is the lower-left corner
  at the blue ALLIANCE WALL; `+x` points toward red. The nominal carpet is
  `690 x 317 in`. Red features are the 180-degree rotation of blue features
  unless stated otherwise.
- A ROBOT may not CONTROL more than the stated holding limit. CONTROL means
  carrying, trapping, or otherwise directing a game piece; merely deflecting
  a loose piece does not count. A violation is a 5-point FOUL. A ROBOT may
  release pieces immediately to return to compliance.
- Standard contact expectations apply: no deliberate damage, entanglement,
  or tipping; no pin longer than 3 seconds without a 1-second release; and
  no contact with an opponent that is fully in its own protected loading or
  scoring zone. Each such protection or pin violation is a 5-point FOUL;
  repeated or dangerous violations may draw a YELLOW CARD. A second YELLOW
  CARD in the tournament phase is a RED CARD.
- In qualifications, a win earns 3 Ranking Points (RP), a tie earns 1 RP,
  and a loss earns 0. The listed bonus RPs are qualification-only. Playoffs
  are decided by MATCH wins and score, not RP.

### Simulation handoff checklist

For each game, students can model the following directly with existing
StrategySim concepts: the field dimensions, `Obstacle`s, `ScoringRegion`s,
finite `PieceSpawnRegion`s, unlimited `IntakeLocation`s, target capacities,
and per-action AUTO/TELEOP values. Endgame parking/climbing and bonus-RP
assessment can initially be post-MATCH checks, then become game-specific
logic once the basic cycle game runs.

---

# 1. AURORA ARRAY

## Game idea

Alliances collect **AURORA CELLS**, compact hexagonal pucks, and fill a
private three-tier ARRAY. The high-value center **CROWN** can be used only
after an alliance has established enough of its own ARRAY, so teams must
choose between fast local cycles and a longer, contested center route.

This is the best starting game for studying a four-piece robot: holding a
batch makes the high goal productive, but makes a missed pickup or a long
defensive crossing much more costly.

## Field and game pieces

**AURORA CELL:** a 6-inch-wide, 2-inch-tall hexagonal puck, mass 0.35 lb.
It may be rolled, placed, or launched. Each ROBOT has a **4-CELL holding
limit**. Each ROBOT starts with up to **2 preloaded CELLS**; the remaining
pre-MATCH CELLS are loose.

| Landmark | Blue-side center / footprint | Purpose |
| --- | --- | --- |
| ARRAY | `(110, 158)`; 64 x 132 in | Alliance-owned low, middle, and high scoring face |
| LOADING PORTS | `(35, 55)` and `(35, 262)`; 36 x 42 in | Unlimited alliance-owned CELL supply |
| CROWN | `(345, 158)`; 72-inch-diameter octagon | Shared center scoring structure |
| SOLAR RAILS | from `(245, 60)` to `(445, 60)` and `(245, 257)` to `(445, 257)`; 24 in wide | Two solid barriers, leaving north, center, and south travel lanes |
| STARTING LINE | `x = 90` | AUTO mobility reference |

The red ARRAY and PORTS are rotated copies: red ARRAY center `(580, 159)`.
Place 12 loose CELLS in each alliance's half, distributed around `(180, 85)`,
`(180, 158)`, and `(180, 232)`. Place 12 neutral CELLS in six pairs at
`x = 285` and `x = 405`, spanning `y = 58` through `259`. The CROWN is a
solid 54-inch-diameter base with eight 9-inch-wide scoring windows around
its perimeter; robots must drive around it rather than through it.

```text
Blue wall                                                      Red wall
| Ports  ARRAY  loose cells  == solar rails ==  CROWN  == rails ==  cells ARRAY Ports |
|          \----------- three drive lanes: north / center / south -----------/          |
```

The 18-inch margin around each ARRAY and each LOADING PORT is its alliance's
protected zone. The CROWN is neutral; it has no protected zone.

## MATCH setup and scoring

At the start of AUTO, all ROBOTS must be fully in their alliance's starting
zone between its wall and starting line. A CELL scores only when it is fully
supported by, or fully at rest within, a legal target. A CELL launched through
a CROWN window is allowed to score without the ROBOT entering the target
region.

| Action | Criteria | AUTO | TELEOP | Capacity |
| --- | --- | ---: | ---: | --- |
| `array_low` | CELL rests in an open low shelf on own ARRAY | 4 | 2 | 6 per alliance |
| `array_mid` | CELL rests in an open middle shelf on own ARRAY | 6 | 4 | 6 per alliance |
| `array_high` | CELL rests in an open high shelf on own ARRAY | 9 | 6 | 6 per alliance |
| `crown` | CELL passes through a CROWN window and stays in its catch tray | 10 | 8 | 12 shared, 6 per alliance |
| `leave` | ROBOT begins AUTO in its starting zone and finishes fully beyond its starting line | 3 | - | once per ROBOT |

The CROWN starts **locked** for an alliance. It unlocks for that alliance as
soon as it has scored **three CELLS in any combination of ARRAY shelves**.
It remains unlocked for the rest of the MATCH. An alliance that has not
unlocked its CROWN window scores zero for any CELL placed there; the CELL is
returned to the nearest neutral loose-piece location at the next field reset.

After three ARRAY shelves in one vertical column are filled, that column is
**lit**. The three columns are independent; a CELL may not be moved after it
has scored. This gives the field 18 private ARRAY capacity and 12 shared
CROWN capacity.

## Endgame and ranking points

During the last 30 seconds, a ROBOT may park on its own **AURORA PAD**, a
48 x 72-inch platform centered at `(68, 158)` for blue and `(622, 159)` for
red. A ROBOT is **parked** when fully supported by the platform, not
contacting carpet, and stationary at the end of the scoring grace period.
Each parked ROBOT earns 8 points. Two or three parked ROBOTS earn an
additional 6-point **array alignment** bonus if all have their bumpers
inside the platform boundary. Contact with an opponent attempting to park on
its AURORA PAD in ENDGAME is a 10-point TECH FOUL, and the contacted robot is
credited as parked if the contact prevents it from qualifying.

| RP | Qualification condition |
| --- | --- |
| **Radiance RP** | Score at least 30 points from ARRAY shelves, including at least one CELL on every tier. |
| **Crown RP** | Unlock the CROWN and score at least 4 CELLS in it. |
| **Alignment RP** | Have at least 2 parked ROBOTS at MATCH end. |

## StrategySim implementation brief

Use one piece type, `aurora_cell`, with `RobotCharacteristics.capacity = 4`.
Build six alliance-owned `array_*` regions (three tiers times two
three-column faces, capacities of 3 each) and a neutral passive `crown`
region with total capacity 12. The per-alliance CROWN cap and the
three-ARRAY-score unlock are small game-specific state checks. Represent the
rails and CROWN base as obstacles; model the ports as unlimited
`IntakeLocation`s and all floor CELLS as finite spawn regions. The first
student experiments should compare a 1-, 2-, and 4-CELL collector, and
compare fast ARRAY cycling against unlocking CROWN early.

---

# 2. RIVET RUSH

## Game idea

Alliances build their side of a bridge from a single flexible **RIVET**
element. Every scored RIVET either completes a short, low-risk **DECK** slot
or a harder **ARCH** slot. Completing paired ARCHES unlocks an alliance's
central **SPAN** target, creating a clear construction dependency without
introducing a second game piece type.

This game emphasizes slot capacity, sequencing, and high-capacity batch
delivery. It is intentionally a placement game, rather than a shooter game.

## Field and game pieces

**RIVET:** a 4-inch-diameter foam ring with a 1.25-inch center hole, mass
0.15 lb. A ROBOT may CONTROL up to **5 RIVETs**. Each ROBOT may preload 3.

| Landmark | Blue-side center / footprint | Purpose |
| --- | --- | --- |
| DECK | `(155, 78)`; 132 x 34 in | Six floor-level, alliance-owned RIVET slots |
| ARCH | `(230, 220)`; 90 x 46 in | Four elevated alliance-owned hooks |
| SPAN | `(345, 158)`; 96 x 32 in | Four shared center hooks, initially unavailable |
| DEPOT | `(85, 250)`; 40 x 46 in | Unlimited alliance-owned human-player handoff |
| RIVET RACKS | `(290, 105)` and `(400, 212)`; 44 x 44 in | Two neutral finite floor racks, 12 RIVETs each |
| GIRDER | rectangle `(300..390, 138..179)` | Solid center barrier; routes pass above or below |

Red DECK, ARCH, and DEPOT are rotated. Stage six loose RIVETs along each
alliance's DECK approach and 24 on the two neutral RACKS. Each DECK has six
distinct one-RIVET sockets. Each ARCH and SPAN has four distinct overhead
hooks. The physical scoring zones are 10 x 12-inch rectangles centered at
each socket or hook approach; their elevation is handled by the robot's
action capability/reliability, not carpet geometry.

```text
Blue wall                                                        Red wall
| DEPOT     ARCH          RACK   [ solid GIRDER ]   RACK          ARCH DEPOT |
|     DECK slots ======= lower open lane / upper open lane ======= DECK slots |
|                         shared SPAN hooks                        |
```

The DEPOT and the 14-inch area around each alliance's ARCH are protected for
that alliance. Neither the central racks nor the SPAN is protected.

## MATCH setup and scoring

Only a RIVET that remains fully seated in a socket or fully captured on a
hook scores. A RIVET that falls after the 5-second scoring grace period is
not scored and becomes a loose field piece. A ROBOT may score in any order,
but an ARCH's upper hook is unavailable until its paired lower hook has a
RIVET. Thus the four ARCH actions are `arch_lower_1`, `arch_upper_1`,
`arch_lower_2`, and `arch_upper_2`.

| Action | Criteria | AUTO | TELEOP | Capacity |
| --- | --- | ---: | ---: | --- |
| `deck` | Seat a RIVET in an empty own-DECK socket | 3 | 2 | 6 per alliance |
| `arch_lower` | Hang a RIVET on an empty lower own-ARCH hook | 6 | 4 | 2 per alliance |
| `arch_upper` | Hang a RIVET on its paired upper own-ARCH hook after lower is filled | 10 | 7 | 2 per alliance |
| `span` | Hang a RIVET on an empty central SPAN hook after own ARCH is complete | 12 | 9 | 4 shared, 2 per alliance |
| `mobility` | ROBOT crosses its starting line during AUTO | 3 | - | once per ROBOT |

An alliance's ARCH is **complete** when both lower and both upper hooks are
filled. Only then may that alliance score RIVETs on the central SPAN. A SPAN
RIVET is credited to the alliance that scores it; the four hooks are shared
and a full hook cannot be reused. This creates a real denial decision:
reaching SPAN first can close it to the other alliance, but doing so requires
spending four RIVETs on one’s own ARCH first.

## Endgame and ranking points

At ENDGAME, each alliance may raise the 60 x 42-inch **LIFT DECK** at its
alliance wall. A ROBOT earns 6 points if its bumper is fully above the deck
surface at MATCH end. If the alliance has at least one completed ARCH, that
value is 10 points instead. Up to two ROBOTS may score it; a third robot may
be fully under the raised deck and earns 4 **shelter** points. The elevated
deck is not a terrain obstacle in the first simulation version: assess it
from a robot state/zone flag at the end of the match.

| RP | Qualification condition |
| --- | --- |
| **Structure RP** | Complete the alliance's ARCH and score at least 3 DECK RIVETs. |
| **Span RP** | Score 2 RIVETs on the central SPAN. |
| **Lift RP** | Earn 20 or more total ENDGAME points. |

## StrategySim implementation brief

Register one `rivet` piece with capacity 5. Make DECK and each hook a
capacity-one scoring region; model `arch_upper_*` and `span` availability as
blocked until their prerequisite regions are full. The framework already has
`blocked_until_collected` for a collection prerequisite, so a new
game-specific `blocked_until_scored` check is the cleanest extension here.
Use unlimited DEPOT intake, 36 finite loose RIVETs, a rectangular GIRDER
obstacle, and alliance ARCH protected zones. Key experiments: compare 2- vs.
5-piece transport, route to a neutral rack versus home DEPOT, and the score
loss from attempting the full ARCH/SPAN sequence too late.

---

# 3. VECTOR VAULT

## Game idea

Alliances move a single flying **VECTOR** disc between a reliable low goal,
a long-range vault, and a limited central target. The game rewards accurate
launching but never requires it: a low-placement robot can still make a
useful alliance partner. In ENDGAME, robots decide whether to keep cycling
or use the same climbing mechanism to occupy a high-value traversal bar.

This is the best game for studying the relationship among shot reliability,
range, intake throughput, and teammate role specialization.

## Field and game pieces

**VECTOR:** an 8-inch foam disc, 1 inch thick, mass 0.18 lb. A ROBOT may
CONTROL up to **2 VECTORs**. Each ROBOT may preload 1.

| Landmark | Blue-side center / footprint | Purpose |
| --- | --- | --- |
| NEST | `(92, 158)`; 72 x 58 in | Alliance-owned low placement target |
| VAULT | `(210, 158)`; 36 x 96 in | Alliance-owned high-passive target |
| TRANSFER STATION | `(38, 58)` and `(38, 259)`; 42 x 42 in | Unlimited alliance-owned human-player supply |
| CROSSWIND POSTS | `(300, 78)` and `(390, 239)`; 28-inch diameter | Solid obstacles that create non-straight shooting lanes |
| CORE | `(345, 158)`; 64-inch-diameter hexagon | Shared center, six limited high-value ports |
| TRAVERSAL BAR | `x = 115`, `y = 158`; 84 x 18 in | Alliance endgame climbing structure |

The red NEST, VAULT, TRANSFER STATIONS, and TRAVERSAL BAR are rotated. Put
10 loose VECTORs in each alliance half, in two five-disc arcs centered at
`(175, 75)` and `(175, 242)`, plus 12 neutral VECTORs arranged around the
CORE at a radius of 60 inches. The VAULT is a passive scoring region: a
launched VECTOR that clears its 54-inch-high lower edge and comes to rest in
its net scores even when its ROBOT is elsewhere. NEST scoring is an active
deposit at carpet level. The CORE has six passive ports and a solid 44-inch
diameter base.

```text
Blue wall                                                        Red wall
| stations  NEST  VAULT      post    CORE    post      VAULT  NEST stations |
|             \------------ clear shots only through chosen lanes -----------/ |
|                 traversal bar                         traversal bar        |
```

The NEST and TRANSFER STATION have 18-inch alliance-owned protected zones.
There is no protected zone at VAULT or CORE; defenders may legally deny an
approach or shooting lane without contact violations.

## MATCH setup and scoring

A VECTOR scores only once. NEST VECTORs remain in a floor bin and are
removed at the next reset. VAULT and CORE VECTORs are caught, held, and
removed. A shot that rebounds is a live loose VECTOR. A ROBOT may not push a
VECTOR through a VAULT or CORE opening while touching the target frame; it
must release the disc before it crosses the target plane.

| Action | Criteria | AUTO | TELEOP | Capacity |
| --- | --- | ---: | ---: | --- |
| `nest` | Deposit VECTOR fully in own NEST bin | 3 | 2 | unlimited |
| `vault` | Launch VECTOR into own VAULT net | 7 | 5 | unlimited |
| `core` | Launch VECTOR through a CORE port into its catch tray | 12 | 9 | 6 shared, 3 per alliance |
| `leave` | ROBOT exits starting zone in AUTO | 3 | - | once per ROBOT |

The CORE opens at the start of TELEOP. In AUTO, its ports are covered;
attempted shots do not score and return as loose pieces. A **VECTOR BURST**
is active for 20 seconds immediately after an alliance scores its third
VAULT VECTOR in TELEOP. During its burst, its own VAULT VECTORs are worth
7 rather than 5 points. The burst can be earned once per alliance. This is
a deliberately simple, explicit state machine: `vault_teleop_count >= 3`
starts a one-shot 20-second timer, then `vault` looks up the bonus value
while that timer is active.

## Endgame and ranking points

In the final 30 seconds, a ROBOT may climb its own TRAVERSAL BAR. A ROBOT
is **latched** when its mechanism supports its full weight from the bar and
its wheels are not touching carpet. It earns 10 points. Two latched ROBOTS
on the same bar earn an additional 6-point **synchronized traversal** bonus;
the bar supports at most two robots. A robot fully within the 84 x 60-inch
area under its own bar, with no part above 24 inches, earns 3 **park** points
instead. A robot contacted while latched is credited with 10 points and the
opponent receives a 10-point TECH FOUL.

| RP | Qualification condition |
| --- | --- |
| **Velocity RP** | Score at least 18 total points in AUTO. |
| **Precision RP** | Score 5 or more VAULT VECTORs and at least 1 CORE VECTOR. |
| **Traversal RP** | Have 2 latched ROBOTS, or one latched ROBOT plus 2 parked ROBOTS. |

## StrategySim implementation brief

Register `vector` with a holding limit of 2. Build `nest` as a normal
alliance scoring region and `vault`/`core` as passive scoring regions with
shot/deposit reliability values. Use six capacity-one CORE ports, enforce a
per-alliance CORE count of 3, and schedule the CORE availability after AUTO.
The burst mechanic needs a per-alliance timer, but no new geometry primitive.
Model the posts and CORE base as obstacles, stations as unlimited intakes,
and the floor inventory as finite spawns. Initial design sweeps should vary
VAULT reliability, shot time, and climb time: a high shooter may dominate
open-field scoring but lose if its endgame commitment costs too many cycles.

---

## Source and design notes

The concepts follow the historical patterns documented in
[`saved_references/FRC_GAME_TRENDS.md`](../saved_references/FRC_GAME_TRENDS.md):
15-second AUTO plus 135-second TELEOP, more valuable AUTO scoring, tiered or
spatially distinct targets, finite/shared capacity where useful, protected
zones, a last-30-second endgame, and win plus task-based qualification RP.
The sectioning (setup, phases, scoring criteria, values, violations, and
ranking) is deliberately modeled after the 2024 manual's game-description
structure. None of the game names or rules are sourced from FIRST.
