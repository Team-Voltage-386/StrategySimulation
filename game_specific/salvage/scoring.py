"""
SALVAGE 2027 point values.

Three things here are deliberately unlike REEFSCAPE's table, because
REEFSCAPE's shape lets a planner get the right answer for the wrong
reason:

1. **HOLD_HIGH is worth more in TELEOP than in AUTO.** Every REEFSCAPE
   action is worth the same or *more* in AUTO, so nothing has ever
   checked that a planner reads the phase rather than assuming the early
   game is the valuable one.
2. **The value ordering changes with the phase.** In AUTO, REACTOR (10)
   > BEACON (5) > HOLD_LOW (4) > HOLD_HIGH (3); in TELEOP, HOLD_HIGH (8)
   > REACTOR (7) > BEACON (5) > HOLD_LOW (2). A robot that picks a
   target once and keeps it is playing the wrong game after t=15s.
3. **The cheap target is not the near one.** In REEFSCAPE, L1..L4 sit on
   the same physical face, so value and travel are independent axes and
   choosing a level is a pure points/reliability question. Here each
   hold is somewhere else on the field, so value, reliability, deposit
   time and travel are all coupled -- which is the case the
   points-per-second currency was written for and has never actually
   been given.
"""
from __future__ import annotations

from common_sim.match.scoring import TableScoringRules

SALVAGE_SCORING_RULES = TableScoringRules({
    # near the ALLIANCE WALL, quick, and worth less once AUTO ends
    ("hold_low", "auto"): 4.0, ("hold_low", "teleop"): 2.0,
    # deep in the field, slow, unreliable -- and the big TELEOP payer
    ("hold_high", "auto"): 3.0, ("hold_high", "teleop"): 8.0,
    # CELLs only, and CELLs only come from the contested neutral depot
    ("reactor", "auto"): 10.0, ("reactor", "teleop"): 7.0,
    # a lob: no robot has to be in position when the SCRAP lands
    ("airlock", "auto"): 2.0, ("airlock", "teleop"): 3.0,
    # neutral, shared capacity -- first alliance there takes the slots
    ("beacon", "auto"): 5.0, ("beacon", "teleop"): 5.0,
})

# Per-action reliability multiplier, same contract as REEFSCAPE's:
# harder target, longer attempt, more misses. AIRLOCK is the lob and the
# least certain thing this robot does; HOLD_LOW is a chute.
DEFAULT_SCORING_RELIABILITY_BY_ACTION = {
    "hold_low": 0.97,
    "hold_high": 0.78,
    "reactor": 0.88,
    "airlock": 0.70,
    "beacon": 0.92,
}
