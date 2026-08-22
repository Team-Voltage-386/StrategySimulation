"""
REEFSCAPE point values, from the 2025 Game Manual (V13), Table 6-2.
BARGE/CAGE (endgame climb) and the AUTO LEAVE mobility bonus are not
included -- see game_specific/reefscape/field.py's module docstring for
why those two aren't modeled by this first pass.
"""
from __future__ import annotations

from common_sim.match.scoring import TableScoringRules

REEFSCAPE_SCORING_RULES = TableScoringRules({
    ("l1", "auto"): 3.0, ("l1", "teleop"): 2.0,
    ("l2", "auto"): 4.0, ("l2", "teleop"): 3.0,
    ("l3", "auto"): 6.0, ("l3", "teleop"): 4.0,
    ("l4", "auto"): 7.0, ("l4", "teleop"): 5.0,
    ("processor", "auto"): 6.0, ("processor", "teleop"): 6.0,
    ("net", "auto"): 4.0, ("net", "teleop"): 4.0,
})

# How often each scoring action lands, as a multiplier on the piece
# type's own reliability above. Ordered the way DEFAULT_DEPOSIT_TIMES is:
# the harder a target is to hit, the longer the attempt takes AND the
# more often it misses. NET is a lob into the BARGE and the least certain
# thing a robot does; L1 is a trough.
#
# Illustrative rather than measured from real event data -- they are
# defaults for the benches, and every robot can override them.
DEFAULT_SCORING_RELIABILITY_BY_ACTION = {
    "l1": 0.98, "l2": 0.95, "l3": 0.90, "l4": 0.82,
    "processor": 0.95, "net": 0.75,
}

