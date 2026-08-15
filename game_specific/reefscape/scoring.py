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
