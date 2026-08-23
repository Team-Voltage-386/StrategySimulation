"""
REEFSCAPE point values, from the 2025 Game Manual (V13), Table 6-2.
BARGE/CAGE (endgame climb) and the AUTO LEAVE mobility bonus are not
included -- see game_specific/reefscape/field.py's module docstring for
why those two aren't modeled by this first pass.
"""
from __future__ import annotations

from common_sim.field.field_config import SecondaryAward
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

# Table 6-2 pays 6 for PROCESSOR against 4 for NET, so on point value
# alone the sim prefers PROCESSOR unconditionally. But a scored PROCESSOR
# ALGAE rolls through to the *opponent's* PROCESSOR AREA, where their
# human player can put it in their own NET for 4 -- so the real swing is
# (6 - 4p) against a flat 4, where p is how often the opponent actually
# converts the gift. See field_config.SecondaryAward.
#
# PROCESSOR_CONVERSION_PROBABILITY was a deliberate 0.5 (the break-even
# point) until this was measured: score_breakdown's processor counts
# against netAlgaeCount across a season's matches puts the real rate
# above 90%. That's high enough to just treat as certain rather than
# model as a coin flip -- 1.0 here, giving a flat effective swing of
# 6 - 4 = +2 for PROCESSOR over NET, rather than carrying a probability
# distribution for a few points of precision nobody's asked for yet.
# Revisit with the measured rate itself if that precision starts to
# matter (e.g. once driver_skill or a margin-aware planner cares about
# the last 10%).
# PROCESSOR_GIFT_DELAY is a dry-run placeholder for how long the round
# trip through the opponent's PROCESSOR AREA and human player takes.
PROCESSOR_CONVERSION_PROBABILITY = 1.0
PROCESSOR_GIFT_DELAY = 5.0

PROCESSOR_TO_OPPONENT_NET_AWARD = SecondaryAward(
    action="processor",
    alliance_of="opponent",
    award_action="net",
    probability=PROCESSOR_CONVERSION_PROBABILITY,
    delay=PROCESSOR_GIFT_DELAY,
)

