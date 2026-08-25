"""Step 4, first half: read the live REBUILT world off NetworkTables.

The strategy layer needs to know what is on the field. It could be told
by a REBUILT implementation in `game_specific/` -- or it could be told by
the simulator that is already running the game on the other side of the
wire. maple-sim publishes fuel positions, the HUB clock, and its own
running score, without a line of robot-side code. This module reads them.

That is why step 4 does not wait on REBUILT being implemented here. A
second implementation would not be a shortcut skipped, it would be a
model to keep in agreement with the one that actually decides what
happens -- and a bridge whose two halves disagree about the score reports
failures that are really disagreements.

Static geometry, which nobody publishes, is transcribed in `arena.py`.
`check_geometry` reconciles the two: maple-sim *does* publish the HUB and
OUTPOST poses, so the transcription can be checked against the running
arena on every connection instead of trusted.

Units are the wire's: metres, radians, blue-origin. The conversion into
sparky-sim's inches happens in the adapter that consumes this, so there
is exactly one place per module where units change.
"""
from __future__ import annotations

from dataclasses import dataclass

from bridge import arena
from bridge.robot_state import BALL_COUNT, FUEL_POSES, POSE_TRUTH, Pose2d, Pose3d, RobotStateLink

# -- maple-sim's own NT surface ----------------------------------------
# Published by SimulatedArena and Arena2026Rebuilt directly, not through
# AdvantageKit, so these are plain SmartDashboard keys and they exist
# whatever the robot code does.
#
# The capitalisation is maple-sim's and is not consistent -- "Red Alliance"
# but "blue Alliance". Do not tidy it; it is what is on the wire.
MAPLE = "/SmartDashboard/MapleSim"
MATCH_DATA = f"{MAPLE}/MatchData/Breakdown"
ALLIANCE_TABLE = {"blue": f"{MATCH_DATA}/blue Alliance", "red": f"{MATCH_DATA}/Red Alliance"}

MATCH_CLOCK = f"{MATCH_DATA}/Match Clock"

# Seconds until the active HUB swaps. Arena2026Rebuilt.simulationSubTick
# counts this down from 25 whenever the robot is enabled and out of
# autonomous, and holds it at 25 otherwise.
PHASE_CLOCK = f"{MATCH_DATA}/Time left in current phase"

# Which HUB is currently accepting. Exactly one is true outside
# autonomous; both are true during it.
HUB_ACTIVE = {
    "blue": f"{ALLIANCE_TABLE['blue']}/Blue is active",
    "red": f"{ALLIANCE_TABLE['red']}/Red is active",
}

# maple-sim adjudicates REBUILT and publishes the result. This is the
# scoring model -- there is no reason to write a second one here, and a
# strong reason not to. It is also what oracle 04 (differential scoring)
# will eventually compare against.
GOALS = f"{MAPLE}/Goals"
HUB_POSE = {"blue": f"{GOALS}/BlueHub", "red": f"{GOALS}/RedHub"}
OUTPOST_DUMP_POSE = {"blue": f"{GOALS}/BlueOutpostDump", "red": f"{GOALS}/RedOutpostDump"}


def score_key(alliance: str, name: str) -> str:
    """A per-alliance MatchData value: TotalScore, TeleopScore,
    Auto/AutoScore, TotalFuelInHub, TotalFuelInOutpost,
    CurrentFuelInOutpost, WastedFuel."""
    return f"{ALLIANCE_TABLE[alliance]}/{name}"


@dataclass(frozen=True)
class WorldState:
    """One instant of the field, as maple-sim sees it. Metres.

    Deliberately a snapshot rather than a live view. The strategy layer
    reads the same quantity several times while making one decision, and
    a view that re-read NT each time could have a piece move between two
    of those reads and produce a choice that was never consistent with
    any actual state of the field.
    """

    robot: Pose2d | None
    fuel: tuple[Pose3d, ...]
    held: int
    match_clock: float
    phase_clock: float
    hub_active: dict[str, bool]
    score: dict[str, float]

    @property
    def fuel_count(self) -> int:
        return len(self.fuel)

    def our_hub_active(self, alliance: str) -> bool:
        return self.hub_active.get(alliance, False)

    def nearest_fuel(self, to: Pose2d | None = None) -> Pose3d | None:
        origin = to or self.robot
        if origin is None or not self.fuel:
            return None
        return min(self.fuel, key=lambda p: (p.x - origin.x) ** 2 + (p.y - origin.y) ** 2)


class WorldStateReader:
    """Reads `WorldState` snapshots from a connected `RobotStateLink`.

    Holds no state of its own beyond the link, so it is safe to build one
    per match and throw it away with the JVM.
    """

    def __init__(self, link: RobotStateLink, alliance: str = "blue"):
        self.link = link
        self.alliance = alliance

    def read(self) -> WorldState:
        return WorldState(
            robot=self.link.pose(POSE_TRUTH),
            fuel=tuple(self.link.pose3d_array(FUEL_POSES) or ()),
            held=self.link.integer(BALL_COUNT),
            match_clock=self.link.number(MATCH_CLOCK),
            phase_clock=self.link.number(PHASE_CLOCK),
            hub_active={side: self.link.boolean(key) for side, key in HUB_ACTIVE.items()},
            score={
                side: self.link.number(score_key(side, "TotalScore"))
                for side in ALLIANCE_TABLE
            },
        )

    def fuel_in_hub(self, alliance: str) -> float:
        return self.link.number(score_key(alliance, "TotalFuelInHub"))

    def wasted_fuel(self, alliance: str) -> float:
        """FUEL that went somewhere it scored nothing.

        A shot that misses is not a failure of the robot *code*, so this
        is not oracle material on its own. It is the differential signal
        for a scoring oracle later: a change that makes the robot waste
        more fuel for the same score is a regression that no liveness
        detector would ever notice.
        """
        return self.link.number(score_key(alliance, "WastedFuel"))


@dataclass(frozen=True)
class GeometryCheck:
    """One transcribed constant weighed against what the arena publishes."""

    what: str
    expected: tuple[float, float]  # metres, from arena.py
    published: tuple[float, float] | None  # metres, from NT
    error: float | None  # metres

    @property
    def ok(self) -> bool:
        return self.error is not None and self.error <= GEOMETRY_TOLERANCE_M

    def __str__(self) -> str:
        if self.published is None:
            return f"{self.what}: not published -- cannot check"
        mark = "ok" if self.ok else "MISMATCH"
        return (
            f"{self.what}: transcribed ({self.expected[0]:.4f}, {self.expected[1]:.4f}) "
            f"vs published ({self.published[0]:.4f}, {self.published[1]:.4f})  "
            f"{self.error * 1000:.1f} mm  [{mark}]"
        )


# A millimetre is far tighter than anything downstream needs -- the
# navigator would not notice a centimetre. It is set this tight because
# the check is not asking "is the geometry good enough", it is asking
# "did I transcribe the same field". Those want different tolerances, and
# a loose one here would pass a constant copied from the wrong season.
GEOMETRY_TOLERANCE_M = 0.001


def check_geometry(link: RobotStateLink) -> list[GeometryCheck]:
    """Weigh `arena.py`'s transcribed constants against the live arena.

    maple-sim publishes the HUB and OUTPOST poses it is actually using,
    so the numbers copied out of its source can be verified against the
    running simulation rather than trusted. Worth doing on every
    connection: the failure this catches is a maple-sim upgrade moving
    the field under a transcription that still looks perfectly reasonable,
    and the symptom would be a navigator quietly routing around empty
    floor.

    It cannot check everything. The obstacle sizes -- including the
    47 x 217 inch HUB collider that decides which gaps a robot fits
    through -- are not published anywhere, so those stay trusted. What
    this establishes is that the arena is the one these constants were
    read from, which is the assumption the rest of them rest on.
    """
    checks = []
    for alliance, key in HUB_POSE.items():
        checks.append(_compare(f"{alliance} HUB centre", arena.HUB_CENTRE_M[alliance], link.pose3d(key)))
    for alliance, key in OUTPOST_DUMP_POSE.items():
        checks.append(_compare(f"{alliance} OUTPOST dump", arena.OUTPOST_DUMP_M[alliance], link.pose3d(key)))
    return checks


def _compare(what: str, expected: tuple[float, float], published: Pose3d | None) -> GeometryCheck:
    if published is None:
        return GeometryCheck(what, expected, None, None)
    got = (published.x, published.y)
    error = ((got[0] - expected[0]) ** 2 + (got[1] - expected[1]) ** 2) ** 0.5
    return GeometryCheck(what, expected, got, error)
