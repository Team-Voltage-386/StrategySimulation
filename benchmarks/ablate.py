"""Undistorted breakdown of Match.step by wall clock.

cProfile inflates call-heavy Python by ~3.5x here while leaving the C
physics call alone, which is what made plan_path look dominant when
removing it entirely buys 1%. These timers are coarse (a handful of
perf_counter calls per tick), so they distort almost nothing.
"""
import sys
import time

sys.argv = ["x"]
from bench import build_jobs  # noqa: E402
from common_sim.control.behavior import BehaviorContext  # noqa: E402
from common_sim.match.match import Match  # noqa: E402
from game_specific.reefscape.sweep_trial import build_match_for_job  # noqa: E402

BUCKETS = {"controllers": 0.0, "physics": 0.0, "protection_pins": 0.0, "robot_io": 0.0, "other": 0.0}


def instrumented_step(self, dt):
    t_start = time.perf_counter()
    if self.ended:
        return
    self.elapsed += dt
    from common_sim.match.match import Phase
    if self.phase == Phase.AUTO and self.elapsed >= self.config.auto_duration:
        self.phase = Phase.TELEOP
        self.events.log(self.elapsed, "phase_change", {"phase": self.phase.value})
    if self.elapsed >= self.config.total_duration:
        self.ended = True
        self.events.log(self.elapsed, "match_end", {})

    t0 = time.perf_counter()
    BUCKETS["other"] += t0 - t_start

    for robot in self.robots:
        t1 = time.perf_counter()
        if robot.controller is not None:
            ctx = BehaviorContext(robot=robot, dt=dt, elapsed=self.elapsed, match=self)
            robot.controller.tick(ctx)
        t2 = time.perf_counter()
        BUCKETS["controllers"] += t2 - t1

        captured = robot.update_intake(dt)
        if captured is not None:
            captured.last_holder_alliance = robot.alliance
            self.events.log(self.elapsed, "intake", {"alliance": robot.alliance, "piece_type": captured.piece_type})
        target_station = robot.nearby_station()
        station_has_supply = self.station_supply.get(target_station, 1) > 0 if target_station is not None else True
        dispensed_at = robot.update_station_intake(dt, station_has_supply)
        if dispensed_at is not None:
            if dispensed_at in self.station_supply:
                self.station_supply[dispensed_at] -= 1
            color_override = {"color": dispensed_at.piece_color} if dispensed_at.piece_color is not None else {}
            piece = self.spawn_piece(dispensed_at.piece_type, robot.pose.as_tuple()[:2], source="station", **color_override)
            piece.held_by = robot
            piece.shape.sensor = True
            piece.last_holder_alliance = robot.alliance
            robot.held_pieces.append(piece)
            self.events.log(self.elapsed, "intake", {"alliance": robot.alliance, "piece_type": piece.piece_type})
        target_piece = self.deposit_piece_for(robot)
        ready_region = self.deposit_region_for(robot, target_piece)
        released = robot.update_manipulator(dt, target_piece, scoring_ready=ready_region is not None)
        if released is not None:
            self.events.log(self.elapsed, "deposit", {"alliance": robot.alliance, "piece_type": released.piece_type})
            if ready_region is not None:
                if self._roll_scoring_success(robot, released):
                    self._try_score(released, ready_region, explicit=True)
                else:
                    self.events.log(self.elapsed, "score_miss", {"alliance": robot.alliance, "piece_type": released.piece_type})
            else:
                self._check_region_scoring(released)
        robot.sync_held_piece_positions()
        BUCKETS["robot_io"] += time.perf_counter() - t2

    t3 = time.perf_counter()
    if self.config.emit_coral_to_field:
        self._step_emitters(dt)
    self.engine.step(dt)
    t4 = time.perf_counter()
    BUCKETS["physics"] += t4 - t3

    self._step_protection(dt)
    self._step_pins(dt)
    t5 = time.perf_counter()
    BUCKETS["protection_pins"] += t5 - t4

    if any(p.scored for p in self.active_pieces):
        still_active, scored = [], []
        for p in self.active_pieces:
            (scored if p.scored else still_active).append(p)
        for p in scored:
            p.remove_from_space()
        self.active_pieces = still_active
    BUCKETS["other"] += time.perf_counter() - t5


Match.step = instrumented_step

for target in ("cycle_v_cycle", "evasive_v_def"):
    for k in BUCKETS:
        BUCKETS[k] = 0.0
    job = next(job for name, job in build_jobs() if name == target)
    match, _, _ = build_match_for_job(job)
    t0 = time.perf_counter()
    while not match.ended:
        match.step(job.dt)
    wall = time.perf_counter() - t0
    print(f"\n{target}: {wall:.2f}s wall")
    for k, v in sorted(BUCKETS.items(), key=lambda kv: -kv[1]):
        print(f"  {k:<18} {v:6.2f}s  {v / wall:5.1%}")
    print(f"  {'(accounted)':<18} {sum(BUCKETS.values()):6.2f}s  {sum(BUCKETS.values()) / wall:5.1%}")
