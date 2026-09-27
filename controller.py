"""Motion state machine shared by the CLI, tests and kinematic preview."""
from __future__ import annotations

import math

from gait_core import CrawlPlanner, IKBinding, PoseSlewLimiter, ease
from robot_config import JOINTS, SafetyError, finite_number


class MotionController:
    def __init__(self, config, initial_pose="STAND"):
        self.config = config
        self.ik = IKBinding(config)
        self.planner = CrawlPlanner(config)
        self.limiter = PoseSlewLimiter(config, config.pose(initial_pose))
        self.last_pose = self.limiter.current.copy()
        self.posture = initial_pose
        self.pending_pose = None
        self.transition = None
        self.tracking_error_deg = 0.
        self.frame = self.planner.frame()

    @property
    def settled(self):
        return self.planner.standing and self.transition is None and self.pending_pose is None

    @property
    def state(self):
        if self.transition:
            return "POSE_TRANSITION"
        return self.posture if self.posture != "STAND" else self.planner.state

    def set_command(self, vx=0., vy=0., wz=0.):
        for value in (vx, vy, wz):
            finite_number(value, "command", -10, 10)
        moving = any(abs(v) > 1e-12 for v in (vx, vy, wz))
        if moving and (self.posture != "STAND" or self.transition or self.pending_pose):
            raise SafetyError("Walking requires STAND; request pose 3 and wait for the transition")
        self.planner.set_command(vx, vy, wz)

    def request_pose(self, name):
        self.planner.set_command()  # even a rejected legacy pose first requests a normal stop
        self.config.pose(name)
        self.pending_pose = name

    def update(self, dt):
        finite_number(dt, "controller dt", 1e-6, self.config.control["max_loop_gap_s"])
        if self.transition is None and self.posture == "STAND":
            self.frame = self.planner.update(dt)
            target = self.ik.solve(self.frame.feet, self.frame.body_shift)
        else:
            target = self.last_pose.copy()
        if self.pending_pose and self.planner.standing and self.transition is None:
            target_pose = self.config.pose(self.pending_pose)
            duration = self.config.control["pose_transition_s"]
            for j in JOINTS:
                # Maximum derivative of the minimum-jerk curve is 1.875.
                duration = max(duration, 1.875 * abs(target_pose[j] - self.last_pose[j]) /
                               self.config.servos[j]["max_rate_deg_s"])
            self.transition = {"from": self.last_pose.copy(), "to": target_pose,
                               "name": self.pending_pose, "duration": duration, "time": 0.}
            self.pending_pose = None
        if self.transition is not None:
            transition = self.transition
            transition["time"] = min(transition["duration"], transition["time"] + dt)
            u = ease(transition["time"] / transition["duration"])
            target = {j: transition["from"][j] + u*(transition["to"][j] - transition["from"][j]) for j in JOINTS}
            if transition["time"] >= transition["duration"] - 1e-10:
                self.posture = transition["name"]
                self.transition = None
        self.last_pose = self.limiter.update(target, dt)
        self.tracking_error_deg = max(abs(target[j] - self.last_pose[j]) for j in JOINTS)
        if self.tracking_error_deg > self.config.control["max_tracking_error_deg"]:
            raise SafetyError(f"Servo trajectory cannot follow the configured speed limits: "
                              f"{self.tracking_error_deg:.2f} degrees behind target")
        return self.last_pose.copy()
