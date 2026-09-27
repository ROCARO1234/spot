"""Deterministic crawl + calibrated IK; no ROS, terminal, clock or I2C dependency.

Foot locations are in a translating/yawing virtual body frame. Actual body
shift moves the approximate COM into the three-foot support triangle before
lift-off. This is kinematics on flat ground, not a dynamics or contact model.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from ik_fin import SpotIK, LEG_ID_ORDER
from robot_config import JOINTS, LEGS, SafetyError, finite_number


def ease(u):
    u = min(1., max(0., u))
    return u ** 3 * (10 + u * (-15 + 6 * u))


def convex_hull(points):
    points = sorted(set(tuple(map(float, p)) for p in points))
    if len(points) < 3:
        return points
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower, upper = [], []
    for p in points:
        while len(lower) > 1 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper) > 1 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def support_margin(points, com):
    """Signed distance to the closest support edge; positive means inside."""
    hull = convex_hull(points)
    if len(hull) < 3:
        return -math.inf
    distances = []
    for a, b in zip(hull, hull[1:] + hull[:1]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        distances.append((dx * (com[1] - a[1]) - dy * (com[0] - a[0])) / math.hypot(dx, dy))
    return min(distances)


def inverse_body_step(point, command, dt):
    """Exact planar SE(2) inverse motion for a constant body-frame twist."""
    vx, vy, wz = command
    angle = wz * dt
    if abs(angle) < 1e-9:
        dx, dy = vx * dt, vy * dt
    else:
        dx = (math.sin(angle) * vx - (1 - math.cos(angle)) * vy) / wz
        dy = ((1 - math.cos(angle)) * vx + math.sin(angle) * vy) / wz
    x, y = point[0] - dx, point[1] - dy
    c, s = math.cos(angle), math.sin(angle)
    return np.array([c*x + s*y, -s*x + c*y, point[2]])


def foot_velocity(point, command):
    vx, vy, wz = command
    return np.array([vx - wz * point[1], vy + wz * point[0], 0.])


@dataclass
class Frame:
    feet: dict
    body_shift: np.ndarray
    swing_legs: tuple
    state: str
    effective_velocity: np.ndarray
    support_margin_m: float


class IKBinding:
    """Preserve the supplied STAND, apply joint-angle DELTAS, then validate.

    Offsets are never computed from an already clipped 0..180 conversion.
    Geometry and servo directions are independent and explicitly configured.
    """
    def __init__(self, config):
        self.config = config
        g = config.geometry
        self.kin = SpotIK(g["body_length_m"], g["body_width_m"], g["hip_link_m"],
                          g["upper_link_m"], g["lower_link_m"])
        self.ids = {leg: LEG_ID_ORDER.index(leg) for leg in LEGS}
        self.stand = config.pose("STAND")
        self.reference = {leg: np.asarray(self.kin.leg_ik(config.nominal[leg] - config.hips[leg],
                                                         legID=self.ids[leg])[:3]) for leg in LEGS}

    def solve(self, feet, body_shift=None):
        if set(feet) != set(LEGS):
            raise SafetyError("All four foot targets are required")
        shift = np.zeros(3) if body_shift is None else np.asarray(body_shift, dtype=float)
        if shift.shape != (3,) or not np.all(np.isfinite(shift)):
            raise SafetyError("Invalid body shift")
        pose = {}
        for leg in LEGS:
            point = np.asarray(feet[leg], dtype=float)
            if point.shape != (3,) or not np.all(np.isfinite(point)):
                raise SafetyError(f"Invalid {leg} foot target")
            try:
                angles = np.asarray(self.kin.leg_ik(point - self.config.hips[leg] - shift,
                                                   legID=self.ids[leg])[:3])
            except ValueError as exc:
                raise SafetyError(f"{leg}: {exc}") from exc
            delta = (angles - self.reference[leg] + math.pi) % (2*math.pi) - math.pi
            for i, part in enumerate(("sh", "up", "lo")):
                name = f"{leg}_{part}"
                pose[name] = self.stand[name] + self.config.servos[name]["direction"] * math.degrees(delta[i])
        return self.config.validate_pose(pose)

    def forward(self, pose, body_shift=None):
        pose = self.config.validate_pose(pose)
        shift = np.zeros(3) if body_shift is None else np.asarray(body_shift, dtype=float)
        result = {}
        for leg in LEGS:
            angles = self.reference[leg].copy()
            for i, part in enumerate(("sh", "up", "lo")):
                name = f"{leg}_{part}"
                angles[i] += math.radians((pose[name] - self.stand[name]) / self.config.servos[name]["direction"])
            result[leg] = np.asarray(self.kin.leg_fk(angles, self.ids[leg])) + self.config.hips[leg] + shift
        return result


class PoseSlewLimiter:
    def __init__(self, config, initial_pose):
        self.config = config
        self.current = config.validate_pose(initial_pose)

    def update(self, target, dt):
        finite_number(dt, "control dt", 1e-6, self.config.control["max_loop_gap_s"])
        target = self.config.validate_pose(target)  # validate WHOLE frame first
        scale = 1.0
        for joint in JOINTS:
            delta = abs(target[joint] - self.current[joint])
            if delta > 1e-12:
                scale = min(scale, self.config.servos[joint]["max_rate_deg_s"] * dt / delta)
        self.current = {j: self.current[j] + scale * (target[j] - self.current[j]) for j in JOINTS}
        return self.current.copy()


class CrawlPlanner:
    def __init__(self, config):
        self.config = config
        self.settings = config.gait
        self.order = tuple(self.settings["order"])
        self.feet = {leg: p.copy() for leg, p in config.nominal.items()}
        self.body_shift = np.zeros(3)
        self.requested = np.zeros(3)
        self.velocity = np.zeros(3)
        self.state = "STAND"
        self.swing = None
        self.next_leg = None
        self.index = 0
        self.t = 0.
        self.homing = False
        self.start_shift = np.zeros(3)
        self.cycle_time = 4 * (self.settings["transfer_time_s"] + self.settings["swing_time_s"])
        self.stance_time = self.cycle_time - self.settings["swing_time_s"]
        self.lift_count = 0
        margin = support_margin([p[:2] for p in self.feet.values()], self.settings["com_offset_xy_m"])
        if margin < self.settings["support_margin_m"]:
            raise SafetyError("Nominal COM projection is outside the four-foot support area")

    @property
    def standing(self):
        return self.state == "STAND" and not np.any(self.requested) and np.linalg.norm(self.velocity) < 1e-10

    def set_command(self, vx=0., vy=0., wz=0.):
        command = np.array([finite_number(v, "velocity", -10, 10) for v in (vx, vy, wz)])
        scale = 1.
        planar = np.linalg.norm(command[:2])
        if planar > 0:
            scale = min(scale, self.settings["max_translation_mps"] / planar)
        if abs(command[2]) > 0:
            scale = min(scale, self.settings["max_yaw_rps"] / abs(command[2]))
        # Scale the whole twist, preserving its direction and turn radius.
        # Radius allowance covers small offsets from nominal stance.
        max_foot_speed = max(np.linalg.norm(foot_velocity(p, command)) for p in self.config.nominal.values())
        max_foot_speed += abs(command[2]) * self.settings["max_stride_m"]
        if max_foot_speed > 0:
            scale = min(scale, self.settings["max_stride_m"] / self.stance_time / max_foot_speed)
        self.requested = command * scale

    def _moving(self):
        return np.linalg.norm(self.requested) > 1e-9 or np.linalg.norm(self.velocity) > 1e-7

    def _choose_next(self):
        self.swing = None
        self.t = 0.
        self.start_shift = self.body_shift.copy()
        if self._moving():
            self.next_leg = self.order[self.index]
            self.index = (self.index + 1) % 4
            self.homing = False
        else:
            self.next_leg = next((leg for leg in self.order if
                np.linalg.norm(self.feet[leg] - self.config.nominal[leg]) > self.settings["settle_tolerance_m"]), None)
            self.homing = True
        self.state = "TRANSFER" if self.next_leg else "CENTER"

    def _support_center(self, excluded):
        points = np.array([p[:2] for leg, p in self.feet.items() if leg != excluded])
        target = points.mean(axis=0) - np.array(self.settings["com_offset_xy_m"])
        if np.linalg.norm(target) > self.settings["max_body_shift_m"]:
            raise SafetyError("Required body shift exceeds configured workspace")
        return np.array([target[0], target[1], 0.])

    def _start_swing(self):
        leg = self.next_leg
        com = self.body_shift[:2] + np.array(self.settings["com_offset_xy_m"])
        margin = support_margin([p[:2] for name, p in self.feet.items() if name != leg], com)
        if margin < self.settings["support_margin_m"]:
            raise SafetyError(f"Not enough support margin to lift {leg}: {margin:.5f} m")
        self.swing = leg
        self.p0 = self.feet[leg].copy()
        self.p1 = self.config.nominal[leg].copy()
        if not self.homing:
            self.p1 += .5 * self.stance_time * foot_velocity(self.p1, self.velocity)
        delta = self.p1[:2] - self.p0[:2]
        distance = np.linalg.norm(delta)
        if distance > self.settings["max_stride_m"]:
            self.p1[:2] = self.p0[:2] + delta * (self.settings["max_stride_m"] / distance)
        self.v0 = -foot_velocity(self.p0, self.velocity)
        self.v1 = -foot_velocity(self.p1, self.velocity)
        self.t = 0.
        self.state = "SWING"
        self.lift_count += 1

    def _swing_point(self, u):
        # Quintic Hermite: match stance velocities at lift-off and touchdown.
        h1 = ease(u)
        h2 = u - 6*u**3 + 8*u**4 - 3*u**5
        h3 = -4*u**3 + 7*u**4 - 3*u**5
        duration = self.settings["swing_time_s"]
        point = (1-h1)*self.p0 + h1*self.p1 + duration*(h2*self.v0 + h3*self.v1)
        point[2] += self.settings["lift_m"] * 64*u**3*(1-u)**3
        return point

    def update(self, dt):
        finite_number(dt, "planner dt", 1e-6, self.config.control["max_loop_gap_s"])
        remaining = dt
        while remaining > 1e-10:
            if self.state == "STAND":
                if not self._moving():
                    break
                self._choose_next()
            if self.state == "CENTER" and self._moving():
                self._choose_next()
            duration = self.settings["swing_time_s"] if self.state == "SWING" else self.settings["transfer_time_s"]
            h = min(remaining, .02, duration - self.t)
            if h <= 1e-10:
                self._finish_stage()
                continue
            delta = self.requested[:2] - self.velocity[:2]
            length = np.linalg.norm(delta)
            limit = self.settings["translation_accel_mps2"] * h
            self.velocity[:2] += delta if length <= limit else delta * limit / length
            yaw_delta = self.requested[2] - self.velocity[2]
            yaw_limit = self.settings["yaw_accel_rps2"] * h
            self.velocity[2] += min(yaw_limit, max(-yaw_limit, yaw_delta))
            for leg in LEGS:
                if leg != self.swing:
                    self.feet[leg] = inverse_body_step(self.feet[leg], self.velocity, h)
            self.t += h
            u = min(1., self.t / duration)
            if self.state == "TRANSFER":
                target = self._support_center(self.next_leg)
                self.body_shift = self.start_shift + ease(u) * (target - self.start_shift)
            elif self.state == "SWING":
                self.feet[self.swing] = self._swing_point(u)
                self.body_shift = self._support_center(self.swing)
            else:
                self.body_shift = self.start_shift * (1 - ease(u))
            remaining -= h
            if self.t >= duration - 1e-10:
                self._finish_stage()
        return self.frame()

    def _finish_stage(self):
        if self.state == "TRANSFER":
            if not self._moving() and not self.homing:
                self._choose_next()  # do not initiate a new walking step after STOP
            else:
                self.homing = not self._moving()
                self._start_swing()
        elif self.state == "SWING":
            self.feet[self.swing] = self.p1.copy()
            self._choose_next()
        else:
            self.body_shift[:] = 0.
            self.velocity[:] = 0.
            self.state = "STAND"
            self.t = 0.

    def frame(self):
        com = self.body_shift[:2] + np.asarray(self.settings["com_offset_xy_m"])
        margin = support_margin([p[:2] for leg, p in self.feet.items() if leg != self.swing], com)
        return Frame({leg: p.copy() for leg, p in self.feet.items()}, self.body_shift.copy(),
                     () if self.swing is None else (self.swing,), self.state, self.velocity.copy(), margin)
