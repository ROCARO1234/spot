#!/usr/bin/env python3
"""
SpotMicro inverse kinematics toolkit (final version).

This module bundles:
  * the full 3DOF leg IK solver (`SpotIK` class)
  * stance presets in world coordinates, converted to leg-local frames
  * helpers that return joint angles or 0..180 degree servo commands

Everything is self-contained so a gait AI can import this file and immediately
compute motor targets for any of the reference poses or for custom foot targets.
"""

from __future__ import annotations

import numpy as np
from numpy import array, asarray, matrix
from numpy.linalg import inv, norm
from math import sin, cos, asin, acos, atan2, pi, radians, isfinite


class IKError(ValueError):
    """An invalid, singular or unreachable target; never silently projected."""


def _vector3(value, label):
    out = np.asarray(value, dtype=float)
    if out.shape != (3,) or not np.all(np.isfinite(out)):
        raise IKError(f"{label} must contain three finite numbers")
    return out


# ---------------------------------------------------------------------------
# Low-level math helpers
# ---------------------------------------------------------------------------

def RotMatrix3D(rot, is_radians=True):
    """Return the ZYX rotation matrix for roll, pitch, yaw."""
    if len(rot) != 3:
        raise ValueError("rot must contain roll, pitch, yaw.")

    roll, pitch, yaw = rot
    if not is_radians:
        roll, pitch, yaw = tuple(radians(v) for v in rot)

    sr, cr = sin(roll), cos(roll)
    sp, cp = sin(pitch), cos(pitch)
    sy, cy = sin(yaw), cos(yaw)

    rx = matrix([[1.0, 0.0, 0.0],
                 [0.0, cr, -sr],
                 [0.0, sr, cr]])
    ry = matrix([[cp, 0.0, sp],
                 [0.0, 1.0, 0.0],
                 [-sp, 0.0, cp]])
    rz = matrix([[cy, -sy, 0.0],
                 [sy, cy, 0.0],
                 [0.0, 0.0, 1.0]])
    return rz * ry * rx


def point_to_rad(x, y):
    """Return angle (0..2pi) between +X axis and vector (x, y)."""
    theta = atan2(y, x)
    if theta < 0:
        theta += 2 * pi
    return theta


# ---------------------------------------------------------------------------
# Robot geometry + stance definitions
# ---------------------------------------------------------------------------

BODY_LENGTH = 0.25205  # meters
BODY_WIDTH = 0.105577

HIP_WORLD = {
    "FL": np.array([BODY_LENGTH / 2, BODY_WIDTH / 2, 0.0]),
    "RL": np.array([-BODY_LENGTH / 2, BODY_WIDTH / 2, 0.0]),
    "RR": np.array([-BODY_LENGTH / 2, -BODY_WIDTH / 2, 0.0]),
    "FR": np.array([BODY_LENGTH / 2, -BODY_WIDTH / 2, 0.0]),
}

# Matches `SpotIK.leg_origins`: 0=FL, 1=RL, 2=RR, 3=FR.
LEG_ID_ORDER = ("FL", "RL", "RR", "FR")
DISPLAY_LEG_ORDER = ("FL", "FR", "RL", "RR")

SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 180.0
SERVO_CENTER_DEG = 90.0
SERVO_DIRECTION = {
    "FL": np.array([1.0, 1.0, 1.0]),
    "FR": np.array([1.0, 1.0, -1.0]),
    "RL": np.array([1.0, 1.0, 1.0]),
    "RR": np.array([1.0, 1.0, -1.0]),
}


def _stance_from_world(world_targets):
    """Convert stance dict with world coords -> leg-local coordinates."""
    stance = {}
    for leg in LEG_ID_ORDER:
        stance[leg] = np.array(world_targets[leg]) - HIP_WORLD[leg]
    return stance


STANCE_LIBRARY = {
    "stand_neutral": _stance_from_world({
        "FL": np.array([0.140, 0.090, -0.180]),
        "FR": np.array([0.140, -0.090, -0.180]),
        "RL": np.array([-0.165, 0.090, -0.180]),
        "RR": np.array([-0.165, -0.090, -0.180]),
    }),
    "stand_low": _stance_from_world({
        "FL": np.array([0.140, 0.090, -0.150]),
        "FR": np.array([0.140, -0.090, -0.150]),
        "RL": np.array([-0.165, 0.090, -0.150]),
        "RR": np.array([-0.165, -0.090, -0.150]),
    }),
    "stand_high": _stance_from_world({
        "FL": np.array([0.140, 0.090, -0.210]),
        "FR": np.array([0.140, -0.090, -0.210]),
        "RL": np.array([-0.165, 0.090, -0.210]),
        "RR": np.array([-0.165, -0.090, -0.210]),
    }),
    "shift_forward": _stance_from_world({
        "FL": np.array([0.160, 0.090, -0.180]),
        "FR": np.array([0.160, -0.090, -0.180]),
        "RL": np.array([-0.140, 0.090, -0.180]),
        "RR": np.array([-0.140, -0.090, -0.180]),
    }),
    "shift_backward": _stance_from_world({
        "FL": np.array([0.120, 0.090, -0.180]),
        "FR": np.array([0.120, -0.090, -0.180]),
        "RL": np.array([-0.185, 0.090, -0.180]),
        "RR": np.array([-0.185, -0.090, -0.180]),
    }),
    "step_ready_front_left": _stance_from_world({
        "FL": np.array([0.140, 0.105, -0.120]),
        "FR": np.array([0.155, -0.075, -0.185]),
        "RL": np.array([-0.155, 0.085, -0.185]),
        "RR": np.array([-0.170, -0.095, -0.180]),
    }),
    "step_ready_front_right": _stance_from_world({
        "FL": np.array([0.155, 0.075, -0.185]),
        "FR": np.array([0.140, -0.105, -0.120]),
        "RL": np.array([-0.170, 0.095, -0.180]),
        "RR": np.array([-0.155, -0.085, -0.185]),
    }),
}

DEFAULT_STANCE = "stand_neutral"


def stance_as_array(name: str):
    """Return stance targets ordered for SpotIK.leg IDs."""
    try:
        stance = STANCE_LIBRARY[name]
    except KeyError as exc:
        raise KeyError(f"Unknown stance '{name}'. Available: {list(STANCE_LIBRARY)}") from exc
    return [stance[leg].copy() for leg in LEG_ID_ORDER]


# ---------------------------------------------------------------------------
# Core IK solver
# ---------------------------------------------------------------------------

class SpotIK:
    """Full 3DOF SpotMicro leg inverse kinematics engine."""

    def __init__(self, body_length=BODY_LENGTH, body_width=BODY_WIDTH,
                 hip_link=0.045, upper_link=0.1115, lower_link=0.155):
        for value in (body_length, body_width, hip_link, upper_link, lower_link):
            if not isfinite(value) or value <= 0:
                raise IKError("Link lengths and body dimensions must be positive and finite")
        self.right_legs = [2, 3]  # leg IDs RR, FR
        self.link_1 = hip_link
        self.link_2 = upper_link
        self.link_3 = lower_link
        self.phi = radians(90)
        self.length = body_length
        self.width = body_width
        self.hight = 0.0
        self.leg_origins = np.matrix([
            [self.length / 2, self.width / 2, 0.0],
            [-self.length / 2, self.width / 2, 0.0],
            [-self.length / 2, -self.width / 2, 0.0],
            [self.length / 2, -self.width / 2, 0.0],
            [self.length / 2, self.width / 2, 0.0],
        ])

    # Public API -------------------------------------------------------------
    def leg_ik(self, xyz, rot=None, legID=0, is_radians=True, center_offset=None):
        """Solve a HIP-LOCAL target in the neutral body's axes, in metres.

        With a body rotation, transform the neutral foot about center_offset
        before solving. The first three returned angles use the original
        runner's corrected joint convention, in radians. The remaining four
        points are the hip, shoulder, knee and foot in the leg-local frame.
        """
        if isinstance(legID, bool) or not isinstance(legID, (int, np.integer)) or not 0 <= legID < 4:
            raise IKError("legID must be 0=FL, 1=RL, 2=RR or 3=FR")
        if rot is None:
            rot = [0, 0, 0]
        if center_offset is None:
            center_offset = [0, 0, 0]

        is_right = legID in self.right_legs
        leg_origin = array(self.leg_origins[legID, :]).flatten()
        xyz_vec = _vector3(xyz, "foot target")
        center = _vector3(center_offset, "rotation centre")
        rot = _vector3(rot, "rotation")

        rotated = inv(RotMatrix3D(rot, is_radians)) * np.reshape(
            xyz_vec + leg_origin - center,
            (3, 1),
        )
        XYZ = asarray(rotated.transpose())
        xyz_ = asarray(XYZ - leg_origin + center).flatten()

        return self._leg_ik_calc(xyz_, is_right)

    def _leg_ik_calc(self, xyz, is_right=False):
        xyz = _vector3(xyz, "foot target")
        x, y, z = xyz

        len_A = norm([0, y, z])
        min_len_A = abs(sin(self.phi) * self.link_1) + 1e-9
        if len_A <= min_len_A:
            raise IKError("Target is inside or on the hip-link singular cylinder")

        a_1 = point_to_rad(y, z)
        ratio_a2 = sin(self.phi) * self.link_1 / len_A
        ratio_a2 = float(np.clip(ratio_a2, -1.0, 1.0))
        a_2 = asin(ratio_a2)
        a_3 = pi - a_2 - self.phi

        theta_1 = a_1 - a_3 if is_right else a_1 + a_3
        if not is_right and theta_1 >= 2 * pi:
            theta_1 -= 2 * pi

        j2 = array([0.0, self.link_1 * cos(theta_1), self.link_1 * sin(theta_1)])
        j4 = array(xyz)
        j4_2_vec = j4 - j2

        R = theta_1 - self.phi - pi / 2 if is_right else theta_1 + self.phi - pi / 2
        rot_mtx = RotMatrix3D([-R, 0, 0], is_radians=True)
        j4_2_vec_ = rot_mtx * np.reshape(j4_2_vec, (3, 1))
        x_, _, z_ = (j4_2_vec_[0, 0], j4_2_vec_[1, 0], j4_2_vec_[2, 0])

        len_B = norm([x_, z_])
        max_len_B = self.link_2 + self.link_3 - 1e-8
        min_len_B = abs(self.link_2 - self.link_3) + 1e-8
        if not min_len_B < len_B < max_len_B:
            raise IKError(f"Unreachable/singular target: planar reach {len_B:.6f} m; "
                          f"allowed ({min_len_B:.6f}, {max_len_B:.6f}) m")

        b2_num = self.link_2 ** 2 + len_B ** 2 - self.link_3 ** 2
        b2_den = 2 * self.link_2 * len_B
        b3_num = self.link_2 ** 2 + self.link_3 ** 2 - len_B ** 2
        b3_den = 2 * self.link_2 * self.link_3
        b_1 = point_to_rad(x_, z_)
        b_2 = acos(np.clip(b2_num / b2_den, -1.0, 1.0))
        b_3 = acos(np.clip(b3_num / b3_den, -1.0, 1.0))

        theta_2 = b_1 - b_2
        theta_3 = pi - b_3

        j1 = np.array([0.0, 0.0, 0.0])
        j3_ = np.reshape(
            np.array([self.link_2 * cos(theta_2), 0.0, self.link_2 * sin(theta_2)]),
            (3, 1),
        )
        j3 = np.asarray(j2 + np.reshape(inv(rot_mtx) * j3_, (1, 3))).flatten()

        j4_ = j3_ + np.reshape(
            np.array([
                self.link_3 * cos(theta_2 + theta_3),
                0.0,
                self.link_3 * sin(theta_2 + theta_3),
            ]),
            (3, 1),
        )
        j4 = np.asarray(j2 + np.reshape(inv(rot_mtx) * j4_, (1, 3))).flatten()

        angles = self._angle_corrector([theta_1, theta_2, theta_3], is_right=is_right)
        if norm(j4 - xyz) > 1e-7:
            raise IKError("IK reconstruction does not match the requested target")
        return [angles[0], angles[1], angles[2], j1, j2, j3, j4]

    def leg_fk(self, angles, legID=0):
        """Independent forward kinematics for the corrected joint convention.

        Returns four HIP-LOCAL joint points. This is used to visualize the
        angles actually sent after slew limiting, not ideal planner targets.
        """
        if isinstance(legID, bool) or not isinstance(legID, (int, np.integer)) or not 0 <= legID < 4:
            raise IKError("Invalid leg ID")
        q1, q2, q3 = _vector3(angles, "joint angles")
        right = legID in self.right_legs
        t1 = q1 + pi if right else q1
        t2 = q2 + 5 * pi / 4 if right else 5 * pi / 4 - q2
        t3 = pi / 4 - q3
        r = t1 - pi if right else t1
        j1 = np.zeros(3)
        j2 = np.array([0., self.link_1 * cos(t1), self.link_1 * sin(t1)])
        def rotate_planar(x, z):
            return np.array([x, -sin(r) * z, cos(r) * z])
        j3 = j2 + rotate_planar(self.link_2 * cos(t2), self.link_2 * sin(t2))
        j4 = j3 + rotate_planar(self.link_3 * cos(t2 + t3), self.link_3 * sin(t2 + t3))
        return [j1, j2, j3, j4]

    # Visualization aids ----------------------------------------------------
    def base_pose(self, rot=None, is_radians=True, center_offset=None):
        if rot is None:
            rot = [0, 0, 0]
        if center_offset is None:
            center_offset = [0, 0, 0]

        offset = RotMatrix3D(rot, is_radians) * matrix(np.reshape(center_offset, (3, 1))) \
            - matrix(np.reshape(center_offset, (3, 1)))
        rotated_base = RotMatrix3D(rot, is_radians) * self.leg_origins.transpose() - offset
        return rotated_base.transpose()

    def leg_pose(self, xyz, rot, legID, is_radians, center_offset=None):
        if center_offset is None:
            center_offset = [0, 0, 0]
        pose_relative = self.leg_ik(
            xyz,
            rot,
            legID,
            is_radians,
            center_offset,
        )[3:]
        pose_true = RotMatrix3D(rot, is_radians) * (array(pose_relative).transpose())
        return pose_true.transpose()

    @staticmethod
    def ax_view(limit):
        import matplotlib.pyplot as plt
        ax = plt.axes(projection="3d")
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_zlim(-limit, limit)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        return ax

    def plot_robot(self, xyz, rot=None, leg_N=4, is_radians=True, limit=0.250, center_offset=None):
        import matplotlib.pyplot as plt
        if rot is None:
            rot = [0, 0, 0]
        if center_offset is None:
            center_offset = [0, 0, 0]

        ax = self.ax_view(limit)
        base = self.base_pose(rot, is_radians, center_offset)
        self._plot_base(ax, base)
        for leg in range(leg_N):
            self._plot_leg(ax, xyz, base, rot, leg, is_radians, center_offset)
        plt.show()

    def _plot_base(self, ax, base):
        ax.plot3D(
            asarray(base.transpose()[0, :]).flatten(),
            asarray(base.transpose()[1, :]).flatten(),
            asarray(base.transpose()[2, :]).flatten(),
            "r",
        )

    def _plot_leg(self, ax, xyz, base, rot, legID, is_radians, center_offset):
        pose = (
            self.leg_pose(xyz[legID], rot, legID, is_radians, center_offset)
            + base[legID]
        ).transpose()
        ax.plot3D(
            asarray(pose[0, :]).flatten(),
            asarray(pose[1, :]).flatten(),
            asarray(pose[2, :]).flatten(),
            "b",
        )

    # Servo-specific correction ---------------------------------------------
    def _angle_corrector(self, angles=None, is_right=True):
        if angles is None:
            angles = [0, 0, 0]

        angles[1] -= 1.5 * pi

        if is_right:
            theta_1 = angles[0] - pi
            theta_2 = angles[1] + 45 * pi / 180
        else:
            theta_1 = angles[0] - 2 * pi if angles[0] > pi else angles[0]
            theta_2 = -angles[1] - 45 * pi / 180

        theta_3 = -angles[2] + 45 * pi / 180
        return [theta_1, theta_2, theta_3]


# ---------------------------------------------------------------------------
# Convenience helpers for AI/gait generators
# ---------------------------------------------------------------------------

def stance_motor_angles(name: str, degrees: bool = False):
    """Return theta1-3 for each leg in the stance."""
    kin = SpotIK()
    targets = stance_as_array(name)
    angles = {}
    for leg_id, leg_name in enumerate(LEG_ID_ORDER):
        sol = kin.leg_ik(targets[leg_id], legID=leg_id)
        vals = np.array(sol[:3])
        if degrees:
            vals = np.degrees(vals)
        angles[leg_name] = vals
    return angles


def angle_to_servo_deg(angle_deg, direction=1.0, center=SERVO_CENTER_DEG):
    if not all(isfinite(x) for x in (angle_deg, direction, center)) or direction not in (-1, 1):
        raise IKError("Invalid angle, centre or servo direction")
    command = center + direction * angle_deg
    if not SERVO_MIN_DEG <= command <= SERVO_MAX_DEG:
        raise IKError(f"Servo command {command:.3f} degrees is outside 0..180")
    return float(command)


def stance_servo_commands(name: str):
    """Return servo 0..180° commands for each leg/joint."""
    angles = stance_motor_angles(name, degrees=True)
    commands = {}
    for leg, vals in angles.items():
        direction = SERVO_DIRECTION.get(leg, np.ones_like(vals))
        commands[leg] = np.array([
            angle_to_servo_deg(v, d)
            for v, d in zip(vals, direction)
        ])
    return commands


def print_stance_chart():
    header = ["Stance"] + [f"{leg} (x,y,z) [mm]" for leg in DISPLAY_LEG_ORDER]
    widths = [20] + [24] * len(DISPLAY_LEG_ORDER)
    row_fmt = " ".join("{:<" + str(w) + "}" for w in widths)
    print(row_fmt.format(*header))
    print("-" * (sum(widths) + len(widths) - 1))
    for name, stance in STANCE_LIBRARY.items():
        row = [name]
        for leg in DISPLAY_LEG_ORDER:
            world = HIP_WORLD[leg] + stance[leg]
            row.append(f"({world[0]*1000:6.1f},{world[1]*1000:6.1f},{world[2]*1000:6.1f})")
        print(row_fmt.format(*row))


def print_servo_chart(name: str):
    commands = stance_servo_commands(name)
    header = [f"{name} servo cmd (0-180 deg)"] + [leg for leg in DISPLAY_LEG_ORDER]
    widths = [28] + [18] * len(DISPLAY_LEG_ORDER)
    row_fmt = " ".join("{:<" + str(w) + "}" for w in widths)
    print(row_fmt.format(*header))
    print("-" * (sum(widths) + len(widths) - 1))
    rows = ["servo1", "servo2", "servo3"]
    for idx, label in enumerate(rows):
        row = [label]
        for leg in DISPLAY_LEG_ORDER:
            row.append(f"{commands[leg][idx]:8.2f}")
        print(row_fmt.format(*row))


if __name__ == "__main__":
    print_stance_chart()
    print()
    print_servo_chart(DEFAULT_STANCE)
