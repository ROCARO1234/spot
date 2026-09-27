import copy
import math
import unittest

import numpy as np

from gait_core import IKBinding
from ik_fin import IKError, LEG_ID_ORDER, SpotIK, angle_to_servo_deg, stance_as_array
from robot_config import JOINTS, LEGS, RobotConfig, SafetyError, load_config


class KinematicsTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.binding = IKBinding(self.config)

    def test_stand_exactly_preserves_all_twelve_original_angles(self):
        self.assertEqual(self.binding.solve(self.config.nominal), self.config.pose("STAND"))
        self.assertEqual(len(self.binding.stand), 12)

    def test_ik_fk_roundtrip_random_reachable_targets_all_legs(self):
        rng = np.random.default_rng(912)
        for leg_id, leg in enumerate(LEG_ID_ORDER):
            for _ in range(70):
                point = self.config.nominal[leg]-self.config.hips[leg] + rng.uniform([-.025, -.012, -.018], [.025, .012, .025])
                solution = self.binding.kin.leg_ik(point, legID=leg_id)
                fk = self.binding.kin.leg_fk(solution[:3], leg_id)
                np.testing.assert_allclose(fk[-1], point, atol=1e-9)
                lengths = [np.linalg.norm(fk[i+1]-fk[i]) for i in range(3)]
                np.testing.assert_allclose(lengths, [.045, .1115, .155], atol=1e-10)

    def test_independent_forward_geometry_at_known_joint_angles(self):
        # Left leg with shoulder horizontal, upper and lower links both down.
        q = [0., -math.pi/4, math.pi/4]
        points = SpotIK().leg_fk(q, 0)
        np.testing.assert_allclose(points[-1], [0., .045, -.2665], atol=1e-10)

    def test_left_right_mirror_geometry_and_angle_conventions(self):
        kin = self.binding.kin
        left = kin.leg_ik([.012, .05, -.18], legID=0)
        right = kin.leg_ik([.012, -.05, -.18], legID=3)
        np.testing.assert_allclose(left[:2], -np.asarray(right[:2]), atol=1e-10)
        self.assertAlmostEqual(left[2], right[2])
        for a, b in zip(left[3:], right[3:]):
            np.testing.assert_allclose(a, np.asarray(b)*[1, -1, 1], atol=1e-10)

    def test_outer_inner_and_hip_cylinder_targets_are_rejected(self):
        for target in ([1., .09, -.18], [0., .045, -.005], [0., .001, -.001]):
            with self.subTest(target=target), self.assertRaises(IKError):
                SpotIK().leg_ik(target)

    def test_nan_inf_and_invalid_leg_ids_are_rejected(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.assertRaises(IKError):
                SpotIK().leg_ik([value, .05, -.18])
        for leg_id in (-1, 4, True, 1.5):
            with self.assertRaises(IKError):
                SpotIK().leg_ik([.01, .05, -.18], legID=leg_id)

    def test_body_rotation_inverse_frame_and_fk_roundtrip(self):
        from ik_fin import RotMatrix3D
        point = np.array([.015, .05, -.18])
        rotation = [.03, -.06, .08]
        hip = self.config.hips["FL"]
        expected = np.asarray(RotMatrix3D(rotation)).T @ (point+hip) - hip
        solution = self.binding.kin.leg_ik(point, rotation, legID=0)
        np.testing.assert_allclose(solution[-1], expected, atol=1e-9)

    def test_calibrated_servo_fk_matches_shifted_body_target(self):
        shift = np.array([.025, -.020, 0.])
        pose = self.binding.solve(self.config.nominal, shift)
        joints = self.binding.forward(pose, shift)
        for leg in LEGS:
            np.testing.assert_allclose(joints[leg][-1], self.config.nominal[leg], atol=1e-9)

    def test_offsets_applied_once_and_direction_inverts_delta_only(self):
        data = copy.deepcopy(self.config.data)
        data["servos"]["FL_up"]["offset_deg"] = 4.
        data["servos"]["FL_up"]["direction"] = -1
        other = IKBinding(RobotConfig(data, copy.deepcopy(self.config.poses)))
        feet = {leg: p.copy() for leg, p in self.config.nominal.items()}
        feet["FL"][0] += .01
        original, changed = self.binding.solve(feet), other.solve(feet)
        self.assertAlmostEqual(changed["FL_up"]-84., -(original["FL_up"]-80.))
        np.testing.assert_allclose(other.forward(changed)["FL"][-1], feet["FL"], atol=1e-9)

    def test_servo_limits_rejected_not_silently_clipped(self):
        data = copy.deepcopy(self.config.data)
        data["servos"]["FL_up"]["min_deg"] = 79.9
        data["servos"]["FL_up"]["max_deg"] = 80.1
        binding = IKBinding(RobotConfig(data, copy.deepcopy(self.config.poses)))
        feet = {leg: p.copy() for leg, p in self.config.nominal.items()}
        feet["FL"][0] += .015
        with self.assertRaises(SafetyError):
            binding.solve(feet)
        with self.assertRaises(IKError):
            angle_to_servo_deg(200.)

    def test_low_stance_is_lower_body_than_high_stance(self):
        self.assertGreater(stance_as_array("stand_low")[0][2], stance_as_array("stand_high")[0][2])


class ConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.base = load_config()
        self.data = copy.deepcopy(self.base.data)
        self.poses = copy.deepcopy(self.base.poses)

    def test_mapping_is_latest_runner_not_incomplete_legacy_csv(self):
        expected = {"FL": [15,14,13], "FR": [3,2,1], "RL": [11,10,9], "RR": [7,6,5]}
        for leg, channels in expected.items():
            self.assertEqual([self.base.servos[f"{leg}_{j}"]["channel"] for j in ("sh", "up", "lo")], channels)

    def test_duplicate_channel_rejected(self):
        self.data["servos"]["FR_sh"]["channel"] = 15
        with self.assertRaisesRegex(SafetyError, "Duplicate"):
            RobotConfig(self.data, self.poses)

    def test_missing_joint_not_replaced_with_90_degrees(self):
        del self.poses["STAND"]["RR_lo"]
        with self.assertRaises(SafetyError):
            RobotConfig(self.data, self.poses)

    def test_invalid_servo_calibrations_fail_early(self):
        for key, value in (("min_deg", 170), ("max_deg", 0), ("channel", 1.5),
                           ("direction", 0), ("direction", True), ("offset_deg", math.nan),
                           ("max_rate_deg_s", -5), ("min_pulse_us", 2500)):
            data = copy.deepcopy(self.data)
            data["servos"]["FL_sh"][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(SafetyError):
                RobotConfig(data, self.poses)

    def test_unverified_flags_block_hardware_but_not_offline(self):
        self.assertEqual(len(self.base.hardware_blockers()), 2)
        IKBinding(self.base)

    def test_unsafe_legacy_poses_preserved_but_disabled(self):
        self.assertEqual(self.poses["LIE"]["FL_lo"], 0)
        self.assertEqual(self.poses["UP_READY"]["RR_up"], 0)
        for name in ("LIE", "UP_READY"):
            with self.assertRaises(SafetyError):
                self.base.pose(name)
        self.base.pose("CROUCH_READY")

    def test_bad_frame_keys_boolean_nan_and_range_fail(self):
        for val in (True, math.nan, math.inf, -1, 999):
            pose = self.base.pose("STAND"); pose["FL_sh"] = val
            with self.assertRaises(SafetyError):
                self.base.validate_pose(pose)
        pose = self.base.pose("STAND"); del pose["FL_sh"]
        with self.assertRaises(SafetyError):
            self.base.validate_pose(pose)


if __name__ == "__main__":
    unittest.main()
