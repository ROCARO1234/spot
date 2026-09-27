import copy
import math
import unittest

import numpy as np

from controller import MotionController
from gait_core import CrawlPlanner, IKBinding, PoseSlewLimiter, inverse_body_step, support_margin
from robot_config import LEGS, RobotConfig, SafetyError, load_config
from simulation import simulate, summary


class GaitTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config()

    def test_zero_command_means_no_steps_and_no_motion(self):
        gait = CrawlPlanner(self.config)
        for _ in range(1500):
            frame = gait.update(.02)
        self.assertEqual(gait.lift_count, 0)
        self.assertTrue(gait.standing)
        for leg in LEGS:
            np.testing.assert_array_equal(frame.feet[leg], self.config.nominal[leg])

    def test_forward_backward_and_zero_differ_with_expected_sign(self):
        positions = []
        for vx in (.003, -.003, 0.):
            gait = CrawlPlanner(self.config); gait.set_command(vx, 0, 0)
            for _ in range(25):
                frame = gait.update(.02)
            positions.append(frame.feet["FL"][0])
        self.assertLess(positions[0], positions[2])
        self.assertGreater(positions[1], positions[2])
        self.assertAlmostEqual(positions[0]+positions[1], 2*positions[2], places=9)

    def test_lateral_motion_does_not_invent_forward_stride(self):
        for direction in (-1, 1):
            gait = CrawlPlanner(self.config); gait.set_command(0, direction*.003, 0)
            for _ in range(30):
                f = gait.update(.02)
            for leg in LEGS:
                self.assertAlmostEqual(f.feet[leg][0], self.config.nominal[leg][0])
                self.assertLess(direction*(f.feet[leg][1]-self.config.nominal[leg][1]), 0)

    def test_yaw_rotates_stance_feet_not_static_offsets(self):
        left = CrawlPlanner(self.config); left.set_command(0, 0, .015)
        right = CrawlPlanner(self.config); right.set_command(0, 0, -.015)
        for _ in range(30):
            a, b = left.update(.02), right.update(.02)
        self.assertLess(a.feet["FL"][1], self.config.nominal["FL"][1])
        self.assertGreater(b.feet["FL"][1], self.config.nominal["FL"][1])
        for leg in LEGS:
            self.assertAlmostEqual(np.linalg.norm(a.feet[leg][:2]), np.linalg.norm(self.config.nominal[leg][:2]), places=9)

    def test_se2_twist_composes_without_discretization_drift(self):
        p = np.array([.14, .09, -.18]); command = np.array([.012, -.008, .11])
        one = inverse_body_step(p, command, 1.)
        many = p.copy()
        for _ in range(100):
            many = inverse_body_step(many, command, .01)
        np.testing.assert_allclose(one, many, atol=1e-12)

    def test_velocity_limits_preserve_command_ratio(self):
        gait = CrawlPlanner(self.config); gait.set_command(.2, -.1, .5)
        ratios = gait.requested / np.array([.2, -.1, .5])
        np.testing.assert_allclose(ratios, ratios[0], atol=1e-12)
        self.assertLess(ratios[0], 1)

    def test_crawl_contacts_support_and_all_direction_trajectories(self):
        commands = ((.003, 0, 0), (-.003, 0, 0), (0, .003, 0), (0, -.003, 0),
                    (0, 0, .015), (0, 0, -.015), (.003, -.002, .01))
        for command in commands:
            with self.subTest(command=command):
                records = simulate(self.config, command, duration=13.)
                self.assertEqual(records[-1]["state"], "STAND")
                self.assertEqual(summary(records)["max_swing_legs"], 1)
                for r in records:
                    raised = sum(r["feet"][leg][2]-self.config.nominal[leg][2] > 1e-7 for leg in LEGS)
                    self.assertLessEqual(raised, 1)
                    self.assertGreaterEqual(r["margin"], self.config.gait["support_margin_m"]-1e-9)
                    for leg in LEGS:
                        if leg not in r["swing"]:
                            self.assertAlmostEqual(r["feet"][leg][2], self.config.nominal[leg][2], places=10)
                for leg in LEGS:
                    np.testing.assert_allclose(records[-1]["feet"][leg], self.config.nominal[leg], atol=.0001)

    def test_stop_midair_finishes_landing_then_stands(self):
        mc = MotionController(self.config); mc.set_command(.003, 0, 0)
        for _ in range(500):
            mc.update(.02)
            if mc.frame.swing_legs and any(p[2] > -.171 for p in mc.frame.feet.values()):
                break
        self.assertTrue(mc.frame.swing_legs)
        last_feet = {leg: p.copy() for leg, p in mc.frame.feet.items()}
        mc.set_command()
        mc.update(.02)
        self.assertTrue(mc.frame.swing_legs)
        for leg in LEGS:
            self.assertLess(np.linalg.norm(mc.frame.feet[leg]-last_feet[leg]), .005)
        for _ in range(1600):
            mc.update(.02)
            if mc.settled:
                break
        self.assertTrue(mc.settled)
        self.assertEqual(mc.frame.swing_legs, ())

    def test_stop_before_lift_does_not_launch_unrequested_step(self):
        gait = CrawlPlanner(self.config)
        gait.set_command(.003, 0, 0)
        gait.update(.02)
        gait.set_command()
        for _ in range(300):
            gait.update(.02)
        self.assertEqual(gait.lift_count, 0)
        self.assertTrue(gait.standing)

    def test_timestep_invariance_and_no_large_unaccounted_gap(self):
        states = []
        for dt in (.01, .02, .04):
            gait = CrawlPlanner(self.config); gait.set_command(.003, .001, .01)
            for _ in range(round(8/dt)):
                f = gait.update(dt)
            states.append(f)
        for f in states[1:]:
            for leg in LEGS:
                np.testing.assert_allclose(f.feet[leg], states[0].feet[leg], atol=.0001)
        for value in (0, -.1, .3, math.nan, math.inf, True):
            with self.assertRaises(SafetyError):
                CrawlPlanner(self.config).update(value)

    def test_snapshot_does_not_mutate_planner(self):
        gait = CrawlPlanner(self.config)
        frame = gait.frame(); frame.feet["FL"][:] = 100
        np.testing.assert_array_equal(gait.feet["FL"], self.config.nominal["FL"])

    def test_support_triangle_signed_margin(self):
        triangle = [(0,0), (1,0), (0,1)]
        self.assertGreater(support_margin(triangle, [.2,.2]), 0)
        self.assertLess(support_margin(triangle, [.8,.8]), 0)
        self.assertEqual(support_margin(triangle[:2], [.1,.1]), -math.inf)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()

    def test_slew_limit_checked_for_each_joint_and_frame(self):
        original = self.config.pose("STAND")
        limiter = PoseSlewLimiter(self.config, original)
        target = {j: v+20 for j,v in original.items()}
        result = limiter.update(target, .02)
        for j in original:
            self.assertLessEqual(abs(result[j]-original[j]), self.config.servos[j]["max_rate_deg_s"]*.02+1e-10)

    def test_pose_transition_starts_from_last_command_and_waits_for_landing(self):
        mc = MotionController(self.config); mc.set_command(.003,0,0)
        for _ in range(110):
            mc.update(.02)
        mc.request_pose("CROUCH_READY")
        previous = mc.last_pose
        transitions = 0
        for _ in range(1600):
            current = mc.update(.02)
            for j in current:
                self.assertLessEqual(abs(current[j]-previous[j]), 1.2+1e-9)
            if mc.transition:
                transitions += 1
                self.assertEqual(mc.frame.swing_legs, ())
            previous = current
            if mc.posture == "CROUCH_READY" and mc.settled:
                break
        self.assertGreater(transitions, 0)
        self.assertEqual(mc.last_pose, self.config.pose("CROUCH_READY"))
        with self.assertRaises(SafetyError):
            mc.set_command(.003,0,0)
        mc.request_pose("STAND")
        for _ in range(200):
            mc.update(.02)
        self.assertEqual(mc.posture, "STAND")
        mc.set_command(.003,0,0)

    def test_rejected_extreme_pose_requests_normal_stop(self):
        mc = MotionController(self.config); mc.set_command(.003,0,0)
        with self.assertRaises(SafetyError):
            mc.request_pose("LIE")
        np.testing.assert_array_equal(mc.planner.requested, np.zeros(3))

    def test_invalid_commands_do_not_enter_pipeline(self):
        for value in (math.nan, math.inf, True, "0.01"):
            with self.assertRaises(SafetyError):
                MotionController(self.config).set_command(value, 0, 0)


if __name__ == "__main__":
    unittest.main()
