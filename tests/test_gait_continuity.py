"""Offline regression checks for conservative timing and stage tangents."""
import copy
import math
import unittest

import numpy as np

from gait_core import CrawlPlanner, IKBinding, PoseSlewLimiter
from robot_config import LEGS, SafetyError, load_config
from robot_io import checked_frame


class ContinuityTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()

    def first_swing(self):
        planner = CrawlPlanner(self.config)
        planner.set_command(.003, 0., 0.)
        for _ in range(200):
            planner.update(.02)
            if planner.swing:
                return planner
        self.fail("No swing started")

    def test_touchdown_tangent_matches_reversal_and_braking(self):
        for next_velocity in (-.003, 0.):
            with self.subTest(next_velocity=next_velocity):
                planner = self.first_swing()
                leg = planner.swing
                for _ in range(20):
                    planner.update(.02)
                planner.set_command(next_velocity, 0., 0.)
                h = 1e-4
                endpoint = planner.p1.copy()
                end = planner.settings["swing_time_s"]
                while planner.t < end - h - 1e-10:
                    planner.update(min(.02, end-h-planner.t))
                before = planner.feet[leg].copy()
                planner.update(h)
                at = planner.feet[leg].copy()
                planner.update(h)
                after = planner.feet[leg].copy()
                np.testing.assert_allclose(at, endpoint, atol=1e-12)
                np.testing.assert_allclose((at-before)/h, (after-at)/h, atol=2e-6)

    def test_body_tangent_is_preserved_after_touchdown(self):
        planner = self.first_swing()
        end = planner.settings["swing_time_s"]
        h = 1e-4
        while planner.t < end - h - 1e-10:
            planner.update(min(.02, end-h-planner.t))
        before = planner.body_shift.copy()
        planner.update(h)
        at = planner.body_shift.copy()
        planner.update(h)
        after = planner.body_shift.copy()
        np.testing.assert_allclose((at-before)/h, (after-at)/h, atol=2e-6)

    def test_resume_during_centering_completes_current_body_path(self):
        planner = self.first_swing()
        planner.set_command()
        for _ in range(2000):
            planner.update(.02)
            if planner.state == "CENTER":
                break
        self.assertEqual(planner.state, "CENTER")
        for _ in range(20):
            planner.update(.02)
        reference = copy.deepcopy(planner)
        planner.set_command(-.003, 0., .008)
        for _ in range(300):
            planner.update(.02)
            reference.update(.02)
            np.testing.assert_allclose(planner.body_shift, reference.body_shift, atol=1e-12)
            for leg in LEGS:
                np.testing.assert_allclose(planner.feet[leg], reference.feet[leg], atol=1e-12)
            self.assertEqual(planner.lift_count, reference.lift_count)
            if reference.standing:
                break
        self.assertTrue(reference.standing)
        planner.update(.02)
        self.assertEqual(planner.state, "TRANSFER")
        self.assertLess(planner.velocity[0], 0.)

    def test_dynamicity_is_time_dilation_and_does_not_mutate_configuration(self):
        data = copy.deepcopy(self.config.data)
        fast = CrawlPlanner(self.config)
        slow = CrawlPlanner(self.config, dynamicity=.5)
        command = (.003, -.001, .008)
        fast.set_command(*command)
        slow.set_command(*command)
        np.testing.assert_allclose(slow.requested, .5*fast.requested, atol=1e-12)
        self.assertEqual(slow.cycle_time, 2*fast.cycle_time)
        for _ in range(600):
            a = fast.update(.01)
            b = slow.update(.02)
        for leg in LEGS:
            np.testing.assert_allclose(a.feet[leg], b.feet[leg], atol=2e-6)
        np.testing.assert_allclose(a.body_shift, b.body_shift, atol=2e-6)
        self.assertEqual(data, self.config.data)

    def test_dynamicity_rejects_unsafe_or_nonfinite_values(self):
        for value in (True, 0., .29, 1.01, math.nan, math.inf, "0.5"):
            with self.subTest(value=value), self.assertRaises(SafetyError):
                CrawlPlanner(self.config, dynamicity=value)

    def test_reversals_respect_support_and_original_servo_limits(self):
        for dynamicity in (.3, 1.):
            with self.subTest(dynamicity=dynamicity):
                planner = CrawlPlanner(self.config, dynamicity=dynamicity)
                ik = IKBinding(self.config)
                previous = self.config.pose("STAND")
                limiter = PoseSlewLimiter(self.config, previous)
                for command in ((.003, -.001, .008), (-.003, .001, -.008), (0., 0., 0.)):
                    planner.set_command(*command)
                    stopping = not any(command)
                    deadline = (3 if stopping else 1)*planner.cycle_time
                    for _ in range(math.ceil(deadline/.02)):
                        frame = planner.update(.02)
                        self.assertLessEqual(len(frame.swing_legs), 1)
                        self.assertGreaterEqual(frame.support_margin_m,
                                                self.config.gait["support_margin_m"]-1e-9)
                        self.assertLessEqual(np.linalg.norm(frame.body_shift),
                                             self.config.gait["max_body_shift_m"])
                        target = ik.solve(frame.feet, frame.body_shift)
                        pose = limiter.update(target, .02)
                        checked_frame(self.config, pose, previous, .02)
                        previous = pose
                        if stopping and planner.standing:
                            break
                self.assertTrue(planner.standing)


if __name__ == "__main__":
    unittest.main()
