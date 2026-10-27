import copy
import math
import unittest

from sensors import JOINT_NAMES, SensorHub, load_sensor_settings


class Backend:
    def __init__(self, sample):
        self.sample = sample
        self.calls = 0

    def read(self):
        self.calls += 1
        return self.sample


class SensorTests(unittest.TestCase):
    def setUp(self):
        self.settings = {"schema_version": 1, "enabled": True, "max_age_s": .2, "backend": "injected"}
        # Synthetic values belong only to this test fixture, never runtime fallback.
        self.sample = {"schema_version": 1, "source": "unit-test fixture", "timestamp_monotonic_s": 10.,
                       "imu": {"accel_mps2": [0., 0., 9.81], "gyro_rps": [0., 0., 0.]},
                       "joint_angles_deg": {name: 90. for name in JOINT_NAMES}}

    def hub(self, sample=None):
        return SensorHub(self.settings, backend=Backend(self.sample if sample is None else sample), clock=lambda: 10.1)

    def test_default_is_unavailable_without_hardware_access(self):
        reading = SensorHub().poll()
        self.assertFalse(reading.available)
        self.assertIsNone(reading.imu)
        self.assertIsNone(reading.joint_angles_deg)
        self.assertIn("not mounted", reading.reason)
        self.assertFalse(load_sensor_settings()["enabled"])

    def test_disabled_backend_is_never_read(self):
        settings = dict(self.settings, enabled=False)
        backend = Backend(self.sample)
        self.assertFalse(SensorHub(settings, backend=backend).poll().available)
        self.assertEqual(backend.calls, 0)

    def test_enable_without_backend_never_invents_data(self):
        reading = SensorHub(self.settings).poll()
        self.assertFalse(reading.available)
        self.assertIsNone(reading.timestamp_monotonic_s)

    def test_valid_measurements_have_source_units_and_defensive_copies(self):
        reading = self.hub().poll()
        self.assertTrue(reading.available)
        self.assertEqual(reading.source, "unit-test fixture")
        self.assertAlmostEqual(reading.age_s, .1)
        self.sample["joint_angles_deg"]["FL_sh"] = 30.
        self.sample["imu"]["gyro_rps"][0] = 20.
        self.assertEqual(reading.joint_angles_deg["FL_sh"], 90.)
        self.assertEqual(reading.imu["gyro_rps"][0], 0.)

    def test_stale_future_and_regressing_timestamps_are_unavailable(self):
        for timestamp, reason in ((9., "stale"), (11., "future")):
            with self.subTest(timestamp=timestamp):
                sample = dict(self.sample, timestamp_monotonic_s=timestamp)
                reading = self.hub(sample).poll()
                self.assertFalse(reading.available)
                self.assertIn(reason, reading.reason)
                self.assertIsNone(reading.joint_angles_deg)
        hub = self.hub()
        self.assertTrue(hub.poll().available)
        hub.backend.sample = dict(self.sample, timestamp_monotonic_s=9.99)
        self.assertIn("backwards", hub.poll().reason)

    def test_nan_missing_joints_and_wrong_vector_shape_reject_whole_frame(self):
        samples = []
        for value in (math.nan, math.inf, True, "90", -1, 181):
            sample = copy.deepcopy(self.sample)
            sample["joint_angles_deg"]["FL_sh"] = value
            samples.append(sample)
        sample = copy.deepcopy(self.sample)
        del sample["joint_angles_deg"]["RR_lo"]
        samples.append(sample)
        sample = copy.deepcopy(self.sample)
        sample["imu"]["accel_mps2"] = [0, 0]
        samples.append(sample)
        for sample in samples:
            with self.subTest(sample=sample):
                reading = self.hub(sample).poll()
                self.assertFalse(reading.available)
                self.assertIsNone(reading.imu)
                self.assertIsNone(reading.joint_angles_deg)

    def test_partial_sensor_installation_is_explicit(self):
        sample = dict(self.sample, joint_angles_deg=None)
        reading = self.hub(sample).poll()
        self.assertTrue(reading.available)
        self.assertIsNone(reading.joint_angles_deg)
        self.assertIsNotNone(reading.imu)

    def test_backend_failure_clears_data(self):
        hub = self.hub()
        self.assertTrue(hub.poll().available)
        def fail():
            raise OSError("disconnected")
        hub.backend.read = fail
        reading = hub.poll()
        self.assertFalse(reading.available)
        self.assertIsNone(reading.imu)
        self.assertIn("disconnected", reading.reason)

    def test_empty_or_malformed_samples_never_produce_feedback(self):
        hub = self.hub()
        for sample in (None, {}, [], dict(self.sample, source=""),
                       dict(self.sample, imu=None, joint_angles_deg=None)):
            with self.subTest(sample=sample):
                hub.backend.sample = sample
                reading = hub.poll()
                self.assertFalse(reading.available)
                self.assertIsNone(reading.imu)
                self.assertIsNone(reading.joint_angles_deg)

    def test_rejected_frame_does_not_advance_timestamp_history(self):
        hub = self.hub()
        hub.backend.sample = dict(self.sample, timestamp_monotonic_s=10.05, imu={})
        self.assertFalse(hub.poll().available)
        hub.backend.sample = self.sample
        self.assertTrue(hub.poll().available)

    def test_unknown_drivers_and_malformed_configuration_fail(self):
        for key, value in (("backend", "ads1115"), ("enabled", 1), ("max_age_s", math.nan),
                           ("schema_version", True)):
            with self.subTest(key=key), self.assertRaises(ValueError):
                SensorHub(dict(self.settings, **{key: value}))


if __name__ == "__main__":
    unittest.main()
