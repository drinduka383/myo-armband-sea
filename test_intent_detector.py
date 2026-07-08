import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from myo_scan import MYO_CONTROL_SERVICE, is_myo
from run_myo_sea_demo import IntentDetector, MYO_STREAM_COMMAND


def detector():
    args = SimpleNamespace(
        window_ms=200,
        step_ms=50,
        gyro_threshold="auto",
        min_score=0.08,
        target_override="auto",
        calibration_repetitions=3,
    )
    value = IntentDetector(args)
    value.orientation_quaternions = {
        "handshake": [1, 0, 0, 0],
        "palm_down": [0.707, 0.707, 0, 0],
        "palm_up": [0.707, -0.707, 0, 0],
    }
    value.rest_thresholds = {name: 10.0 for name in value.orientation_quaternions}
    value.target_templates = {name: [1, 0, 0, 0, 0, 0, 0, 0] for name in value.orientation_quaternions}
    value.target_activity = {name: {"mean": 20.0, "std": 1.0} for name in value.orientation_quaternions}
    value.static_templates = {
        "open_hand": {name: [0, 1, 0, 0, 0, 0, 0, 0] for name in value.orientation_quaternions},
        "full_fist": {name: [0, 0, 1, 0, 0, 0, 0, 0] for name in value.orientation_quaternions},
    }
    value.static_activity = {
        "open_hand": {name: {"mean": 20.0, "std": 1.0} for name in value.orientation_quaternions},
        "full_fist": {name: {"mean": 20.0, "std": 1.0} for name in value.orientation_quaternions},
    }
    value.motion_templates = {
        label: [[0, 0, 0, 1, 0, 0, 0, 0]]
        for label in ("wrist_flex", "wrist_extend", "forearm_pronate", "forearm_supinate")
    }
    value.chosen_target = "claw_hook"
    value.gyro_threshold = 20.0
    value.on_hold_s = value.off_hold_s = 0.0
    return value


class FakeStream:
    def __init__(self, feature, quaternion=None, gyro=0):
        self.feature = feature
        self.quaternion = quaternion or [1, 0, 0, 0]
        self.has_imu = True
        self.gyro_mag = gyro

    def window_feature(self, *_):
        return self.feature


class IntentDetectorTest(unittest.TestCase):
    def test_myo_stream_mode_enables_imu(self):
        self.assertEqual(MYO_STREAM_COMMAND, bytes((0x01, 0x03, 0x02, 0x01, 0x00)))

    def test_myo_scan_requires_name_or_service_uuid(self):
        self.assertFalse(is_myo("", []))
        self.assertTrue(is_myo("", [MYO_CONTROL_SERVICE]))

    def test_repeated_trials_pass_cross_orientation_rules(self):
        value = detector()
        trials = [{"windows": [[20, 1, 0, 0, 0, 0, 0, 0]] * 4} for _ in range(3)]
        template = value._template(trials)
        self.assertGreater(value._cross_similarity(trials), 0.99)
        result = value._validate_orientation("handshake", trials, template)
        self.assertTrue(result["passed"])
        self.assertGreater(result["score_margin"], 0.05)

    def test_movement_templates_do_not_compete_in_spatial_ranking(self):
        value = detector()
        target = [{"windows": [[20, 1, 0, 0, 0, 0, 0, 0]] * 4} for _ in range(3)]
        static = {
            "open_hand": {"handshake": [{"windows": [[0, 20, 0, 0, 0, 0, 0, 0]] * 4}]},
            "full_fist": {"handshake": [{"windows": [[0, 0, 20, 0, 0, 0, 0, 0]] * 4}]},
        }
        row = value._rank_candidate("claw_hook", target, static)
        self.assertIn(row["worst_rejection_label"], ("open_hand", "full_fist"))
        self.assertGreater(row["score"], 0.9)

    def test_activity_separates_identical_spatial_patterns(self):
        feature = [20, 0, 0, 0, 0, 0, 0, 0]
        pattern = [1, 0, 0, 0, 0, 0, 0, 0]
        target_score = IntentDetector._class_score(feature, pattern, {"mean": 20, "std": 1})
        full_fist_score = IntentDetector._class_score(feature, pattern, {"mean": 80, "std": 5})
        self.assertGreater(target_score, full_fist_score + 0.1)

    def test_orientation_rest_target_and_motion_rejection(self):
        value = detector()
        self.assertEqual(value.orientation_for([0.7, -0.7, 0, 0]), "palm_up")
        stream = FakeStream([20, 0, 0, 0, 0, 0, 0, 0])
        value.update(stream)
        target = value.update(stream)
        self.assertEqual(target["state"], "TARGET")
        self.assertNotEqual(target["best_rejection_label"], "rest")
        stream.gyro_mag = 100
        value.update(stream)
        rejected = value.update(stream)
        self.assertEqual(rejected["state"], "MOTION_REJECT")
        stream.feature = [0.1] * 8
        stream.gyro_mag = 0
        self.assertEqual(value.update(stream)["state"], "REST")

    def test_preset_round_trip_and_version_guard(self):
        value = detector()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "preset.json"
            value.save_preset(path)
            loaded = detector()
            loaded.load_preset(path)
            self.assertEqual(loaded.chosen_target, "claw_hook")
            self.assertEqual(loaded.target_templates, value.target_templates)
            data = path.read_text().replace('"version": 2', '"version": 999')
            path.write_text(data)
            with self.assertRaises(ValueError):
                loaded.load_preset(path)


if __name__ == "__main__":
    unittest.main()
