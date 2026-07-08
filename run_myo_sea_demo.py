#!/usr/bin/env python3
import argparse
import asyncio
import csv
import json
import math
import multiprocessing as mp
import queue
import signal
import statistics
import struct
import time
from collections import deque
from datetime import datetime
from pathlib import Path

from sea_model import SEAModel

COMMAND_UUID = "d5060401-a904-deb9-4748-2c7f4a124842"
CONTROL_SERVICE_UUID = "d5060001-a904-deb9-4748-2c7f4a124842"
IMU_UUID = "d5060402-a904-deb9-4748-2c7f4a124842"
EMG_UUIDS = [f"d5060{i}05-a904-deb9-4748-2c7f4a124842" for i in range(1, 5)]
MYO_STREAM_COMMAND = bytes((0x01, 0x03, 0x02, 0x01, 0x00))  # raw EMG, IMU data, classifier off
EPSILON = 1e-6

CANDIDATE_LABELS = ("claw_hook", "pinch_attempt", "partial_hand_close", "fist_lite")
ORIENTATIONS = ("handshake", "palm_down", "palm_up")
STATIC_REJECTIONS = ("open_hand", "full_fist")
MOTION_REJECTIONS = ("wrist_flex", "wrist_extend", "forearm_pronate", "forearm_supinate")
PRESET_VERSION = 2

CALIBRATION_PROMPTS = {
    "rest": "Keep the wrist straight and let the fingers rest in their natural curved position.",
    "pinch_attempt": "Touch the thumb and index fingertips gently; do not squeeze or curl the other fingers.",
    "claw_hook": "RAWR/claw pose: keep the base knuckles straight; bend the middle and fingertip joints on each finger.",
    "partial_hand_close": "Close all fingers about halfway, as if loosely holding a large cup.",
    "fist_lite": "Make a loose fist with the thumb resting gently; do not squeeze.",
    "wrist_flex": "During recording, bend the palm about 20 degrees toward the inner forearm/torso.",
    "wrist_extend": "During recording, bend the back of the hand about 20 degrees toward the outer forearm.",
    "forearm_pronate": "During recording, slowly rotate from handshake to palm-down.",
    "forearm_supinate": "During recording, slowly rotate from handshake to palm-up.",
    "open_hand": "Straighten and gently spread the fingers while keeping the wrist straight.",
    "full_fist": "Close the whole hand firmly, including the thumb, but do not use maximum force.",
}

CALIBRATION_TIPS = {
    "rest": "HOLD, effort 0/10. Support the arm and release all deliberate muscle effort.",
    "pinch_attempt": "Effort: 2-3/10. Gentle thumb-index pinch intent. Do not curl the whole hand.",
    "claw_hook": "HOLD, effort 3/10. Base knuckles stay straight; thumb and wrist stay relaxed.",
    "partial_hand_close": "Effort: 2-3/10. Small whole-hand close, clearly less than a full fist.",
    "fist_lite": "Effort: 3-4/10. Light fist only. Repeatable and deliberate, not maximal.",
    "wrist_flex": "MOVE once, effort 2-3/10. Start straight; fingers stay loose.",
    "wrist_extend": "MOVE once, effort 2-3/10. Start straight; fingers stay loose.",
    "forearm_pronate": "MOVE once, effort 2-3/10. Direction is palm-down, not clockwise.",
    "forearm_supinate": "MOVE once, effort 2-3/10. Direction is palm-up, not clockwise.",
    "open_hand": "Effort: 2-3/10. Spread or open the hand without bending the wrist backward hard.",
    "full_fist": "Effort: 5/10. Firm full-hand close, not maximum force. This is a rejection class.",
}

CALIBRATION_POSTURE = (
    "Sit with the shoulder relaxed and elbow near 90 degrees.",
    "Support the forearm on a desk or folded towel; do not press the Myo electrodes against the desk.",
    "Let the hand extend beyond the desk edge so the wrist and fingers are free.",
    "The program will request handshake, palm-down, or palm-up; keep the wrist straight unless told to move it.",
    "Keep this arm, chair, armband position, and effort scale unchanged for the whole calibration.",
    "HOLD prompts stay motionless; MOVE prompts perform one slow movement during recording.",
)


def calibration_goal(label):
    if label in CANDIDATE_LABELS:
        return "Goal: make this gesture clear and repeatable without adding wrist motion or extra squeeze."
    if label == "rest":
        return "Goal: truly quiet baseline. Let the forearm settle before recording starts."
    return "Goal: deliberately perform this non-target so the detector learns to reject it."


def mean_abs_feature(samples):
    if not samples:
        return [0.0] * 8
    count = float(len(samples))
    return [sum(abs(sample[i]) for sample in samples) / count for i in range(8)]


def normalize_pattern(feature):
    total_activity = sum(feature)
    return [value / (total_activity + EPSILON) for value in feature]


def mean_vector(vectors):
    if not vectors:
        return [0.0] * 8
    count = float(len(vectors))
    return [sum(vector[i] for vector in vectors) / count for i in range(len(vectors[0]))]


def cosine_similarity(a, b):
    numerator = sum(x * y for x, y in zip(a, b))
    denom_a = math.sqrt(sum(x * x for x in a))
    denom_b = math.sqrt(sum(y * y for y in b))
    if denom_a <= EPSILON or denom_b <= EPSILON:
        return 0.0
    return numerator / (denom_a * denom_b)


def normalize_quaternion(values):
    magnitude = math.sqrt(sum(value * value for value in values))
    return [value / magnitude for value in values] if magnitude > EPSILON else [1.0, 0.0, 0.0, 0.0]


def mean_quaternion(quaternions):
    if not quaternions:
        return [1.0, 0.0, 0.0, 0.0]
    reference = normalize_quaternion(quaternions[0])
    aligned = []
    for quaternion in quaternions:
        quaternion = normalize_quaternion(quaternion)
        aligned.append([-value for value in quaternion] if sum(a * b for a, b in zip(reference, quaternion)) < 0 else quaternion)
    return normalize_quaternion(mean_vector(aligned))


def quaternion_similarity(a, b):
    return abs(sum(x * y for x, y in zip(normalize_quaternion(a), normalize_quaternion(b))))


def percentile(values, fraction):
    if not values:
        return 0.0
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def windows_from_trace(trace, start, end, window_s, step_s):
    windows = []
    window_end = start + window_s
    while window_end <= end:
        samples = [sample for ts, sample in trace if window_end - window_s <= ts <= window_end]
        if samples:
            windows.append(mean_abs_feature(samples))
        window_end += step_s
    return windows


class EMGStream:
    def __init__(self, history_s=12.0):
        self.history_s = history_s
        self.raw = [0] * 8
        self.updated = 0.0
        self.started = None
        self.samples = deque()
        self.sample_count = 0
        self.gyro_mag = 0.0
        self.quaternion = [1.0, 0.0, 0.0, 0.0]
        self.imu_samples = deque()
        self.has_imu = False

    def update_emg(self, channels):
        now = time.monotonic()
        if self.started is None:
            self.started = now
        sample = list(channels)
        self.raw = sample
        self.updated = now
        self.sample_count += 1
        self.samples.append((now, sample))
        cutoff = now - self.history_s
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def update_imu(self, data):
        if len(data) != 20:
            return
        values = struct.unpack("<10h", data)
        now = time.monotonic()
        self.quaternion = normalize_quaternion(values[:4])
        gx, gy, gz = values[7:10]
        self.gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
        self.imu_samples.append((now, self.gyro_mag, self.quaternion))
        cutoff = now - self.history_s
        while self.imu_samples and self.imu_samples[0][0] < cutoff:
            self.imu_samples.popleft()
        self.has_imu = True

    def force_activation(self, activation):
        level = int(max(0.0, min(1.0, activation)) * 127)
        self.update_emg([level] * 8)

    def samples_between(self, start, end):
        return [(ts, sample) for ts, sample in self.samples if start <= ts <= end]

    def imu_between(self, start, end):
        return [(gyro, quaternion) for ts, gyro, quaternion in self.imu_samples if start <= ts <= end]

    def window_feature(self, window_s, end_time=None):
        if end_time is None:
            end_time = time.monotonic()
        samples = [sample for ts, sample in self.samples if end_time - window_s <= ts <= end_time]
        return mean_abs_feature(samples)

    def fail_safe_if_stale(self):
        if self.updated and time.monotonic() - self.updated > 1.0:
            self.raw = [0] * 8
            self.samples.clear()


class ThresholdDetector:
    def __init__(self, threshold_mode, threshold_value):
        self.envelope = 0.0
        self.activation = 0.0
        self.active = False
        self.started = None
        self.baseline_samples = []
        self.baseline = 0.0
        self.high = threshold_value if threshold_mode == "manual" else None
        self.low = threshold_value * 0.7 if threshold_mode == "manual" else None

    @property
    def calibrated(self):
        return self.high is not None

    def update(self, total_activity):
        now = time.monotonic()
        if self.started is None:
            self.started = now
        value = total_activity / 8.0
        self.envelope = 0.25 * value + 0.75 * self.envelope
        if not self.calibrated:
            self.baseline_samples.append(value)
            if now - self.started >= 5.0:
                self.baseline = statistics.median(self.baseline_samples)
                self.high = max(self.baseline + 5.0, self.baseline * 2.0)
                self.low = self.baseline + 0.6 * (self.high - self.baseline)
        if self.calibrated:
            span = max(1.0, self.high - self.baseline)
            self.activation = min(1.0, max(0.0, (self.envelope - self.baseline) / span))
            self.active = self.envelope >= (self.low if self.active else self.high)
        else:
            self.activation = 0.0
            self.active = False
        return {
            "activation": self.activation,
            "chosen_target": "threshold",
            "total_activity": total_activity,
            "target_similarity": self.activation,
            "best_rejection_label": "rest",
            "best_rejection_similarity": 0.0,
            "score_margin": self.activation,
            "orientation": "unknown",
            "activity_threshold": self.high or 0.0,
            "emg_effort": self.activation,
            "decision_reason": "threshold",
            "state": "TARGET" if self.active and self.calibrated else "REST",
        }


class IntentDetector:
    def __init__(self, args):
        self.window_s = args.window_ms / 1000.0
        self.step_s = args.step_ms / 1000.0
        self.gyro_override = None if str(args.gyro_threshold).lower() == "auto" else float(args.gyro_threshold)
        self.gyro_threshold = self.gyro_override or 200.0
        self.min_score = args.min_score
        self.target_override = args.target_override
        self.repetitions = args.calibration_repetitions
        self.on_similarity = 0.80
        self.off_similarity = 0.70
        self.on_margin = 0.05
        self.off_margin = 0.02
        self.on_hold_s = 0.15
        self.off_hold_s = 0.20
        self.orientation_quaternions = {}
        self.rest_thresholds = {}
        self.target_templates = {}
        self.target_activity = {}
        self.static_templates = {}
        self.static_activity = {}
        self.motion_templates = {}
        self.rankings = []
        self.chosen_target = None
        self.active = False
        self.pending_on_since = None
        self.pending_off_since = None
        self.newly_calibrated = False
        self.loaded_preset = None
        self.weak_calibration = False

    async def calibrate(self, stream, seconds, settle_s, transition_s):
        print("Intent calibration starting.")
        print("\nSTANDARD POSTURE")
        for instruction in CALIBRATION_POSTURE:
            print(f"- {instruction}")
        await asyncio.to_thread(input, "\nSet up this posture, then press Enter to begin: ")

        rest_trials = {}
        for orientation in ORIENTATIONS:
            rest_trials[orientation] = await collect_trials(
                stream, "rest", orientation, self.repetitions, seconds, settle_s, transition_s,
                self.window_s, self.step_s,
            )
        if not stream.has_imu:
            raise RuntimeError("Myo IMU data is required for three-orientation calibration")

        candidate_labels = (self.target_override,) if self.target_override != "auto" else CANDIDATE_LABELS
        candidate_trials = {
            label: {"handshake": await collect_trials(
                stream, label, "handshake", self.repetitions, seconds, settle_s, transition_s,
                self.window_s, self.step_s,
            )}
            for label in candidate_labels
        }
        static_trials = {
            label: {
                orientation: await collect_trials(
                    stream, label, orientation, self.repetitions, seconds, settle_s, transition_s,
                    self.window_s, self.step_s,
                )
                for orientation in ORIENTATIONS
            }
            for label in STATIC_REJECTIONS
        }
        motion_trials = {
            label: await collect_trials(
                stream, label, "handshake", self.repetitions, seconds, settle_s, transition_s,
                self.window_s, self.step_s,
            )
            for label in MOTION_REJECTIONS
        }

        self.orientation_quaternions = {
            orientation: mean_quaternion([trial["quaternion"] for trial in trials])
            for orientation, trials in rest_trials.items()
        }
        self.rest_thresholds = {}
        for orientation, trials in rest_trials.items():
            totals = [sum(window) for trial in trials for window in trial["windows"]]
            self.rest_thresholds[orientation] = statistics.fmean(totals) + 3.0 * statistics.pstdev(totals)
        self.static_templates = {
            label: {
                orientation: self._template(trials)
                for orientation, trials in orientations.items()
            }
            for label, orientations in static_trials.items()
        }
        self.static_activity = {
            label: {
                orientation: self._activity_stats(trials)
                for orientation, trials in orientations.items()
            }
            for label, orientations in static_trials.items()
        }
        self.motion_templates = {
            label: [self._template([trial]) for trial in trials]
            for label, trials in motion_trials.items()
        }
        rest_gyro = [value for trials in rest_trials.values() for trial in trials for value in trial["gyro"]]
        motion_gyro = [value for trials in motion_trials.values() for trial in trials for value in trial["gyro"]]
        if self.gyro_override is None:
            rest_limit, motion_floor = percentile(rest_gyro, 0.99), percentile(motion_gyro, 0.50)
            if motion_floor > rest_limit:
                self.gyro_threshold = (rest_limit + motion_floor) / 2.0
            else:
                self.gyro_threshold = 200.0
                print("Warning: rest and motion gyro ranges overlap; using conservative threshold 200.")

        rankings = [
            self._rank_candidate(label, trials["handshake"], static_trials)
            for label, trials in candidate_trials.items()
        ]
        rankings.sort(key=lambda item: item["score"], reverse=True)
        self.rankings = rankings
        print()
        print("Ranked candidate targets:")
        print("gesture_name         score  target_sim  worst_rejection     worst_sim  activity_margin")
        for row in rankings:
            print(
                f"{row['gesture_name']:18} {row['score']:5.2f}     {row['target_similarity']:5.2f}      "
                f"{row['worst_rejection_label']:16} {row['worst_rejection_similarity']:5.2f}      {row['activity_margin']:7.1f}"
            )

        ordered = [row["gesture_name"] for row in rankings]
        for candidate in ordered:
            for orientation in ORIENTATIONS[1:]:
                candidate_trials[candidate][orientation] = await collect_trials(
                    stream, candidate, orientation, self.repetitions, seconds, settle_s, transition_s,
                    self.window_s, self.step_s,
                )
            while True:
                row = self._rank_candidate(candidate, candidate_trials[candidate]["handshake"], static_trials)
                templates = {
                    orientation: self._template(trials)
                    for orientation, trials in candidate_trials[candidate].items()
                }
                activity = {
                    orientation: self._activity_stats(trials)
                    for orientation, trials in candidate_trials[candidate].items()
                }
                results = [
                    self._validate_orientation(
                        orientation, candidate_trials[candidate][orientation], templates[orientation], activity[orientation]
                    )
                    for orientation in ORIENTATIONS
                ]
                for result in results:
                    print(
                        f"validate {candidate:18} {result['orientation']:10} target={result['target_similarity']:.2f} "
                        f"reject={result['best_rejection_similarity']:.2f} margin={result['score_margin']:.2f} "
                        f"activity_margin={result['activity_margin']:.1f} {'PASS' if result['passed'] else 'FAIL'}"
                    )
                if row["score"] >= self.min_score and all(result["passed"] for result in results):
                    answer = "use"
                else:
                    answer = (await asyncio.to_thread(
                        input,
                        "Weak target: Enter=use diagnostic, a=redo all, h=redo handshake, "
                        "d=redo palm-down, u=redo palm-up, n=reject target: ",
                    )).strip().lower() or "use"
                redo = {
                    "a": ORIENTATIONS,
                    "h": ("handshake",),
                    "d": ("palm_down",),
                    "u": ("palm_up",),
                }.get(answer)
                if redo:
                    for orientation in redo:
                        candidate_trials[candidate][orientation] = await collect_trials(
                            stream, candidate, orientation, self.repetitions, seconds, settle_s, transition_s,
                            self.window_s, self.step_s,
                        )
                    continue
                if answer in {"use", "y", "yes"}:
                    self.chosen_target = candidate
                    self.target_templates = templates
                    self.target_activity = activity
                    self.weak_calibration = not (row["score"] >= self.min_score and all(result["passed"] for result in results))
                    if self.weak_calibration:
                        minimum_margin = min(result["score_margin"] for result in results)
                        self.on_margin = min(self.on_margin, max(-0.05, minimum_margin * 0.8))
                        self.off_margin = self.on_margin - 0.02
                    break
                if answer == "n":
                    break
                print("Enter one of: a, h, d, u, n, or press Enter.")
            if self.chosen_target:
                break
            print(f"{candidate} failed cross-orientation validation; trying the next ranked candidate.")

        if self.chosen_target is None:
            raise RuntimeError("No target passed all three orientations; motor remains stopped")
        validation_ok = await self.validate_labels(stream, seconds, settle_s, transition_s)
        self.weak_calibration = self.weak_calibration or not validation_ok
        self.newly_calibrated = True
        quality = "WEAK / DIAGNOSTIC ONLY" if self.weak_calibration else "PASS"
        print(f"Selected target: {self.chosen_target}; quality={quality}; gyro threshold={self.gyro_threshold:.1f}")

    async def validate_labels(self, stream, seconds, settle_s, transition_s):
        print("\nIndependent labeled validation (handshake orientation).")
        expected = {
            "rest": "REST",
            self.chosen_target: "TARGET",
            "open_hand": "GESTURE_REJECT",
            "full_fist": "GESTURE_REJECT",
        }
        passed = True
        print("label                 expected         accuracy")
        for label, expected_state in expected.items():
            trial = (await collect_trials(
                stream, label, "handshake", 1, seconds, settle_s, transition_s,
                self.window_s, self.step_s,
            ))[0]
            gyro = statistics.median(trial["gyro"])
            states = [
                self.evaluate_feature(window, trial["quaternion"], True, gyro)["state"]
                for window in trial["windows"]
            ]
            accuracy = sum(state == expected_state for state in states) / len(states)
            passed = passed and accuracy >= 0.60
            print(f"{label:20} {expected_state:16} {accuracy * 100:6.1f}% {'PASS' if accuracy >= 0.60 else 'FAIL'}")
        if not passed:
            print("Validation is below 60%; this calibration remains motor-disabled diagnostic only.")
        return passed

    @staticmethod
    def _template(trials):
        patterns = [normalize_pattern(window) for trial in trials for window in trial["windows"]]
        return normalize_pattern(mean_vector(patterns))

    @staticmethod
    def _activity_stats(trials):
        totals = [sum(window) for trial in trials for window in trial["windows"]]
        return {"mean": statistics.fmean(totals), "std": statistics.pstdev(totals)}

    @staticmethod
    def _class_score(feature, template, activity):
        total = sum(feature)
        activity_penalty = min(0.30, 0.12 * abs(math.log((total + 1.0) / (activity["mean"] + 1.0))))
        return cosine_similarity(normalize_pattern(feature), template) - activity_penalty

    @staticmethod
    def _cross_similarity(trials):
        scores = []
        for index, trial in enumerate(trials):
            training = [other for i, other in enumerate(trials) if i != index]
            template = IntentDetector._template(training)
            activity = IntentDetector._activity_stats(training)
            scores.extend(IntentDetector._class_score(window, template, activity) for window in trial["windows"])
        return statistics.fmean(scores)

    def _rank_candidate(self, label, trials, static_trials):
        template = self._template(trials)
        activity = self._activity_stats(trials)
        target_similarity = self._cross_similarity(trials)
        rejection_means = []
        for rejection in STATIC_REJECTIONS:
            windows = [window for trial in static_trials[rejection]["handshake"] for window in trial["windows"]]
            rejection_means.append((rejection, statistics.fmean(
                self._class_score(window, template, activity) for window in windows
            )))
        worst_label, worst_similarity = max(rejection_means, key=lambda item: item[1])
        target_activity = statistics.fmean(sum(window) for trial in trials for window in trial["windows"])
        activity_margin = target_activity - self.rest_thresholds["handshake"]
        penalty = max(0.0, -activity_margin / max(self.rest_thresholds["handshake"], 1.0))
        return {
            "gesture_name": label,
            "score": target_similarity - worst_similarity - penalty,
            "target_similarity": target_similarity,
            "worst_rejection_label": worst_label,
            "worst_rejection_similarity": worst_similarity,
            "activity_margin": activity_margin,
        }

    def _validate_orientation(self, orientation, trials, target_template, target_activity=None):
        target_activity = target_activity or self._activity_stats(trials)
        target_similarity = self._cross_similarity(trials)
        windows = [window for trial in trials for window in trial["windows"]]
        rejection_scores = {
            label: statistics.fmean(
                self._class_score(window, templates[orientation], self.static_activity[label][orientation])
                for window in windows
            )
            for label, templates in self.static_templates.items()
        }
        best_label = max(rejection_scores, key=rejection_scores.get)
        best_similarity = rejection_scores[best_label]
        mean_activity = target_activity["mean"]
        activity_margin = mean_activity - self.rest_thresholds[orientation]
        margin = target_similarity - best_similarity
        return {
            "orientation": orientation,
            "target_similarity": target_similarity,
            "best_rejection_label": best_label,
            "best_rejection_similarity": best_similarity,
            "score_margin": margin,
            "activity_margin": activity_margin,
            "passed": target_similarity >= self.on_similarity and margin >= self.on_margin and activity_margin > 0,
        }

    def orientation_for(self, quaternion, has_imu=True):
        if not has_imu or not self.orientation_quaternions:
            return "handshake"
        return max(self.orientation_quaternions, key=lambda name: quaternion_similarity(quaternion, self.orientation_quaternions[name]))

    def preset_data(self):
        return {
            "version": PRESET_VERSION,
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "chosen_target": self.chosen_target,
            "orientation_quaternions": self.orientation_quaternions,
            "rest_thresholds": self.rest_thresholds,
            "target_templates": self.target_templates,
            "target_activity": self.target_activity,
            "static_templates": self.static_templates,
            "static_activity": self.static_activity,
            "motion_templates": self.motion_templates,
            "gyro_threshold": self.gyro_threshold,
            "weak_calibration": self.weak_calibration,
            "on_margin": self.on_margin,
            "off_margin": self.off_margin,
            "rankings": self.rankings,
        }

    def load_preset(self, path):
        with Path(path).open() as file:
            data = json.load(file)
        if data.get("version") != PRESET_VERSION:
            raise ValueError(f"unsupported calibration preset version: {data.get('version')}")
        required = ("chosen_target", "orientation_quaternions", "rest_thresholds", "target_templates", "target_activity", "static_templates", "static_activity", "motion_templates", "gyro_threshold")
        if any(key not in data for key in required) or set(data["rest_thresholds"]) != set(ORIENTATIONS):
            raise ValueError("invalid calibration preset")
        if data["chosen_target"] not in CANDIDATE_LABELS:
            raise ValueError("preset target is not supported")
        valid_vector = lambda values, length: isinstance(values, list) and len(values) == length and all(isinstance(value, (int, float)) and math.isfinite(value) for value in values)
        valid_stats = lambda values: isinstance(values, dict) and set(values) == {"mean", "std"} and all(isinstance(value, (int, float)) and math.isfinite(value) for value in values.values()) and values["mean"] > 0 and values["std"] >= 0
        valid_orientations = (
            set(data["orientation_quaternions"]) == set(ORIENTATIONS)
            and set(data["target_templates"]) == set(ORIENTATIONS)
            and all(valid_vector(data["orientation_quaternions"][name], 4) for name in ORIENTATIONS)
            and all(valid_vector(data["target_templates"][name], 8) for name in ORIENTATIONS)
            and set(data["target_activity"]) == set(ORIENTATIONS)
            and all(valid_stats(data["target_activity"][name]) for name in ORIENTATIONS)
        )
        valid_static = set(data["static_templates"]) == set(STATIC_REJECTIONS) and all(
            set(data["static_templates"][label]) == set(ORIENTATIONS)
            and all(valid_vector(data["static_templates"][label][name], 8) for name in ORIENTATIONS)
            for label in STATIC_REJECTIONS
        )
        valid_static_activity = set(data["static_activity"]) == set(STATIC_REJECTIONS) and all(
            set(data["static_activity"][label]) == set(ORIENTATIONS)
            and all(valid_stats(data["static_activity"][label][name]) for name in ORIENTATIONS)
            for label in STATIC_REJECTIONS
        )
        valid_motion = set(data["motion_templates"]) == set(MOTION_REJECTIONS) and all(
            data["motion_templates"][label]
            and all(valid_vector(template, 8) for template in data["motion_templates"][label])
            for label in MOTION_REJECTIONS
        )
        if not valid_orientations or not valid_static or not valid_static_activity or not valid_motion:
            raise ValueError("calibration preset contains invalid templates")
        if any(not math.isfinite(float(value)) or float(value) <= 0 for value in data["rest_thresholds"].values()):
            raise ValueError("calibration preset contains invalid activity thresholds")
        if not math.isfinite(float(data["gyro_threshold"])) or float(data["gyro_threshold"]) <= 0:
            raise ValueError("calibration preset contains invalid gyro threshold")
        self.chosen_target = data["chosen_target"]
        self.orientation_quaternions = data["orientation_quaternions"]
        self.rest_thresholds = {key: float(value) for key, value in data["rest_thresholds"].items()}
        self.target_templates = data["target_templates"]
        self.target_activity = data["target_activity"]
        self.static_templates = data["static_templates"]
        self.static_activity = data["static_activity"]
        self.motion_templates = data["motion_templates"]
        self.gyro_threshold = self.gyro_override or float(data["gyro_threshold"])
        self.weak_calibration = bool(data.get("weak_calibration", False))
        self.on_margin = float(data.get("on_margin", self.on_margin))
        self.off_margin = float(data.get("off_margin", self.off_margin))
        self.rankings = data.get("rankings", [])
        self.loaded_preset = str(path)

    def save_preset(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as file:
            json.dump(self.preset_data(), file, indent=2)
        return path

    def evaluate_feature(self, feature, quaternion, has_imu, gyro_mag):
        total_activity = sum(feature)
        orientation = self.orientation_for(quaternion, has_imu)
        activity_threshold = self.rest_thresholds[orientation]
        target_similarity = self._class_score(feature, self.target_templates[orientation], self.target_activity[orientation])
        rejection_scores = {
            label: self._class_score(feature, templates[orientation], self.static_activity[label][orientation])
            for label, templates in self.static_templates.items()
        }
        best_rejection_label = max(rejection_scores, key=rejection_scores.get)
        best_rejection_similarity = rejection_scores[best_rejection_label]
        score_margin = target_similarity - best_rejection_similarity
        gyro_blocked = has_imu and gyro_mag > self.gyro_threshold
        target_span = max(5.0, self.target_activity[orientation]["mean"] - activity_threshold)
        emg_effort = min(1.5, max(0.0, (total_activity - activity_threshold) / target_span))
        target_condition = (
            total_activity > activity_threshold
            and target_similarity >= self.on_similarity
            and score_margin >= self.on_margin
            and not gyro_blocked
        )
        if target_condition:
            state = "TARGET"
        elif total_activity <= activity_threshold:
            state = "REST"
        elif gyro_blocked:
            state = "MOTION_REJECT"
        else:
            state = "GESTURE_REJECT"
        return {
            "total_activity": total_activity,
            "target_similarity": target_similarity,
            "best_rejection_label": best_rejection_label,
            "best_rejection_similarity": best_rejection_similarity,
            "score_margin": score_margin,
            "orientation": orientation,
            "activity_threshold": activity_threshold,
            "emg_effort": emg_effort,
            "state": state,
        }

    def update(self, stream):
        now = time.monotonic()
        feature = stream.window_feature(self.window_s, now)
        result = self.evaluate_feature(feature, stream.quaternion, stream.has_imu, stream.gyro_mag)
        on_condition = result["state"] == "TARGET"
        off_condition = (
            result["total_activity"] <= result["activity_threshold"]
            or result["target_similarity"] < self.off_similarity
            or result["score_margin"] < self.off_margin
            or result["state"] == "MOTION_REJECT"
        )

        if self.active:
            self.pending_on_since = None
            if off_condition:
                if self.pending_off_since is None:
                    self.pending_off_since = now
                elif now - self.pending_off_since >= self.off_hold_s:
                    self.active = False
                    self.pending_off_since = None
            else:
                self.pending_off_since = None
        else:
            self.pending_off_since = None
            if on_condition:
                if self.pending_on_since is None:
                    self.pending_on_since = now
                elif now - self.pending_on_since >= self.on_hold_s:
                    self.active = True
                    self.pending_on_since = None
            else:
                self.pending_on_since = None

        if self.active:
            state = "TARGET"
        else:
            state = result["state"]

        return {
            "activation": 1.0 if self.active else 0.0,
            "chosen_target": self.chosen_target,
            **{key: result[key] for key in (
                "total_activity", "target_similarity", "best_rejection_label",
                "best_rejection_similarity", "score_margin", "orientation",
                "activity_threshold", "emg_effort",
            )},
            "decision_reason": state.lower(),
            "state": state,
        }


class Motor:
    def __init__(self, port, baud):
        try:
            import serial
            from serial.tools import list_ports
        except ImportError as error:
            raise SystemExit("pyserial is missing; run: pip install -r requirements.txt") from error
        if not port:
            ports = list(list_ports.comports())
            matching = [p.device for p in ports if "STM" in p.description.upper() or "ST-LINK" in p.description.upper()]
            port = matching[0] if matching else (ports[0].device if ports else "/dev/ttyACM0")
        self.link = serial.Serial(port, baud, timeout=0.1, write_timeout=1)
        self.command = None
        self.send("0")

    def send(self, command):
        self.link.write((command + "\n").encode("ascii"))
        self.link.flush()
        self.command = command

    def stop(self):
        try:
            self.send("0")
        except Exception:
            pass

    def close(self):
        self.stop()
        self.link.close()


ORIENTATION_PROMPTS = {
    "handshake": "Forearm supported; thumb points upward as if offering a handshake.",
    "palm_down": "Forearm supported; rotate the whole forearm until the palm faces down.",
    "palm_up": "Forearm supported; rotate the whole forearm until the palm faces up.",
}


async def collect_trials(stream, label, orientation, repetitions, seconds, settle_s, transition_s, window_s, step_s):
    trials = []
    moving = label in MOTION_REJECTIONS
    for repetition in range(1, repetitions + 1):
        if repetition > 1:
            print(f"Relax before repeating. Next attempt in {transition_s:.1f} s.")
            await asyncio.sleep(transition_s)
        while True:
            print()
            print(f"{label} — {orientation} — attempt {repetition}/{repetitions} — {'MOVE' if moving else 'HOLD'}")
            print(ORIENTATION_PROMPTS[orientation])
            print(CALIBRATION_PROMPTS[label])
            print(CALIBRATION_TIPS[label])
            print(calibration_goal(label))
            print(f"Recording: {seconds:.1f} s. Press Enter only when the starting position is correct.")
            await asyncio.to_thread(input, "Ready: ")
            for remaining in range(int(settle_s), 0, -1):
                print(f"Stay ready. Recording starts in {remaining}...", end="\r", flush=True)
                await asyncio.sleep(1)
            print("MOVE slowly now.                         " if moving else "HOLD completely still now.                ")
            start = time.monotonic()
            await asyncio.sleep(seconds)
            end = time.monotonic()
            trace = stream.samples_between(start, end)
            windows = windows_from_trace(trace, start, end, window_s, step_s)
            imu = stream.imu_between(start, end)
            if len(windows) < 5:
                sample_age = end - stream.updated if stream.updated else float("inf")
                print(
                    f"Recording discarded: only {len(trace)} EMG samples produced {len(windows)} windows "
                    f"(last sample age {sample_age:.1f} s). Repeating this attempt only."
                )
                print("Check that the armband is snug and awake. Press Ctrl+C if this repeats continuously.")
                await asyncio.sleep(transition_s)
                continue
            question = "Was that one clean, slow movement with no finger squeeze?" if moving else "Was that a clean, motionless hold at the requested effort?"
            answer = (await asyncio.to_thread(input, f"{question} [Y/n]: ")).strip().lower()
            if answer in {"", "y", "yes"}:
                trials.append({
                    "windows": windows,
                    "gyro": [gyro for gyro, _ in imu] or [stream.gyro_mag],
                    "quaternion": mean_quaternion([quaternion for _, quaternion in imu] or [stream.quaternion]),
                })
                mean_activity = statistics.fmean(sum(window) for window in windows)
                print(f"Accepted: windows={len(windows)} mean_activity={mean_activity:.1f}")
                break
            print("Discarded. Relax completely, then repeat this attempt.")
    return trials


async def myo_source(stream, address, ready):
    try:
        from bleak import BleakClient, BleakScanner
    except ImportError as error:
        raise SystemExit("bleak is missing; run: pip install -r requirements.txt") from error
    def is_myo(device, advertisement):
        name = device.name or advertisement.local_name or ""
        return CONTROL_SERVICE_UUID in {uuid.lower() for uuid in advertisement.service_uuids} or any(
            marker in name.lower() for marker in ("myo", "thalmic")
        )

    if address:
        device = await BleakScanner.find_device_by_address(address, timeout=15)
    else:
        device = None
    if device is None:
        if address:
            print(f"Myo address {address} is not advertising; scanning for its service UUID...")
        device = await BleakScanner.find_device_by_filter(is_myo, timeout=20)
    if device is None:
        raise RuntimeError("Myo not advertising; run: .venv/bin/python myo_scan.py --seconds 30")

    disconnected = asyncio.Event()
    async with BleakClient(device, disconnected_callback=lambda _: disconnected.set()) as client:
        uuids = {characteristic.uuid.lower() for service in client.services for characteristic in service.characteristics}

        def emg_callback(_, data):
            values = struct.unpack("16b", data)
            stream.update_emg(values[:8])
            stream.update_emg(values[8:])

        def imu_callback(_, data):
            stream.update_imu(data)

        for uuid in EMG_UUIDS:
            await client.start_notify(uuid, emg_callback)
        if IMU_UUID in uuids:
            try:
                await client.start_notify(IMU_UUID, imu_callback)
            except Exception as error:
                print(f"Myo IMU notification setup failed: {error}")
        await client.write_gatt_char(COMMAND_UUID, bytes((0x09, 0x01, 0x01)), response=True)
        await client.write_gatt_char(COMMAND_UUID, bytes((0x0A, 0x01, 0x02)), response=True)
        await client.write_gatt_char(COMMAND_UUID, MYO_STREAM_COMMAND, response=True)
        print(f"Connected to Myo {device.address}.")
        ready.set()
        await disconnected.wait()
        raise ConnectionError("Myo disconnected")


async def keyboard_source(stream, stop, ready):
    print("Keyboard fallback: type 1 (target), 0 (rest), or q.")
    ready.set()
    while not stop.is_set():
        value = (await asyncio.to_thread(input, "activation> ")).strip().lower()
        if value == "q":
            stop.set()
        elif value in {"0", "1"}:
            stream.force_activation(float(value))


async def synthetic_source(stream, stop, duration, ready):
    started = time.monotonic()
    ready.set()
    while not stop.is_set() and (duration <= 0 or time.monotonic() - started < duration):
        stream.force_activation(0.5 + 0.5 * math.sin(2 * math.pi * (time.monotonic() - started) / 4.0))
        await asyncio.sleep(0.02)
    stop.set()


def choose_calibration_preset(directory=Path("calibrations")):
    paths = sorted(directory.glob("*.json"), reverse=True) if directory.exists() else []
    presets = []
    for path in paths:
        try:
            with path.open() as file:
                data = json.load(file)
            if data.get("version") == PRESET_VERSION:
                presets.append((path, data))
            else:
                print(f"Ignoring incompatible calibration preset: {path.name}")
        except (OSError, ValueError):
            print(f"Ignoring invalid calibration preset: {path.name}")
    if not presets:
        return None
    print("\nSaved calibration presets (valid only for the same wearer, arm, and unchanged Myo placement):")
    print("  0: new calibration")
    for index, (path, data) in enumerate(presets, 1):
        description = f"{data.get('created_at', 'unknown date')} — {data.get('chosen_target', 'unknown target')}"
        print(f"  {index}: {path.name} — {description}")
    while True:
        answer = input("Choose a preset number, or 0 for new calibration [0]: ").strip() or "0"
        if answer.isdigit() and 0 <= int(answer) <= len(presets):
            return None if answer == "0" else presets[int(answer) - 1][0]
        print("Enter one of the listed numbers.")


async def run(args):
    template_like = args.detector in {"intent", "template"}
    if template_like and args.source != "myo":
        raise SystemExit("intent detector requires --source myo")
    if args.threshold == "manual" and args.threshold_value <= 0:
        raise SystemExit("--threshold manual requires --threshold-value greater than zero")
    if args.live_plot:
        from plot_log import live_plot_error
        error = live_plot_error()
        if error:
            print(f"Live plot disabled: {error}")
            args.live_plot = False

    stream = EMGStream()
    detector = IntentDetector(args) if template_like else ThresholdDetector(args.threshold, args.threshold_value)
    if template_like and args.calibration_preset:
        detector.load_preset(args.calibration_preset)
        print(f"Loaded calibration: {args.calibration_preset} ({detector.chosen_target})")
    if args.detector == "threshold" and args.source != "myo" and args.threshold == "auto":
        detector.high, detector.low = 63.5, 44.45
    model = SEAModel(args.mode)
    motor = Motor(args.serial_port, args.baud) if args.motor == "enable" else None
    stop = asyncio.Event()
    ready = asyncio.Event()
    log_path = Path(args.log or f"logs/session_{datetime.now():%Y%m%d_%H%M%S}.csv")
    log_path.parent.mkdir(parents=True, exist_ok=True)

    fields = [
        "timestamp",
        *[f"emg_{i}" for i in range(1, 9)],
        *[f"mav_{i}" for i in range(1, 9)],
        "total_activity",
        "chosen_target",
        "target_similarity",
        "best_rejection_label",
        "best_rejection_similarity",
        "score_margin",
        "gyro_mag",
        "orientation",
        "activity_threshold",
        "emg_effort",
        "decision_reason",
        "state",
        "command",
        "mode",
        "detector",
        "x_motor_mm",
        "x_finger_mm",
        "x_spring_mm",
        "force_N",
        "theta_deg",
        "EMD_ms",
        "safety_limited",
    ]

    if args.source == "myo":
        source = asyncio.create_task(myo_source(stream, args.myo_address, ready))
    elif args.source == "keyboard":
        source = asyncio.create_task(keyboard_source(stream, stop, ready))
    else:
        source = asyncio.create_task(synthetic_source(stream, stop, args.duration, ready))

    last_heartbeat = 0.0
    plot_queue = None
    plot_process = None
    plot_cursor = time.monotonic()
    signal_installed = False
    graceful = False
    try:
        while not ready.is_set():
            if source.done():
                source.result()
            await asyncio.sleep(0.1)

        ready_at = time.monotonic()
        while args.source == "myo" and stream.started is None:
            if source.done():
                source.result()
            if time.monotonic() - ready_at > 3.0:
                raise TimeoutError("Myo connected but no EMG samples arrived")
            await asyncio.sleep(0.1)

        if template_like and args.source == "myo":
            imu_ready_at = time.monotonic()
            while not stream.has_imu:
                if source.done():
                    source.result()
                if time.monotonic() - imu_ready_at > 3.0:
                    raise TimeoutError("Myo connected but no IMU samples arrived; calibration did not start")
                await asyncio.sleep(0.1)

        if template_like and not detector.loaded_preset:
            await detector.calibrate(
                stream,
                args.calibration_seconds,
                args.calibration_settle_seconds,
                args.calibration_transition_seconds,
            )
        elif template_like:
            print("Skipping calibration. Verify REST and every rejection before enabling the motor.")
        elif args.source == "myo" and args.threshold == "auto":
            print("Threshold mode: keep relaxed for 5 s auto calibration.")

        if template_like and detector.weak_calibration and args.motor == "enable":
            raise RuntimeError("Weak calibration is diagnostic-only; rerun with --motor disable")

        if args.live_plot:
            from plot_log import run_live_plot
            plot_queue = mp.Queue(maxsize=5)
            plot_process = mp.Process(target=run_live_plot, args=(plot_queue, 10.0), daemon=True)
            try:
                plot_process.start()
                plot_cursor = time.monotonic()
            except (OSError, RuntimeError) as error:
                print(f"Live plot unavailable ({error}); continuing with terminal output.")
                plot_queue.close()
                plot_queue = None
                plot_process = None

        def request_stop():
            if motor:
                motor.stop()
            stop.set()

        try:
            asyncio.get_running_loop().add_signal_handler(signal.SIGINT, request_stop)
            signal_installed = True
        except (NotImplementedError, RuntimeError):
            pass

        with log_path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            while not stop.is_set():
                if args.duration > 0 and stream.started is not None and time.monotonic() - stream.started >= args.duration:
                    stop.set()
                    continue
                if source.done():
                    source.result()
                    break

                stream.fail_safe_if_stale()
                feature = stream.window_feature(args.window_ms / 1000.0)
                total_activity = sum(feature)
                detection = detector.update(stream) if template_like else detector.update(total_activity)
                now = time.time()
                result = model.update(detection["activation"], now)
                command = "1" if detection["state"] == "TARGET" else "0"
                if motor and (motor.command != command or time.monotonic() - last_heartbeat >= 0.5):
                    motor.send(command)
                    last_heartbeat = time.monotonic()
                shown_command = command if motor else f"DRY-{command}"
                print(
                    f"\rtarget={detection['chosen_target']:18} orientation={detection['orientation']:10} activity={detection['total_activity']:6.1f} "
                    f"effort={detection['emg_effort'] * 100:5.0f}% target_score={detection['target_similarity']:.2f} "
                    f"reject={detection['best_rejection_label']}:{detection['best_rejection_similarity']:.2f} margin={detection['score_margin']:.2f} "
                    f"virtual_force={result['force_N']:4.2f}N virtual_angle={result['theta_deg']:4.1f} "
                    f"state={detection['state']:14} command={shown_command:6}",
                    end="",
                    flush=True,
                )
                writer.writerow({
                    "timestamp": now,
                    **{f"emg_{i}": value for i, value in enumerate(stream.raw, 1)},
                    **{f"mav_{i}": value for i, value in enumerate(feature, 1)},
                    "total_activity": detection["total_activity"],
                    "chosen_target": detection["chosen_target"],
                    "target_similarity": detection["target_similarity"],
                    "best_rejection_label": detection["best_rejection_label"],
                    "best_rejection_similarity": detection["best_rejection_similarity"],
                    "score_margin": detection["score_margin"],
                    "gyro_mag": stream.gyro_mag,
                    "orientation": detection["orientation"],
                    "activity_threshold": detection["activity_threshold"],
                    "emg_effort": detection["emg_effort"],
                    "decision_reason": detection["decision_reason"],
                    "state": detection["state"],
                    "command": command,
                    "mode": args.mode,
                    "detector": "intent" if template_like else "threshold",
                    **{key: result[key] for key in ("x_motor_mm", "x_finger_mm", "x_spring_mm", "force_N", "theta_deg", "EMD_ms", "safety_limited")},
                })
                file.flush()
                if plot_queue is not None:
                    plot_now = time.monotonic()
                    message = {
                        "time": plot_now,
                        "raw": stream.samples_between(plot_cursor, plot_now),
                        "mav": feature,
                        "detection": detection,
                        "command": command,
                        "gyro": stream.gyro_mag,
                    }
                    plot_cursor = plot_now
                    try:
                        plot_queue.put_nowait(message)
                    except queue.Full:
                        pass
                await asyncio.sleep(0.1)
        graceful = True
    finally:
        print()
        if motor:
            motor.stop()
        source.cancel()
        await asyncio.gather(source, return_exceptions=True)
        if motor:
            motor.close()
        if plot_queue is not None:
            try:
                plot_queue.put_nowait(None)
            except queue.Full:
                pass
        if plot_process is not None and plot_process.pid is not None:
            plot_process.join(timeout=2)
            if plot_process.is_alive():
                plot_process.terminate()
                plot_process.join(timeout=1)
        if signal_installed:
            asyncio.get_running_loop().remove_signal_handler(signal.SIGINT)
        if log_path.exists():
            print(f"Log saved: {log_path}")
    if graceful and template_like and detector.newly_calibrated:
        answer = (await asyncio.to_thread(input, "Save this validated calibration preset? [Y/n]: ")).strip().lower()
        if answer in {"", "y", "yes"}:
            path = Path(f"calibrations/calibration_{datetime.now():%Y%m%d_%H%M%S}.json")
            detector.save_preset(path)
            print(f"Calibration saved: {path}")


def main():
    parser = argparse.ArgumentParser(description="Myo -> intent gesture detector -> virtual SEA -> optional STM32 motor demo")
    parser.add_argument("--myo-address")
    parser.add_argument("--serial-port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mode", choices=("diagnostic", "assistive"), default="diagnostic")
    parser.add_argument("--motor", choices=("enable", "disable"), default="disable")
    parser.add_argument("--detector", choices=("intent", "template", "threshold"), default="intent")
    parser.add_argument("--target-override", choices=("auto", *CANDIDATE_LABELS), default="pinch_attempt")
    parser.add_argument("--threshold", choices=("auto", "manual"), default="auto")
    parser.add_argument("--threshold-value", type=float, default=20.0)
    parser.add_argument("--window-ms", type=int, default=200)
    parser.add_argument("--step-ms", type=int, default=50)
    parser.add_argument("--calibration-seconds", type=float, default=2.0)
    parser.add_argument("--calibration-repetitions", type=int, default=3)
    parser.add_argument("--calibration-settle-seconds", type=float, default=4.0)
    parser.add_argument("--calibration-transition-seconds", type=float, default=3.0)
    parser.add_argument("--gyro-threshold", default="auto", help="auto or a positive numeric override")
    parser.add_argument("--min-score", type=float, default=0.08)
    parser.add_argument("--live-plot", action="store_true", help="open rolling raw EMG/MAV dashboard")
    parser.add_argument("--log")
    parser.add_argument("--source", choices=("myo", "keyboard", "synthetic"), default="myo")
    parser.add_argument("--duration", type=float, default=0.0, help="run duration in seconds; 0 is unlimited")
    args = parser.parse_args()
    if args.calibration_repetitions < 2:
        parser.error("--calibration-repetitions must be at least 2 for cross-validation")
    if args.calibration_seconds <= args.window_ms / 1000.0:
        parser.error("--calibration-seconds must be longer than --window-ms")
    if str(args.gyro_threshold).lower() != "auto":
        try:
            if float(args.gyro_threshold) <= 0:
                raise ValueError
        except ValueError:
            parser.error("--gyro-threshold must be auto or a positive number")
    args.calibration_preset = choose_calibration_preset() if args.detector in {"intent", "template"} and args.source == "myo" else None
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
