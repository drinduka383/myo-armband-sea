#!/usr/bin/env python3
import math
import time


class SEAModel:
    def __init__(self, mode="diagnostic", spool_radius_mm=5.0, finger_moment_arm_mm=8.0,
                 spring_k_N_per_mm=2.0, theta_max_deg=180.0, x_max_mm=10.0,
                 force_limit_N=5.0, activation_threshold=0.5, force_threshold_N=0.1,
                 max_speed_deg_s=90.0, finger_time_constant_s=0.18):
        if mode not in {"diagnostic", "assistive"}:
            raise ValueError("mode must be diagnostic or assistive")
        self.mode = mode
        self.spool_radius_mm = spool_radius_mm
        self.finger_moment_arm_mm = finger_moment_arm_mm
        self.spring_k_N_per_mm = spring_k_N_per_mm
        self.theta_max_deg = theta_max_deg
        self.x_max_mm = x_max_mm
        self.force_limit_N = force_limit_N
        self.activation_threshold = activation_threshold
        self.force_threshold_N = force_threshold_N
        self.max_speed_deg_s = max_speed_deg_s
        self.finger_time_constant_s = finger_time_constant_s
        self.theta_deg = theta_max_deg
        self.motor_theta_deg = theta_max_deg
        self._last_timestamp = None
        self._emg_onset = None
        self._force_onset = None
        self._above_activation = False
        self._above_force = False
        self.emd_ms = None

    def update(self, activation, timestamp=None):
        timestamp = time.time() if timestamp is None else timestamp
        activation = min(1.0, max(0.0, float(activation)))
        dt = 0.0 if self._last_timestamp is None else min(0.25, max(0.0, timestamp - self._last_timestamp))
        self._last_timestamp = timestamp
        target_theta_deg = self.theta_max_deg * (1.0 - activation)
        if self.mode == "diagnostic":
            self.theta_deg = 0.0
            x_finger = 0.0
            x_motor = activation * self.x_max_mm
        else:
            motor_step = self.max_speed_deg_s * dt
            motor_error = target_theta_deg - self.motor_theta_deg
            self.motor_theta_deg += min(motor_step, max(-motor_step, motor_error))
            follow = 0.0 if dt == 0 else 1.0 - math.exp(-dt / self.finger_time_constant_s)
            self.theta_deg += (self.motor_theta_deg - self.theta_deg) * follow
            x_motor = self.finger_moment_arm_mm * math.radians(self.theta_max_deg - self.motor_theta_deg)
            x_finger = self.finger_moment_arm_mm * math.radians(self.theta_max_deg - self.theta_deg)
        x_spring = abs(x_motor - x_finger)
        force = self.spring_k_N_per_mm * x_spring
        safety_limited = force > self.force_limit_N
        if safety_limited:
            force = self.force_limit_N
            x_spring = force / self.spring_k_N_per_mm

        above_activation = activation >= self.activation_threshold
        above_force = force >= self.force_threshold_N
        if above_activation and not self._above_activation:
            self._emg_onset, self._force_onset, self.emd_ms = timestamp, None, None
        if above_force and self._emg_onset is not None and self._force_onset is None:
            self._force_onset = timestamp
            self.emd_ms = 1000.0 * (self._force_onset - self._emg_onset)
        if not above_activation:
            self._emg_onset = self._force_onset = None
        self._above_activation, self._above_force = above_activation, above_force

        return {
            "time": timestamp, "activation": activation, "mode": self.mode,
            "target_theta_deg": target_theta_deg, "theta_deg": self.theta_deg,
            "x_motor_mm": x_motor, "x_finger_mm": x_finger,
            "x_spring_mm": x_spring, "force_N": force,
            "safety_limited": safety_limited, "EMD_ms": self.emd_ms,
        }


def _self_check():
    diagnostic_model = SEAModel("diagnostic")
    diagnostic = diagnostic_model.update(1, 1.0)
    assert diagnostic["theta_deg"] == 0 and diagnostic["x_spring_mm"] == 2.5
    assert diagnostic["force_N"] == 5 and diagnostic["safety_limited"]
    assert diagnostic["EMD_ms"] == 0
    assistive_model = SEAModel("assistive")
    assistive_model.update(0, 1.0)
    assistive = assistive_model.update(1, 2.0)
    assert assistive["target_theta_deg"] == 0
    assert 0 <= assistive["theta_deg"] < 180
    assert assistive["force_N"] > 0


if __name__ == "__main__":
    _self_check()
    print("SEA model self-check passed")
