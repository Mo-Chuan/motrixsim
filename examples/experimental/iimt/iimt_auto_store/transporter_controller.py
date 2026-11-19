# Copyright (C) 2020-2026 Motphys Technology Co., Ltd. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .common import ROBOT_BASE_LINK, inverse_transform_point, pose, yaw_from_quat


class PidController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.output_limit: tuple[float, float] | None = None

    def set_output_limit(self, min_value: float, max_value: float) -> None:
        self.output_limit = (min_value, max_value)

    def update(self, error: float, dt: float) -> float:
        proportional = self.kp * error
        self.integral += error * dt
        integral = self.ki * self.integral
        derivative = self.kd * (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error
        output = proportional + integral + derivative
        if self.output_limit is not None:
            output = max(self.output_limit[0], min(self.output_limit[1], output))
        return output

    def reset(self) -> None:
        self.prev_error = 0.0
        self.integral = 0.0


def world_space_target_to_local_delta(
    target_pos: np.ndarray, current_pos: np.ndarray, rotation_quat: np.ndarray
) -> tuple[float, float]:
    local_err = inverse_transform_point(
        pose(current_pos[:3], rotation_quat), np.array([target_pos[0], target_pos[1], 0.0], dtype=np.float32)
    )
    planar_distance = float(np.linalg.norm(local_err[:2]))
    target_rad = float(np.arctan2(local_err[1], local_err[0]))
    return planar_distance, target_rad


@dataclass
class TransporterLowController:
    position_pid: PidController = field(default_factory=lambda: PidController(1.0, 0.0, 0.5))
    velocity_pid: PidController = field(default_factory=lambda: PidController(20.0, 0.02, 0.5))
    rotation_pid: PidController = field(default_factory=lambda: PidController(5.0, 0.0, 0.5))
    angular_velocity_pid: PidController = field(default_factory=lambda: PidController(10.0, 0.0, 0.1))
    action_forward: float = 0.0
    action_rotate: float = 0.0
    action_brake: float = 0.0

    def __post_init__(self) -> None:
        self.position_pid.set_output_limit(-3.0, 3.0)
        self.velocity_pid.set_output_limit(-5.0, 5.0)
        self.angular_velocity_pid.set_output_limit(-10.0, 10.0)

    def update_pid(
        self, delta_forward: float, delta_rotate: float, current_linvel: float, current_gyro: float, dt: float
    ) -> None:
        target_velocity = self.position_pid.update(delta_forward, dt)
        target_angular_velocity = self.rotation_pid.update(delta_rotate, dt)
        velocity_error = target_velocity - current_linvel
        angular_velocity_error = target_angular_velocity - current_gyro
        self.action_forward = self.velocity_pid.update(velocity_error, dt)
        self.action_rotate = -self.angular_velocity_pid.update(angular_velocity_error, dt)

    def update_guide(
        self,
        target_pos: np.ndarray,
        current_pos: np.ndarray,
        current_quat: np.ndarray,
        current_linvel: float,
        current_gyro: float,
        dt: float,
    ) -> None:
        delta_distance, delta_rotate = world_space_target_to_local_delta(target_pos, current_pos, current_quat)
        # Keep the robot approaching waypoints with a consistent forward-facing strategy.
        # When heading error is large, rotate first and suppress forward motion to avoid
        # the "reverse toward the point and spin in place" behavior on one side.
        forward_scale = max(0.0, float(np.cos(delta_rotate)))
        delta_forward = delta_distance * forward_scale
        self.update_pid(delta_forward, delta_rotate, current_linvel, current_gyro, dt)
        self.action_brake = 0.0

    def brake(self, current_linvel: float, current_gyro: float, dt: float) -> None:
        self.update_pid(0.0, 0.0, current_linvel, current_gyro, dt)
        self.action_brake = 1.0

    def direct(self, forward: float, rotate: float) -> None:
        self.action_forward = forward
        self.action_rotate = rotate
        self.action_brake = 0.0

    def actions(self) -> np.ndarray:
        return np.array([self.action_forward, self.action_rotate, self.action_brake], dtype=np.float32)


@dataclass
class TransporterController:
    low: TransporterLowController = field(default_factory=TransporterLowController)
    mode: str = "idle"
    waypoints: list[np.ndarray] = field(default_factory=list)
    forward: float = 0.0
    rotate: float = 0.0
    current_pos: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    current_quat: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32))
    current_linvel: float = 0.0
    current_gyro: float = 0.0
    last_reached_waypoint: np.ndarray | None = None
    reach_position_threshold: float = 0.1
    reach_linvel_threshold: float = 0.12
    reach_gyro_threshold: float = 0.2
    align_target_yaw: float = 0.0
    align_yaw_threshold: float = 0.08

    def is_idle(self) -> bool:
        return self.mode == "idle"

    def start_waypoint_navigation(self, waypoints: list[list[float] | np.ndarray]) -> None:
        self.mode = "waypoint"
        self.waypoints = [np.asarray(point, dtype=np.float32) for point in waypoints]

    def direct_control(self, forward: float, rotate: float) -> None:
        self.mode = "direct"
        self.forward = forward
        self.rotate = rotate

    def start_yaw_alignment(self, target_yaw: float) -> None:
        self.mode = "align_yaw"
        self.align_target_yaw = float(target_yaw)

    def stop(self) -> None:
        self.mode = "idle"
        self.waypoints.clear()
        self.forward = 0.0
        self.rotate = 0.0

    def set_direct_input(self, forward: float, rotate: float) -> None:
        if abs(forward) > 1e-6 or abs(rotate) > 1e-6:
            self.direct_control(forward, rotate)
        elif self.mode == "direct":
            self.stop()

    def current_waypoint(self) -> np.ndarray | None:
        if self.mode == "waypoint" and self.waypoints:
            return self.waypoints[-1]
        return None

    def all_waypoints(self) -> list[np.ndarray]:
        return [point.copy() for point in self.waypoints]

    def update_state_from_model(self, model, data, dt: float) -> None:
        self.current_linvel = float(np.asarray(model.get_sensor_value("local_linvel", data), dtype=np.float32)[0])
        self.current_gyro = float(np.asarray(model.get_sensor_value("gyro", data), dtype=np.float32)[2])
        base_pose = np.asarray(model.get_link(ROBOT_BASE_LINK).get_pose(data), dtype=np.float32)
        self.current_pos = base_pose[:3].copy()
        self.current_quat = base_pose[3:7].copy()

        if self.mode == "idle":
            self.low.brake(self.current_linvel, self.current_gyro, dt)
            return

        if self.mode == "direct":
            self.low.direct(self.forward, self.rotate)
            return

        if self.mode == "align_yaw":
            current_yaw = float(yaw_from_quat(self.current_quat))
            yaw_error = math.atan2(
                math.sin(self.align_target_yaw - current_yaw),
                math.cos(self.align_target_yaw - current_yaw),
            )
            settled_yaw = (
                abs(yaw_error) < self.align_yaw_threshold and abs(self.current_gyro) < self.reach_gyro_threshold
            )
            if settled_yaw:
                self.stop()
                self.low.brake(self.current_linvel, self.current_gyro, dt)
                return

            self.low.update_pid(0.0, yaw_error, self.current_linvel, self.current_gyro, dt)
            self.low.action_forward = 0.0
            self.low.action_brake = 0.0
            return

        if not self.waypoints:
            self.stop()
            self.low.brake(self.current_linvel, self.current_gyro, dt)
            return

        target = self.waypoints[-1]
        dx = abs(float(target[0] - self.current_pos[0]))
        dy = abs(float(target[1] - self.current_pos[1]))
        reached_position = dx < self.reach_position_threshold and dy < self.reach_position_threshold
        settled_motion = (
            abs(self.current_linvel) < self.reach_linvel_threshold
            and abs(self.current_gyro) < self.reach_gyro_threshold
        )

        if reached_position and settled_motion:
            self.last_reached_waypoint = self.waypoints.pop().copy()
            if not self.waypoints:
                self.stop()
                self.low.brake(self.current_linvel, self.current_gyro, dt)
                return
            target = self.waypoints[-1]

        self.low.update_guide(target, self.current_pos, self.current_quat, self.current_linvel, self.current_gyro, dt)

    def actions(self) -> np.ndarray:
        return self.low.actions()
