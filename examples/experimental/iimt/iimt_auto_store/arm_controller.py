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

from dataclasses import dataclass, field

import numpy as np

from motrixsim import ik

from .common import (
    END_EFFECTOR_LINK,
    END_EFFECTOR_OFFSET,
    RESET_ARM_DOFS,
    pose,
    pose_rotation,
    pose_translation,
    quat_slerp,
    transform_vec,
)


def generate_track(start: np.ndarray, target: np.ndarray, hanging_offset: np.ndarray) -> list[np.ndarray]:
    track_points = [np.asarray(start, dtype=np.float32).copy()]

    hanging_point = np.asarray(target, dtype=np.float32).copy()
    hanging_point[:3] = hanging_point[:3] - transform_vec(hanging_point[3:7], hanging_offset)

    delta = hanging_point[:3] - start[:3]
    distance = float(np.linalg.norm(delta))
    if distance > 1e-6:
        direction = delta / distance
        step = 0.2
        num_points = int(np.floor(distance / step))
        track_translation = start[:3].copy()
        for i in range(num_points):
            track_translation = track_translation + direction * step
            t = (i + 1) / max(1, num_points)
            track_points.append(pose(track_translation, quat_slerp(start[3:7], target[3:7], t)))

    track_points.append(hanging_point.copy())
    track_points.append(hanging_point.copy())
    track_points.append(np.asarray(target, dtype=np.float32).copy())
    track_points.append(np.asarray(target, dtype=np.float32).copy())
    return track_points


def default_accuracy(way_points: list[np.ndarray]) -> list[float]:
    accuracy = [0.15 for _ in way_points]
    if len(accuracy) >= 1:
        accuracy[-1] = 0.02
    if len(accuracy) >= 2:
        accuracy[-2] = 0.02
    if len(accuracy) >= 3:
        accuracy[-3] = 0.05
    if len(accuracy) >= 4:
        accuracy[-4] = 0.01
    return accuracy


@dataclass
class ArmController:
    model: object
    chain: ik.IkChain = field(init=False)
    solver: ik.GaussNewtonSolver = field(init=False)
    mode: str = "idle"
    waypoints: list[np.ndarray] = field(default_factory=list)
    accuracy: list[float] = field(default_factory=list)
    current_segment: int = 0
    base_target: float = 0.0
    desired_dof_pos: np.ndarray = field(default_factory=lambda: RESET_ARM_DOFS.copy())
    current_pose: np.ndarray | None = None
    residual: float = 0.0

    def __post_init__(self) -> None:
        self.chain = ik.IkChain(
            self.model,
            end_link=END_EFFECTOR_LINK,
            start_link="Link_CR_1",
            end_effector_offset=END_EFFECTOR_OFFSET,
        )
        self.solver = ik.GaussNewtonSolver(max_iter=200, step_size=0.5, tolerance=1e-3)
        self.desired_dof_pos = RESET_ARM_DOFS.copy()

    def is_idle(self) -> bool:
        return self.mode == "idle"

    def reset(self) -> None:
        self.mode = "idle"
        self.waypoints = []
        self.accuracy = []
        self.current_segment = 0
        self.base_target = 0.0
        self.desired_dof_pos = RESET_ARM_DOFS.copy()

    def start_move_to_waypoints(self, waypoints: list[np.ndarray], accuracy: list[float]) -> None:
        self.mode = "moving"
        self.waypoints = [np.asarray(point, dtype=np.float32).copy() for point in waypoints]
        self.accuracy = list(accuracy)
        self.current_segment = 0

    def start_base_rotate(self, value: float) -> None:
        self.mode = "base_rotate"
        self.base_target = float(value)

    def current_debug_waypoints(self) -> list[np.ndarray]:
        return self.waypoints

    def update(self, data) -> None:
        self.current_pose = np.asarray(self.chain.get_end_effector_pose(data), dtype=np.float32)
        chain_dof_pos = np.asarray(self.chain.get_dof_pos(data), dtype=np.float32)

        if self.mode == "idle":
            return

        if self.mode == "base_rotate":
            self.desired_dof_pos[0] = self.base_target
            if abs(float(chain_dof_pos[0] - self.base_target)) < 0.2:
                self.mode = "idle"
            return

        if not self.waypoints:
            self.mode = "idle"
            return

        target_pose = self._compute_target_pose(self.current_pose)
        if target_pose is None:
            self.mode = "idle"
            return

        result = np.asarray(self.solver.solve(self.chain, data, target_pose), dtype=np.float32)
        self.residual = float(result[1])
        if result.shape[0] >= 8:  # success_flag + residual + 6_dofs
            self.desired_dof_pos = result[2:].copy()

    def _compute_target_pose(self, current_pose: np.ndarray) -> np.ndarray | None:
        segment_count = len(self.waypoints) - 1
        if segment_count < 0:
            return None

        if self.current_segment == segment_count:
            distance = float(
                np.linalg.norm(pose_translation(current_pose) - pose_translation(self.waypoints[self.current_segment]))
            )
            if distance < self.accuracy[self.current_segment]:
                return None
            return self.waypoints[self.current_segment]

        p0 = self.waypoints[self.current_segment]
        p1 = self.waypoints[self.current_segment + 1]
        segment_vector = pose_translation(p1) - pose_translation(p0)
        segment_length = float(np.linalg.norm(segment_vector))
        if segment_length < 1e-6:
            self.current_segment += 1
            return self._compute_target_pose(current_pose)

        to_current_vec = pose_translation(current_pose) - pose_translation(p0)
        t = float(np.dot(to_current_vec, segment_vector) / (segment_length * segment_length))
        if t >= 1.0 and self.current_segment < segment_count - 1:
            self.current_segment += 1
            return self._compute_target_pose(current_pose)

        advance_t = t + self.accuracy[self.current_segment] / segment_length
        if advance_t > 1.0:
            self.current_segment += 1
            return self._compute_target_pose(current_pose)

        advance_t = max(0.0, advance_t)
        translation = pose_translation(p0) + advance_t * segment_vector
        rotation = quat_slerp(pose_rotation(p0), pose_rotation(p1), advance_t)
        return pose(translation, rotation)
