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
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from motrixsim import SceneData
from motrixsim.render import RenderApp

SCENE_PATH = str(Path(__file__).resolve().parents[1] / "assets" / "scene.xml")
CAMERA_NAME = "aim"
ROBOT_BASE_LINK = "base_link"
END_EFFECTOR_LINK = "Link_CR_6"
GRIPPER_LINK_NAME = "right_driver"
GRIPPER_OFFSET = 0.22
END_EFFECTOR_OFFSET = np.array([0.0, 0.0, GRIPPER_OFFSET, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
VERTICAL_HANGING_OFFSET = np.array([0.0, 0.0, 0.2], dtype=np.float32)
RESET_ARM_DOFS = np.array([0.0, -2.0, 2.0, -math.pi / 2.0, math.pi / 2.0, 0.0], dtype=np.float32)
ARM_ACTUATOR_NAMES = [
    "act_Joint_CR_1",
    "act_Joint_CR_2",
    "act_Joint_CR_3",
    "act_Joint_CR_4",
    "act_Link_CR_5",
    "act_Link_CR_6",
]
VEHICLE_FORWARD_ACTUATOR = "forward"
VEHICLE_TURN_ACTUATOR = "turn"
VEHICLE_BRAKE_ACTUATOR = "act_braking"
GRIPPER_ACTUATOR = "fingers_actuator"
GRIPPER_OPEN_CTRL = 0.0
GRIPPER_CLOSE_CTRL = 225.0
ITEM_PREFIXES = [
    "V_Bottle_350",
    "V_Bottle_360",
    "V_Bottle_400",
    "V_Bottle_600",
    "V_Cans_N250",
    "V_Cans_N330",
    "V_Cans_N355",
    "V_Cans_N500",
    "V_Cans_S250",
    "V_Cans_S330",
    "V_Cans_S355",
    "V_Tetra_250_F",
    "V_Tetra_250_T",
    "V_Tetra_300",
    "V_Tetra_500",
]


@dataclass
class ItemTargetInfo:
    name: str
    position: np.ndarray
    rotation: np.ndarray
    item_type: str


def normalize(vec: np.ndarray) -> np.ndarray:
    arr = np.asarray(vec, dtype=np.float32)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-8:
        return np.zeros_like(arr)
    return arr / norm


def quat_normalize(quat: np.ndarray) -> np.ndarray:
    return normalize(np.asarray(quat, dtype=np.float32))


def quat_conjugate(quat: np.ndarray) -> np.ndarray:
    return Rotation.from_quat(quat_normalize(quat)).inv().as_quat().astype(np.float32)


def quat_multiply(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    return (
        (Rotation.from_quat(quat_normalize(lhs)) * Rotation.from_quat(quat_normalize(rhs))).as_quat().astype(np.float32)
    )


def quat_rotate(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    return Rotation.from_quat(quat_normalize(quat)).apply(np.asarray(vec, dtype=np.float32)).astype(np.float32)


def quat_slerp(lhs: np.ndarray, rhs: np.ndarray, t: float) -> np.ndarray:
    clamped_t = max(0.0, min(1.0, float(t)))
    key_rots = Rotation.from_quat(np.stack([quat_normalize(lhs), quat_normalize(rhs)], axis=0))
    return Slerp([0.0, 1.0], key_rots)([clamped_t]).as_quat()[0].astype(np.float32)


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = normalize(axis)
    return Rotation.from_rotvec(axis * float(angle)).as_quat().astype(np.float32)


def pose(translation: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    return np.concatenate([np.asarray(translation, dtype=np.float32), quat_normalize(rotation)]).astype(np.float32)


def pose_translation(pose_value: np.ndarray) -> np.ndarray:
    return np.asarray(pose_value[:3], dtype=np.float32)


def pose_rotation(pose_value: np.ndarray) -> np.ndarray:
    return quat_normalize(np.asarray(pose_value[3:7], dtype=np.float32))


def transform_vec(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    return quat_rotate(quat, vec)


def transform_point(pose_value: np.ndarray, point: np.ndarray) -> np.ndarray:
    return pose_translation(pose_value) + transform_vec(pose_rotation(pose_value), point)


def inverse_transform_point(pose_value: np.ndarray, point: np.ndarray) -> np.ndarray:
    return quat_rotate(
        quat_conjugate(pose_rotation(pose_value)), np.asarray(point, dtype=np.float32) - pose_translation(pose_value)
    )


def yaw_from_quat(quat: np.ndarray) -> float:
    matrix = Rotation.from_quat(quat_normalize(quat)).as_matrix()
    return float(math.atan2(matrix[1, 0], matrix[0, 0]))


def quat_from_yaw(yaw: float) -> np.ndarray:
    return Rotation.from_euler("z", float(yaw)).as_quat().astype(np.float32)


def quat_from_rotation_matrix(matrix: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(np.asarray(matrix, dtype=np.float32)).as_quat().astype(np.float32)


def quat_face_towards(direction: np.ndarray, up: np.ndarray) -> np.ndarray:
    z_axis = normalize(np.asarray(direction, dtype=np.float32))
    y_hint = normalize(np.asarray(up, dtype=np.float32))
    if np.linalg.norm(z_axis) < 1e-6 or np.linalg.norm(y_hint) < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)

    x_axis = normalize(np.cross(y_hint, z_axis))
    if np.linalg.norm(x_axis) < 1e-6:
        fallback = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        if abs(float(np.dot(fallback, z_axis))) > 0.9:
            fallback = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        x_axis = normalize(np.cross(fallback, z_axis))
    y_axis = normalize(np.cross(z_axis, x_axis))
    rotation_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
    return quat_from_rotation_matrix(rotation_matrix)


def quat_top_down_grasp(opening_axis: np.ndarray) -> np.ndarray:
    x_axis = normalize(np.asarray(opening_axis, dtype=np.float32))
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    z_axis = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    y_axis = normalize(np.cross(z_axis, x_axis))
    if np.linalg.norm(y_axis) < 1e-6:
        x_axis = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        y_axis = normalize(np.cross(z_axis, x_axis))
    x_axis = normalize(np.cross(y_axis, z_axis))
    rotation_matrix = np.stack([x_axis, y_axis, z_axis], axis=1)
    return quat_from_rotation_matrix(rotation_matrix)


def ray_point_distance(ray_origin: np.ndarray, ray_dir: np.ndarray, point: np.ndarray) -> tuple[float, float]:
    delta = np.asarray(point, dtype=np.float32) - np.asarray(ray_origin, dtype=np.float32)
    along = float(np.dot(delta, ray_dir))
    closest = ray_origin + ray_dir * max(along, 0.0)
    perp = float(np.linalg.norm(point - closest))
    return along, perp


def item_kind(name: str) -> str:
    for prefix in ITEM_PREFIXES:
        if name.startswith(prefix):
            return prefix
    return "unknown"


def item_height_offset(kind: str) -> float:
    return {
        "V_Bottle_350": 0.07,
        "V_Bottle_360": 0.07,
        "V_Bottle_400": 0.07,
        "V_Bottle_600": 0.07,
        "V_Cans_N500": 0.10,
    }.get(kind, 0.05)


def selection_radius(kind: str) -> float:
    return {
        "V_Bottle_600": 0.11,
        "V_Cans_N500": 0.10,
    }.get(kind, 0.08)


def highlight_size(kind: str) -> np.ndarray:
    size = {
        "V_Bottle_600": [0.09, 0.09, 0.30],
        "V_Cans_N500": [0.09, 0.09, 0.22],
    }.get(kind, [0.08, 0.08, 0.20])
    return np.asarray(size, dtype=np.float32)


def get_selection_ray(render: RenderApp) -> np.ndarray:
    ray = np.asarray(render.input.mouse_ray(), dtype=np.float32)
    if ray.shape[0] >= 6 and np.linalg.norm(ray[3:6]) > 1e-6:
        return ray[:6].copy()

    camera_pose = np.asarray(render.system_camera.pose, dtype=np.float32)
    origin = camera_pose[:3].copy()
    direction = quat_rotate(camera_pose[3:7], np.array([0.0, 0.0, -1.0], dtype=np.float32))
    return np.concatenate([origin, normalize(direction)]).astype(np.float32)


def pick_item_target(model, data: SceneData, item_names: list[str], ray: np.ndarray, limit: int = 5):
    ray_origin = np.asarray(ray[:3], dtype=np.float32)
    ray_dir = normalize(ray[3:])
    if np.linalg.norm(ray_dir) < 1e-6:
        return None, []

    candidates = []
    for name in item_names:
        body = model.get_body(name)
        if body is None:
            continue
        body_pose = np.asarray(body.get_pose(data), dtype=np.float32)
        kind = item_kind(name)
        along, perp = ray_point_distance(ray_origin, ray_dir, body_pose[:3])
        if along <= 0.0:
            continue
        tight_radius = max(selection_radius(kind), 0.18)
        score = along + perp * 6.0
        candidates.append(
            {
                "name": name,
                "pose": body_pose,
                "kind": kind,
                "along": along,
                "perp": perp,
                "tight_radius": tight_radius,
                "score": score,
                "inside": perp <= tight_radius,
            }
        )

    candidates.sort(key=lambda item: item["score"])
    target = None
    for candidate in candidates:
        if candidate["inside"]:
            target = ItemTargetInfo(
                name=candidate["name"],
                position=candidate["pose"][:3].copy(),
                rotation=candidate["pose"][3:7].copy(),
                item_type=candidate["kind"],
            )
            break
    return target, candidates[:limit]


def get_pose_by_sensor_name(model, data: SceneData, name: str) -> np.ndarray:
    pos = np.asarray(model.get_sensor_value(f"{name}_pos", data), dtype=np.float32)
    quat = np.asarray(model.get_sensor_value(f"{name}_quat", data), dtype=np.float32)
    return pose(pos, quat)
