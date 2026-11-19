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
from pathlib import Path

import numpy as np

from motrixsim import SceneData, ik, msd
from motrixsim.render import Color, RenderApp

SCENE_PATH = str(Path(__file__).resolve().parent / "assets" / "scene.xml")
CAMERA_NAME = "aim"
END_EFFECTOR_LINK = "Link_CR_6"
END_EFFECTOR_OFFSET = np.array([0.0, 0.0, 0.22, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
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


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vec))
    if norm < 1e-8:
        return np.zeros_like(vec)
    return vec / norm


def quat_normalize(quat: np.ndarray) -> np.ndarray:
    return normalize(quat.astype(np.float32))


def quat_conjugate(quat: np.ndarray) -> np.ndarray:
    return np.array([-quat[0], -quat[1], -quat[2], quat[3]], dtype=np.float32)


def quat_multiply(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float32,
    )


def quat_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = normalize(axis.astype(np.float32))
    half = angle * 0.5
    s = math.sin(half)
    return quat_normalize(np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half)], dtype=np.float32))


def quat_rotate(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    quat = quat_normalize(quat.astype(np.float32))
    pure = np.array([vec[0], vec[1], vec[2], 0.0], dtype=np.float32)
    rotated = quat_multiply(quat_multiply(quat, pure), quat_conjugate(quat))
    return rotated[:3]


def ray_point_distance(ray_origin: np.ndarray, ray_dir: np.ndarray, point: np.ndarray) -> tuple[float, float]:
    delta = point - ray_origin
    along = float(np.dot(delta, ray_dir))
    closest = ray_origin + ray_dir * max(along, 0.0)
    perp = float(np.linalg.norm(point - closest))
    return along, perp


def item_kind(name: str) -> str:
    prefixes = [
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
    for prefix in prefixes:
        if name.startswith(prefix):
            return prefix
    return "unknown"


def item_height_offset(name: str) -> float:
    return {
        "V_Bottle_350": 0.07,
        "V_Bottle_360": 0.07,
        "V_Bottle_400": 0.07,
        "V_Bottle_600": 0.07,
        "V_Cans_N500": 0.10,
    }.get(item_kind(name), 0.05)


def selection_radius(name: str) -> float:
    return {
        "V_Bottle_600": 0.11,
        "V_Cans_N500": 0.10,
    }.get(item_kind(name), 0.08)


def highlight_size(name: str) -> np.ndarray:
    size = {
        "V_Bottle_600": [0.09, 0.09, 0.30],
        "V_Cans_N500": [0.09, 0.09, 0.22],
    }.get(item_kind(name), [0.08, 0.08, 0.20])
    return np.asarray(size, dtype=np.float32)


def pick_item(model, data: SceneData, item_names: list[str], ray: np.ndarray) -> str | None:
    ray_origin = ray[:3]
    ray_dir = normalize(ray[3:])
    if np.linalg.norm(ray_dir) < 1e-6:
        return None

    best_name = None
    best_score = None
    for name in item_names:
        body = model.get_body(name)
        if body is None:
            continue
        pose = np.asarray(body.get_pose(data), dtype=np.float32)
        along, perp = ray_point_distance(ray_origin, ray_dir, pose[:3])
        if along <= 0.0 or perp > 0.35:
            continue

        tight_radius = max(selection_radius(name), 0.18)
        score = along + perp * 6.0
        if perp <= tight_radius and (best_score is None or score < best_score):
            best_score = score
            best_name = name

    return best_name


def pick_item_debug(model, data: SceneData, item_names: list[str], ray: np.ndarray, limit: int = 5):
    ray_origin = ray[:3]
    ray_dir = normalize(ray[3:])
    if np.linalg.norm(ray_dir) < 1e-6:
        return None, []

    candidates = []
    for name in item_names:
        body = model.get_body(name)
        if body is None:
            continue
        pose = np.asarray(body.get_pose(data), dtype=np.float32)
        along, perp = ray_point_distance(ray_origin, ray_dir, pose[:3])
        if along <= 0.0:
            continue
        tight_radius = max(selection_radius(name), 0.18)
        score = along + perp * 6.0
        candidates.append(
            {
                "name": name,
                "pose": pose,
                "along": along,
                "perp": perp,
                "tight_radius": tight_radius,
                "score": score,
                "inside": perp <= tight_radius,
            }
        )

    candidates.sort(key=lambda item: item["score"])
    picked = next((item["name"] for item in candidates if item["inside"]), None)
    return picked, candidates[:limit]


def get_selection_ray(render: RenderApp) -> np.ndarray:
    ray = np.asarray(render.input.mouse_ray(), dtype=np.float32)
    if ray.shape[0] >= 6 and np.linalg.norm(ray[3:6]) > 1e-6:
        return ray[:6].copy()

    pose = np.asarray(render.system_camera.pose, dtype=np.float32)
    origin = pose[:3].copy()
    direction = quat_rotate(pose[3:7], np.array([0.0, 0.0, -1.0], dtype=np.float32))
    direction = normalize(direction)
    return np.concatenate([origin, direction]).astype(np.float32)


def main():
    # Configure the aim camera tracking on the MSD layer before building the immutable model
    world = msd.from_file(SCENE_PATH)
    aim_desc = next(c for c in world.hierarchy.cameras if c.name == CAMERA_NAME)
    aim_desc.position_track = msd.PositionTrack.FixedLocal
    aim_desc.rotation_track = msd.RotationTrack.LookAtLink
    aim_desc.track_target = "base_link"
    model = world.build()
    data = SceneData(model)
    model.step(data)

    camera = model.cameras[CAMERA_NAME]

    chain = ik.IkChain(
        model,
        end_link=END_EFFECTOR_LINK,
        start_link="Link_CR_1",
        end_effector_offset=END_EFFECTOR_OFFSET,
    )
    solver = ik.DlsSolver(max_iter=100, step_size=0.4, tolerance=1e-3, damping=1e-3)

    arm_actuators = [model.get_actuator(name) for name in ARM_ACTUATOR_NAMES]
    forward_actuator = model.get_actuator(VEHICLE_FORWARD_ACTUATOR)
    turn_actuator = model.get_actuator(VEHICLE_TURN_ACTUATOR)
    brake_actuator = model.get_actuator(VEHICLE_BRAKE_ACTUATOR)
    gripper_actuator = model.get_actuator(GRIPPER_ACTUATOR)

    desired_arm_dofs = RESET_ARM_DOFS.copy()
    target_pose = np.asarray(chain.get_end_effector_pose(data), dtype=np.float32).copy()
    selected_item = None
    gripper_closed = False
    pulse_time = 0.0
    item_names = [name for name in model.body_names if isinstance(name, str) and name.startswith("V_")]
    last_pick_ray = None
    debug_candidates = []

    print("IIMT Auto Store Example")
    print("Controls:")
    print("  Left click    : select and highlight a shelf item")
    print("  Left drag     : orbit system camera")
    print("  Right drag    : pan system camera")
    print("  Click the viewport before using the camera shortcuts.")
    print("  1             : switch to system camera")
    print("  2             : switch to scene camera 'aim'")
    print("  W / S / A / D : drive the base")
    print("  Space         : move arm target above highlighted item")
    print("  Arrow keys    : move arm target in x / y")
    print("  U / J         : move arm target in z")
    print("  Q / E         : rotate arm target around z")
    print("  Z / X         : rotate arm target around x")
    print("  C / O         : close / open gripper")
    print("  R             : reset arm target")
    print("  P             : reset scene")
    print("  Esc           : quit")

    with RenderApp() as render:
        render.launch(model)

        while True:
            render.sync(data)
            input_mgr = render.input
            if render.is_closed or input_mgr.is_key_just_pressed("esc"):
                break

            pulse_time += float(model.options.timestep)

            if input_mgr.is_key_just_pressed("1"):
                render.set_active_viewport_camera(None)
            if input_mgr.is_key_just_pressed("2"):
                render.set_active_viewport_camera(camera)

            if input_mgr.is_key_just_pressed("p"):
                data = SceneData(model)
                model.step(data)
                desired_arm_dofs = RESET_ARM_DOFS.copy()
                target_pose = np.asarray(chain.get_end_effector_pose(data), dtype=np.float32).copy()
                selected_item = None
                last_pick_ray = None
                debug_candidates = []

            if input_mgr.is_key_just_pressed("r"):
                desired_arm_dofs = RESET_ARM_DOFS.copy()
                target_pose = np.asarray(chain.get_end_effector_pose(data), dtype=np.float32).copy()

            if input_mgr.is_key_just_pressed("c"):
                gripper_closed = True
            if input_mgr.is_key_just_pressed("o"):
                gripper_closed = False

            if input_mgr.is_mouse_just_pressed("left"):
                ray = get_selection_ray(render)
                last_pick_ray = ray.copy()
                picked, debug_candidates = pick_item_debug(model, data, item_names, ray)
                if picked is not None:
                    selected_item = picked
                    print(f"Selected target: {selected_item}")
                else:
                    print("No shelf item selected.")
                    for idx, item in enumerate(debug_candidates):
                        print(
                            f"  cand[{idx}] {item['name']} "
                            f"along={item['along']:.3f} perp={item['perp']:.3f} "
                            f"thr={item['tight_radius']:.3f} inside={item['inside']}"
                        )

            if input_mgr.is_key_just_pressed("space") and selected_item is not None:
                body = model.get_body(selected_item)
                if body is not None:
                    body_pose = np.asarray(body.get_pose(data), dtype=np.float32)
                    target_pose[:3] = body_pose[:3] + np.array(
                        [0.0, 0.0, item_height_offset(selected_item)],
                        dtype=np.float32,
                    )

            if input_mgr.is_key_pressed("left"):
                target_pose[0] -= 0.003
            if input_mgr.is_key_pressed("right"):
                target_pose[0] += 0.003
            if input_mgr.is_key_pressed("up"):
                target_pose[1] += 0.003
            if input_mgr.is_key_pressed("down"):
                target_pose[1] -= 0.003
            if input_mgr.is_key_pressed("u"):
                target_pose[2] += 0.003
            if input_mgr.is_key_pressed("j"):
                target_pose[2] -= 0.003
            if input_mgr.is_key_pressed("q"):
                target_pose[3:7] = quat_multiply(
                    quat_from_axis_angle(np.array([0.0, 0.0, 1.0], dtype=np.float32), 0.02),
                    target_pose[3:7],
                )
            if input_mgr.is_key_pressed("e"):
                target_pose[3:7] = quat_multiply(
                    quat_from_axis_angle(np.array([0.0, 0.0, 1.0], dtype=np.float32), -0.02),
                    target_pose[3:7],
                )
            if input_mgr.is_key_pressed("z"):
                target_pose[3:7] = quat_multiply(
                    quat_from_axis_angle(np.array([1.0, 0.0, 0.0], dtype=np.float32), 0.02),
                    target_pose[3:7],
                )
            if input_mgr.is_key_pressed("x"):
                target_pose[3:7] = quat_multiply(
                    quat_from_axis_angle(np.array([1.0, 0.0, 0.0], dtype=np.float32), -0.02),
                    target_pose[3:7],
                )
            target_pose[3:7] = quat_normalize(target_pose[3:7])

            forward_value = 0.0
            turn_value = 0.0
            if input_mgr.is_key_pressed("w"):
                forward_value = 2.5
            elif input_mgr.is_key_pressed("s"):
                forward_value = -2.5
            if input_mgr.is_key_pressed("a"):
                turn_value = 3.0
            elif input_mgr.is_key_pressed("d"):
                turn_value = -3.0

            brake_value = 0.0 if abs(forward_value) > 1e-6 or abs(turn_value) > 1e-6 else 0.3
            forward_actuator.set_ctrl(data, forward_value)
            turn_actuator.set_ctrl(data, turn_value)
            brake_actuator.set_ctrl(data, brake_value)
            gripper_actuator.set_ctrl(data, 225.0 if gripper_closed else 0.0)

            result = np.asarray(solver.solve(chain, data, target_pose), dtype=np.float32)
            residual = float(result[1])
            if residual < 0.1:
                desired_arm_dofs = result[2:].copy()

            for actuator, value in zip(arm_actuators, desired_arm_dofs, strict=True):
                actuator.set_ctrl(data, float(value))

            model.step(data)

            if selected_item is not None:
                body = model.get_body(selected_item)
                if body is not None:
                    pose = np.asarray(body.get_pose(data), dtype=np.float32)
                    pulse = 1.0 + 0.12 * math.sin(pulse_time * 6.0)
                    size = highlight_size(selected_item) * pulse
                    top = pose[:3] + np.array([0.0, 0.0, size[2] * 0.75], dtype=np.float32)
                    render.gizmos.draw_cuboid(size, pose[:3], rot=pose[3:7], color=Color.rgb(1.0, 0.2, 0.2))
                    render.gizmos.draw_sphere(0.03 * pulse, top, color=Color.rgb(1.0, 0.9, 0.2))
                    render.gizmos.draw_line(pose[:3], top, color=Color.rgb(1.0, 0.9, 0.2))
                    render.gizmos.draw_axes(pose[:3], pose[3:7], length=0.12)

            if last_pick_ray is not None:
                ray_origin = last_pick_ray[:3]
                ray_dir = normalize(last_pick_ray[3:])
                if np.linalg.norm(ray_dir) > 1e-6:
                    ray_end = ray_origin + ray_dir * 6.0
                    render.gizmos.draw_line(ray_origin, ray_end, color=Color.rgb(1.0, 1.0, 0.2))

            for item in debug_candidates:
                color = Color.rgb(0.2, 1.0, 0.2) if item["inside"] else Color.rgb(1.0, 0.4, 0.4)
                render.gizmos.draw_sphere(0.025, item["pose"][:3], color=color)

            render.gizmos.draw_cuboid(
                np.array([0.05, 0.05, 0.05], dtype=np.float32),
                target_pose[:3],
                rot=target_pose[3:7],
                color=Color.rgb(0.1, 0.8, 1.0),
            )
            render.gizmos.draw_axes(target_pose[:3], target_pose[3:7], length=0.12)


if __name__ == "__main__":
    main()
