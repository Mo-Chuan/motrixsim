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

import numpy as np

from motrixsim import SceneData, msd
from motrixsim.render import Color, RenderApp

from .common import (
    CAMERA_NAME,
    SCENE_PATH,
    get_pose_by_sensor_name,
    get_selection_ray,
    highlight_size,
    pick_item_target,
)
from .task_flow import IimtTaskRuntime


def main() -> None:
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

    runtime = IimtTaskRuntime(model, data)
    pulse_time = 0.0

    print("IIMT Auto Store Task Example")
    print("Controls:")
    print("  Right click   : select and highlight a shelf item")
    print("  Left drag     : orbit system camera")
    print("  Click the viewport before using the camera shortcuts.")
    print("  1             : switch to system camera")
    print("  2             : switch to scene camera 'aim'")
    print("  T             : initialize task")
    print("  Space         : fetch selected target")
    print("  Y             : finish task")
    print("  R             : reset arm and clear commands")
    print("  P             : reset scene")
    print("  Esc           : quit")

    with RenderApp() as render:
        render.launch(model)

        while True:
            render.sync(data)
            input_mgr = render.input
            if render.is_closed or input_mgr.is_key_just_pressed("esc"):
                break

            dt = float(model.options.timestep)
            pulse_time += dt

            if input_mgr.is_key_just_pressed("1"):
                render.set_active_viewport_camera(None)
            if input_mgr.is_key_just_pressed("2"):
                render.set_active_viewport_camera(camera)

            if input_mgr.is_key_just_pressed("p"):
                data = SceneData(model)
                model.step(data)
                runtime = IimtTaskRuntime(model, data)
                continue

            if input_mgr.is_key_just_pressed("r"):
                runtime.reset_arm()

            if input_mgr.is_key_just_pressed("t"):
                runtime.initialize()

            if input_mgr.is_key_just_pressed("y"):
                runtime.finish(data)

            if input_mgr.is_key_just_pressed("space"):
                runtime.fetch_selected_target(data)

            if input_mgr.is_mouse_just_pressed("right") and not runtime.commands:
                ray = get_selection_ray(render)
                target, candidates = pick_item_target(model, data, runtime.item_names, ray)
                runtime.select_target(target, ray, candidates)
                if target is not None:
                    print(f"Selected target: {target.name}")
                else:
                    print("No shelf item selected.")

            forward = 0.0
            rotate = 0.0
            if input_mgr.is_key_pressed("w"):
                forward = 2.5
            elif input_mgr.is_key_pressed("s"):
                forward = -2.5
            if input_mgr.is_key_pressed("a"):
                rotate = 3.0
            elif input_mgr.is_key_pressed("d"):
                rotate = -3.0
            runtime.set_direct_drive(forward, rotate)

            runtime.step(data, dt)

            if runtime.selected_target is not None:
                body = model.get_body(runtime.selected_target.name)
                if body is not None:
                    body_pose = np.asarray(body.get_pose(data), dtype=np.float32)
                    pulse = 1.0 + 0.12 * math.sin(pulse_time * 6.0)
                    size = highlight_size(runtime.selected_target.item_type) * pulse
                    top = body_pose[:3] + np.array([0.0, 0.0, size[2] * 0.75], dtype=np.float32)
                    render.gizmos.draw_cuboid(size, body_pose[:3], rot=body_pose[3:7], color=Color.rgb(1.0, 0.2, 0.2))
                    render.gizmos.draw_sphere(0.03 * pulse, top, color=Color.rgb(1.0, 0.9, 0.2))
                    render.gizmos.draw_line(body_pose[:3], top, color=Color.rgb(1.0, 0.9, 0.2))
                    render.gizmos.draw_axes(body_pose[:3], body_pose[3:7], length=0.12)

            if runtime.last_pick_ray is not None:
                ray_origin = runtime.last_pick_ray[:3]
                ray_dir = runtime.last_pick_ray[3:]
                ray_dir = ray_dir / max(float(np.linalg.norm(ray_dir)), 1e-6)
                render.gizmos.draw_line(ray_origin, ray_origin + ray_dir * 6.0, color=Color.rgb(1.0, 1.0, 0.2))

            for candidate in runtime.debug_candidates:
                color = Color.rgb(0.2, 1.0, 0.2) if candidate["inside"] else Color.rgb(1.0, 0.4, 0.4)
                render.gizmos.draw_sphere(0.025, candidate["pose"][:3], color=color)

            basket_pose = get_pose_by_sensor_name(model, data, "basket")
            render.gizmos.draw_axes(basket_pose[:3], basket_pose[3:7], length=0.18)
            render.gizmos.draw_sphere(0.02, basket_pose[:3], color=Color.rgb(0.1, 0.3, 1.0))

            table_pose = get_pose_by_sensor_name(model, data, "table")
            render.gizmos.draw_axes(table_pose[:3], table_pose[3:7], length=0.22)
            render.gizmos.draw_sphere(0.025, table_pose[:3], color=Color.rgb(0.7, 0.2, 1.0))

            base_pose = np.asarray(model.get_link("base_link").get_pose(data), dtype=np.float32)
            render.gizmos.draw_axes(base_pose[:3], base_pose[3:7], length=0.2)
            render.gizmos.draw_sphere(0.025, base_pose[:3], color=Color.rgb(0.1, 1.0, 1.0))

            arm_base_pose = np.asarray(model.get_link("Link_CR_1").get_pose(data), dtype=np.float32)
            render.gizmos.draw_axes(arm_base_pose[:3], arm_base_pose[3:7], length=0.16)
            render.gizmos.draw_sphere(0.02, arm_base_pose[:3], color=Color.rgb(1.0, 0.4, 0.1))

            current_waypoint = runtime.transporter_current_waypoint()
            for waypoint in runtime.transporter_debug_waypoints():
                is_current = current_waypoint is not None and np.allclose(waypoint, current_waypoint)
                color = Color.rgb(0.9, 0.3, 0.2) if is_current else Color.rgb(1.0, 0.7, 0.2)
                render.gizmos.draw_sphere(
                    0.08 if is_current else 0.05,
                    np.array([waypoint[0], waypoint[1], 0.15], dtype=np.float32),
                    color=color,
                )

            arm_waypoints = runtime.arm.current_debug_waypoints()
            for index, waypoint_pose in enumerate(arm_waypoints):
                if index == len(arm_waypoints) - 1:
                    color = Color.rgb(0.05, 0.15, 0.65)
                elif index == runtime.arm.current_segment:
                    color = Color.rgb(1.0, 1.0, 0.2)
                else:
                    color = Color.rgb(1.0, 0.2, 0.2)
                render.gizmos.draw_sphere(0.02, waypoint_pose[:3], color=color)
                render.gizmos.draw_line(
                    waypoint_pose[:3],
                    waypoint_pose[:3] + np.array([0.0, 0.0, 0.12], dtype=np.float32),
                    color=Color.rgb(0.2, 1.0, 0.2),
                )

            if runtime.arm.current_pose is not None:
                render.gizmos.draw_axes(runtime.arm.current_pose[:3], runtime.arm.current_pose[3:7], length=0.1)
