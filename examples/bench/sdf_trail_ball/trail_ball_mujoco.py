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

"""MuJoCo counterpart of trail_ball.py, using the supplied MuJoCo XML.

Run with Python from the docs environment (mujoco, glfw, numpy required).
GLFW runs on the main thread, including on macOS; no mjpython is needed.
Left drag rotates, right drag pans, wheel zooms. R resets, Space pauses,
Home restores the initial camera, and Escape closes the window.
CPU frame generation measures only mj_step calls, excluding rendering,
telemetry, forward kinematics and frame pacing, just like trail_ball.py.
"""

import time
from pathlib import Path

import glfw
import mujoco
import numpy as np

ASSET_PATH = Path(__file__).resolve().parents[2] / "assets/sdf/trail_ball/trail ball for mujoco.xml"
FRAME_PERIOD = 1 / 60
SIDEBAR_WIDTH = 400


def sidebar_buttons(width, height, sidebar_width):
    """Button rectangles, shared by drawing and hit testing at any DPI."""
    padding = max(1, round(10 * sidebar_width / SIDEBAR_WIDTH))
    return tuple(
        mujoco.MjrRect(
            width - sidebar_width + padding,
            round(height * (1 - (top + 40) / 960)),
            max(1, sidebar_width - 2 * padding),
            max(1, round(height * 40 / 960)),
        )
        for top in (520, 570)
    )


def reset_camera(camera):
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.23, -0.23, 0.25]
    camera.distance = 12.7
    camera.elevation = -30
    camera.azimuth = 221


# Match trail_ball.py colors/width; even short normals extend beyond the ball.
GIZMOS_LINE_WIDTH = 6.0
CONTACT_MARKER_RADIUS = 0.03
CONTACT_POINT_COLOR = (1.0, 0.55, 0.1)
CONTACT_NORMAL_COLOR = (0.2, 0.9, 1.0)
NORMAL_ARROW_MIN = 0.6
NORMAL_ARROW_MAX = 1.6
DEPTH_REF = 5e-4


def normal_arrow_length(depth):
    return NORMAL_ARROW_MIN + (NORMAL_ARROW_MAX - NORMAL_ARROW_MIN) * min(abs(depth) / DEPTH_REF, 1.0)


def add_contacts(data, scene, show_arrows=True):
    """Add depth-tested contact markers and 6-pixel normal arrows to the scene."""
    camera = mujoco.mjv_averageCamera(scene.camera[0], scene.camera[1])
    for contact in data.contact:
        if contact.dist >= 0 or scene.ngeom + (4 if show_arrows else 1) > scene.maxgeom:
            continue
        marker = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            marker,
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.full(3, CONTACT_MARKER_RADIUS),
            contact.pos,
            np.eye(3).ravel(),
            np.array([*CONTACT_POINT_COLOR, 1.0]),
        )
        scene.ngeom += 1
        if not show_arrows:
            continue
        # The first contact-frame axis points from geom[0] to geom[1].
        normal = contact.frame[:3]
        length = normal_arrow_length(contact.dist)
        tip = contact.pos + normal * length
        # Face the arrowhead toward the camera while preserving its world-space
        # endpoint. All three line segments use MuJoCo's normal depth testing.
        side = np.cross(normal, camera.forward)
        if np.linalg.norm(side) < 1e-6:
            side = np.cross(normal, camera.up)
        side /= np.linalg.norm(side)
        head = 0.2 * length
        for start, end in (
            (contact.pos, tip),
            (tip, tip - head * normal + 0.5 * head * side),
            (tip, tip - head * normal - 0.5 * head * side),
        ):
            line = scene.geoms[scene.ngeom]
            mujoco.mjv_initGeom(
                line,
                mujoco.mjtGeom.mjGEOM_LINE,
                np.zeros(3),
                np.zeros(3),
                np.eye(3).ravel(),
                np.array([*CONTACT_NORMAL_COLOR, 1.0]),
            )
            # Unlike solid arrow widths, mjGEOM_LINE width is measured in pixels.
            mujoco.mjv_connector(line, mujoco.mjtGeom.mjGEOM_LINE, GIZMOS_LINE_WIDTH, start, end)
            scene.ngeom += 1


def main():
    model = mujoco.MjModel.from_xml_path(str(ASSET_PATH))
    data = mujoco.MjData(model)
    ball_id = model.body("ball").id
    camera = mujoco.MjvCamera()
    reset_camera(camera)
    option = mujoco.MjvOption()
    scene = mujoco.MjvScene(model, maxgeom=10000)
    velocity = np.zeros(6)
    mujoco.mj_forward(model, data)

    if not glfw.init():
        raise RuntimeError("GLFW could not initialize a display")
    window = None
    context = None
    try:
        window = glfw.create_window(1440, 960, "Trail Ball Demo - MuJoCo", None, None)
        if window is None:
            raise RuntimeError("GLFW could not create the render window")
        glfw.make_context_current(window)
        glfw.swap_interval(1)
        context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        paused = False
        show_arrows = True
        target_time = 0.0
        frame_ns = average_ns = 0.0
        frames = 0
        cursor = glfw.get_cursor_pos(window)
        dragging = None

        def reset():
            nonlocal target_time, frame_ns, average_ns, frames
            mujoco.mj_resetData(model, data)
            mujoco.mj_forward(model, data)
            target_time = frame_ns = average_ns = 0.0
            frames = 0

        def key_callback(window, key, scancode, action, mods):
            nonlocal paused
            if action != glfw.PRESS:
                return
            if key == glfw.KEY_ESCAPE:
                glfw.set_window_should_close(window, True)
            elif key == glfw.KEY_R:
                reset()
            elif key == glfw.KEY_SPACE:
                paused = not paused
            elif key == glfw.KEY_HOME:
                reset_camera(camera)

        def mouse_button(window, button, action, mods):
            nonlocal cursor, dragging, show_arrows
            cursor = glfw.get_cursor_pos(window)
            width, height = glfw.get_window_size(window)
            sidebar_width = min(SIDEBAR_WIDTH, width - 1)
            if action == glfw.RELEASE:
                dragging = None
            elif cursor[0] >= width - sidebar_width:
                if button == glfw.MOUSE_BUTTON_LEFT:
                    for index, rect in enumerate(sidebar_buttons(width, height, sidebar_width)):
                        if (
                            rect.left <= cursor[0] < rect.left + rect.width
                            and rect.bottom <= height - cursor[1] < rect.bottom + rect.height
                        ):
                            if index == 0:
                                reset()
                            else:
                                show_arrows = not show_arrows
            else:
                dragging = button

        def mouse_move(window, x, y):
            nonlocal cursor
            dx, dy = x - cursor[0], y - cursor[1]
            cursor = (x, y)
            if dragging is None:
                return
            _, height = glfw.get_window_size(window)
            if height <= 0:
                return
            if dragging == glfw.MOUSE_BUTTON_LEFT:
                camera.azimuth -= dx / height * 180
                camera.elevation = np.clip(camera.elevation - dy / height * 180, -89, 89)
            elif dragging == glfw.MOUSE_BUTTON_RIGHT:
                azimuth, elevation = np.deg2rad([camera.azimuth, camera.elevation])
                right = np.array([np.sin(azimuth), -np.cos(azimuth), 0])
                up = np.array(
                    [
                        -np.sin(elevation) * np.cos(azimuth),
                        -np.sin(elevation) * np.sin(azimuth),
                        np.cos(elevation),
                    ]
                )
                camera.lookat[:] += (up * dy - right * dx) * camera.distance / height

        def scroll(window, dx, dy):
            width, _ = glfw.get_window_size(window)
            if glfw.get_cursor_pos(window)[0] < width - SIDEBAR_WIDTH:
                camera.distance = np.clip(camera.distance * np.exp(-0.1 * dy), 0.1, 100)

        glfw.set_key_callback(window, key_callback)
        glfw.set_mouse_button_callback(window, mouse_button)
        glfw.set_cursor_pos_callback(window, mouse_move)
        glfw.set_scroll_callback(window, scroll)

        while not glfw.window_should_close(window):
            started = time.perf_counter()
            glfw.poll_events()
            width, height = glfw.get_framebuffer_size(window)
            logical_width, logical_height = glfw.get_window_size(window)
            if min(width, height, logical_width, logical_height) <= 0:
                glfw.wait_events_timeout(0.05)
                continue
            frame_ns = 0
            if not paused:
                # Advance one display interval, carrying the fractional step to
                # the next frame. Slow frames slow playback rather than skipping.
                target_time += FRAME_PERIOD
                while data.time + model.opt.timestep <= target_time + 1e-10:
                    t0 = time.perf_counter_ns()
                    mujoco.mj_step(model, data)
                    frame_ns += time.perf_counter_ns() - t0
            average_ns = frame_ns if frames == 0 else average_ns + 0.1 * (frame_ns - average_ns)
            frames += 1
            # mj_step leaves derived quantities at the preceding integration
            # state. Refresh for consistent current-pose rendering and velocity.
            mujoco.mj_forward(model, data)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, ball_id, velocity, 0)
            angular, linear = velocity[:3], velocity[3:]
            sidebar_width = min(int(SIDEBAR_WIDTH * width / logical_width), width - 1)
            viewport = mujoco.MjrRect(0, 0, width - sidebar_width, height)
            sidebar = mujoco.MjrRect(viewport.width, 0, sidebar_width, height)
            # Scale text with the panel so telemetry, buttons and help remain
            # separate when resizing the window or using a high-DPI display.
            font_scale = max(50, min(300, int(150 * min(height / 960, sidebar_width / SIDEBAR_WIDTH) / 50) * 50))
            if context.fontScale != font_scale:
                mujoco.mjr_changeFont(font_scale, context)
            mujoco.mjv_updateScene(model, data, option, None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
            add_contacts(data, scene, show_arrows)
            mujoco.mjr_render(viewport, scene, context)
            mujoco.mjr_rectangle(sidebar, 0.10, 0.12, 0.15, 1)
            text = (
                "Trail Ball Demo - MuJoCo\n\n"
                f"Simulation: {data.time:.2f} s {'(paused)' if paused else ''}\n\n"
                "World linear velocity (m/s)\n"
                f"[{linear[0]:+.3f} {linear[1]:+.3f} {linear[2]:+.3f}]\n"
                f"Linear speed: {np.linalg.norm(linear):.3f} m/s\n\n"
                "World angular velocity (rad/s)\n"
                f"[{angular[0]:+.3f} {angular[1]:+.3f} {angular[2]:+.3f}]\n"
                f"Spin rate: {np.linalg.norm(angular):.3f} rad/s\n\n"
                f"CPU frame gen: {frame_ns / 1e3:.1f} us\n"
                f"Average: {average_ns / 1e3:.1f} us\n"
                "(physics stepping only)"
            )
            mujoco.mjr_overlay(
                mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, sidebar, text, "", context
            )
            button, arrow_button = sidebar_buttons(width, height, sidebar_width)
            mujoco.mjr_rectangle(button, 0.25, 0.32, 0.40, 1)
            mujoco.mjr_overlay(
                mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, button, "Reset ball (R)", "", context
            )
            mujoco.mjr_rectangle(arrow_button, 0.25, 0.32, 0.40, 1)
            mujoco.mjr_overlay(
                mujoco.mjtFont.mjFONT_NORMAL,
                mujoco.mjtGridPos.mjGRID_TOPLEFT,
                arrow_button,
                "Hide arrows" if show_arrows else "Show arrows",
                "",
                context,
            )
            mujoco.mjr_overlay(
                mujoco.mjtFont.mjFONT_NORMAL,
                mujoco.mjtGridPos.mjGRID_BOTTOMLEFT,
                sidebar,
                "Orange: contact point\nCyan: contact normal\n\n"
                "Left drag: rotate\nRight drag: pan\nWheel: zoom\n"
                "Space: pause / resume\nHome: restore camera\nEsc: close",
                "",
                context,
            )
            glfw.swap_buffers(window)
            remaining = FRAME_PERIOD - (time.perf_counter() - started)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        if context is not None:
            context.free()
        if window is not None:
            glfw.destroy_window(window)
        glfw.terminate()


if __name__ == "__main__":
    main()
