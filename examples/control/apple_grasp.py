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

"""Pick-up-apple replay example driven by the native MotrixSim Python bindings.

The robot follows the joint-target replay recorded from the reference run
(wheel ``0.10.1+2186``): pinch the stem, lift, place back, release, grasp the
fruit body, lift again. The apple settles live in the simulation before the
replay starts. During playback, the terminal shows the apple height and
contact counts for the index-finger and thumb pads once per second.

Run from the repository root (opens a window and starts playback)::

    uv run examples/control/apple_grasp.py

The replay lasts 40 seconds and supports pause and resume in the window.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np

import motrixsim as mx

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parent / "assets" / "apple_grasp"
DATA = ASSETS / "data"

SCENE = ASSETS / "scene.xml"
RECORD = DATA / "joint_targets.npz"

DT = 0.002
REPLAY_SECONDS = 40.0
GAIN_SWITCH_TIME = 23.0
APPLE_BODY = "apple_with_stem"
RENDER_INTERVAL = 0.05
RENDER_STEPS = round(RENDER_INTERVAL / DT)


def build_position_gains(joint_names, *, body_grasp: bool = False):
    """Reference gain schedule, in the given joint-name order."""
    kp = np.zeros(len(joint_names), dtype=np.float32)
    kd = np.zeros(len(joint_names), dtype=np.float32)
    for index, name in enumerate(joint_names):
        if name.startswith(("l_", "r_")):
            if body_grasp and name.startswith("r_"):
                kp[index], kd[index] = 0.8, 0.01
            else:
                kp[index] = 30 if name.startswith(("r_index_finger", "r_thumb")) else 0.8
                kd[index] = 0.02
        elif name.startswith("arm_openarm_"):
            kp[index], kd[index] = 1000.0, 40.0
        else:
            raise ValueError(f"Unsupported robot hinge for controller gains: {name}")
    return kp, kd


def apply_gains(model, data, *, body_grasp: bool) -> list[str]:
    actuators = sorted(model.actuators, key=lambda actuator: actuator.index)
    names = tuple(actuator.target_name for actuator in actuators)
    kp, kd = build_position_gains(names, body_grasp=body_grasp)
    for actuator, stiffness, damping in zip(actuators, kp, kd, strict=True):
        actuator.set_kp_override(data, np.asarray(stiffness, dtype=np.float32))
        actuator.set_damping_override(data, np.asarray(damping, dtype=np.float32))
    return list(names)


def apple_floating(model):
    body = model.get_body(APPLE_BODY)
    if body is None or body.floatingbase is None:
        raise RuntimeError("Missing the free apple body")
    return body.floatingbase


def settle_apple(model, data, ctrl):
    """Settle the scene with the servo holding the pre-grasp pose (>= 3 s, 0.5 s window)."""
    data.actuator_ctrls = np.ascontiguousarray(ctrl, dtype=np.float32)
    floating = apple_floating(model)
    poses = []
    for step in range(1, round(10.0 / DT) + 1):
        model.step(data)
        pose = np.asarray(floating.get_dof_pos(data)).reshape(-1)[[0, 1, 2, 6, 3, 4, 5]]
        velocity = np.asarray(floating.get_dof_vel(data)).reshape(-1)
        if not np.isfinite(np.r_[pose, velocity]).all():
            raise RuntimeError("Apple settling diverged")
        poses.append(pose)
        if step * DT < 3.0 or len(poses) < round(0.5 / DT):
            continue
        array = np.asarray(poses[-round(0.5 / DT) :], dtype=float)
        quats = array[:, 3:] / np.linalg.norm(array[:, 3:], axis=1, keepdims=True)
        position_error = float(np.max(np.linalg.norm(array[:, :3] - array[0, :3], axis=1)))
        angle_error = float(np.max(2 * np.arccos(np.clip(np.abs(quats @ quats[0]), 0, 1))))
        if position_error <= 0.00025 and angle_error <= np.deg2rad(0.25):
            return pose.copy(), velocity.copy(), step * DT
    raise RuntimeError("Apple did not settle within 10 s")


class TaskViewer:
    """Render live data while keeping task stepping and timing under the runner's control."""

    def __init__(self, model):
        from motrixsim.render import RenderSettings

        self.paused = False
        self._closed = False
        self._resume_pending = False
        self._clock_origin = time.monotonic()
        ui = mx.ui
        with ui.View(title="Task controls") as controls:
            with ui.Column():
                with ui.Row():
                    self.play_button = ui.Button("Play")
                    self.pause_button = ui.Button("Pause")
                ui.Text("Playback starts automatically. Close the window to stop the task.")
        blueprint = ui.Horizontal(
            [
                ui.Tabs([controls]),
                ui.Viewport(title="Apple grasp", camera=ui.CameraRef.system()),
            ],
            shares=[0.2, 0.8],
        )
        self.app = mx.render.RenderApp(blueprint=blueprint)
        self.app.launch(model, render_settings=RenderSettings.quality())

    @property
    def is_closed(self):
        return self._closed or self.app.is_closed

    def sync(self, data):
        if self.is_closed:
            return
        try:
            self.app.sync(data)
        except mx.render.RenderClosedError:
            self._closed = True
            return
        was_paused = self.paused
        if self.play_button.is_clicked():
            self.paused = False
        if self.pause_button.is_clicked():
            self.paused = True
        if was_paused and not self.paused:
            self._resume_pending = True

    def reset_clock(self, task_time):
        """Exclude pauses and cold-path planning from real-time pacing."""
        self._clock_origin = time.monotonic() - task_time

    def wait_until_playing(self, data, task_time):
        """Service UI events with frozen physics; return False when the window closes."""
        was_paused = self.paused
        while self.paused and not self.is_closed:
            self.sync(data)
            if self.paused and not self.is_closed:
                time.sleep(RENDER_INTERVAL)
        if (was_paused or self._resume_pending) and not self.is_closed:
            self.reset_clock(task_time)
            self._resume_pending = False
        return not self.is_closed

    def frame(self, data, task_time):
        self.sync(data)
        if not self.is_closed:
            time.sleep(max(0.0, self._clock_origin + task_time - time.monotonic()))

    def close(self):
        self.app.close()


def run_task():
    record = np.load(RECORD, allow_pickle=True)
    plan_names = [str(name) for name in record["names"]]
    targets_plan = record["targets"]  # 20000 x 54, planner joint order (post-replan).

    model = mx.load_model(str(SCENE))
    data = mx.SceneData(model)

    actuators = sorted(model.actuators, key=lambda actuator: actuator.index)
    if any(actuator.typ != "position" for actuator in actuators):
        raise RuntimeError("Expected position actuators only")
    actuator_names = [actuator.target_name for actuator in actuators]
    permutation = np.asarray([plan_names.index(name) for name in actuator_names])
    targets = np.ascontiguousarray(targets_plan[:, permutation], dtype=np.float32)

    initial = dict(zip(plan_names, record["sampled_actual"][0].tolist()))
    qpos = np.asarray(data.dof_pos, dtype=np.float32).copy()
    for name, value in initial.items():
        joint = model.get_joint(name)
        if joint is None:
            raise RuntimeError(f"Joint {name} from the replay is missing in the model")
        qpos[joint.dof_pos_index] = value
    data.set_dof_pos(np.ascontiguousarray(qpos), model)
    data.set_dof_vel(np.zeros(len(np.asarray(data.dof_vel)), dtype=np.float32))

    print("Settling the apple...", flush=True)
    pose, velocity, settling_seconds = settle_apple(model, data, targets[0])
    print(f"Settled in {settling_seconds:.2f}s at {np.round(pose[:3], 5).tolist()}", flush=True)

    floating = apple_floating(model)
    qpos[floating.dof_pos_indices] = pose[[0, 1, 2, 4, 5, 6, 3]]  # back to native order
    vel = np.asarray(data.dof_vel, dtype=np.float32).copy()
    vel[floating.dof_vel_indices] = velocity
    data.set_dof_pos(np.ascontiguousarray(qpos), model)
    data.set_dof_vel(np.ascontiguousarray(vel))

    renderer = TaskViewer(model)
    renderer.sync(data)
    renderer.reset_clock(0.0)

    completed = 0
    total_steps = round(REPLAY_SECONDS / DT)
    try:
        for step in range(total_steps):
            time_s = step * DT
            if not renderer.wait_until_playing(data, time_s):
                break
            if step == round(GAIN_SWITCH_TIME / DT):
                apply_gains(model, data, body_grasp=True)
                renderer.reset_clock(time_s)
            data.actuator_ctrls = targets[step]
            model.step(data)
            completed = step + 1
            if not np.isfinite(np.asarray(data.dof_pos)).all():
                raise RuntimeError(f"Non-finite state at {completed * DT:.3f}s")
            if completed % RENDER_STEPS == 0:
                renderer.frame(data, completed * DT)
            if completed % round(1.0 / DT) == 0:
                apple_z = np.asarray(floating.get_dof_pos(data)).reshape(-1)[2]
                # The sensors report only "found": the number of contact points.
                index_contacts = int(model.get_sensor_value("apple_contact_r_index_finger_pad_collision", data)[0])
                thumb_contacts = int(model.get_sensor_value("apple_contact_r_thumb_pad_collision", data)[0])
                print(
                    f"{completed * DT:5.1f}s  apple_z={apple_z:.4f}  "
                    f"index contacts={index_contacts}  thumb contacts={thumb_contacts}",
                    flush=True,
                )
    finally:
        renderer.close()

    if completed == total_steps:
        print("Playback complete.", flush=True)
    else:
        print(f"Playback stopped at {completed * DT:.2f}s.", flush=True)


if __name__ == "__main__":
    run_task()
