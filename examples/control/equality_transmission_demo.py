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

"""CPU equality demos: a welded flange platform and a joint transmission gallery.

Both scenes drive the model through the same control functions, and the dials
are coupled by equality/joint rows, not by teeth, belts or contact.

Run from the repository's motrixsim-python/motrixsim-docs directory:

    # view the bolted-flange platform
    python examples/control/equality_transmission_demo.py --scene weld

    # view the joint transmission gallery, stopping after 5 simulated seconds
    python examples/control/equality_transmission_demo.py --scene joint --window-seconds 5

The MJCF assets live in examples/assets/equality within motrixsim-docs;
this script does not require the repository-root assets directory.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

import motrixsim as mx
from motrixsim.render import RenderApp, RenderClosedError

# Resolve the docs-local assets from this file so the cwd does not matter.
ASSET_DIR = Path(__file__).resolve().parents[1] / "assets" / "equality"

JOINT_SCENE = ASSET_DIR / "joint_transmission.xml"
WELD_SCENE = ASSET_DIR / "weld_flange_platform.xml"

# Quartic transmission law of the right-hand dial: y = p(x).
QUARTIC_COEFFS = (0.0, 0.6, 0.4, -0.15, 0.05)
# Linear reference law of the middle dial: y = q(x).
LINEAR_COEFFS = (0.0, -0.8, 0.0, 0.0, 0.0)

# Welded flange platform drives (lift + three non-coaxial rotations).
WELD_PERIODS = {
    "lift_drive": 4.0,
    "yaw_drive": 5.0,
    "pitch_drive": 3.7,
    "roll_drive": 4.6,
}
WELD_AMPLITUDES = {
    "lift_drive": 0.05,
    "yaw_drive": 0.12,
    "pitch_drive": 0.10,
    "roll_drive": 0.14,
}
WELD_PHASES = {
    "lift_drive": 0.0,
    "yaw_drive": 0.0,
    "pitch_drive": 1.1,
    "roll_drive": 2.2,
}

JOINT_INPUT_AMPLITUDE = 1.0
JOINT_INPUT_PERIOD = 4.0


def _actuator(model: mx.SceneModel, name: str) -> mx.Actuator:
    actuator = model.get_actuator(name)
    if actuator is None:
        raise RuntimeError(f"actuator {name!r} not found")
    return actuator


def _joint(model: mx.SceneModel, name: str) -> mx.Joint:
    joint = model.get_joint(name)
    if joint is None:
        raise RuntimeError(f"joint {name!r} not found")
    return joint


def _site(model: mx.SceneModel, name: str) -> mx.Site:
    site = model.get_site(name)
    if site is None:
        raise RuntimeError(f"site {name!r} not found")
    return site


def _quat_angle(q_a: np.ndarray, q_b: np.ndarray) -> float:
    """Shortest relative rotation angle in radians for two (x, y, z, w) quats."""
    dot = abs(float(np.dot(q_a, q_b)) / (np.linalg.norm(q_a) * np.linalg.norm(q_b)))
    return 2.0 * math.acos(min(1.0, dot))


def _poly(coeffs: tuple[float, ...], x: float) -> float:
    return sum(a * x**i for i, a in enumerate(coeffs))


def _summary(values: list[float]) -> tuple[float, float, float]:
    data = np.asarray(values, dtype=np.float64)
    rms = float(np.sqrt(np.mean(data * data)))
    p99 = float(np.percentile(data, 99.0))
    return rms, float(np.max(data)), p99


@dataclass
class SceneRun:
    name: str
    path: Path
    model: mx.SceneModel
    data: mx.SceneData
    apply_controls: Callable[[float], None]
    sample: Callable[[float], None]
    summary: Callable[[], str]
    sim_time: float = 0.0
    step_time: float = 0.0
    steps: int = 0
    _every: int = 1
    _skip: int = 0
    view_lookat: tuple[float, float, float] = (0.0, 0.0, 0.0)
    view_distance: float = 3.0
    view_elevation: float = -25.0
    view_azimuth: float = 90.0


class WeldScene(SceneRun):
    def __init__(self) -> None:
        model = mx.load_model(WELD_SCENE)
        data = mx.SceneData(model)
        self.drives = {name: _actuator(model, name) for name in WELD_PERIODS}
        self.site_a = _site(model, "flange_a")
        self.site_b = _site(model, "flange_b")
        self.tracked = {
            name: _joint(model, name)
            for name in (
                "a_lift",
                "a_yaw",
                "a_pitch",
                "a_roll",
                "a_slide_x",
                "a_slide_y",
                "b_yaw",
                "b_pitch",
                "b_roll",
            )
        }
        self.pos_errors: list[float] = []
        self.rot_errors: list[float] = []
        self.joint_min = {name: math.inf for name in self.tracked}
        self.joint_max = {name: -math.inf for name in self.tracked}
        self.platform_turn = 0.0
        self._q_a0: np.ndarray | None = None
        super().__init__(
            name="equality-weld-flange",
            path=WELD_SCENE,
            model=model,
            data=data,
            apply_controls=self._apply,
            sample=self._sample,
            summary=self._summary,
            view_lookat=(0.0, 0.0, 0.75),
            view_distance=2.4,
            view_elevation=-34.0,
            view_azimuth=90.0,
        )

    def _apply(self, t: float) -> None:
        for name, drive in self.drives.items():
            phase = WELD_PHASES[name]
            ctrl = WELD_AMPLITUDES[name] * math.sin(2.0 * math.pi * t / WELD_PERIODS[name] + phase)
            drive.set_ctrl(self.data, float(ctrl))

    def _sample(self, _: float) -> None:
        pose_a = self.site_a.get_pose(self.data).astype(np.float64)
        pose_b = self.site_b.get_pose(self.data).astype(np.float64)
        self.pos_errors.append(float(np.linalg.norm(pose_a[:3] - pose_b[:3])))
        self.rot_errors.append(_quat_angle(pose_a[3:], pose_b[3:]))
        q_a = pose_a[3:]
        if self._q_a0 is None:
            self._q_a0 = q_a
        self.platform_turn = max(self.platform_turn, _quat_angle(self._q_a0, q_a))
        for name, joint in self.tracked.items():
            q = float(joint.get_dof_pos(self.data)[0])
            self.joint_min[name] = min(self.joint_min[name], q)
            self.joint_max[name] = max(self.joint_max[name], q)

    def _summary(self) -> str:
        if not self.pos_errors:
            return f"[{self.name}] no simulation samples"
        pos_rms, pos_max, pos_p99 = _summary(self.pos_errors)
        rot_rms, rot_max, rot_p99 = _summary(self.rot_errors)
        spans = ", ".join(f"{name}=[{self.joint_min[name]:+.3f}, {self.joint_max[name]:+.3f}]" for name in self.tracked)
        return (
            f"[{self.name}] steps={self.steps} sim={self.sim_time:.3f}s dt={self.dt:.4f} "
            f"step_us={self.step_us:.1f}\n"
            f"  flange position error: rms={pos_rms:.3e} m max={pos_max:.3e} m p99={pos_p99:.3e} m\n"
            f"  flange rotation error: rms={rot_rms:.3e} rad max={rot_max:.3e} rad "
            f"p99={rot_p99:.3e} rad\n"
            f"  platform orientation change: max={self.platform_turn:.3f} rad\n"
            f"  joint spans (rad or m): {spans}"
        )

    @property
    def dt(self) -> float:
        return float(self.model.options.timestep)

    @property
    def step_us(self) -> float:
        return 1e6 * self.step_time / max(self.steps, 1)


class JointScene(SceneRun):
    def __init__(self) -> None:
        model = mx.load_model(JOINT_SCENE)
        data = mx.SceneData(model)
        self.drive = _actuator(model, "input_drive")
        self.input_joint = _joint(model, "input_angle")
        self.linear_joint = _joint(model, "linear_angle")
        self.quartic_joint = _joint(model, "quartic_angle")
        self.input_x: list[float] = []
        self.linear_y: list[float] = []
        self.quartic_y: list[float] = []
        self.linear_error: list[float] = []
        self.quartic_error: list[float] = []
        self.max_curvature = 0.0
        self.max_input_speed = 0.0
        super().__init__(
            name="equality-joint-transmission",
            path=JOINT_SCENE,
            model=model,
            data=data,
            apply_controls=self._apply,
            sample=self._sample,
            summary=self._summary,
            view_lookat=(0.0, 0.0, 0.1),
            view_distance=3.9,
            view_elevation=-23.0,
            view_azimuth=90.0,
        )

    def _apply(self, t: float) -> None:
        ctrl = JOINT_INPUT_AMPLITUDE * math.sin(2.0 * math.pi * t / JOINT_INPUT_PERIOD)
        self.drive.set_ctrl(self.data, float(ctrl))

    def _sample(self, _: float) -> None:
        x = float(self.input_joint.get_dof_pos(self.data)[0])
        xdot = float(self.input_joint.get_dof_vel(self.data)[0])
        y_linear = float(self.linear_joint.get_dof_pos(self.data)[0])
        y_quartic = float(self.quartic_joint.get_dof_pos(self.data)[0])
        self.input_x.append(x)
        self.linear_y.append(y_linear)
        self.quartic_y.append(y_quartic)
        self.linear_error.append(y_linear - _poly(LINEAR_COEFFS, x))
        self.quartic_error.append(y_quartic - _poly(QUARTIC_COEFFS, x))
        # Second derivative of the quartic law: p''(x) = 2 a2 + 6 a3 x + 12 a4 x^2.
        p_second = 2.0 * QUARTIC_COEFFS[2] + 6.0 * QUARTIC_COEFFS[3] * x + 12.0 * QUARTIC_COEFFS[4] * x * x
        self.max_curvature = max(self.max_curvature, abs(p_second) * xdot * xdot)
        self.max_input_speed = max(self.max_input_speed, abs(xdot))

    def _summary(self) -> str:
        if not self.linear_error:
            return f"[{self.name}] no simulation samples"
        lin_rms, lin_max, lin_p99 = _summary(self.linear_error)
        qua_rms, qua_max, qua_p99 = _summary(self.quartic_error)
        return (
            f"[{self.name}] steps={self.steps} sim={self.sim_time:.3f}s dt={self.dt:.4f} "
            f"step_us={self.step_us:.1f}\n"
            f"  input x range: [{min(self.input_x):+.3f}, {max(self.input_x):+.3f}] rad, "
            f"max |xdot|={self.max_input_speed:.3f} rad/s\n"
            f"  linear  y-q(x) error: rms={lin_rms:.3e} rad max={lin_max:.3e} rad "
            f"p99={lin_p99:.3e} rad (y range [{min(self.linear_y):+.3f}, {max(self.linear_y):+.3f}])\n"
            f"  quartic y-p(x) error: rms={qua_rms:.3e} rad max={qua_max:.3e} rad "
            f"p99={qua_p99:.3e} rad (y range [{min(self.quartic_y):+.3f}, {max(self.quartic_y):+.3f}])\n"
            f"  max |p''(x) xdot^2|: {self.max_curvature:.3f} rad/s^2\n"
            "  note: motion is transmitted by equality/joint rows only, not by contact or teeth."
        )

    @property
    def dt(self) -> float:
        return float(self.model.options.timestep)

    @property
    def step_us(self) -> float:
        return 1e6 * self.step_time / max(self.steps, 1)


def build_scene(name: str) -> SceneRun:
    return WeldScene() if name == "weld" else JointScene()


def _set_timestep(scene: SceneRun, dt: float | None) -> None:
    if dt is not None:
        scene.model.options.timestep = float(dt)
    if float(scene.model.options.timestep) <= 0.0:
        raise RuntimeError("timestep must be positive")


def _step_scene(scene: SceneRun) -> None:
    start = time.perf_counter()
    scene.model.step(scene.data)
    scene.step_time += time.perf_counter() - start
    scene.steps += 1


def run_viewer(scene: SceneRun, max_seconds: float | None) -> None:
    dt = scene.dt
    wall_limit = max_seconds
    try:
        with RenderApp() as render:
            render.launch(scene.model)
            # Keep the interactive system camera (left drag rotates, right drag
            # pans) but start it from a view that frames the demo.
            render.system_camera.set_view(
                lookat=scene.view_lookat,
                distance=scene.view_distance,
                elevation=scene.view_elevation,
                azimuth=scene.view_azimuth,
            )
            last = time.perf_counter()
            accumulator = 0.0
            while not render.is_closed:
                now = time.perf_counter()
                accumulator += min(now - last, 0.05)
                last = now
                while accumulator >= dt:
                    scene.apply_controls(scene.sim_time + dt)
                    _step_scene(scene)
                    scene.sim_time += dt
                    accumulator -= dt
                    if scene._skip == 0:
                        scene.sample(scene.sim_time)
                    scene._skip = (scene._skip + 1) % scene._every
                render.sync(scene.data)
                if wall_limit is not None and scene.sim_time >= wall_limit:
                    break
    except RenderClosedError:
        pass
    print(scene.summary())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", choices=("weld", "joint"), default="weld")
    parser.add_argument("--dt", type=float, default=None, help="override the model timestep")
    parser.add_argument("--window-seconds", type=float, default=None, help="stop the viewer after this many seconds")
    args = parser.parse_args()

    scene = build_scene(args.scene)
    _set_timestep(scene, args.dt)
    run_viewer(scene, args.window_seconds)


if __name__ == "__main__":
    main()
