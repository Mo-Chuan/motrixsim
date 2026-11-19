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

"""Single-crank Strandbeest; run with --headless for measured physics validation."""

import argparse
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import motrixsim as mx

ASSETS = Path(__file__).resolve().parents[1] / "assets" / "strandbeest"


def load_scene(path, suspended=False, timestep=None, screenshot=False):
    """Load an assembled MJCF, optionally fixing its base for a no-gravity check."""
    root = ET.parse(path).getroot()
    # String loading has no source-directory context for relative texture paths.
    for texture in root.findall("asset/texture"):
        if texture.get("file"):
            texture.set("file", str((Path(path).parent / texture.get("file")).resolve()))
    option = root.find("option")
    if timestep is not None:
        option.set("timestep", str(timestep))
    if suspended:
        option.set("gravity", "0 0 0")
        for parent in root.iter("body"):
            for joint in list(parent):
                if joint.tag == "freejoint" or (joint.tag == "joint" and joint.get("type") in ("free", "slide")):
                    parent.remove(joint)
        for geom in root.iter("geom"):
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
    world = mx.msd.from_str(ET.tostring(root, encoding="unicode"))
    if screenshot:
        # Offscreen target for the system camera, configured before building the model
        world.system_camera.render_target = mx.msd.RenderTarget.Image
        world.system_camera.resolution = [1280, 800]
    model = world.build()
    data = mx.SceneData(model)
    model.forward_kinematic(data)
    return model, data


def run(args):
    path = ASSETS / "strandbeest.xml"
    model, data = load_scene(path, args.suspended, args.timestep, args.screenshot)
    crank_name, actuator_name, base_name = "joint_crossbar_crank", "crank_drive", "crossbar"
    crank = model.get_joint(crank_name)
    drive = model.get_actuator(actuator_name)
    base = model.get_link(base_name)
    pairs = [(model.get_site(f"loop_{i:02d}_a"), model.get_site(f"loop_{i:02d}_b")) for i in range(36)]
    dt = model.options.timestep
    camera_distance = 3.5
    initial_position = base.get_pose(data)[:3].copy()
    initial_crank = float(crank.get_dof_pos(data)[0])
    initial_errors = [np.linalg.norm(a.get_position(data) - b.get_position(data)) for a, b in pairs]
    samples = []
    min_height = float(initial_position[2])
    steps = round(args.seconds / dt)
    sample_stride = max(1, round(0.02 / dt))
    render_stride = max(1, round(1.0 / (60 * dt)))
    renderer = None
    start = time.perf_counter()
    if not args.headless or args.screenshot:
        renderer = mx.render.RenderApp(headless=args.headless)
        settings = mx.render.RenderSettings.quality()
        settings.enable_shadow = True
        settings.enable_ssao = True
        renderer.launch(model, render_settings=settings)
        renderer.system_camera.set_view(
            lookat=initial_position.tolist(), distance=camera_distance, elevation=-25, azimuth=135
        )
    status = np.zeros((), dtype=np.int32)
    try:
        for i in range(steps):
            t = (i + 1) * dt
            speed = 2 * np.pi * args.hz * min(1.0, t / args.ramp) if args.ramp > 0 else 2 * np.pi * args.hz
            drive.set_ctrl(data, speed)
            mx.step(model, data, status)
            if status.item():
                raise RuntimeError(f"Physics step failed at {t:.6f}s, numerical status={status.item()}")
            if i % sample_stride == 0 or i == steps - 1:
                model.forward_kinematic(data)
                if not np.isfinite(data.dof_pos).all() or not np.isfinite(data.dof_vel).all():
                    raise RuntimeError(f"Non-finite simulation state at {t:.6f}s")
                errors = [float(np.linalg.norm(a.get_position(data) - b.get_position(data))) for a, b in pairs]
                samples.extend(errors)
                min_height = min(min_height, float(base.get_pose(data)[2]))
            if renderer is not None and i % render_stride == 0:
                renderer.system_camera.set_view(
                    lookat=base.get_pose(data)[:3].tolist(), distance=camera_distance, elevation=-25, azimuth=135
                )
                renderer.sync(data)
                if not args.fast and not args.headless:
                    delay = t - (time.perf_counter() - start)
                    if delay > 0:
                        time.sleep(delay)
        model.forward_kinematic(data)
        elapsed = time.perf_counter() - start
        displacement = base.get_pose(data)[:3] - initial_position
        result = {
            "engine": "MotrixSim CPU",
            "numerical_status": int(status),
            "sampling_interval_seconds": sample_stride * dt,
            "base_constraint": "fixed" if args.suspended else "free_6dof",
            "legs": 12,
            "mode": "suspended" if args.suspended else "ground",
            "seconds": steps * dt,
            "timestep": dt,
            "dof_velocity": model.num_dof_vel,
            "actuators": model.num_actuators,
            "connect_loops": len(pairs),
            "initial_closure_max_m": float(max(initial_errors)),
            "closure_rms_m": float(np.sqrt(np.mean(np.square(samples)))),
            "closure_max_m": float(max(samples)),
            "crank_turns": (float(crank.get_dof_pos(data)[0]) - initial_crank) / (2 * np.pi),
            "base_displacement_m": displacement.tolist(),
            "base_min_height_m": min_height,
            "wall_seconds": elapsed,
            "wall_us_per_step": elapsed * 1e6 / max(1, steps),
        }
        if args.screenshot:
            for _ in range(8):
                renderer.sync(data, wait=True)
            task = renderer.system_camera.capture()
            renderer.sync(data, wait=True)
            image = task.take_image()
            if image is None:
                raise RuntimeError("Screenshot capture returned no image")
            image.save_to_disk(str(Path(args.screenshot).resolve()))
        print(json.dumps(result, indent=2))
        if args.output:
            Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
        return result
    finally:
        if renderer is not None:
            renderer.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--legs", type=int, choices=[12], default=12)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--suspended", action="store_true")
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--hz", type=float, default=1)
    parser.add_argument("--ramp", type=float, default=1, help="Speed ramp duration in seconds; zero disables it")
    parser.add_argument("--timestep", type=float)
    parser.add_argument("--fast", action="store_true", help="Do not pace the viewer to real time")
    parser.add_argument("--output", help="Write measured metrics as JSON")
    parser.add_argument("--screenshot", help="Save a final MotrixSim-rendered PNG; works with --headless")
    args = parser.parse_args()
    if args.seconds <= 0 or args.ramp < 0 or (args.timestep is not None and args.timestep <= 0):
        parser.error("seconds and timestep must be positive; ramp must be nonnegative")
    run(args)


if __name__ == "__main__":
    main()
