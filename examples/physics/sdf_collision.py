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

"""SDF collision demo: two meshing gears colliding through signed distance fields.

The scene is loaded from an MJCF asset whose gear geoms are ``type="sdf"``, so the
contact points shown here are solved by the engine's SDF narrow phase. A velocity
actuator spins the driven gear at a target speed (default 2 rad/s); the idler gear
is dragged along purely by tooth-to-tooth SDF contacts.

The render window draws immediate-mode gizmos for all detected contacts, including
margin contacts before geometric overlap. This scene uses zero-gap soft contacts,
so no extra depth filter is applied before drawing. An orange sphere marks the
contact point; a cyan normal arrow grows with negative solver depth, not force.

Note:
    The first launch cooks the SDF for the gear meshes, which may take a few seconds.

    On macOS the render event loop must live on the main thread, so this script
    re-executes itself through the ``mxpython`` launcher when started with plain
    ``python``. Other operating systems run the script directly.

Mouse controls:
- Press and hold left button then drag to rotate the camera/view
- Press and hold right button then drag to pan/translate the view
"""

import os
import shutil
import sys
import threading
from pathlib import Path

import numpy as np

from motrixsim import SceneData, load_model, run, step, ui
from motrixsim.render import Color, RenderApp

# Target speed of the driven gear in rad/s. Change it here, or type a new value
# in the "Gear speed" text field of the window and press Enter.
GEAR_SPEED = 2.0
# Actuator ctrlrange of the asset is [-100, 100]; clamp typed speeds to it.
SPEED_LIMIT = 100.0

# Gizmos style. The gears have a radius of about 2.05 scene units, so the marker
# and arrow sizes below are clearly visible at the default camera distance.
GIZMOS_LINE_WIDTH = 6.0
CONTACT_MARKER_RADIUS = 0.06
CONTACT_POINT_COLOR = Color.rgb(1.0, 0.55, 0.1)
CONTACT_NORMAL_COLOR = Color.rgb(0.2, 0.9, 1.0)
# Normal arrows lengthen with negative solver depth (including contact margin),
# from MIN at zero solver depth to MAX at DEPTH_REF. MAX exceeds the gear
# radius so the deepest arrows reach beyond the gear silhouette; the gears also
# render semi-transparent (material alpha_mode="blend" in the asset) so arrows
# behind teeth stay visible.
NORMAL_ARROW_MIN = 0.5
NORMAL_ARROW_MAX = 2.8
DEPTH_REF = 3e-4

# Refresh the measured-speed readouts every N rendered frames (60 fps render).
SPEED_READOUT_INTERVAL = 10

# Resolve the asset relative to this file (examples/physics/../assets/...) so the
# demo runs from any working directory, not only the motrixsim-docs root.
ASSET_PATH = Path(__file__).resolve().parent.parent / "assets" / "sdf" / "gear_sdf" / "M2_Z20_dual.xml"


def normal_arrow_length(depth: float) -> float:
    """Map margin-adjusted solver depth to an arrow length, not a force."""
    t = min(max(-depth, 0.0) / DEPTH_REF, 1.0)
    return NORMAL_ARROW_MIN + (NORMAL_ARROW_MAX - NORMAL_ARROW_MIN) * t


def main():
    # Loading cooks the SDF of both gear meshes (cached afterwards).
    model = load_model(str(ASSET_PATH))
    data = SceneData(model)
    motor = model.get_actuator("velocity_driven")
    joint_driven = model.get_joint("j_driven")
    joint_idler = model.get_joint("j_idler")
    assert motor is not None and joint_driven is not None and joint_idler is not None
    speed = GEAR_SPEED

    def set_speed(value: str):
        """TextInput submit callback: parse, clamp, and apply the new gear speed."""
        nonlocal speed
        try:
            requested = float(value)
        except ValueError:
            print(f"[sdf_collision] ignoring invalid speed: {value!r}")
            return
        speed = max(-SPEED_LIMIT, min(SPEED_LIMIT, requested))
        motor.set_ctrl(data, speed)
        print(f"[sdf_collision] gear speed set to {speed:g} rad/s")

    def build_telemetry():
        """Frame build function: query the measured DoF velocities, render them fresh."""
        driven = float(joint_driven.get_dof_vel(data)[0])
        idler = float(joint_idler.get_dof_vel(data)[0])
        with ui.Column():
            ui.Text(f"Driven gear speed: {driven:+.3f} rad/s")
            ui.Text(f"Idler gear speed:  {idler:+.3f} rad/s")

    motor.set_ctrl(data, speed)

    show_arrows = True

    def toggle_arrows():
        nonlocal show_arrows
        show_arrows = not show_arrows
        render.rebuild(arrows_frame)

    def build_arrow_button():
        ui.Button("Hide arrows" if show_arrows else "Show arrows", on_click=toggle_arrows)

    viewport = ui.Viewport(camera=ui.CameraRef.system(), title="Viewport")
    telemetry_frame = None
    with ui.View(title="SDF Collision Demo") as controls:
        with ui.Column():
            ui.Text("Gear meshing solved by SDF collision.")
            ui.Text("The idler gear is driven only by tooth contacts.")
            with ui.Section("Drive"):
                ui.Text("Type a target speed (rad/s) and press Enter.")
                ui.TextInput("Gear speed", value=f"{GEAR_SPEED:g}", on_submit=set_speed)
                # Live readout of the measured DoF velocities: the Frame build
                # function re-queries them on each throttled rebuild.
                telemetry_frame = ui.Frame(build_fn=build_telemetry)
            with ui.Section("Gizmos"):
                arrows_frame = ui.Frame(build_fn=build_arrow_button)
                ui.Text("Orange sphere: contact point.")
                ui.Text("Cyan arrow: normal; length shows solver depth, not force.")
    # The with-blocks above run unconditionally, so the Frame is always assigned
    # before the view closes.
    assert telemetry_frame is not None
    blueprint = ui.Horizontal([viewport, controls])

    frame = 0

    with RenderApp(blueprint=blueprint) as render:
        render.launch(model)
        render.gizmos.line_width = GIZMOS_LINE_WIDTH
        # Both hinges spin around world +Y, so look straight down that axis (the
        # camera faces the gear disks) from a distance that frames both gears.
        render.system_camera.set_view(lookat=[2.05, 0.0, 0.0], distance=12.0, elevation=0.0, azimuth=90.0)

        def phys_step():
            step(model, data)

        def render_step():
            nonlocal frame
            gizmos = render.gizmos
            # tag::sdf_collision_contacts[]
            # Read the latest detected contacts (SDF pairs included). Use the
            # margin-adjusted depth used by this scene's solver. Gizmos are
            # immediate-mode, so redraw them every frame.
            for contact in data.low.get_contacts():
                # This scene has zero gap and soft contacts: constraint construction
                # applies no depth-sign filter, so draw every returned contact too.
                point = np.asarray(contact.point_a)
                gizmos.draw_sphere(CONTACT_MARKER_RADIUS, point, color=CONTACT_POINT_COLOR)
                if show_arrows:
                    normal = np.asarray(contact.normal)
                    end = point + normal * normal_arrow_length(contact.depth)
                    gizmos.draw_arrow(point, end, color=CONTACT_NORMAL_COLOR)
            # end::sdf_collision_contacts[]
            # Live readout of the measured DoF velocities (throttled rebuild).
            frame += 1
            if frame % SPEED_READOUT_INTERVAL == 0:
                render.rebuild(telemetry_frame)
            render.sync(data)

        run.render_loop(model.options.timestep, 60, phys_step, render_step)


def _ensure_render_main_thread():
    """Re-exec through mxpython when plain ``python`` cannot render on macOS.

    winit requires the render event loop to sit on the main thread (macOS only).
    The mxpython launcher reserves the main thread for rendering and runs the
    script on a side thread, so a script running on a non-main thread is already
    launched. Windows and Linux have no such requirement.
    """
    if sys.platform != "darwin":
        return
    if threading.current_thread() is not threading.main_thread():
        return  # Already running under mxpython (or another launcher).
    # Prefer the mxpython installed next to the active interpreter (venv bin dir),
    # then fall back to PATH.
    candidates = [Path(sys.executable).with_name("mxpython"), Path(shutil.which("mxpython") or "")]
    launcher = next((p for p in candidates if p.is_file()), None)
    if launcher is None:
        raise SystemExit(
            "On macOS this demo must run through the mxpython launcher "
            f"(not found next to {sys.executable} or on PATH). "
            "Try: mxpython sdf_collision.py"
        )
    print("[sdf_collision] re-launching through mxpython for macOS rendering", file=sys.stderr)
    os.execv(str(launcher), [str(launcher), *sys.argv])


if __name__ == "__main__":
    _ensure_render_main_thread()
    main()
