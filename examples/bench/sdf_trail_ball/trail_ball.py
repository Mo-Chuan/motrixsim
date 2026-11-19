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

"""Trail ball demo: a ball dropping and rolling down a spiral slide.

The scene is loaded from an MJCF asset whose slide (trail) and ball geoms are
both ``type="sdf"``, so every contact below is solved by the engine's SDF narrow
phase. The asset defines no actuators; the ball simply free-falls onto the top
of the slide, then rolls along the near-zero-friction groove to the ground.

The render window draws immediate-mode gizmos for every penetrating contact:
an orange sphere at the contact point and a cyan normal arrow whose length grows
with the penetration depth. A side panel shows the ball's live world linear and
angular velocity, the CPU frame generation time (physics stepping only), and a
button resets the ball to its initial drop pose.

Note:
    The first launch cooks the SDF for both meshes, which may take a few seconds.

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
import time
from pathlib import Path

import numpy as np

from motrixsim import SceneData, load_model, run, step, ui
from motrixsim.render import Color, RenderApp

# Gizmos style. The ball has a radius of about 0.27 scene units, so the marker
# and arrow sizes below are clearly visible while staying small next to the ball.
GIZMOS_LINE_WIDTH = 6.0
CONTACT_MARKER_RADIUS = 0.03
CONTACT_POINT_COLOR = Color.rgb(1.0, 0.55, 0.1)
CONTACT_NORMAL_COLOR = Color.rgb(0.2, 0.9, 1.0)
# Normal arrows lengthen with penetration depth, from MIN at first touch to MAX
# at DEPTH_REF deep (both geoms of the asset use a 0.0005 margin).
NORMAL_ARROW_MIN = 0.6
NORMAL_ARROW_MAX = 1.6
DEPTH_REF = 5e-4

# Initial drop pose of the ball, taken from the asset ("body name=ball pos=1 0 4.8").
BALL_START_POS = np.array([1.0, 0.0, 4.8])
# Identity quaternion in [i, j, k, w] format.
IDENTITY_QUAT = np.array([0.0, 0.0, 0.0, 1.0])

# Refresh the velocity readouts every N rendered frames (60 fps render).
SPEED_READOUT_INTERVAL = 10

# Smoothing factor of the exponential moving average over frame generation time.
FRAME_TIME_EMA_ALPHA = 0.1

# Resolve the asset relative to this file (examples/bench/trail_ball/../../assets/...)
# so the demo runs from any working directory, not only the motrixsim-docs root.
# parents[2] is the examples/ dir (file -> trail_ball/ -> bench/ -> examples/).
ASSET_PATH = Path(__file__).resolve().parents[2] / "assets" / "sdf" / "trail_ball" / "trail ball.xml"


def normal_arrow_length(depth: float) -> float:
    """Map the penetration depth (negative when penetrating) to an arrow length."""
    t = min(abs(depth) / DEPTH_REF, 1.0)
    return NORMAL_ARROW_MIN + (NORMAL_ARROW_MAX - NORMAL_ARROW_MIN) * t


def main():
    # Loading cooks the SDF of both meshes (cached afterwards).
    model = load_model(str(ASSET_PATH))
    data = SceneData(model)
    ball = model.get_body("ball")
    assert ball is not None, "Expect ball to be a body in model"
    ball_link = ball.base_link
    floatingbase = ball.floatingbase
    assert floatingbase is not None, "Expect the ball body to have a freejoint"
    trail_geom = model.get_geom("trail_sdf")
    ball_geom = model.get_geom("ball_sdf")
    assert trail_geom is not None and ball_geom is not None
    # The ground is visual-only; this pair has no pair/global margin overrides.
    pair_margin = max(trail_geom.margin, ball_geom.margin)

    def reset_ball():
        """Button callback: teleport the ball back to its initial drop state."""
        floatingbase.set_translation(data, BALL_START_POS)
        floatingbase.set_rotation(data, IDENTITY_QUAT)
        floatingbase.set_global_linear_velocity(data, np.zeros(3))
        floatingbase.set_global_angular_velocity(data, np.zeros(3))

    # CPU frame generation timing: each rendered frame runs 0..N physics steps
    # (their wall time accumulates in phys_accum_ns); the sum is the frame's CPU
    # generation cost. Rendering-side work (gizmos, sync, UI) is excluded, so
    # the metric is pure simulation compute time per generated frame. Initialize
    # before the UI block below: ui.Frame(build_fn=...) invokes build_telemetry
    # immediately, so the closure must find these variables already bound.
    phys_accum_ns = 0
    frame_gen_ns = 0.0
    frame_gen_avg_ns = 0.0

    def build_telemetry():
        """Frame build function: query the ball velocities, render them fresh."""
        linear = np.asarray(ball_link.get_linear_velocity(data))
        angular = np.asarray(ball_link.get_angular_velocity(data))
        with ui.Column():
            ui.Text(f"Linear vel:  [{linear[0]:+.3f} {linear[1]:+.3f} {linear[2]:+.3f}] m/s")
            ui.Text(f"Linear speed:  {np.linalg.norm(linear):.3f} m/s")
            ui.Text(f"Angular vel: [{angular[0]:+.3f} {angular[1]:+.3f} {angular[2]:+.3f}] rad/s")
            ui.Text(f"Spin rate:  {np.linalg.norm(angular):.3f} rad/s")
            ui.Text(f"CPU frame gen:  {frame_gen_ns / 1e3:8.1f} µs (avg {frame_gen_avg_ns / 1e3:.1f} µs)")

    show_arrows = True

    def toggle_arrows():
        nonlocal show_arrows
        show_arrows = not show_arrows
        render.rebuild(arrows_frame)

    def build_arrow_button():
        ui.Button("Hide arrows" if show_arrows else "Show arrows", on_click=toggle_arrows)

    viewport = ui.Viewport(camera=ui.CameraRef.system(), title="Viewport")
    telemetry_frame = None
    with ui.View(title="Trail Ball Demo") as controls:
        with ui.Column():
            ui.Text("Ball dropping along an SDF slide.")
            ui.Text("No actuators: gravity and contacts drive everything.")
            with ui.Section("Ball state"):
                # Live readout of the ball velocities: the Frame build function
                # re-queries them on each throttled rebuild.
                telemetry_frame = ui.Frame(build_fn=build_telemetry)
                ui.Button("Reset ball", on_click=reset_ball)
            with ui.Section("Gizmos"):
                arrows_frame = ui.Frame(build_fn=build_arrow_button)
                ui.Text("Orange sphere: contact point.")
                ui.Text("Cyan arrow: contact normal; longer means deeper.")
    # The with-blocks above run unconditionally, so the Frame is always assigned
    # before the view closes.
    assert telemetry_frame is not None
    blueprint = ui.Horizontal([viewport, controls])

    frame = 0

    with RenderApp(blueprint=blueprint) as render:
        render.launch(model)
        render.gizmos.line_width = GIZMOS_LINE_WIDTH
        # Match the reference framing: the full spiral fills the view, with
        # the top groove visible and the slide exit at the lower left.
        render.system_camera.set_view(lookat=[0.23, -0.23, 0.25], distance=12.7, elevation=-30.0, azimuth=221.0)

        def phys_step():
            nonlocal phys_accum_ns
            t0 = time.perf_counter_ns()
            step(model, data)
            phys_accum_ns += time.perf_counter_ns() - t0

        def render_step():
            nonlocal frame, phys_accum_ns, frame_gen_ns, frame_gen_avg_ns
            gizmos = render.gizmos
            # data.low.get_contacts() returns every contact solved this step (SDF pairs
            # included); read each penetrating contact's point/normal/depth. Gizmos are
            # immediate-mode, so redraw them every frame.
            for contact in data.low.get_contacts():
                # Solver depth includes the margin; restore geometric separation.
                geometric_distance = contact.depth + pair_margin
                if geometric_distance >= 0.0:
                    continue
                point = np.asarray(contact.point_a)
                gizmos.draw_sphere(CONTACT_MARKER_RADIUS, point, color=CONTACT_POINT_COLOR)
                if show_arrows:
                    normal = np.asarray(contact.normal)
                    end = point + normal * normal_arrow_length(geometric_distance)
                    gizmos.draw_arrow(point, end, color=CONTACT_NORMAL_COLOR)
            # CPU frame generation time: physics stepping spent on this rendered
            # frame; telemetry rebuild and sync stay out of the metric. Live
            # readout of the ball velocities (throttled rebuild).
            frame_gen_ns = phys_accum_ns
            phys_accum_ns = 0
            if frame == 0:
                frame_gen_avg_ns = frame_gen_ns
            else:
                frame_gen_avg_ns += FRAME_TIME_EMA_ALPHA * (frame_gen_ns - frame_gen_avg_ns)
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
            "Try: mxpython trail_ball.py"
        )
    print("[trail_ball] re-launching through mxpython for macOS rendering", file=sys.stderr)
    os.execv(str(launcher), [str(launcher), *sys.argv])


if __name__ == "__main__":
    _ensure_render_main_thread()
    main()
