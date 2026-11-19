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

"""Run a Python-controlled simulation with a custom RenderApp layout."""

import motrixsim as mx
from motrixsim import run, ui
from motrixsim.render import RenderApp, RenderClosedError


def main():
    model = mx.load_model("examples/assets/boston_dynamics_spot/scene.xml")
    data = mx.SceneData(model)

    with ui.View(title="Controls") as controls:
        with ui.Column():
            ui.Text("Simulation is controlled by the Python loop.")
            running = ui.Checkbox("Run simulation", value=True)
            single_step = ui.Button("Step once (when paused)")
            reset = ui.Button("Reset state")

    viewport = ui.Viewport(camera=ui.CameraRef.system(), title="Simulation")
    blueprint = ui.Horizontal(
        [viewport, ui.Tabs([controls, ui.Options(), ui.Log()])],
        shares=[3, 1],
    )

    with RenderApp(blueprint=blueprint) as renderer:
        renderer.launch(model)

        def physics_step():
            if running.value:
                mx.step(model, data)

        def render_frame():
            nonlocal data
            # Synchronization also delivers control changes to Python.
            renderer.sync(data)
            if reset.is_clicked():
                data = mx.SceneData(model)
            elif single_step.is_clicked() and not running.value:
                mx.step(model, data)

        try:
            run.render_loop(model.options.timestep, 60, physics_step, render_frame)
        except RenderClosedError:
            pass


if __name__ == "__main__":
    main()
