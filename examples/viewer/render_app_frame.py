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

"""Replace parameter controls when the selected mode changes."""

import motrixsim as mx
from motrixsim import run, ui
from motrixsim.render import RenderApp, RenderClosedError


def main():
    model = mx.load_model("examples/assets/boston_dynamics_spot/scene.xml")
    data = mx.SceneData(model)
    target = None

    def build_parameters():
        nonlocal target
        with ui.ScrollArea(max_height=240):
            if mode.value == "Position":
                target = ui.TextInput("Target position (rad)", value="0")
            else:
                target = ui.TextInput("Target velocity (rad/s)", value="0")
                ui.Checkbox("Enable velocity limit", value=True)

    with ui.View(title="Parameters") as parameters:
        with ui.Column():
            mode = ui.Select("Mode", options=["Position", "Velocity"], value="Position")
            frame = ui.Frame(build_fn=build_parameters)

    blueprint = ui.Horizontal([ui.Viewport(camera=ui.CameraRef.system()), parameters], shares=[3, 1])
    with RenderApp(blueprint=blueprint) as renderer:
        renderer.launch(model)

        def render_frame():
            renderer.sync(data)
            if mode.is_changed():
                renderer.rebuild(frame)
            elif target is not None and target.is_submitted():
                print(f"{mode.value}: {target.value}")

        try:
            run.render_loop(
                model.options.timestep,
                60,
                lambda: mx.step(model, data),
                render_frame,
            )
        except RenderClosedError:
            pass


if __name__ == "__main__":
    main()
