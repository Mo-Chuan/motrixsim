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

"""Draft example for the viewport camera API; requires the redesigned SDK.

Click a viewport before using the arrow keys. F changes only the right viewport.
The camera widget stays attached to the system camera when bindings change.
"""

from motrixsim import SceneData, msd, run, step, ui
from motrixsim.render import Layout, RenderApp


def main():
    world = msd.from_file("examples/assets/go1/scene.xml")
    sensor = world.find_body("trunk").find_camera("track")
    sensor.render_target = msd.RenderTarget.Image
    sensor.resolution = [320, 240]
    model = world.build()
    cameras = model.cameras
    left = ui.Viewport(camera=ui.CameraRef.system(), title="Free")
    right = ui.Viewport(camera=ui.CameraRef(2), title="Scene")
    blueprint = ui.Horizontal([left, right])
    with RenderApp(blueprint=blueprint) as render:
        render.launch(model)
        data = SceneData(model)
        switch_interactive = render.set_active_viewport_camera
        render.widgets.create_camera_viewport(
            cameras[0],
            target_camera=model.cameras.system_camera,
            layout=Layout(right=10, top=10, width=160, height=120),
        )

        def render_step():
            if render.input.is_key_just_pressed("f"):
                render.set_viewport_camera(right, cameras[3])
            if render.input.is_key_just_pressed("right"):
                switch_interactive(cameras[3])
            if render.input.is_key_just_pressed("left"):
                switch_interactive(cameras[2])
            render.sync(data)

        run.render_loop(model.options.timestep, 60, lambda: step(model, data), render_step)


if __name__ == "__main__":
    main()
