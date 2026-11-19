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

import sys

from motrixsim import msd, run
from motrixsim.render import RenderApp

USE_GPU = "--gpu" in sys.argv
if USE_GPU:
    from motrixsim.gpu import SceneData, step
else:
    from motrixsim import SceneData, step


def main():
    print("Press 'A' to disable the system camera.")
    print("Press 'D' to enable the system camera.")
    print("Press 'Q' to disable the scene camera.")
    print("Press 'E' to enable the scene camera.")
    # Create render window for visualization
    with RenderApp() as render:
        # The scene description file
        path = "examples/assets/control_cameras.xml"
        # Load the scene world and configure the camera before building
        world = msd.from_file(path)
        world.hierarchy.cameras[0].render_target = msd.RenderTarget.Image
        world.hierarchy.cameras[0].resolution = [320, 240]
        model = world.build()
        if USE_GPU:
            model = model.to_gpu()

        # Create the render instance of the model
        render.launch(model)
        # Create the physics data of the model
        data = SceneData(model)

        def render_step():
            system_camera = render.system_camera
            # Disable the system camera.
            if render.input.is_key_just_pressed("a"):
                system_camera.active = False
            # Enable the system camera.
            if render.input.is_key_just_pressed("d"):
                system_camera.active = True
            print(f"system camera pose: {system_camera.pose} \n")
            scene_camera = render.get_camera(0)
            # Disable the main camera.
            if render.input.is_key_just_pressed("q"):
                scene_camera.active = False
            # Enable the main camera.
            if render.input.is_key_just_pressed("e"):
                scene_camera.active = True
            render.sync(data)

        run.render_loop(model.options.timestep, 60, lambda: step(model, data), render_step)


if __name__ == "__main__":
    main()
