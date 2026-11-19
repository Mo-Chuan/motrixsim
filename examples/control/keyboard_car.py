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

from motrixsim import msd, run, ui
from motrixsim.render import RenderApp

USE_GPU = "--gpu" in sys.argv
if USE_GPU:
    from motrixsim.gpu import SceneData, step
else:
    from motrixsim import SceneData, step


# Mouse controls:
# - Press and hold left button then drag to rotate the camera/view
# - Press and hold right button then drag to pan/translate the view
def main():
    # The scene description file
    path = "examples/assets/keyboard_car.xml"
    # Configure the camera tracking on the MSD layer before building the immutable model
    world = msd.from_file(path)
    camera_desc = world.hierarchy.cameras[0]
    camera_desc.rotation_track = msd.RotationTrack.LookAtLink
    camera_desc.track_target = "car"
    model = world.build()
    if USE_GPU:
        model = model.to_gpu()
    camera = model.cameras[0]

    # Create render window for visualization
    view = ui.Viewport(camera=ui.CameraRef(camera.index), title="Scene")
    with RenderApp(blueprint=view) as render:
        # Create the render instance of the model
        render.launch(model)
        # Create the physics data of the model
        data = SceneData(model)
        # Get actuator for car controlling
        forward = model.get_actuator("forward")
        turn = model.get_actuator("turn")
        forward_value = 0.0
        turn_value = 0.0
        print("==========================")
        print("Press 'W' to move forward, 'S' to move backward, 'A' to turn left, 'D' to turn right.")

        def phys_step():
            nonlocal forward_value, turn_value
            forward.set_ctrl(data, forward_value)
            turn.set_ctrl(data, turn_value)
            step(model, data)

        def render_step():
            nonlocal forward_value, turn_value
            render.sync(data)
            input = render.input
            # Read keyboard from render app
            forward_value = 0.0
            turn_value = 0.0
            if input.is_key_pressed("A"):
                turn_value = 1
            if input.is_key_pressed("d"):
                turn_value = -1
            if input.is_key_pressed("S"):
                forward_value = -1
            if input.is_key_pressed("w"):
                forward_value = 1

        run.render_loop(model.options.timestep, 60, phys_step, render_step)


if __name__ == "__main__":
    main()
