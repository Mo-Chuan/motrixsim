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


import time

from motrixsim import SceneData, msd, step
from motrixsim.render import RenderApp


def main():
    batch = 4 * 4
    render_offsets = []
    for i in range(batch):
        render_offsets.append([i * 2.0, 0, 0])

    count = 8

    # Create render window for visualization
    with RenderApp(headless=True) as render:
        # The scene description file
        path = "examples/assets/batch_world_render.xml"
        # Load the scene world and configure the camera before building the immutable model
        world = msd.from_file(path)
        scene_cam = world.hierarchy.cameras[0]
        # tag: camera render target
        scene_cam.render_target = msd.RenderTarget.Image

        model = world.build()

        # Create the render instance of the model
        render.launch(model, batch, render_offsets)
        # Create the physics data of the model
        data = SceneData(model, batch=(batch,))
        camera_0 = render.get_camera(0)
        render.sync(data, wait=False)

        step_count = 0

        time.sleep(2)

        while True:
            step_count += 1
            if step_count >= count:
                return

            step(model, data)
            cap_task = camera_0.capture()
            render.sync(data, wait=True)
            image = cap_task.take_image()
            assert image is not None
            import os

            os.makedirs("shot", exist_ok=True)
            image.save_to_disk(f"shot/capture_{step_count}.png")
            print(f"Captured image: shot/capture_{step_count}.png")


if __name__ == "__main__":
    main()
