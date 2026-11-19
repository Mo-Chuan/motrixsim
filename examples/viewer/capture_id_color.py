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


from motrixsim import SceneData, msd, step
from motrixsim.render import RenderApp


def main():

    # Create render window for visualization
    with RenderApp(headless=False) as render:
        # The scene description file
        path = "examples/assets/batch_world_render.xml"
        # Load the scene world and configure the camera before building the immutable model
        world = msd.from_file(path)
        scene_cam = world.hierarchy.cameras[0]
        # tag: camera render target
        scene_cam.render_target = msd.RenderTarget.Image
        # endtag

        # tag: id color camera
        scene_cam.segmentation = msd.Segmentation.Id
        # endtag

        model = world.build()
        render.launch(model)
        data = SceneData(model)

        for _ in range(100):
            step(model, data)
            render.sync(data)

        task = render.get_camera(0).capture()
        render.sync(data, wait=True)
        image = task.take_image()
        assert image is not None
        import os

        os.makedirs("shot", exist_ok=True)
        image.save_to_disk(f"shot/capture_{0}.png")
        print(f"Captured image: shot/capture_{0}.png")

        while True:
            step(model, data)
            render.sync(data)


if __name__ == "__main__":
    main()
