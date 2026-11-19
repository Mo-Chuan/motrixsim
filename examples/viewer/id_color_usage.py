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


def check_geometry(geom):
    index = geom.index
    name = geom.name
    shape = geom.shape
    print(f"geom index :{index}, name :{name}, shape : {shape}")


def main():

    # Create render window for visualization
    with RenderApp(headless=True) as render:
        # The scene description file
        path = "examples/assets/batch_world_render.xml"
        # Load the scene world and configure the camera before building the immutable model
        world = msd.from_file(path)
        scene_cam = world.hierarchy.cameras[0]
        # tag: camera render target
        scene_cam.render_target = msd.RenderTarget.Image
        scene_cam.resolution = [64, 64]
        # endtag

        # tag: segmentation camera
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

        linear_pixels = image.pixels
        print(f"image shape : {linear_pixels.shape}")
        print(f"image data count :{linear_pixels.size}")

        floor = linear_pixels[60][2]
        floor_id = floor[0]

        bar = linear_pixels[43][32]
        bar_id = bar[0]

        cube = linear_pixels[60][32]
        cube_id = cube[0]

        print(f"floor_id is {floor_id}, bar_id is {bar_id}, cube_id is {cube_id}, ")

        # geom id  = color id - 1
        floor_geom = model.geoms[floor_id - 1]
        print(f"floor_id : {floor_id}, geom :")
        check_geometry(floor_geom)

        bar_geom = model.geoms[bar_id - 1]
        print(f"bar_id : {bar_id}, geom :")
        check_geometry(bar_geom)

        cube_geom = model.geoms[cube_id - 1]
        print(f"cube_id : {cube_id}, geom :")
        check_geometry(cube_geom)


if __name__ == "__main__":
    main()
