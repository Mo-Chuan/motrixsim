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

from motrixsim import SceneData, load_model, step
from motrixsim.render import RenderApp
from motrixsim.render._exper import image_to_tensor


def check_tensor(tensor):
    print("--- basic tensor structure information  ---")
    print(f"Shape : {tensor.shape}")
    print(f"Data type: {tensor.dtype}")
    print(f"Itemsize: {tensor.element_size()}")
    print(f"Strides: {tensor.stride()}")
    print(f"Size: {tensor.numel()}")
    print(f"Dim: {tensor.dim()}")
    print(f"Device: {tensor.device}")
    print("----------------")


def cap_task_to_dlpack(cap_task):
    try:
        img = cap_task.take_image()
        print(f"py image is : {img}")
        tensor = image_to_tensor(img)
        check_tensor(tensor)
        pack = tensor.to_dlpack()
        return pack

    except Exception:
        pass


def main():
    batch = 32 * 32
    render_offsets = []
    for i in range(batch):
        render_offsets.append([i * 2.0, 0, 0])

    count = 50

    # Create render window for visualization
    with RenderApp(headless=True) as render:
        # The scene description file
        path = "examples/assets/batch_world_render.xml"
        # Load the scene model
        model = load_model(path)

        # Create the render instance of the model
        render.launch(model, batch, render_offsets)
        # Create the physics data of the model
        data = SceneData(model, batch=(batch,))
        camera_0 = render.get_camera(0)
        cap_task_0 = camera_0.capture_buffer()

        render.sync(data, wait=True)
        if cap_task_0.state != "pending":
            pack_0 = cap_task_to_dlpack(cap_task_0)
            print(f"Capsule: {pack_0}")
            print(f"Type: {type(pack_0)}")
            print(f"ID: {id(pack_0)}")

        step_count = 0

        while True:
            step_count += 1
            if step_count >= count:
                return

            start = time.perf_counter()
            step(model, data)
            end = time.perf_counter()
            step_cost = (end - start) * 1000
            print(f"Once step_cost: {step_cost:.4f} ms")

            cap_task_0 = camera_0.capture()

            start = time.perf_counter()
            render.sync(data, wait=True)
            end = time.perf_counter()
            render_cost = (end - start) * 1000
            print(f"Once render-capture sync: {render_cost:.4f} ms")

            # use pack as tensor ,compute tensor.....
            total_cost = step_cost + render_cost
            fps = 1000.0 / total_cost
            output_image = fps * batch
            print(f"Fps: {int(fps)}, output images : {int(output_image)}/s")


if __name__ == "__main__":
    main()
