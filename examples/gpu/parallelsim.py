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


import warp as wp

from motrixsim import run
from motrixsim.gpu import SceneData, load_model
from motrixsim.render import RenderApp, RenderSettings


@wp.kernel
def randomize_controls(ctrls: wp.array2d(dtype=float), seed: int, selection: int):
    world, actuator = wp.tid()
    # selection: 0 = all worlds, 1 = first world, 2 = odd-indexed worlds.
    if selection == 0 or (selection == 1 and world == 0) or (selection == 2 and world % 2 == 1):
        rng = wp.rand_init(seed, world * ctrls.shape[1] + actuator)
        ctrls[world, actuator] = wp.randf(rng)


def main():
    # Create render window for visualization
    with RenderApp() as render:
        # tag::init batch data
        size = 10
        batch_size = size * size
        path = "examples/assets/stanford_tidybot/scene_position.xml"

        model = load_model(path)
        data = SceneData(model, batch=(batch_size,))
        model.capture_step(data)

        # When create scene data in batch mode, we also need to launch the render in batch mode.
        # The render offset can be assigned for each instance to avoid overlapping.
        # Note: The offset only affects the render objects, the physics instance is still at the origin.
        render_offset = []
        for i in range(size):
            for j in range(size):
                render_offset.append([-i * 2, j * 2, 0])
        render.launch(
            model,
            batch=batch_size,
            render_offset=render_offset,
            render_settings=RenderSettings.performance(),
        )
        # a batch dimension is added to all data fields
        assert data.dof_pos.shape == (batch_size, model.num_dof_pos)

        # end::init batch data

        print("Press 'r' to reset all instances")
        print("Press 'a' to apply a random actuator ctrls to all instances")
        print("Press 's' to apply a random actuator ctrls to the first instance")
        print("Press 'd' to apply a random actuator ctrls to instances with odd index")

        # A new seed per key action gives repeatable, changing random controls.
        seed = 42

        def set_random_controls(selection):
            nonlocal seed
            wp.launch(
                randomize_controls,
                dim=(batch_size, model.num_actuators),
                inputs=[data.actuator_ctrls, seed, selection],
            )
            seed += 1

        def physics_step():
            model.step(data)

        def render_step():
            render.sync(data)
            input = render.input
            if input.is_key_just_pressed("r"):
                data.reset(model)
            if input.is_key_just_pressed("a"):
                set_random_controls(0)
            if input.is_key_just_pressed("s"):
                set_random_controls(1)
            if input.is_key_just_pressed("d"):
                set_random_controls(2)

        run.render_loop(
            model.options.timestep,
            60.0,
            physics_step,
            render_step,
        )


if __name__ == "__main__":
    main()
