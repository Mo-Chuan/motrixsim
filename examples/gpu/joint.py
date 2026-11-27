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

# tag::init[]
from pathlib import Path

import numpy as np

from motrixsim import run
from motrixsim.gpu import SceneData, load_model, step
from motrixsim.render import RenderApp, RenderSettings

# Mouse controls:
# - Press and hold left button then drag to rotate the camera/view
# - Press and hold right button then drag to pan/translate the view

ASSETS = Path(__file__).resolve().parents[1] / "assets"


def main():
    # Create render window for visualization
    with RenderApp() as render:
        # The scene description file
        path = ASSETS / "joint.xml"
        # Load the scene model
        model = load_model(path)

        # Create the render instance of the model with high-quality GI enabled.
        render_settings = RenderSettings.quality()
        render_settings.enable_ssgi = True
        render.launch(model, render_settings=render_settings)
        # Create the physics data of the model
        data = SceneData(model)
        # end::init[]

        # Frame the four booths: hinge, ball, slide and free.
        render.system_camera.set_view(
            lookat=[0.0, 0.0, 0.6], distance=7.5, elevation=-20.0, azimuth=90.0
        )

        # tag::access_all[]
        # ----------Try to access joint data----------
        # How many joints in the model?
        num_joints = model.num_joints
        # Get the all joints in the model
        joints = model.joints
        # The name list of joints in the model
        joint_names = model.joint_names
        print(f"num_joints :{num_joints}, joint_names : {joint_names}, joints : {joints}")
        # end::access_all[]

        # tag::joint_index[]
        # Try to visit "joint_A"
        joint_A_index = model.get_joint_index("joint_A")
        joint_A = model.get_joint(joint_A_index)
        assert joint_A is not None, "Expect joint_A in the model"
        print(f"joint_addr is {joint_A_index}, joint_A is : {joint_A}, name is : {joint_A.name}")
        # end::joint_index[]

        # The num_dof_vel of joint_A
        joint_A_num_dof_vel = joint_A.num_dof_vel
        # The num_dof_pos of joint_A
        joint_A_num_dof_pos = joint_A.num_dof_pos
        # For a hinge joint, the DOFs for position and velocity are both 1.
        print(f"joint_A_num_dof_vel is : {joint_A_num_dof_vel}, joint_A_num_dof_pos is : {joint_A_num_dof_pos}")

        # The same data of joint_B
        joint_B_index = model.get_joint_index("joint_B")
        joint_B = model.get_joint(joint_B_index)
        assert joint_B is not None, "Expect joint_B in the model"
        joint_B_num_dof_vel = joint_B.num_dof_vel  # Since joint_B is ball joint ,the num_dof_vel is 3
        joint_B_num_dof_pos = joint_B.num_dof_pos  # Since joint_B is ball joint ,the num_dof_pos is 4
        print(f"joint_B_num_dof_vel is : {joint_B_num_dof_vel}, joint_B_num_dof_pos is : {joint_B_num_dof_pos}")

        # And joint_C, the slide joint that lifts the green carriage
        joint_C_index = model.get_joint_index("joint_C")
        assert joint_C_index is not None, "Expect joint_C in the model"
        joint_C = model.get_joint(joint_C_index)
        assert joint_C is not None, "Expect joint_C in the model"
        joint_C_num_dof_vel = joint_C.num_dof_vel  # Since joint_C is slide joint, the num_dof_vel is 1
        joint_C_num_dof_pos = joint_C.num_dof_pos  # Since joint_C is slide joint, the num_dof_pos is 1
        print(f"joint_C_num_dof_vel is : {joint_C_num_dof_vel}, joint_C_num_dof_pos is : {joint_C_num_dof_pos}")

        # We can use model to get num_dof_pos/num_dof_vel of all the joints directly
        vel_num = model.joint_dof_vel_nums
        pos_num = model.joint_dof_pos_nums
        print(f"vel_num is : {vel_num}, pos_num is : {pos_num}")

        # tag::joint_dof_pos_vel[]
        # Indicate the first pos data index of each joint in SceneData.dof_pos
        pos_indices = model.joint_dof_pos_indices
        # Indicate the first vel data index of each joint in SceneData.dof_vel
        vel_indices = model.joint_dof_vel_indices

        # Read one world to inspect its DOF arrays on the host.
        dof_pos = data.dof_pos.numpy()[0]
        dof_vel = data.dof_vel.numpy()[0]

        # The pos of joint_A (hinge joint ,num_dof_pos is 1, num_dof_vel is 1)
        joint_A_pos_0 = dof_pos[pos_indices[joint_A_index]]
        joint_A_vel_0 = dof_vel[vel_indices[joint_A_index]]
        print(f"joint_A_pos_0 is :{joint_A_pos_0}, joint_A_vel_0 is : {joint_A_vel_0} ")
        # end::joint_dof_pos_vel[]

        print("--------------------")
        # The pos of joint_B (ball joint, num_dof_pos is 4, num_dof_vel is 3)
        joint_B_pos = dof_pos[pos_indices[joint_B_index] : pos_indices[joint_B_index] + joint_B_num_dof_pos]
        print(f"joint_B_pos is :{joint_B_pos}")
        print("--------------------")

        joint_B_vel = dof_vel[vel_indices[joint_B_index] : vel_indices[joint_B_index] + joint_B_num_dof_vel]
        print(f"joint_B_vel = {joint_B_vel}")

        # The slide joint moves along its axis, one scalar for pos and vel each
        joint_C_pos = dof_pos[pos_indices[joint_C_index]]
        print(f"joint_C_pos is : {joint_C_pos}, in meter")

        # tag::joint_limits[]
        joint_limits = model.joint_limits
        assert joint_limits.shape == (2, num_joints), (
            f"joint_limits shape is {joint_limits.shape}, but should be (2, {num_joints})"
        )
        # Limits of joint_A
        limits = joint_limits[:, joint_A_index]
        print(f"limit of joint_A in radian, min: {limits[0]}, max: {limits[1]}")

        # Limits of joint_B
        limits = joint_limits[:, joint_B_index]
        print(f"limit of joint_B is radian, min: {limits[0]}, max: {limits[1]}")

        # Limits of joint_C, slide joints are limited in meter
        limits = joint_limits[:, joint_C_index]
        print(f"limit of joint_C in meter, min: {limits[0]}, max: {limits[1]}")
        # end::joint_limits[]
        # -----------End----------

        # tag::set_pos_vel[]
        print("-------Set vel and get-------")
        # Try to set vel directly
        joint_A.set_dof_vel(data, np.array([0.1]))
        # Use joint dof_vel_index to get dof_vel
        dof_vel_index = joint_A.dof_vel_index
        dof_vel = data.dof_vel.numpy()[0][dof_vel_index]
        print(f"dof_vel of joint_A is : {dof_vel}")
        # Or use get_dof_vel() directly. It's the same with dof_pos
        dof_vel = joint_A.get_dof_vel(data)
        print(f"dof_vel of joint_A is : {dof_vel}")
        print("-----------------------------")
        # Set positions
        # joint_A, num_dof_pos is 1
        joint_A.set_dof_pos(data, [0.1])
        # joint_B, num_dof_pos is 4
        joint_B.set_dof_pos(data, [0.0, 0.0, 0.0, 1.0])
        # end::set_pos_vel[]
        # joint_C, num_dof_pos is 1, start the carriage mid-rail so the sinusoidal
        # drive can sweep it across the whole -0.5..0.5 m travel
        joint_C.set_dof_pos(data, [0.0])

        # ----------The fourth booth: a free-floating cube----------
        # A freejoint takes no slot in model.joints, it turns the body into a floating base.
        free_cube = model.get_body("free_cube")
        assert free_cube is not None, "Expect free_cube in the model"
        free_fb = free_cube.floatingbase
        assert free_fb is not None, "Expect free_cube to be a floating base body"
        # 7 pos values (position + quaternion) and 6 vel values (linear + angular)
        print(f"free_cube dof_pos_start : {free_fb.dof_pos_start}, dof_vel_start : {free_fb.dof_vel_start}")
        print(f"free_cube dof_pos is : {free_fb.get_dof_pos(data)}")
        print(f"free_cube dof_vel is : {free_fb.get_dof_vel(data)}")
        # Drop the cube onto its pad; the drive loop below keeps kicking it, so all
        # six DOF (translation + rotation) stay alive instead of settling to rest.
        free_fb.set_translation(data, np.array([3.0, 0.0, 1.2]))
        free_fb.set_rotation(data, np.array([0.259, 0.0, 0.0, 0.966]))

        next_kick = 2.5
        step_index = 0

        def control_and_phys_step():
            nonlocal next_kick, step_index

            # Drive on the simulation clock, not the wall clock, so the scripted
            # motion is identical whether the loop runs interactive or headless.
            t = step_index * model.options.timestep
            step_index += 1

            # Booth 1, hinge: write the analytic angle AND its derivative every
            # step -> the arm sweeps deterministically back and forth between
            # its -60..60 degree limits, like a metronome (~4 s per sweep).
            joint_A.set_dof_pos(data, [0.96 * np.sin(1.5 * t)])
            joint_A.set_dof_vel(data, [1.44 * np.cos(1.5 * t)])

            # Booth 2, ball: the tilt nods 8..36 degrees inside the 40 degree
            # cone while the azimuth swings -63..63 degrees, so the rod reverses
            # in more than one plane instead of tracing one fixed circle.
            tilt = 0.385 + 0.245 * np.sin(1.3 * t)
            tilt_rate = 0.245 * 1.3 * np.cos(1.3 * t)
            azim = 1.1 * np.sin(0.9 * t)
            azim_rate = 1.1 * 0.9 * np.cos(0.9 * t)
            half = 0.5 * tilt
            joint_B.set_dof_pos(data, [np.sin(half) * np.cos(azim), np.sin(half) * np.sin(azim), 0.0, np.cos(half)])
            joint_B.set_dof_vel(data, [-tilt_rate * np.sin(azim), tilt_rate * np.cos(azim), azim_rate])

            # Booth 3, slide: analytic position + derivative -> the carriage
            # travels back and forth along the whole -0.5..0.5 m rail.
            joint_C.set_dof_pos(data, [0.44 * np.sin(1.2 * t)])
            joint_C.set_dof_vel(data, [0.528 * np.cos(1.2 * t)])

            # Booth 4, freejoint: between kicks, gravity and floor contact own
            # the cube. Every 2.5 s of sim time re-kick it with a different
            # impulse (hop, hop with spin, sideways hop, counter-spin) -> free
            # 6-DOF motion that keeps repeating without leaving its pad.
            if t >= next_kick:
                variant = int(t / 2.5) % 4
                if variant == 0:
                    free_fb.set_global_linear_velocity(data, np.array([0.6, 0.0, 2.6]))
                    free_fb.set_local_angular_velocity(data, np.array([0.0, 0.0, 2.5]))
                elif variant == 1:
                    free_fb.set_global_linear_velocity(data, np.array([-0.6, 0.2, 2.2]))
                    free_fb.set_local_angular_velocity(data, np.array([2.0, 0.0, 0.0]))
                elif variant == 2:
                    free_fb.set_global_linear_velocity(data, np.array([0.1, 0.6, 2.6]))
                    free_fb.set_local_angular_velocity(data, np.array([0.0, 2.5, 0.0]))
                else:
                    free_fb.set_global_linear_velocity(data, np.array([-0.2, -0.6, 2.2]))
                    free_fb.set_local_angular_velocity(data, np.array([0.0, 0.0, -2.5]))
                next_kick += 2.5

            # Physics world step
            step(model, data)

        def render_step():
            # Sync render objects from physics world
            render.sync(data)

        run.render_loop(model.options.timestep, 60, control_and_phys_step, render_step)


if __name__ == "__main__":
    main()
