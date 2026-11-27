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

"""
Example demonstrating the Link API.

A three-link pendulum chain hangs from a static pillar and swings under
gravity. Every frame the script reads each link's pose / velocity via the
Link API and overlays immediate-mode gizmos so the data is visible:

- RGB axes at each link frame origin  (link.get_pose / get_rotation)
- a white sphere at each center of mass (link.center_of_mass, local)
- a yellow arrow for the linear velocity (link.get_linear_velocity)

Every few seconds link_3 receives an external force kick through
link.add_external_force, visibly deflecting the chain.
"""

import sys
from pathlib import Path

import numpy as np

from motrixsim import run
from motrixsim.render import Color, RenderApp, RenderClosedError, RenderSettings

if "--gpu" in sys.argv:
    from motrixsim.gpu import SceneData, load_model, step
else:
    from motrixsim import SceneData, load_model, step

ASSETS = Path(__file__).resolve().parents[1] / "assets"

CHAIN = ("link_1", "link_2", "link_3")
AXES_LEN = 0.22
COM_RADIUS = 0.03
VEL_SCALE = 0.12
KICK_INTERVAL = 2.5  # seconds between external-force kicks on link_3
PRINT_INTERVAL = 120  # frames between console speed summaries


def main():
    # Create render window for visualization
    with RenderApp() as render:
        # The scene description file
        path = ASSETS / "link.xml"
        # Load the scene model
        model = load_model(path)
        # Create the render instance of the model with high-quality GI enabled.
        render_settings = RenderSettings.quality()
        render_settings.enable_ssgi = True
        render.launch(model, render_settings=render_settings)
        # Create the physics data of the model
        data = SceneData(model)

        # Frame the pendulum chain.
        render.system_camera.set_view(lookat=[0.0, 0.0, 1.3], distance=6.5, elevation=-15.0, azimuth=90.0)

        # tag::link_struct[]
        # ----------Try to access link data----------
        # How many links in the model? (the world does not count)
        num_links = model.num_links
        # The link list and the link name list
        links = model.links
        link_names = model.link_names
        print(f"num_links is : {num_links}, link_names : {link_names}")

        # Get all the links' poses at once, shape (*data.shape, num_links, 7)
        link_poses = model.get_link_poses(data)
        print(f"link_poses shape is : {tuple(link_poses.shape)}")

        # Get the link by name, or by index
        link_1 = model.get_link("link_1")
        link_1_index = model.get_link_index("link_1")
        assert link_1_index == link_1.index
        chain = [model.get_link(name) for name in CHAIN]
        # end::link_struct[]

        # Walk the kinematic tree: parent, children and the joints on a link.
        for link in chain:
            parent_name = link.parent.name if link.parent is not None else "world"
            children_names = [child.name for child in link.children]
            joint_names = [joint.name for joint in link.joints()]
            print(
                f"link {link.name}: index {link.index}, parent {parent_name}, "
                f"children {children_names}, joints {joint_names}, "
                f"mass {link.mass:.3f}, geoms {len(link.geoms)}"
            )

        # tag::link_query[]
        # Per-link state queries: pose, rotation, velocities
        pose = link_1.get_pose(data)  # [x, y, z, i, j, k, w]
        position = link_1.get_position(data)
        rotation = link_1.get_rotation(data)
        linear_vel = link_1.get_linear_velocity(data)
        angular_vel = link_1.get_angular_velocity(data)
        print(f"link_1 pose : {pose}")
        print(f"link_1 linear_vel : {linear_vel}, angular_vel : {angular_vel}")
        print(f"link_1 rotation (xyzw) : {rotation}, mass : {link_1.mass}, center_of_mass : {link_1.center_of_mass}")
        # end::link_query[]

        # Start swinging: tilt the first joints, gravity does the rest.
        model.get_joint("joint_1").set_dof_pos(data, np.array([0.9]))
        model.get_joint("joint_2").set_dof_pos(data, np.array([0.45]))

        # tag::link_force[]
        def kick_link_3():
            # Apply an external force at the center of mass of link_3 (world frame)
            chain[-1].add_external_force(data, np.array([60.0, 0.0, 0.0]), local=False)

        # end::link_force[]

        frame = 0

        def phys_step():
            # Periodic external-force kick: a dynamic demonstration of
            # link.add_external_force without any UI interaction.
            t = frame * model.options.timestep
            if t % KICK_INTERVAL < model.options.timestep:
                kick_link_3()
            step(model, data)

        def render_step():
            nonlocal frame
            gizmos = render.gizmos
            for link in chain:
                pose = np.asarray(link.get_pose(data))
                # Link frame axes at the body origin
                gizmos.draw_axes(pos=pose[:3], rot=pose[3:], length=AXES_LEN)
                # Center of mass, transformed from the link local frame
                rot_mat = np.asarray(link.get_rotation_mat(data))
                com_world = pose[:3] + rot_mat @ np.asarray(link.center_of_mass)
                gizmos.draw_sphere(COM_RADIUS, com_world, color=Color.rgb(1, 1, 1))
                # Linear velocity arrow from the link origin
                v = np.asarray(link.get_linear_velocity(data))
                if np.linalg.norm(v) > 1e-4:
                    gizmos.draw_arrow(
                        start=pose[:3], end=pose[:3] + v * VEL_SCALE, color=Color.rgb(1, 0.9, 0.2)
                    )
            # Throttled console summary (every PRINT_INTERVAL frames) so the
            # teaching points stay visible without per-frame spam.
            if frame % PRINT_INTERVAL == 0 and frame > 0:
                speeds = [np.linalg.norm(np.asarray(l.get_linear_velocity(data))) for l in chain]
                print(f"t = {frame * model.options.timestep:5.1f}s  |v| = " + "  ".join(
                    f"{l.name}:{s:5.2f}" for l, s in zip(chain, speeds)
                ) + " m/s")
            frame += 1
            # Sync render objects from physic world
            render.sync(data)

        try:
            run.render_loop(model.options.timestep, 60, phys_step, render_step)
        except RenderClosedError:
            pass


if __name__ == "__main__":
    main()
