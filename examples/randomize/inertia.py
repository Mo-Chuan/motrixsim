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

"""Rotating arm inertia tensor randomization experiment.

This example assigns a different inertia tensor to each simulated arm while
keeping its mass and center of mass unchanged. The arm is driven by the same
hinge spring in every instance, so its oscillation speed directly reveals the
inertia about the hinge axis. Because the joint is damped, its oscillation
frequency is:

    omega_d^2 = stiffness / inertia_axis - damping^2 / (4 * inertia_axis^2)

Lower inertia produces faster oscillation; higher inertia produces slower
oscillation. The tensors are expressed in the link local frame and about the
current center of mass.

Mouse controls:
- Press and hold left button then drag to rotate the camera/view
- Press and hold right button then drag to pan/translate the view
"""

import numpy as np

from motrixsim import SceneData, load_model
from motrixsim.render import RenderApp

_NUM_INSTANCES = 16
_STIFFNESS = 2.5
_DAMPING = 0.1


def _inertia_tensors(axis_inertia: np.ndarray) -> np.ndarray:
    """Create valid fullinertia vectors with the requested Z-axis inertia."""
    tensors = np.zeros((_NUM_INSTANCES, 6), dtype=np.float32)
    tensors[:, 0] = axis_inertia  # Ixx
    tensors[:, 1] = axis_inertia  # Iyy
    tensors[:, 2] = axis_inertia  # Izz
    return tensors


def main():
    with RenderApp() as render:
        model = load_model("examples/assets/randomize/spring_rotor.xml")

        render_offset = []
        for i in range(4):
            for j in range(4):
                render_offset.append([-i * 2, j * 2, 0])

        render.launch(model, batch=_NUM_INSTANCES, render_offset=render_offset)
        render.system_camera.set_view(lookat=[-3, 3, 0.3], distance=8.0, elevation=-35, azimuth=90)

        data = SceneData(model, batch=(_NUM_INSTANCES,))
        rotor = model.get_link("rotor")

        # Choose evenly spaced target damped periods so the visual speed
        # difference is approximately uniform across the 4x4 grid. Invert
        # omega_d^2 = k/I - c^2/(4*I^2) to obtain the hinge-axis inertia.
        periods = np.linspace(2.5, 8.0, _NUM_INSTANCES, dtype=np.float32)
        angular_frequency = 2 * np.pi / periods
        axis_inertia = (_STIFFNESS + np.sqrt(_STIFFNESS**2 - _DAMPING**2 * angular_frequency**2)) / (
            2 * angular_frequency**2
        )
        inertia = _inertia_tensors(axis_inertia)

        original_mass = rotor.get_mass_override(data).copy()
        original_com = rotor.get_center_of_mass_override(data).copy()
        rotor.set_inertia_override(data, inertia)

        assert np.allclose(rotor.get_inertia_override(data), inertia)
        assert np.allclose(rotor.get_mass_override(data), original_mass)
        assert np.allclose(rotor.get_center_of_mass_override(data), original_com)

        print("\nInertia Tensor Randomization Summary")
        print("=" * 62)
        for index, (moment, period) in enumerate(zip(axis_inertia, periods)):
            print(f"Instance {index:2d}: Izz = {moment:.3f} kg*m^2  target period = {period:.2f} s")
        print("=" * 62 + "\n")

        while True:
            model.step(data)
            render.sync(data)


if __name__ == "__main__":
    main()
