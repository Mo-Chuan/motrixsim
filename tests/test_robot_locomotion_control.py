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

"""Regress the shared runner against the established locomotion physics cadence."""

import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest

from motrixsim import SceneData, msd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))

from control.robot_locomotion import ROBOT_CONFIGS  # noqa: E402
from utils.locomotion import LocomotionSimulation  # noqa: E402


@pytest.mark.parametrize("robot_name", ["go1", "go2", "g1", "g1_12dof"])
def test_existing_policy_cadence_and_reset(robot_name):
    config = ROBOT_CONFIGS[robot_name]
    model = msd.from_file(config["robot"].mjcf_path).build()
    if "timestep" in config:
        model.options.timestep = config["timestep"]
    body = model.get_body(config["robot"].base_link_name)
    reference = config["policy"](config["robot"](body))
    policy = config["policy"](config["robot"](body))
    simulation = LocomotionSimulation(model, policy)
    command = np.array([0.4, 0.1, 0.2], dtype=np.float32)

    for episode in range(2):
        data = SceneData(model)
        if episode:
            simulation.reset()
            if hasattr(reference, "reset"):
                reference.reset()
        # Run the pre-refactor schedule next to the shared runner with real actors and physics.
        with patch.object(policy, "step", wraps=policy.step) as policy_step:
            for step in range(1, 3 * simulation.n_ctrl + 1):
                if hasattr(reference, "pre_physics_step"):
                    reference.pre_physics_step(data)
                model.step(data)
                if step % simulation.n_ctrl == 0:
                    assert not reference.step(data, command)
                simulation.step(command)
                np.testing.assert_array_equal(simulation.data.dof_pos, data.dof_pos)
                np.testing.assert_array_equal(simulation.data.dof_vel, data.dof_vel)
                np.testing.assert_array_equal(simulation.data.actuator_ctrls, data.actuator_ctrls)
            assert policy_step.call_count == 3
