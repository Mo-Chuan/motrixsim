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

"""Run with pytest or directly with Python; no viewer is required."""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from motrixsim import load_model

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))

from control.robot_locomotion import build_model, resolve_locomotion_config  # noqa: E402
from utils.controller import KeyboardCommandAdapter, OnnxController  # noqa: E402
from utils.locomotion import LocomotionSimulation  # noqa: E402
from utils.policy import MicroduckLocomotionPolicy  # noqa: E402
from utils.robot import MicroduckRobot  # noqa: E402


def advance_control(simulation, command):
    for _ in range(simulation.n_ctrl):
        simulation.step(command)


def test_microduck_control_contract():
    np.random.seed(1)
    _, _, scene_file, config = resolve_locomotion_config("microduck")
    model, camera, _ = build_model(scene_file, config["robot"], config["camera_position"], 45)
    assert camera.position_track == "fixed_local"
    assert camera.rotation_track == "look_at_link"
    robot = MicroduckRobot(model.get_body("trunk_base"))
    controller = OnnxController(str(robot.asset_dir / "policy.onnx"))
    policy = MicroduckLocomotionPolicy(robot, controller)
    simulation = LocomotionSimulation(model, policy, policy.ctrl_dt)
    np.testing.assert_allclose(model.options.timestep, 0.005)
    assert simulation.n_ctrl == 4
    np.testing.assert_allclose(model.options.timestep * simulation.n_ctrl, policy.ctrl_dt)
    # MJCF conversion takes a square root; this value reproduces the source solver contract.
    np.testing.assert_allclose(model.options.solver_tolerance, 1e-4)

    # Exercise name-based mapping with a physically nontrivial state and distinct joint values.
    qpos = policy.initial_qpos.copy()
    qpos[3:7] = [np.sin(0.3), 0, 0, np.cos(0.3)]
    offsets = np.linspace(-0.02, 0.02, len(MicroduckRobot.DEFAULT_POSE), dtype=np.float32)
    velocities = np.linspace(-0.5, 0.5, len(MicroduckRobot.DEFAULT_POSE), dtype=np.float32)
    qpos[robot.qpos_indices] += offsets
    qvel = policy.initial_qvel.copy()
    qvel[robot.qvel_indices] = velocities
    simulation.data.reset(model, qpos, qvel, forward_kinematic=True)
    model.get_body("trunk_base").floatingbase.set_global_angular_velocity(simulation.data, [1.0, 2.0, 3.0])
    model.forward_kinematic(simulation.data)
    policy.phase[:] = [-np.pi / 2, np.pi / 2]
    policy.last_action[:] = np.linspace(-1, 1, len(MicroduckRobot.DEFAULT_POSE))
    command = MicroduckLocomotionPolicy.command_limit.copy()
    with patch("numpy.random.uniform", side_effect=lambda lo, hi, shape: np.zeros(shape, dtype=np.float32)):
        obs = policy.get_observation(simulation.data, command)
    rotation = robot.base_link.get_rotation_mat(simulation.data)
    # These slices/scales are the shipped ONNX interface, not tunable reward constants.
    assert obs.shape == (55,) and obs.dtype == np.float32
    np.testing.assert_allclose(obs[:3], rotation.T @ [1, 2, 3] * 0.25, atol=1e-6)
    np.testing.assert_allclose(obs[3:6], rotation.T @ [0, 0, -1], atol=1e-6)
    np.testing.assert_array_equal(obs[6:9], command)
    np.testing.assert_allclose(obs[9:23], offsets, atol=1e-7)
    np.testing.assert_allclose(obs[23:37], velocities * 0.05)
    np.testing.assert_array_equal(obs[37:51], policy.last_action)
    np.testing.assert_allclose(obs[51:], [-1, 1, 0, 0], atol=1e-7)

    action = controller.get_actions(obs)
    assert action.shape == (14,) and np.isfinite(action).all()
    policy.apply_action(simulation.data, np.full(14, 1000, dtype=np.float32))
    np.testing.assert_array_equal(policy.last_action, policy.action_bound)
    np.testing.assert_allclose(
        simulation.data.actuator_ctrls[robot.ctrl_indices],
        policy.default_angles + policy.action_scale * policy.action_bound,
    )

    raw_command = np.array([1.0, 0.5, 2.0], dtype=np.float32)
    simulation.reset()
    advance_control(simulation, raw_command)
    assert policy.episode_steps == 1
    expected_phase = np.fmod(policy.phase_offset + np.pi, 2 * np.pi) - np.pi
    np.testing.assert_allclose(policy.phase, expected_phase, atol=1e-6)
    advance_control(simulation, raw_command)
    np.testing.assert_allclose(
        policy.phase, np.fmod(policy.phase_dt + policy.phase_offset + np.pi, 2 * np.pi) - np.pi, atol=1e-6
    )
    advance_control(simulation, np.zeros(3, dtype=np.float32))
    np.testing.assert_allclose(policy.phase, np.pi)
    assert np.isfinite(simulation.data.dof_pos).all()

    # Crossing the former 20-second limit must continue the same episode.
    policy.episode_steps = 999
    advance_control(simulation, raw_command)
    assert policy.episode_steps == 1000
    advance_control(simulation, raw_command)
    assert policy.episode_steps == 1001

    # Force the trunk into the floor to verify contact-triggered reset without a long rollout.
    qpos = policy.initial_qpos.copy()
    qpos[2] = 0.005
    simulation.data.reset(model, qpos, policy.initial_qvel, forward_kinematic=True)
    advance_control(simulation, raw_command)
    assert policy.episode_steps == 0
    np.testing.assert_array_equal(policy.last_action, 0)
    np.testing.assert_allclose(simulation.data.dof_pos, policy.initial_qpos)
    np.testing.assert_array_equal(simulation.data.dof_vel, 0)

    # The next episode must infer before physics and use a query bound to its new data.
    reset_data = simulation.data
    with patch.object(controller, "get_actions", wraps=controller.get_actions) as infer:
        simulation.step(raw_command)
        assert infer.call_count == 1
    for _ in range(simulation.n_ctrl - 1):
        simulation.step(raw_command)
    assert policy.episode_steps == 1
    qpos = policy.initial_qpos.copy()
    qpos[2] = 0.005
    simulation.data.reset(model, qpos, policy.initial_qvel, forward_kinematic=True)
    advance_control(simulation, raw_command)
    assert simulation.data is not reset_data
    assert policy.episode_steps == 0


def test_command_is_frozen_until_control_interval_finishes():
    model = load_model(str(MicroduckRobot.asset_dir / "scene.xml"))
    robot = MicroduckRobot(model.get_body("trunk_base"))
    actions = []

    def infer(obs):
        actions.append(obs.copy())
        return np.zeros(14, dtype=np.float32)

    policy = MicroduckLocomotionPolicy(robot, SimpleNamespace(get_actions=infer))
    simulation = LocomotionSimulation(model, policy)
    command = np.array([1.0, 0.5, 2.0], dtype=np.float32)
    simulation.step(command)
    assert len(actions) == 1  # Inference occurs before the first physics tick.
    np.testing.assert_array_equal(actions[0][6:9], policy.command_limit)
    command.fill(0)
    for _ in range(simulation.n_ctrl - 1):
        simulation.step(command)
    expected_phase = np.fmod(policy.phase_offset + np.pi, 2 * np.pi) - np.pi
    np.testing.assert_allclose(policy.phase, expected_phase, atol=1e-6)
    assert len(actions) == 1
    advance_control(simulation, command)
    assert len(actions) == 2
    np.testing.assert_array_equal(actions[1][6:9], 0)
    np.testing.assert_allclose(policy.phase, np.pi)


def test_shared_keyboard_commands_reach_microduck_actor():
    model = load_model(str(MicroduckRobot.asset_dir / "scene.xml"))
    robot = MicroduckRobot(model.get_body("trunk_base"))
    observations = []

    def infer(obs):
        observations.append(obs.copy())
        return np.zeros(14, dtype=np.float32)

    policy = MicroduckLocomotionPolicy(robot, SimpleNamespace(get_actions=infer))
    simulation = LocomotionSimulation(model, policy)
    keyboard = KeyboardCommandAdapter()
    cases = [
        ({"w"}, [0.4, 0, 0]),
        ({"up"}, [0.4, 0, 0]),
        ({"s"}, [-0.4, 0, 0]),
        ({"down"}, [-0.4, 0, 0]),
        ({"left"}, [0, 0.3, 0]),
        ({"right"}, [0, -0.3, 0]),
        ({"a"}, [0, 0, 1]),
        ({"d"}, [0, 0, -1]),
        ({"w", "up"}, [0.4, 0, 0]),
        ({"s", "right", "d"}, [-0.4, -0.3, -1]),
        ({"w", "s", "up", "down", "left", "right", "a", "d"}, [0.4, 0.3, 1]),
        ({"up", "s"}, [0.4, 0, 0]),
        ({"w", "down"}, [0.4, 0, 0]),
        (set(), [0, 0, 0]),
        ({"q", "e"}, [0, 0, 0]),
    ]
    for held, expected in cases:
        keyboard.update_from_input(SimpleNamespace(is_key_pressed=held.__contains__))
        policy.pre_control_step(simulation.data, keyboard.command)
        # Assert the actor's physical velocity inputs, including no double scaling.
        np.testing.assert_array_equal(observations[-1][6:9], np.array(expected, dtype=np.float32))


if __name__ == "__main__":
    test_microduck_control_contract()
    test_command_is_frozen_until_control_interval_finishes()
    test_shared_keyboard_commands_reach_microduck_actor()
    print("Microduck control contract passed")
