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

"""Motion planning example: 7-DOF Kinova Gen3 arm plans collision-free paths.

Demonstrates:
1. Build a scene by attaching the robot body and a selectable obstacle with
   msd.attach().
2. Use MotionPlanner.plan() with CartesianGoal: IK solves goal joint
   candidates from a target EE pose, then RRT-Connect finds a collision-free
   joint-space path.
3. Press Space to sample a random EE goal, plan from the current state, and
   replay the resulting trajectory.

Usage:
    cd motrixsim-python/motrixsim-docs
    python examples/pro/motion_planning.py
"""

import argparse
from dataclasses import dataclass, field
from time import perf_counter
from typing import NamedTuple

import numpy as np

import motrixsim as mx
from motrixsim import SceneData, SceneModel, step
from motrixsim.ik import DlsSolver, IkChain
from motrixsim.planner import (
    CartesianGoal,
    FreeOrientationGoal,
    MotionPlanner,
    MotionPlanRequest,
    MotionPlanResult,
    PointPositionGoal,
    QuaternionOrientationGoal,
)
from motrixsim.render import Color, RenderApp

ROBOT_PATH = "examples/assets/stanford_tidybot/tidybot_position.xml"

BASE_SCENE_MJCF = """\
<mujoco model="motion planning base scene">
  <statistic center="0 0 0.525" extent="1.1" meansize="0.05"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.1 0.1 0.1" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="140" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.4314 0.5294 0.6431" rgb2="0 0 0" width="512" height="512"/>
  </asset>
  <worldbody>
    <light pos="0 0 1.5" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" rgba="0.8 0.8 0.8 1"/>
  </worldbody>
</mujoco>"""

WALL_MJCF = """\
<mujoco model="wall">
  <worldbody>
    <body name="wall" pos="0 0 0.4">
      <geom type="box" size="0.02 0.15 0.4" rgba="1 0.3 0.3 0.7"/>
    </body>
  </worldbody>
</mujoco>"""

BOX_MJCF = """\
<mujoco model="box obstacle">
  <worldbody>
    <body name="box" pos="0 0 0.18">
      <geom type="box" size="0.08 0.08 0.18" rgba="0.95 0.55 0.15 0.75"/>
    </body>
  </worldbody>
</mujoco>"""

POSTS_MJCF = """\
<mujoco model="post obstacles">
  <worldbody>
    <body name="left_post" pos="0 -0.18 0.5">
      <geom type="cylinder" size="0.035 0.5" rgba="0.15 0.7 1.0 0.75"/>
    </body>
    <body name="right_post" pos="0 0.18 0.5">
      <geom type="cylinder" size="0.035 0.5" rgba="0.15 0.7 1.0 0.75"/>
    </body>
  </worldbody>
</mujoco>"""

SHELF_MJCF = """\
<mujoco model="shelf obstacle">
  <worldbody>
    <body name="shelf">
      <geom name="shelf_top" type="box" pos="0 0 0.72" size="0.2 0.28 0.025" rgba="0.45 0.35 0.25 0.78"/>
      <geom name="shelf_mid" type="box" pos="0 0 0.46" size="0.2 0.28 0.02" rgba="0.45 0.35 0.25 0.68"/>
      <geom name="shelf_bottom" type="box" pos="0 0 0.20" size="0.2 0.28 0.025" rgba="0.45 0.35 0.25 0.78"/>
      <geom name="shelf_left_post" type="box" pos="0 -0.255 0.46" size="0.035 0.025 0.29" rgba="0.38 0.28 0.18 0.78"/>
      <geom name="shelf_right_post" type="box" pos="0 0.255 0.46" size="0.035 0.025 0.29" rgba="0.38 0.28 0.18 0.78"/>
      <geom name="shelf_back_panel" type="box" pos="0.18 0 0.46" size="0.025 0.28 0.29" rgba="0.32 0.25 0.2 1.0"/>
      <geom name="shelf_center_divider" type="box" pos="0 0 0.33" size="0.035 0.02 0.11" rgba="0.38 0.28 0.18 0.72"/>
    </body>
  </worldbody>
</mujoco>"""

SIM_DT = 0.002
CTRL_DT = 0.02
IK_MAX_ITER = 100
IK_TOLERANCE = 1e-3
GOAL_REACHED_POSITION_TOLERANCE = 0.04
IK_DAMPING = 1e-3
ORIENTATION_TOLERANCE = 0.5
REPLAY_SETTLE_TIMEOUT = 1.0
REPLAY_SETTLE_JOINT_TOLERANCE = 3e-3
NUM_IK_SEEDS = 64
MAX_GOAL_ATTEMPTS = 12
PLANNER_MAX_ITERATIONS = 3000
PLANNER_STEP_SIZE = 0.1
PLANNER_GOAL_BIAS = 0.05
PLANNER_MAX_VELOCITY = 0.25
PLANNER_MAX_ACCELERATION = 0.5
NUM_GOAL_PRESETS = 16
MAX_GOAL_PRESET_GENERATION_ATTEMPTS = 8000
GOAL_PRESET_MIN_DISTANCE = 0.10
GOAL_PRESET_X_RANGE = (0.22, 0.68)
GOAL_PRESET_Y_ABS_MAX = 0.38
GOAL_PRESET_Z_RANGE = (0.28, 0.90)
NUM_INTERIOR_GOAL_PRESETS = 4
MAX_INTERIOR_GOAL_ATTEMPTS = 20000
DEFAULT_SEED = 42
DEFAULT_OBSTACLE = "wall"


class InteriorRegion(NamedTuple):
    min_bound: tuple[float, float, float]
    max_bound: tuple[float, float, float]


class ObstacleSpec(NamedTuple):
    mjcf: str
    translation: tuple[float, float, float]
    interior_regions: tuple[InteriorRegion, ...] = ()


OBSTACLES = {
    "wall": ObstacleSpec(WALL_MJCF, (0.3, 0.0, 0.0)),
    "box": ObstacleSpec(BOX_MJCF, (0.36, 0.0, 0.0)),
    "posts": ObstacleSpec(POSTS_MJCF, (0.36, 0.0, 0.0)),
    "shelf": ObstacleSpec(
        SHELF_MJCF,
        (0.52, 0.0, 0.0),
        interior_regions=(
            InteriorRegion((0.36, -0.18, 0.51), (0.63, 0.18, 0.66)),
            InteriorRegion((0.36, -0.18, 0.27), (0.63, -0.06, 0.40)),
            InteriorRegion((0.36, 0.06, 0.27), (0.63, 0.18, 0.40)),
        ),
    ),
}


@dataclass(frozen=True)
class CartesianGoalPreset:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float] | None = None
    orientation_tolerance: float = ORIENTATION_TOLERANCE

    @property
    def target_pose(self) -> np.ndarray:
        orientation = self.orientation if self.orientation is not None else (0.0, 0.0, 0.0, 1.0)
        return np.array([*self.position, *orientation], dtype=np.float32)

    @property
    def is_orientation_constrained(self) -> bool:
        return self.orientation is not None

    @property
    def goal_type_name(self) -> str:
        if self.is_orientation_constrained:
            return "CartesianGoal(PointPositionGoal, QuaternionOrientationGoal)"
        return "CartesianGoal(PointPositionGoal, FreeOrientationGoal)"

    @property
    def orientation_name(self) -> str:
        if self.is_orientation_constrained:
            return "QuaternionOrientationGoal"
        return "FreeOrientationGoal"


def free_goal(position: tuple[float, float, float]) -> CartesianGoalPreset:
    return CartesianGoalPreset(position)


def quat_goal(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
) -> CartesianGoalPreset:
    return CartesianGoalPreset(position, orientation=orientation)


@dataclass(frozen=True)
class ArmMapping:
    ctrl_start: int = 3
    ctrl_end: int = 10
    dof_start: int = 3
    dof_end: int = 10

    @property
    def ctrl_slice(self) -> slice:
        return slice(self.ctrl_start, self.ctrl_end)

    @property
    def dof_slice(self) -> slice:
        return slice(self.dof_start, self.dof_end)


class GoalSampler:
    def __init__(self, goal_presets: tuple[CartesianGoalPreset, ...], seed: int) -> None:
        self.goal_presets = goal_presets
        self.rng = np.random.default_rng(seed)

    def sample(self, current_ee_pose: np.ndarray | None = None) -> tuple[int, CartesianGoalPreset]:
        candidate_indices = [
            index
            for index, goal_preset in enumerate(self.goal_presets)
            if not is_goal_reached(goal_preset, current_ee_pose)
        ]
        if not candidate_indices:
            candidate_indices = list(range(len(self.goal_presets)))
        goal_index = int(candidate_indices[int(self.rng.integers(len(candidate_indices)))])
        return goal_index, self.goal_presets[goal_index]


class CartesianGoalRequestFactory:
    def make(self, preset: CartesianGoalPreset) -> MotionPlanRequest:
        if preset.is_orientation_constrained:
            orientation_goal = QuaternionOrientationGoal(preset.orientation, preset.orientation_tolerance)
        else:
            orientation_goal = FreeOrientationGoal()
        return MotionPlanRequest(
            CartesianGoal(
                PointPositionGoal(preset.position, IK_TOLERANCE),
                orientation_goal,
            ),
            num_ik_seeds=NUM_IK_SEEDS,
            ik_max_iter=IK_MAX_ITER,
            ik_damping=IK_DAMPING,
        )


def finite_arm_joint_limits(model: SceneModel, mapping: ArmMapping) -> tuple[np.ndarray, np.ndarray]:
    # SceneModel.joint_limits is indexed by joint, while mapping.dof_slice indexes DOFs.
    # This example assumes every joint on the planning chain has exactly one DOF.
    limits = np.asarray(model.joint_limits, dtype=np.float32)[:, mapping.dof_slice]
    low = np.where(np.isfinite(limits[0]), limits[0], -np.pi)
    high = np.where(np.isfinite(limits[1]), limits[1], np.pi)
    return low, high


def set_arm_dof_pos(model: SceneModel, data: SceneData, mapping: ArmMapping, arm_q: np.ndarray) -> None:
    dof_pos = data.dof_pos.copy()
    dof_pos[mapping.dof_slice] = arm_q
    data.set_dof_pos(dof_pos, model)
    data.set_dof_vel(np.zeros_like(data.dof_vel))
    mx.forward_kinematic(model, data)


def is_pose_in_goal_workspace(position: np.ndarray) -> bool:
    return (
        GOAL_PRESET_X_RANGE[0] <= position[0] <= GOAL_PRESET_X_RANGE[1]
        and abs(position[1]) <= GOAL_PRESET_Y_ABS_MAX
        and GOAL_PRESET_Z_RANGE[0] <= position[2] <= GOAL_PRESET_Z_RANGE[1]
    )


def solve_goal_ik(
    solver: DlsSolver, chain: IkChain, data: SceneData, target_pose: np.ndarray
) -> tuple[float, np.ndarray]:
    result = solver.solve(chain, data, target_pose)
    return float(result[1]), result[2:]


def generate_goal_presets(
    model: SceneModel,
    data: SceneData,
    chain: IkChain,
    planner: MotionPlanner,
    mapping: ArmMapping,
    seed: int,
    interior_regions: tuple[InteriorRegion, ...] = (),
) -> tuple[CartesianGoalPreset, ...]:
    rng = np.random.default_rng(seed)
    low, high = finite_arm_joint_limits(model, mapping)
    reference_dof_pos = data.dof_pos.copy()
    reference_dof_vel = data.dof_vel.copy()
    reference_ctrls = data.actuator_ctrls.copy()
    ik_solver = DlsSolver(
        max_iter=IK_MAX_ITER,
        step_size=0.5,
        tolerance=IK_TOLERANCE,
        damping=IK_DAMPING,
        adaptive_damping=True,
    )
    goals: list[CartesianGoalPreset] = []
    goal_positions: list[np.ndarray] = []

    for _ in range(MAX_GOAL_PRESET_GENERATION_ATTEMPTS):
        if len(goals) >= NUM_GOAL_PRESETS:
            break

        data.set_dof_pos(reference_dof_pos, model)
        data.set_dof_vel(reference_dof_vel)
        data.actuator_ctrls = reference_ctrls
        mx.forward_kinematic(model, data)

        sampled_q = rng.uniform(low, high).astype(np.float32)
        if not planner.is_collision_free(data, sampled_q):
            continue

        set_arm_dof_pos(model, data, mapping, sampled_q)
        target_pose = chain.get_end_effector_pose(data)
        position = target_pose[:3]
        if not is_pose_in_goal_workspace(position):
            continue
        if any(np.linalg.norm(position - old) < GOAL_PRESET_MIN_DISTANCE for old in goal_positions):
            continue

        ik_residual, ik_q = solve_goal_ik(ik_solver, chain, data, target_pose)
        if ik_residual > IK_TOLERANCE or not planner.is_collision_free(data, ik_q):
            continue

        preset = (
            quat_goal(tuple(float(value) for value in position), tuple(float(value) for value in target_pose[3:7]))
            if len(goals) % 2 == 1
            else free_goal(tuple(float(value) for value in position))
        )

        goals.append(preset)
        goal_positions.append(position.copy())

    # Phase 2: joint-space sampling targeting interior regions (e.g. shelf compartments).
    # The general workspace filter in Phase 1 rarely produces EE poses inside tight
    # enclosures. Here we sample random joint configs and check whether the resulting
    # EE position falls inside any interior region, bypassing the general workspace filter.
    interior_count = 0
    if interior_regions:
        num_interior_target = min(
            NUM_INTERIOR_GOAL_PRESETS, max(0, NUM_GOAL_PRESETS - len(goals)) + NUM_INTERIOR_GOAL_PRESETS
        )
        interior_attempts = 0
        for _ in range(MAX_INTERIOR_GOAL_ATTEMPTS):
            if interior_count >= num_interior_target:
                break

            data.set_dof_pos(reference_dof_pos, model)
            data.set_dof_vel(reference_dof_vel)
            data.actuator_ctrls = reference_ctrls
            mx.forward_kinematic(model, data)

            interior_attempts += 1
            sampled_q = rng.uniform(low, high).astype(np.float32)
            if not planner.is_collision_free(data, sampled_q):
                continue

            set_arm_dof_pos(model, data, mapping, sampled_q)
            ee_pose = chain.get_end_effector_pose(data)
            position = ee_pose[:3]

            # Check if EE is inside any interior region.
            if not any(
                all(r.min_bound[i] <= float(position[i]) <= r.max_bound[i] for i in range(3)) for r in interior_regions
            ):
                continue

            if any(np.linalg.norm(position - old) < GOAL_PRESET_MIN_DISTANCE for old in goal_positions):
                continue

            ik_residual, ik_q = solve_goal_ik(ik_solver, chain, data, ee_pose)
            if ik_residual > IK_TOLERANCE or not planner.is_collision_free(data, ik_q):
                continue

            preset = free_goal(tuple(float(v) for v in position))
            goals.append(preset)
            goal_positions.append(position.copy())
            interior_count += 1

    data.set_dof_pos(reference_dof_pos, model)
    data.set_dof_vel(reference_dof_vel)
    data.actuator_ctrls = reference_ctrls
    mx.forward_kinematic(model, data)

    if interior_regions:
        print(f"Interior sampling: {interior_count}/{num_interior_target} generated in {interior_attempts} attempts.")
    if len(goals) < NUM_GOAL_PRESETS:
        raise RuntimeError(
            f"failed to generate {NUM_GOAL_PRESETS} valid goal presets; "
            f"got {len(goals)} after {MAX_GOAL_PRESET_GENERATION_ATTEMPTS} attempts"
        )
    return tuple(goals)


class MotionPlanningSceneBuilder:
    def __init__(self, obstacle_name: str) -> None:
        self.obstacle_name = obstacle_name

    def build(self) -> SceneModel:
        scene = mx.msd.from_str(BASE_SCENE_MJCF)
        robot = mx.msd.from_file(ROBOT_PATH)
        scene.attach(robot)

        if self.obstacle_name == "none":
            return scene.build()

        obstacle = OBSTACLES[self.obstacle_name]
        scene.attach(
            mx.msd.from_str(obstacle.mjcf),
            other_translation=obstacle.translation,
            other_prefix=f"{self.obstacle_name}_",
        )
        return scene.build()

    def obstacle_translation(self) -> tuple[float, float, float] | None:
        if self.obstacle_name == "none":
            return None
        return OBSTACLES[self.obstacle_name].translation


class ArmController:
    def __init__(self, model: SceneModel, data: SceneData, mapping: ArmMapping) -> None:
        self.model = model
        self.data = data
        self.mapping = mapping

    def set_state(self, arm_q: np.ndarray) -> None:
        dof_pos = self.data.dof_pos.copy()
        dof_pos[self.mapping.dof_slice] = arm_q
        self.data.set_dof_pos(dof_pos, self.model)
        self.data.set_dof_vel(np.zeros_like(self.data.dof_vel))

        self.set_target(arm_q)
        mx.forward_kinematic(self.model, self.data)

    def set_target(self, q_des: np.ndarray) -> None:
        ctrls = self.data.actuator_ctrls
        ctrls[self.mapping.ctrl_slice] = q_des
        self.data.actuator_ctrls = ctrls


@dataclass
class ReplayState:
    last_safe_arm_q: np.ndarray
    target_pose: np.ndarray | None = None
    target_orientation_constrained: bool = False
    plan_query_index: int = 0
    waypoints: list[np.ndarray] = field(default_factory=list)
    active_trajectory: object | None = None
    active_goal_q: np.ndarray | None = None
    settling_goal_q: np.ndarray | None = None
    settle_time: float = 0.0
    replay_time: float = 0.0
    trajectory_duration: float = 0.0
    planned_ee_path: list[np.ndarray] = field(default_factory=list)
    trajectory_ee_waypoints: list[np.ndarray] = field(default_factory=list)
    ee_trail: list[np.ndarray] = field(default_factory=list)
    max_tracking_error: float = 0.0

    def reset_for_new_plan(self) -> None:
        self.ee_trail.clear()
        self.planned_ee_path.clear()
        self.trajectory_ee_waypoints.clear()
        self.replay_time = 0.0
        self.settle_time = 0.0
        self.trajectory_duration = 0.0
        self.max_tracking_error = 0.0
        self.target_orientation_constrained = False
        self.waypoints = []
        self.active_trajectory = None
        self.active_goal_q = None
        self.settling_goal_q = None


@dataclass
class PlanningColors:
    trail: Color = field(default_factory=lambda: Color.rgb(1.0, 0.6, 0.0))
    planned_path: Color = field(default_factory=lambda: Color.rgb(0.0, 0.9, 1.0))
    trajectory_waypoint: Color = field(default_factory=lambda: Color.rgb(1.0, 0.0, 0.8))
    tracked_path: Color = field(default_factory=lambda: Color.rgb(1.0, 0.95, 0.0))
    current_target: Color = field(default_factory=lambda: Color.rgb(0.4, 0.0, 1.0))
    selected_goal: Color = field(default_factory=lambda: Color.rgb(1.0, 0.45, 0.0))
    goal_preset: Color = field(default_factory=lambda: Color.rgb(0.2, 1.0, 0.35))


def format_repro_array(values: object) -> str:
    return np.array2string(
        np.asarray(values, dtype=np.float32),
        precision=8,
        separator=", ",
        suppress_small=False,
        max_line_width=1_000_000,
    )


def quaternion_angle_error(target: object, current: object) -> float:
    target = np.asarray(target, dtype=np.float64)
    current = np.asarray(current, dtype=np.float64)
    target_norm = np.linalg.norm(target)
    current_norm = np.linalg.norm(current)
    if target_norm <= 0.0 or current_norm <= 0.0:
        return float("nan")
    dot = abs(float(np.dot(target / target_norm, current / current_norm)))
    return float(2.0 * np.arccos(np.clip(dot, -1.0, 1.0)))


def is_goal_reached(goal_preset: CartesianGoalPreset, ee_pose: np.ndarray | None) -> bool:
    if ee_pose is None:
        return False

    position_error = float(np.linalg.norm(np.subtract(ee_pose[:3], goal_preset.position)))
    if position_error > GOAL_REACHED_POSITION_TOLERANCE:
        return False
    if not goal_preset.is_orientation_constrained:
        return True
    return quaternion_angle_error(goal_preset.orientation, ee_pose[3:7]) <= goal_preset.orientation_tolerance


def pose_error_summary(target_pose: object, current_pose: object, include_orientation: bool) -> str:
    target_pose = np.asarray(target_pose, dtype=np.float32)
    current_pose = np.asarray(current_pose, dtype=np.float32)
    position_error = float(np.linalg.norm(current_pose[:3] - target_pose[:3]))
    if not include_orientation:
        return f"position_error={position_error:.6g}"
    orientation_error = quaternion_angle_error(target_pose[3:7], current_pose[3:7])
    return f"position_error={position_error:.6g}, orientation_error={orientation_error:.6g} rad"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--seed",
        type=int,
        default=DEFAULT_SEED,
        help="Deterministic seed for target sampling, IK seeds, and RRT sampling.",
    )
    parser.add_argument(
        "--obstacle",
        choices=["none", *OBSTACLES],
        default=DEFAULT_OBSTACLE,
        help="Obstacle model to attach for planning tests.",
    )
    return parser.parse_args()


class MotionPlanningDemo:
    def __init__(self, args: argparse.Namespace, render: RenderApp) -> None:
        self.args = args
        self.render = render
        self.scene_builder = MotionPlanningSceneBuilder(args.obstacle)
        self.model = self.scene_builder.build()
        self.model.options.timestep = SIM_DT
        self.data = SceneData(self.model)
        self.mapping = ArmMapping()
        self.ctrl_steps = max(1, round(CTRL_DT / SIM_DT))

        self.chain = IkChain(
            self.model,
            end_link="base",
            start_link="gen3/base_link",
            end_effector_offset=[0.0, 0.0, 0.15, 0, 0, 0, 1],
        )
        self.planner = MotionPlanner(
            self.chain,
            max_velocity=PLANNER_MAX_VELOCITY,
            max_acceleration=PLANNER_MAX_ACCELERATION,
            max_iterations=PLANNER_MAX_ITERATIONS,
            step_size=PLANNER_STEP_SIZE,
            goal_bias=PLANNER_GOAL_BIAS,
            seed=args.seed,
            max_goal_attempts=MAX_GOAL_ATTEMPTS,
            return_end_effector_waypoints=True,
            max_end_effector_waypoints=512,
            return_trajectory_end_effector_waypoints=True,
        )

        mx.forward_kinematic(self.model, self.data)

        self.arm = ArmController(self.model, self.data, self.mapping)
        self.goal_request_factory = CartesianGoalRequestFactory()
        print("Generating validated goal presets...", flush=True)
        interior_regions = OBSTACLES[args.obstacle].interior_regions if args.obstacle in OBSTACLES else ()
        self.goal_presets = generate_goal_presets(
            self.model,
            self.data,
            self.chain,
            self.planner,
            self.mapping,
            args.seed,
            interior_regions=interior_regions,
        )
        self.sampler = GoalSampler(self.goal_presets, args.seed)
        self.state = ReplayState(last_safe_arm_q=self.chain.get_dof_pos(self.data).copy())
        self.colors = PlanningColors()
        self.gizmos = render.gizmos

    def run(self) -> None:
        self.print_startup_summary()
        self.render.launch(self.model)
        print("Press Space in the render window to sample a goal, plan, and replay.")

        while not self.render.is_closed:
            self.control_step()
            self.render_step()

    def print_startup_summary(self) -> None:
        print(f"Seed: {self.args.seed}")
        print(f"Robot: {ROBOT_PATH}")
        print(f"Obstacle: {self.args.obstacle}")
        translation = self.scene_builder.obstacle_translation()
        if translation is not None:
            print(f"Obstacle translation: {translation}")
        print(f"Goal presets: {self.goal_presets}")
        print(f"Start EE: {self.chain.get_end_effector_pose(self.data)[:3]}")

    def restore_last_safe_start(self) -> None:
        self.arm.set_state(self.state.last_safe_arm_q)
        self.state.ee_trail.clear()
        print("Current start is in collision; restored the last safe arm state.", flush=True)

    def end_effector_pose_for_arm_q(self, arm_q: np.ndarray) -> np.ndarray:
        original_dof_pos = self.data.dof_pos.copy()
        original_dof_vel = self.data.dof_vel.copy()
        original_ctrls = self.data.actuator_ctrls.copy()

        dof_pos = original_dof_pos.copy()
        dof_pos[self.mapping.dof_slice] = arm_q
        self.data.set_dof_pos(dof_pos, self.model)
        self.data.set_dof_vel(np.zeros_like(original_dof_vel))
        mx.forward_kinematic(self.model, self.data)
        ee_pose = self.chain.get_end_effector_pose(self.data).copy()

        self.data.set_dof_pos(original_dof_pos, self.model)
        self.data.set_dof_vel(original_dof_vel)
        self.data.actuator_ctrls = original_ctrls
        mx.forward_kinematic(self.model, self.data)
        return ee_pose

    def plan_random_goal(self) -> None:
        state = self.state
        state.reset_for_new_plan()

        current_start = self.chain.get_dof_pos(self.data)
        if not self.planner.is_collision_free(self.data, current_start):
            self.restore_last_safe_start()
            current_start = self.chain.get_dof_pos(self.data)
            if not self.planner.is_collision_free(self.data, current_start):
                print("Planning skipped: restored start is still in collision.", flush=True)
                return

        start_ee_pose = self.chain.get_end_effector_pose(self.data).copy()
        goal_index, goal_preset = self.sampler.sample(start_ee_pose)
        candidate_pose = goal_preset.target_pose
        plan_input_full_dof_pos = self.data.dof_pos.copy()
        plan_input_full_dof_vel = self.data.dof_vel.copy()
        state.target_pose = candidate_pose
        state.target_orientation_constrained = goal_preset.is_orientation_constrained
        state.plan_query_index += 1
        print(
            f"Planning query {state.plan_query_index}: {self.planner.num_dof}-DOF path to "
            f"{self.args.obstacle} goal preset {goal_index} "
            f"({goal_preset.goal_type_name}) at EE {candidate_pose[:3]}...",
            flush=True,
        )

        plan_started_at = perf_counter()
        plan_request = self.goal_request_factory.make(goal_preset)
        plan_result = self.planner.plan(self.data, plan_request)
        elapsed = perf_counter() - plan_started_at

        if not plan_result.success:
            self.print_failure_report(plan_result, elapsed, goal_index, goal_preset, candidate_pose, current_start)
            state.last_safe_arm_q = current_start.copy()
            self.arm.set_target(state.last_safe_arm_q)
            return

        self.accept_plan_result(
            plan_result,
            elapsed,
            goal_index,
            goal_preset,
            candidate_pose,
            current_start,
            plan_input_full_dof_pos,
            plan_input_full_dof_vel,
            start_ee_pose,
        )

    def print_failure_report(
        self,
        plan_result: MotionPlanResult,
        elapsed: float,
        goal_index: int,
        goal_preset: CartesianGoalPreset,
        candidate_pose: np.ndarray,
        current_start: np.ndarray,
    ) -> None:
        diagnostics = plan_result.diagnostics
        timings = plan_result.timings
        failed_start_q = current_start.copy()
        failed_full_dof_pos = self.data.dof_pos.copy()
        failed_full_dof_vel = self.data.dof_vel.copy()
        failed_start_ee_pose = self.chain.get_end_effector_pose(self.data)
        lines = [
            f"Planning failed in {elapsed:.3f}s:",
            f"  status: {plan_result.status}",
            f"  reason: {plan_result.failure_reason}",
            "  repro:",
            f"    seed: {self.args.seed}",
            f"    query_index: {self.state.plan_query_index}",
            "    scene: BASE_SCENE_MJCF",
            f"    robot: {ROBOT_PATH}",
            f"    obstacle: {self.args.obstacle}",
            f"    goal_preset_index: {goal_index}",
            f"    start_ee_pose: {format_repro_array(failed_start_ee_pose)}",
            f"    target_pose: {format_repro_array(candidate_pose)}",
            "    goal:",
            "      type: CartesianGoal",
            f"      position_tolerance: {IK_TOLERANCE}",
            f"      orientation: {goal_preset.orientation_name}",
            *(
                [
                    f"      orientation_target: {format_repro_array(goal_preset.orientation)}",
                    f"      orientation_tolerance: {goal_preset.orientation_tolerance}",
                ]
                if goal_preset.is_orientation_constrained
                else []
            ),
            f"    arm_dof_slice: [{self.mapping.dof_start}:{self.mapping.dof_end}]",
            f"    start_arm_q: {format_repro_array(failed_start_q)}",
            f"    full_dof_pos: {format_repro_array(failed_full_dof_pos)}",
            f"    full_dof_vel: {format_repro_array(failed_full_dof_vel)}",
            "    planner:",
            f"      max_velocity: {PLANNER_MAX_VELOCITY}",
            f"      max_acceleration: {PLANNER_MAX_ACCELERATION}",
            f"      max_iterations: {PLANNER_MAX_ITERATIONS}",
            f"      step_size: {PLANNER_STEP_SIZE}",
            f"      goal_bias: {PLANNER_GOAL_BIAS}",
            f"      max_goal_attempts: {MAX_GOAL_ATTEMPTS}",
            "    ik:",
            f"      num_ik_seeds: {NUM_IK_SEEDS}",
            f"      ik_max_iter: {IK_MAX_ITER}",
            f"      ik_tolerance: {IK_TOLERANCE}",
            f"      ik_damping: {IK_DAMPING}",
        ]
        if diagnostics is not None:
            lines.extend(
                [
                    "  diagnostics:",
                    f"    ik: {diagnostics.num_converged_candidates}/{diagnostics.num_ik_candidates} converged",
                    f"    collision-free goals: {diagnostics.num_collision_free_candidates}",
                    f"    planning attempts: {diagnostics.num_planning_attempts}/{diagnostics.max_goal_attempts}",
                    f"    best IK residual: {diagnostics.best_ik_residual}",
                    f"    best position error: {diagnostics.best_position_error}",
                    f"    best orientation error: {diagnostics.best_orientation_error}",
                    f"    position-only goal: {diagnostics.position_only}",
                ]
            )
            rejected = diagnostics.rejected_collision_candidates
            if rejected:
                lines.append(f"    rejected collision candidates: {len(rejected)} shown")
                for candidate in rejected:
                    pair = candidate.first_collision
                    pair_text = f"({pair.collider_a}, {pair.collider_b})" if pair is not None else "none"
                    lines.append(
                        "      "
                        f"seed={candidate.seed_index} "
                        f"residual={candidate.residual:.6g} "
                        f"dist={candidate.distance_from_start:.6g} "
                        f"first_collision={pair_text} "
                        f"contacts={candidate.num_contacts} "
                        f"pairs={candidate.num_pairs_tested} "
                        f"repaired={candidate.repaired}"
                    )
        lines.extend(
            [
                "  timings:",
                f"    total: {timings.total_sec:.3f}s",
                f"    ik: {timings.ik_sec:.3f}s",
                f"    filter: {timings.goal_collision_filter_sec:.3f}s",
                f"    plan: {timings.path_planning_sec:.3f}s",
            ]
        )
        print("\n".join(lines), flush=True)

    def accept_plan_result(
        self,
        plan_result: MotionPlanResult,
        elapsed: float,
        goal_index: int,
        goal_preset: CartesianGoalPreset,
        candidate_pose: np.ndarray,
        current_start: np.ndarray,
        plan_input_full_dof_pos: np.ndarray,
        plan_input_full_dof_vel: np.ndarray,
        start_ee_pose: np.ndarray,
    ) -> None:
        trajectory = plan_result.trajectory
        if trajectory is None:
            print("Planning returned no timed trajectory; replay skipped.", flush=True)
            return

        state = self.state
        state.waypoints = [np.array(wp, dtype=np.float32) for wp in plan_result.waypoints]
        state.trajectory_duration = float(trajectory.duration_sec)
        state.active_trajectory = trajectory
        state.active_goal_q = np.array(trajectory.position_at(state.trajectory_duration), dtype=np.float32)

        self.extend_flattened_points(state.planned_ee_path, plan_result.end_effector_waypoints)
        self.extend_flattened_points(
            state.trajectory_ee_waypoints,
            plan_result.trajectory_end_effector_waypoints,
        )
        state.last_safe_arm_q = np.array(trajectory.position_at(0.0), dtype=np.float32)

        timings = plan_result.timings
        lines = [
            f"Path found: {len(state.waypoints)} waypoints, "
            f"{len(state.trajectory_ee_waypoints)} trajectory EE waypoints, "
            f"{int(np.ceil(state.trajectory_duration / SIM_DT)) + 1} physics-step targets, "
            f"duration {trajectory.duration_sec:.3f}s in {elapsed:.3f}s "
            f"(ik {timings.ik_sec:.3f}s, filter {timings.goal_collision_filter_sec:.3f}s, "
            f"plan {timings.path_planning_sec:.3f}s, simplify {timings.simplification_sec:.3f}s).",
            "  repro:",
            f"    seed: {self.args.seed}",
            f"    query_index: {self.state.plan_query_index}",
            "    scene: BASE_SCENE_MJCF",
            f"    robot: {ROBOT_PATH}",
            f"    obstacle: {self.args.obstacle}",
            f"    goal_preset_index: {goal_index}",
            f"    goal_type: {goal_preset.goal_type_name}",
            f"    target_pose: {format_repro_array(candidate_pose)}",
            f"    position_tolerance: {IK_TOLERANCE}",
            f"    orientation: {goal_preset.orientation_name}",
            *(
                [
                    f"    orientation_target: {format_repro_array(goal_preset.orientation)}",
                    f"    orientation_tolerance: {goal_preset.orientation_tolerance}",
                ]
                if goal_preset.is_orientation_constrained
                else []
            ),
            f"    arm_dof_slice: [{self.mapping.dof_start}:{self.mapping.dof_end}]",
            f"    start_arm_q: {format_repro_array(current_start)}",
            f"    full_dof_pos: {format_repro_array(plan_input_full_dof_pos)}",
            f"    full_dof_vel: {format_repro_array(plan_input_full_dof_vel)}",
            f"    max_velocity: {PLANNER_MAX_VELOCITY}",
            f"    max_acceleration: {PLANNER_MAX_ACCELERATION}",
            f"    max_iterations: {PLANNER_MAX_ITERATIONS}",
            f"    step_size: {PLANNER_STEP_SIZE}",
            f"    goal_bias: {PLANNER_GOAL_BIAS}",
            f"    max_goal_attempts: {MAX_GOAL_ATTEMPTS}",
            f"    num_ik_seeds: {NUM_IK_SEEDS}",
            f"    ik_max_iter: {IK_MAX_ITER}",
            f"    ik_tolerance: {IK_TOLERANCE}",
            f"    ik_damping: {IK_DAMPING}",
            f"  start EE pose: {format_repro_array(start_ee_pose)}",
            f"  target pose: {format_repro_array(state.target_pose)}",
        ]
        if plan_result.goal_config is not None:
            selected_goal_ee_pose = self.end_effector_pose_for_arm_q(plan_result.goal_config)
            selected_goal_error = pose_error_summary(
                state.target_pose,
                selected_goal_ee_pose,
                state.target_orientation_constrained,
            )
            lines.extend(
                [
                    f"  selected goal q: {format_repro_array(plan_result.goal_config)}",
                    f"  selected goal EE pose: {format_repro_array(selected_goal_ee_pose)}",
                    f"  selected goal EE error: {selected_goal_error}",
                ]
            )
        if state.trajectory_ee_waypoints:
            final_trajectory_ee = np.array(state.trajectory_ee_waypoints[-1], dtype=np.float32)
            lines.append(
                "  final trajectory EE position error: "
                f"{np.linalg.norm(final_trajectory_ee - state.target_pose[:3]):.6g}"
            )
        diagnostics = plan_result.diagnostics
        if diagnostics is not None:
            lines.extend(
                [
                    f"  diagnostics position-only: {diagnostics.position_only}",
                    f"  diagnostics best IK residual: {diagnostics.best_ik_residual}",
                    f"  diagnostics best position error: {diagnostics.best_position_error}",
                    f"  diagnostics best orientation error: {diagnostics.best_orientation_error}",
                ]
            )
        print("\n".join(lines), flush=True)

    def extend_flattened_points(self, destination: list[np.ndarray], flattened_points: object | None) -> None:
        if flattened_points is None:
            return
        destination.extend(
            np.array(point, dtype=np.float32) for point in np.asarray(flattened_points, dtype=np.float32).reshape(-1, 3)
        )

    def physics_step(self) -> None:
        step(self.model, self.data)

    def run_control_interval(self, target_q: np.ndarray) -> None:
        self.arm.set_target(target_q)
        for _ in range(self.ctrl_steps):
            self.physics_step()
        self.state.ee_trail.append(self.chain.get_end_effector_pose(self.data)[0:3].copy())

    def control_step(self) -> None:
        state = self.state
        if state.active_trajectory is None:
            if state.settling_goal_q is not None:
                self.run_settle_interval()
                return
            self.run_control_interval(state.last_safe_arm_q)
            return

        for _ in range(self.ctrl_steps):
            desired = np.array(state.active_trajectory.position_at(state.replay_time), dtype=np.float32)
            self.arm.set_target(desired)
            self.physics_step()
            actual = self.chain.get_dof_pos(self.data)
            error = np.linalg.norm(actual - desired, ord=np.inf)
            state.max_tracking_error = max(state.max_tracking_error, float(error))
            state.replay_time = min(state.replay_time + SIM_DT, state.trajectory_duration)

        state.ee_trail.append(self.chain.get_end_effector_pose(self.data)[0:3].copy())

        if state.replay_time >= state.trajectory_duration:
            goal_q = state.active_goal_q.copy()
            state.active_trajectory = None
            state.active_goal_q = None
            state.settling_goal_q = goal_q
            state.settle_time = 0.0
            print(
                f"Replay target stream finished; max streaming joint tracking error "
                f"{state.max_tracking_error:.4f} rad. Settling at goal target...",
                flush=True,
            )

    def run_settle_interval(self) -> None:
        state = self.state
        goal_q = state.settling_goal_q
        self.arm.set_target(goal_q)
        joint_error = 0.0
        for _ in range(self.ctrl_steps):
            self.physics_step()
            actual = self.chain.get_dof_pos(self.data)
            joint_error = float(np.linalg.norm(actual - goal_q, ord=np.inf))
            state.settle_time += SIM_DT
            if joint_error <= REPLAY_SETTLE_JOINT_TOLERANCE:
                break

        state.ee_trail.append(self.chain.get_end_effector_pose(self.data)[0:3].copy())

        if joint_error <= REPLAY_SETTLE_JOINT_TOLERANCE or state.settle_time >= REPLAY_SETTLE_TIMEOUT:
            final_ee_pose = self.chain.get_end_effector_pose(self.data).copy()
            final_error = (
                pose_error_summary(state.target_pose, final_ee_pose, state.target_orientation_constrained)
                if state.target_pose is not None
                else "target_pose=None"
            )
            state.last_safe_arm_q = goal_q.copy()
            state.settling_goal_q = None
            print(
                f"Replay settle finished after {state.settle_time:.3f}s; "
                f"final joint error {joint_error:.4f} rad.\n"
                f"  settled replay EE pose: {format_repro_array(final_ee_pose)}\n"
                f"  settled replay EE error: {final_error}",
                flush=True,
            )

    def current_planned_path_index(self) -> int | None:
        state = self.state
        if len(state.planned_ee_path) == 0:
            return None
        if state.active_trajectory is None:
            if state.settling_goal_q is not None:
                return len(state.planned_ee_path) - 1
            return None
        if state.trajectory_duration <= 0.0:
            return 0

        progress = state.replay_time / state.trajectory_duration
        path_idx = round(progress * (len(state.planned_ee_path) - 1))
        return min(path_idx, len(state.planned_ee_path) - 1)

    def render_step(self) -> None:
        if self.render.input.is_key_just_pressed("space"):
            self.plan_random_goal()

        self.draw_ee_trail()
        self.draw_planned_path()
        self.draw_tracked_path()
        self.draw_goal_presets()
        self.draw_current_target()
        self.draw_current_end_effector()
        self.render.sync(self.data)

    def draw_ee_trail(self) -> None:
        trail = self.state.ee_trail
        for i in range(1, len(trail)):
            self.gizmos.draw_line(start=trail[i - 1], end=trail[i], color=self.colors.trail)

    def draw_planned_path(self) -> None:
        planned_path = self.state.planned_ee_path
        for i in range(1, len(planned_path)):
            self.gizmos.draw_line(
                start=planned_path[i - 1],
                end=planned_path[i],
                color=self.colors.planned_path,
            )
        for point in planned_path:
            self.gizmos.draw_sphere(0.012, point, color=self.colors.planned_path)
        for point in self.state.trajectory_ee_waypoints:
            self.gizmos.draw_sphere(0.018, point, color=self.colors.trajectory_waypoint)

    def draw_tracked_path(self) -> None:
        current_path_idx = self.current_planned_path_index()
        if current_path_idx is None:
            return

        planned_path = self.state.planned_ee_path
        for i in range(1, current_path_idx + 1):
            self.gizmos.draw_line(
                start=planned_path[i - 1],
                end=planned_path[i],
                color=self.colors.tracked_path,
            )
        self.gizmos.draw_sphere(
            0.026,
            planned_path[current_path_idx],
            color=self.colors.current_target,
        )

    def draw_goal_presets(self) -> None:
        for preset in self.goal_presets:
            self.gizmos.draw_sphere(0.012, preset.position, color=self.colors.goal_preset)
            if preset.is_orientation_constrained:
                self.gizmos.draw_axes(preset.position, preset.orientation, length=0.06)

    def draw_current_target(self) -> None:
        if self.state.target_pose is not None:
            self.gizmos.draw_sphere(0.035, self.state.target_pose[:3], color=self.colors.selected_goal)
            if self.state.target_orientation_constrained:
                self.gizmos.draw_axes(self.state.target_pose[:3], self.state.target_pose[3:7], length=0.11)

    def draw_current_end_effector(self) -> None:
        ee_pose = self.chain.get_end_effector_pose(self.data)
        self.gizmos.draw_sphere(0.02, ee_pose[0:3], color=Color.rgb(0.2, 0.5, 1.0))
        self.gizmos.draw_axes(ee_pose[0:3], ee_pose[3:7], length=0.1)


def main() -> None:
    args = parse_args()

    with RenderApp() as render:
        demo = MotionPlanningDemo(args, render)
        demo.run()


if __name__ == "__main__":
    main()
