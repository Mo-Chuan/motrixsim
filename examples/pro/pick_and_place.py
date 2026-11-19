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

"""Pick-and-place with motion planning: Franka Panda grasps a cube and moves it.

Demonstrates:
1. Load a scene with a Franka Panda arm and a free-floating cube.
2. Use MotionPlanner to plan collision-free trajectories to reach the cube.
3. Attach the cube to the planner so subsequent plans treat it as part of the
   gripper (collision between gripper and cube is ignored).
4. Plan a trajectory to a place position and detach the cube.
5. Visualise every step with RenderApp, including EE trail and goal markers.

Usage:
    cd motrixsim-python/motrixsim-docs
    python examples/pro/pick_and_place.py
"""

from collections.abc import Generator

import numpy as np

import motrixsim as mx
from motrixsim import SceneData, step
from motrixsim.ik import IkChain
from motrixsim.planner import (
    CartesianGoal,
    FreeOrientationGoal,
    MotionPlanner,
    MotionPlanRequest,
    PointPositionGoal,
    QuaternionOrientationGoal,
)
from motrixsim.render import Color, RenderApp

# -- Scene ---------------------------------------------------------------------

ROBOT_PATH = "examples/assets/franka_emika_panda/scene_pick_cube.xml"

# -- Constants -----------------------------------------------------------------

SIM_DT = 0.002
CTRL_DT = 0.01
CTRL_STEPS = max(1, round(CTRL_DT / SIM_DT))
SETTLE_INTERVALS = max(1, round(0.5 / CTRL_DT))

# Arm actuator indices (actuator1..actuator7).
ARM_CTRL_SLICE = slice(0, 7)
# Gripper actuator index (actuator8 controls finger_joint1; finger_joint2 is
# coupled via an equality constraint).
GRIPPER_CTRL_IDX = 7

GRIPPER_OPEN = 0.04
GRIPPER_CLOSED = 0.0

# Pick target: cube "obj" centre at (0.65, 0, 0.02), half-extent 0.02.
# The grasp keyframe in scene_pick_cube.xml places the EE (TCP) at z≈0.03.
CUBE_POS = np.array([0.65, 0.0, 0.02])
CUBE_BODY_NAME = "obj"
PRE_GRASP_HEIGHT = 0.18
GRASP_HEIGHT = 0.03

# Place target: move the cube to a different position on the table.
PLACE_POS = np.array([0.45, -0.3, 0.02])
PLACE_HEIGHT = 0.18

# Planner parameters.  Keep velocity / acceleration moderate so the arm
# doesn't jerk too hard and fling the cube.
PLANNER_SEED = 42
NUM_IK_SEEDS = 64
IK_MAX_ITER = 100
IK_DAMPING = 1e-3
POSITION_TOLERANCE = 1e-2
ORIENTATION_TOLERANCE = 0.5

# Orientation: gripper pointing straight down.
# Measured from the "grasp" keyframe: quat ≈ [-1, 0, 0, 0].
GRASP_ORIENTATION = [-1.0, 0.0, 0.0, 0.0]  # [qx, qy, qz, qw]

# Gizmo colours.
COLOR_EE_TRAIL = Color.rgb(1.0, 0.6, 0.0)
COLOR_TARGET = Color.rgb(0.2, 1.0, 0.35)
COLOR_PLACE = Color.rgb(0.0, 0.8, 1.0)
COLOR_EE = Color.rgb(0.2, 0.5, 1.0)


# -- Helpers -------------------------------------------------------------------


def make_cartesian_request(
    position: np.ndarray,
    orientation: list[float] | None = None,
) -> MotionPlanRequest:
    if orientation is not None:
        orient_goal = QuaternionOrientationGoal(orientation, ORIENTATION_TOLERANCE)
    else:
        orient_goal = FreeOrientationGoal()
    return MotionPlanRequest(
        CartesianGoal(
            PointPositionGoal(position.tolist(), POSITION_TOLERANCE),
            orient_goal,
        ),
        num_ik_seeds=NUM_IK_SEEDS,
        ik_max_iter=IK_MAX_ITER,
        ik_damping=IK_DAMPING,
    )


def set_gripper(data: SceneData, opening: float) -> None:
    ctrls = data.actuator_ctrls.copy()
    ctrls[GRIPPER_CTRL_IDX] = opening
    data.actuator_ctrls = ctrls


# -- Demo class ----------------------------------------------------------------


class PickAndPlaceDemo:
    """Pick-and-place demo driven by a single render loop.

    The demo sequence is implemented as a generator (`_sequence`).  Each
    ``yield`` hands control back to the main loop, which advances physics by
    one control interval, draws gizmos, and calls ``render.sync`` — all in
    one place.
    """

    def __init__(self, render: RenderApp) -> None:
        self.render = render
        self.model = mx.load_model(ROBOT_PATH)
        self.model.options.timestep = SIM_DT
        self.data = SceneData(self.model)

        # Reset to the "home" keyframe.
        self.model.keyframes[0].apply(self.data)
        mx.forward_kinematic(self.model, self.data)

        self.chain = IkChain(
            self.model,
            end_link="hand",
            start_link="link0",
            end_effector_offset=[0.0, 0.0, 0.1, 0, 0, 0, 1],
        )
        self.planner = MotionPlanner(
            self.chain,
            max_velocity=1.0,
            max_acceleration=4.0,
            max_iterations=5000,
            step_size=0.1,
            goal_bias=0.05,
            seed=PLANNER_SEED,
            max_goal_attempts=12,
        )

        self.ee_trail: list[np.ndarray] = []
        self.target_marker: np.ndarray | None = None

    # -- Render loop (single sync point) ---------------------------------------

    def run(self) -> None:
        self.render.launch(self.model)
        sequence = self._sequence()
        finished = False

        while not self.render.is_closed:
            # 1. Advance the demo by one control interval.
            if not finished:
                try:
                    next(sequence)
                except StopIteration:
                    finished = True

            # 2. Record the EE trail.
            self.ee_trail.append(self.chain.get_end_effector_pose(self.data)[:3].copy())

            # 3. Draw gizmos.
            self._draw_gizmos()

            # 4. Push frame to the render window.
            self.render.sync(self.data)

    def _draw_gizmos(self) -> None:
        gizmos = self.render.gizmos

        # EE trail.
        for i in range(1, len(self.ee_trail)):
            gizmos.draw_line(start=self.ee_trail[i - 1], end=self.ee_trail[i], color=COLOR_EE_TRAIL)

        # Current EE position.
        ee_pose = self.chain.get_end_effector_pose(self.data)
        gizmos.draw_sphere(0.015, ee_pose[:3], color=COLOR_EE)

        # Current target marker.
        if self.target_marker is not None:
            gizmos.draw_sphere(0.02, self.target_marker, color=COLOR_TARGET)

        # Place position marker (always visible).
        gizmos.draw_sphere(0.02, PLACE_POS, color=COLOR_PLACE)

    # -- Simulation primitives (each yield = one control interval) -------------

    def _step_physics(self) -> None:
        """Advance the simulation by one control interval (CTRL_STEPS sim steps)."""
        for _ in range(CTRL_STEPS):
            step(self.model, self.data)

    def _wait(self, num_intervals: int) -> Generator[None]:
        """Run physics for *num_intervals* control intervals."""
        for _ in range(num_intervals):
            self._step_physics()
            yield

    def _follow_trajectory(self, trajectory, *, settle: int = SETTLE_INTERVALS) -> Generator[None]:
        """Follow a planned trajectory with position control, then settle."""
        samples = trajectory.sample_positions(sample_dt=CTRL_DT, max_points=4096)
        for sample in samples:
            ctrls = self.data.actuator_ctrls.copy()
            ctrls[ARM_CTRL_SLICE] = np.asarray(sample, dtype=np.float32)
            self.data.actuator_ctrls = ctrls
            self._step_physics()
            yield

        # Let the arm settle at the final position.
        yield from self._wait(settle)

    def _plan_and_execute(
        self,
        position: np.ndarray,
        label: str,
        orientation: list[float] | None = None,
    ) -> Generator[None, None, bool]:
        """Plan to a Cartesian target and follow the trajectory.

        Yields control back to the render loop each control interval.
        Returns True on success via ``generator.return`` (captured by the
        caller through ``StopIteration.value``).
        """
        self.target_marker = position.copy()
        result = self.planner.plan(self.data, make_cartesian_request(position, orientation))

        if not result.success:
            print(f"  [{label}] Planning FAILED: {result.status} - {result.failure_reason}")
            if result.diagnostics:
                d = result.diagnostics
                print(
                    f"    IK: {d.num_converged_candidates}/{d.num_ik_candidates} converged, "
                    f"collision-free: {d.num_collision_free_candidates}"
                )
            return False

        traj = result.trajectory
        print(
            f"  [{label}] Plan OK: {len(result.waypoints)} waypoints, "
            f"trajectory {traj.duration_sec:.2f}s ({traj.num_points} pts)"
        )
        yield from self._follow_trajectory(traj)

        ee_pose = self.chain.get_end_effector_pose(self.data)
        pos_err = float(np.linalg.norm(ee_pose[:3] - position))
        print(f"  [{label}] EE position error: {pos_err:.4f} m")
        return True

    # -- Demo sequence ---------------------------------------------------------

    def _sequence(self) -> Generator[None]:
        """The full pick-and-place sequence, yielding each control interval."""
        model, data = self.model, self.data

        print("=== Pick-and-Place Demo ===")
        ee_start = self.chain.get_end_effector_pose(data)
        print(f"Home EE position: [{ee_start[0]:.3f}, {ee_start[1]:.3f}, {ee_start[2]:.3f}]")

        # Phase 1: Open gripper.
        print("\n[Phase 1] Open gripper")
        set_gripper(data, GRIPPER_OPEN)
        yield from self._wait(100)

        # Phase 2: Move to pre-grasp (above the cube).
        pre_grasp_pos = CUBE_POS.copy()
        pre_grasp_pos[2] = PRE_GRASP_HEIGHT
        print(f"\n[Phase 2] Plan to pre-grasp at {pre_grasp_pos}")
        self.planner.set_excluded_bodies(model, [CUBE_BODY_NAME])
        if not (yield from self._plan_and_execute(pre_grasp_pos, "pre-grasp", GRASP_ORIENTATION)):
            return

        # Phase 3: Move down to grasp position.
        grasp_pos = CUBE_POS.copy()
        grasp_pos[2] = GRASP_HEIGHT
        print(f"\n[Phase 3] Plan to grasp at {grasp_pos}")
        if not (yield from self._plan_and_execute(grasp_pos, "grasp", GRASP_ORIENTATION)):
            return
        self.planner.clear_excluded_bodies(model)

        # Phase 4: Close gripper and attach cube.
        print("\n[Phase 4] Close gripper & attach cube")
        set_gripper(data, GRIPPER_CLOSED)
        yield from self._wait(100)
        grasp_offset = np.array([0, 0, 0.1, 0, 0, 0, 1], dtype=np.float32)
        # The cube follows the "hand" mount link. Ignore its collisions against the whole
        # gripper (hand + both fingers) so contact with the fingers does not register as a
        # start collision.
        self.planner.attach_body(
            model,
            CUBE_BODY_NAME,
            "hand",
            ["hand", "left_finger", "right_finger"],
            grasp_offset,
        )
        print(f"  Cube attached: {self.planner.has_attached_body}")

        # Phase 5: Lift.
        lift_pos = CUBE_POS.copy()
        lift_pos[2] = PRE_GRASP_HEIGHT
        print(f"\n[Phase 5] Lift to {lift_pos}")
        if not (yield from self._plan_and_execute(lift_pos, "lift", GRASP_ORIENTATION)):
            return

        # Phase 6: Move to place position.
        place_pos = PLACE_POS.copy()
        place_pos[2] = PLACE_HEIGHT
        print(f"\n[Phase 6] Move to place at {place_pos}")
        if not (yield from self._plan_and_execute(place_pos, "place", GRASP_ORIENTATION)):
            return

        # Phase 7: Lower to place surface.
        lower_pos = PLACE_POS.copy()
        lower_pos[2] = GRASP_HEIGHT
        print(f"\n[Phase 7] Lower to {lower_pos}")
        if not (yield from self._plan_and_execute(lower_pos, "lower", GRASP_ORIENTATION)):
            return

        # Phase 8: Open gripper and detach.
        print("\n[Phase 8] Open gripper & detach")
        set_gripper(data, GRIPPER_OPEN)
        yield from self._wait(100)
        self.planner.detach_body(model)
        self.target_marker = None
        print(f"  Cube detached: has_attached_body={self.planner.has_attached_body}")

        # Phase 9: Retreat upward.
        retreat_pos = PLACE_POS.copy()
        retreat_pos[2] = PRE_GRASP_HEIGHT
        print(f"\n[Phase 9] Retreat to {retreat_pos}")
        if not (yield from self._plan_and_execute(retreat_pos, "retreat")):
            return

        print("\n=== Pick-and-Place Complete ===")
        ee_final = self.chain.get_end_effector_pose(data)
        print(f"Final EE position: [{ee_final[0]:.3f}, {ee_final[1]:.3f}, {ee_final[2]:.3f}]")


def main() -> None:
    with RenderApp() as render:
        demo = PickAndPlaceDemo(render)
        demo.run()


if __name__ == "__main__":
    main()
