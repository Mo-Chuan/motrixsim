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

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .arm_controller import ArmController, default_accuracy, generate_track
from .common import (
    ARM_ACTUATOR_NAMES,
    GRIPPER_ACTUATOR,
    GRIPPER_CLOSE_CTRL,
    GRIPPER_LINK_NAME,
    GRIPPER_OPEN_CTRL,
    ROBOT_BASE_LINK,
    VEHICLE_BRAKE_ACTUATOR,
    VEHICLE_FORWARD_ACTUATOR,
    VEHICLE_TURN_ACTUATOR,
    VERTICAL_HANGING_OFFSET,
    get_pose_by_sensor_name,
    inverse_transform_point,
    item_height_offset,
    pose,
    quat_face_towards,
    quat_from_yaw,
    quat_multiply,
    quat_top_down_grasp,
    transform_vec,
    yaw_from_quat,
)
from .transporter_controller import TransporterController

BASKET_FRONT_WAYPOINT = np.array([0.30, -2.26], dtype=np.float32)
PRE_BASKET_FRONT_WAYPOINT = np.array([-1.5, -2.26], dtype=np.float32)
TABLE_FRONT_WAYPOINT = BASKET_FRONT_WAYPOINT.copy()
BASKET_PICK_HOVER_OFFSET = np.array([0.0, 0.0, 0.40], dtype=np.float32)
BASKET_PICK_OFFSET = np.array([0.00, 0.07, -0.18], dtype=np.float32)
BASKET_ON_BASE_OFFSET = np.array([-0.40, 0.30, 0.10], dtype=np.float32)
TABLE_PLACE_OFFSET = np.array([-0.45, -0.90, 0.22], dtype=np.float32)
DEFAULT_NAVIGATION_TARGET = np.array([0.0, -1.0], dtype=np.float32)
GRIPPER_DRIVER_JOINT = "right_driver_joint"
GRIPPER_OPEN_POSITION_THRESHOLD = 0.02
GRIPPER_SETTLE_POSITION_THRESHOLD = 5e-5
SHELF_FRONT_WAYPOINTS = [
    np.array([-1.80, -0.15], dtype=np.float32),
    np.array([-1.00, 0.00], dtype=np.float32),
    np.array([-0.10, 0.15], dtype=np.float32),
    np.array([1.10, -0.05], dtype=np.float32),
]
SHELF_X_THRESHOLDS = (-0.87, 0.0, 0.87)


@dataclass
class FetchItemCommand:
    force_direction: np.ndarray | None = None


@dataclass
class MoveToSensorTargetCommand:
    target_name: str
    hanging_offset: np.ndarray
    force_direction: np.ndarray | None


@dataclass
class MoveToTargetPointCommand:
    target: np.ndarray
    hanging_offset: np.ndarray
    force_direction: np.ndarray | None


@dataclass
class MoveToRelativeTargetCommand:
    link_name: str
    local_offset: np.ndarray
    hanging_offset: np.ndarray
    force_direction: np.ndarray | None


@dataclass
class GripperMoveCommand:
    value: float


@dataclass
class BaseRotateCommand:
    value: float


@dataclass
class BaseAlignYawCommand:
    value: float


@dataclass
class CarMoveWayPointsCommand:
    waypoints: list[list[float]]


class CarRunningCommand:
    pass


class ArmRunningCommand:
    pass


@dataclass
class GripperRunningCommand:
    reference_pos: np.ndarray
    target_value: float


class ResetCommand:
    pass


class IimtTaskRuntime:
    def __init__(self, model, data):
        self.model = model
        self.arm = ArmController(model)
        self.transporter = TransporterController()
        self.arm_actuators = [model.get_actuator(name) for name in ARM_ACTUATOR_NAMES]
        self.forward_actuator = model.get_actuator(VEHICLE_FORWARD_ACTUATOR)
        self.turn_actuator = model.get_actuator(VEHICLE_TURN_ACTUATOR)
        self.brake_actuator = model.get_actuator(VEHICLE_BRAKE_ACTUATOR)
        self.gripper_actuator = model.get_actuator(GRIPPER_ACTUATOR)
        self.gripper_driver_joint = model.get_joint(GRIPPER_DRIVER_JOINT)
        self.default_grasp_rotation = np.asarray(self.arm.chain.get_end_effector_pose(data), dtype=np.float32)[
            3:7
        ].copy()
        self.commands: list[object] = []
        self.selected_target = None
        self.last_pick_ray = None
        self.debug_candidates = []
        self.item_names = [name for name in model.body_names if isinstance(name, str) and name.startswith("V_")]
        self.gripper_target = GRIPPER_OPEN_CTRL
        self.current_navigation_target = DEFAULT_NAVIGATION_TARGET.copy()

    def is_mode_idle(self) -> bool:
        return not self.commands and self.arm.is_idle() and self.transporter.is_idle()

    def has_pending_commands(self) -> bool:
        return bool(self.commands)

    def select_target(self, target_info, ray, candidates) -> None:
        self.selected_target = target_info
        self.last_pick_ray = None if ray is None else np.asarray(ray, dtype=np.float32).copy()
        self.debug_candidates = list(candidates)

    def initialize(self) -> None:
        self.commands = list(
            reversed(
                [
                    CarMoveWayPointsCommand([[0.0, 0.0], [-1.0, 0.0]]),
                    ResetCommand(),
                    GripperMoveCommand(GRIPPER_OPEN_CTRL),
                    MoveToSensorTargetCommand(ROBOT_BASE_LINK, np.zeros(3, dtype=np.float32), None),
                    GripperMoveCommand(GRIPPER_CLOSE_CTRL),
                    MoveToSensorTargetCommand("basket", VERTICAL_HANGING_OFFSET.copy(), None),
                    CarMoveWayPointsCommand([TABLE_FRONT_WAYPOINT.tolist(), [-2.0, -2.15]]),
                    ResetCommand(),
                ]
            )
        )

    def finish(self, data) -> None:
        self.commands.clear()
        self.arm.reset()
        self.transporter.stop()
        basket_on_base = self._basket_on_base(data)
        table_waypoints = [TABLE_FRONT_WAYPOINT.tolist()]
        pre_table_waypoint = [PRE_BASKET_FRONT_WAYPOINT.tolist()]
        already_at_table_waypoint = self._is_at_waypoint(data, TABLE_FRONT_WAYPOINT) or self._has_reached_waypoint(
            TABLE_FRONT_WAYPOINT
        )
        basket_pick_target, basket_pick_direction = self._basket_pick_target(data)

        if basket_on_base:
            navigation_sequence = (
                []
                if already_at_table_waypoint
                else [
                    CarMoveWayPointsCommand(pre_table_waypoint),
                    BaseAlignYawCommand(0.0),
                    CarMoveWayPointsCommand(table_waypoints),
                ]
            )
            sequence = navigation_sequence + [
                ResetCommand(),
                GripperMoveCommand(GRIPPER_OPEN_CTRL),
                ResetCommand(),
                MoveToRelativeTargetCommand(
                    "basket",
                    BASKET_PICK_OFFSET.copy(),
                    VERTICAL_HANGING_OFFSET.copy(),
                    basket_pick_direction,
                ),
                GripperMoveCommand(GRIPPER_CLOSE_CTRL),
                MoveToRelativeTargetCommand(
                    ROBOT_BASE_LINK,
                    TABLE_PLACE_OFFSET.copy(),
                    VERTICAL_HANGING_OFFSET.copy(),
                    basket_pick_direction,
                ),
                GripperMoveCommand(GRIPPER_OPEN_CTRL),
                ResetCommand(),
            ]
        else:
            navigation_sequence = (
                []
                if already_at_table_waypoint
                else [
                    CarMoveWayPointsCommand(table_waypoints),
                    BaseAlignYawCommand(0.0),
                ]
            )
            sequence = navigation_sequence + [
                GripperMoveCommand(GRIPPER_OPEN_CTRL),
                ResetCommand(),
                MoveToTargetPointCommand(
                    basket_pick_target,
                    BASKET_PICK_HOVER_OFFSET.copy(),
                    basket_pick_direction,
                ),
                GripperMoveCommand(GRIPPER_CLOSE_CTRL),
                MoveToRelativeTargetCommand(
                    ROBOT_BASE_LINK,
                    BASKET_ON_BASE_OFFSET.copy(),
                    VERTICAL_HANGING_OFFSET.copy(),
                    basket_pick_direction,
                ),
                GripperMoveCommand(GRIPPER_OPEN_CTRL),
                ResetCommand(),
            ]

        self.commands = list(reversed(sequence))

    def reset_arm(self) -> None:
        self.commands.clear()
        self.arm.reset()
        self.gripper_target = GRIPPER_OPEN_CTRL

    def set_direct_drive(self, forward: float, rotate: float) -> None:
        if self.commands:
            return
        self.transporter.set_direct_input(forward, rotate)

    def fetch_selected_target(self, data) -> None:
        if self.selected_target is None:
            return

        if not self._basket_on_base(data):
            selected_target = self.selected_target
            self.finish(data)
            finish_commands = self.commands.copy()
            self.selected_target = selected_target
            self._enqueue_fetch_target_item(data)
            fetch_commands = self.commands.copy()
            self.commands = fetch_commands + finish_commands
            self.selected_target = None
            return

        self._enqueue_fetch_target_item(data)
        self.selected_target = None

    def _enqueue_fetch_target_item(self, data) -> None:
        target = self.selected_target
        if target is None:
            return

        normal_take_direction = quat_face_towards(
            np.array([0.0, 1.0, -0.5], dtype=np.float32),
            np.array([1.0, 0.0, 0.0], dtype=np.float32),
        )
        yaw_quat = quat_from_yaw(yaw_from_quat(target.rotation))
        rotated_take_direction = quat_multiply(yaw_quat, normal_take_direction)

        # Box-like tetra packs, plus the narrow Bottle_350 tuning case, need the gripper
        # orientation to follow the item yaw. Cylindrical bottles/cans are
        # effectively yaw-symmetric here, so the default top-down grasp is enough.
        if target.item_type in {"V_Bottle_350", "V_Tetra_250_F", "V_Tetra_250_T", "V_Tetra_300", "V_Tetra_500"}:
            item_catch_direction = rotated_take_direction
        else:
            item_catch_direction = normal_take_direction.copy()

        target_pos = np.array(
            [target.position[0], target.position[1], target.position[2] + item_height_offset(target.item_type)],
            dtype=np.float32,
        )
        base_pose = np.asarray(self.model.get_link(ROBOT_BASE_LINK).get_pose(data), dtype=np.float32)
        relative_pos = inverse_transform_point(base_pose, target_pos)
        if relative_pos[1] < 0.0:
            base_rotate, back_rotate = 1.5, 1.1
        else:
            base_rotate, back_rotate = -1.5, -1.1

        shelf_waypoints = self._build_shelf_approach_waypoints(target_pos)
        sequence = [
            CarMoveWayPointsCommand(shelf_waypoints),
            BaseAlignYawCommand(0.0),
            GripperMoveCommand(GRIPPER_OPEN_CTRL),
            ResetCommand(),
            BaseRotateCommand(base_rotate),
            MoveToTargetPointCommand(target_pos, VERTICAL_HANGING_OFFSET.copy(), item_catch_direction),
            GripperMoveCommand(GRIPPER_CLOSE_CTRL),
            FetchItemCommand(),
            BaseRotateCommand(base_rotate),
            MoveToSensorTargetCommand("basket_place", np.zeros(3, dtype=np.float32), None),
            GripperMoveCommand(GRIPPER_OPEN_CTRL),
            BaseRotateCommand(back_rotate),
            ResetCommand(),
        ]
        self.commands = list(reversed(sequence))

    def _build_shelf_approach_waypoints(self, target: np.ndarray) -> list[list[float]]:
        shelf_waypoint = self._shelf_front_waypoint_for_target(target)
        return [shelf_waypoint.tolist()]

    def _shelf_front_waypoint_for_target(self, target: np.ndarray) -> np.ndarray:
        target_x = float(target[0])
        if target_x < SHELF_X_THRESHOLDS[0]:
            return SHELF_FRONT_WAYPOINTS[0].copy()
        if target_x < SHELF_X_THRESHOLDS[1]:
            return SHELF_FRONT_WAYPOINTS[1].copy()
        if target_x < SHELF_X_THRESHOLDS[2]:
            return SHELF_FRONT_WAYPOINTS[2].copy()
        return SHELF_FRONT_WAYPOINTS[3].copy()

    def _basket_pick_target(self, data) -> tuple[np.ndarray, np.ndarray]:
        basket_pose = get_pose_by_sensor_name(self.model, data, "basket")
        base_pose = np.asarray(self.model.get_link(ROBOT_BASE_LINK).get_pose(data), dtype=np.float32)
        base_in_basket = inverse_transform_point(basket_pose, base_pose[:3])

        if base_in_basket[1] <= 0.0:
            local_offset = np.array([0.0, -0.33, -0.02], dtype=np.float32)
        else:
            local_offset = np.array([0.0, 0.33, -0.02], dtype=np.float32)

        opening_axis_local = np.array([1.0, 0.0, 0.0], dtype=np.float32)

        pick_target = basket_pose[:3] + transform_vec(basket_pose[3:7], local_offset)
        opening_axis_world = transform_vec(basket_pose[3:7], opening_axis_local)
        grip_direction = quat_top_down_grasp(opening_axis_world)
        return pick_target.astype(np.float32), grip_direction.astype(np.float32)

    def _basket_on_base(self, data) -> bool:
        base_pose = np.asarray(self.model.get_link(ROBOT_BASE_LINK).get_pose(data), dtype=np.float32)
        basket_pose = get_pose_by_sensor_name(self.model, data, "basket")
        local_translation = inverse_transform_point(base_pose, basket_pose[:3])
        horizontal_on_base = abs(float(local_translation[0])) < 1.00 and abs(float(local_translation[1])) < 1.00
        height_delta = float(basket_pose[2] - base_pose[2])
        height_on_base = -0.10 < height_delta < 0.10
        return horizontal_on_base and height_on_base

    def step(self, data, dt: float) -> None:
        self._run_command(data)
        self.transporter.update_state_from_model(self.model, data, dt)
        self.arm.update(data)
        self._apply_actions(data)
        self.model.step(data)

    def transporter_debug_waypoints(self) -> list[np.ndarray]:
        return self.transporter.all_waypoints()

    def transporter_current_waypoint(self) -> np.ndarray | None:
        return self.transporter.current_waypoint()

    def _apply_actions(self, data) -> None:
        for actuator, value in zip(self.arm_actuators, self.arm.desired_dof_pos, strict=True):
            actuator.set_ctrl(data, float(value))

        transporter_actions = self.transporter.actions()
        self.forward_actuator.set_ctrl(data, float(transporter_actions[0]))
        self.turn_actuator.set_ctrl(data, float(transporter_actions[1]))
        self.brake_actuator.set_ctrl(data, float(transporter_actions[2]))
        self.gripper_actuator.set_ctrl(data, float(self.gripper_target))

    def _run_command(self, data) -> None:
        if not self.commands:
            return

        last_index = len(self.commands) - 1
        cmd = self.commands[last_index]
        if isinstance(cmd, FetchItemCommand):
            self._fetch_item(data, cmd.force_direction)
            self.commands[last_index] = ArmRunningCommand()
        elif isinstance(cmd, MoveToSensorTargetCommand):
            self._move_to_sensor_target(data, cmd.target_name, cmd.hanging_offset, cmd.force_direction)
            self.commands[last_index] = ArmRunningCommand()
        elif isinstance(cmd, MoveToTargetPointCommand):
            self._move_to_target_point(data, cmd.target, cmd.hanging_offset, cmd.force_direction)
            self.commands[last_index] = ArmRunningCommand()
        elif isinstance(cmd, MoveToRelativeTargetCommand):
            self._move_to_relative_target(
                data,
                cmd.link_name,
                cmd.local_offset,
                cmd.hanging_offset,
                cmd.force_direction,
            )
            self.commands[last_index] = ArmRunningCommand()
        elif isinstance(cmd, CarMoveWayPointsCommand):
            target_waypoint = self._command_navigation_target(cmd.waypoints)
            if target_waypoint is None:
                self.commands.pop()
            elif self._same_navigation_target(target_waypoint):
                self.commands.pop()
            else:
                self.transporter.start_waypoint_navigation(cmd.waypoints)
                self.current_navigation_target = target_waypoint.copy()
                self.commands[last_index] = CarRunningCommand()
        elif isinstance(cmd, BaseAlignYawCommand):
            self.transporter.start_yaw_alignment(cmd.value)
            self.commands[last_index] = CarRunningCommand()
        elif isinstance(cmd, BaseRotateCommand):
            self.arm.start_base_rotate(cmd.value)
            self.commands[last_index] = ArmRunningCommand()
        elif isinstance(cmd, GripperMoveCommand):
            self.gripper_target = float(cmd.value)
            grip_pose = np.asarray(self.model.get_link(GRIPPER_LINK_NAME).get_pose(data), dtype=np.float32)
            self.commands[last_index] = GripperRunningCommand(grip_pose[:3].copy(), float(cmd.value))
        elif isinstance(cmd, ResetCommand):
            self.gripper_target = GRIPPER_OPEN_CTRL
            self.arm.reset()
            self.commands[last_index] = ArmRunningCommand()
        elif isinstance(cmd, CarRunningCommand):
            if self.transporter.is_idle():
                self.commands.pop()
        elif isinstance(cmd, ArmRunningCommand):
            if self.arm.is_idle():
                self.commands.pop()
        elif isinstance(cmd, GripperRunningCommand):
            current = np.asarray(self.model.get_link(GRIPPER_LINK_NAME).get_pose(data), dtype=np.float32)[:3]
            if self._gripper_motion_completed(data, cmd, current):
                self.commands.pop()
            else:
                cmd.reference_pos[:] = current

    def _move_to_sensor_target(
        self, data, sensor_name: str, hanging_offset: np.ndarray, force_direction: np.ndarray | None
    ) -> None:
        target = get_pose_by_sensor_name(self.model, data, sensor_name)
        if force_direction is not None:
            target[3:7] = force_direction
        start = np.asarray(self.arm.chain.get_end_effector_pose(data), dtype=np.float32)
        waypoints = generate_track(start, target, hanging_offset)
        accuracy = default_accuracy(waypoints)
        self.arm.start_move_to_waypoints(waypoints, accuracy)

    def _fetch_item(self, data, force_direction: np.ndarray | None = None) -> None:
        base_pose = np.asarray(self.model.get_link(ROBOT_BASE_LINK).get_pose(data), dtype=np.float32)
        start = np.asarray(self.arm.chain.get_end_effector_pose(data), dtype=np.float32)
        start[2] += 0.05
        safe_plane_target = start.copy()
        safe_plane_target[1] = 0.45
        if force_direction is not None:
            safe_plane_target[3:7] = force_direction
        waypoints = generate_track(start, safe_plane_target, np.zeros(3, dtype=np.float32))
        safe_height = safe_plane_target.copy()
        safe_height[2] = 1.3
        if force_direction is not None:
            safe_height[3:7] = force_direction
        waypoints.extend(generate_track(safe_plane_target, safe_height, np.zeros(3, dtype=np.float32)))
        accuracy = default_accuracy(waypoints)
        if accuracy:
            accuracy[0] = 0.05
        self._forbidden_zone_modify(base_pose, waypoints)
        self.arm.start_move_to_waypoints(waypoints, accuracy)

    def _move_to_target_point(
        self, data, target_translation: np.ndarray, hanging_offset: np.ndarray, force_direction: np.ndarray | None
    ) -> None:
        target = pose(target_translation, self.default_grasp_rotation if force_direction is None else force_direction)
        base_pose = np.asarray(self.model.get_link(ROBOT_BASE_LINK).get_pose(data), dtype=np.float32)
        start = np.asarray(self.arm.chain.get_end_effector_pose(data), dtype=np.float32)
        waypoints = generate_track(start, target, hanging_offset)
        accuracy = default_accuracy(waypoints)
        self._forbidden_zone_modify(base_pose, waypoints)
        self.arm.start_move_to_waypoints(waypoints, accuracy)

    def _move_to_relative_target(
        self,
        data,
        link_name: str,
        local_offset: np.ndarray,
        hanging_offset: np.ndarray,
        force_direction: np.ndarray | None,
    ) -> None:
        link_pose = np.asarray(self.model.get_link(link_name).get_pose(data), dtype=np.float32)
        target_translation = link_pose[:3] + transform_vec(link_pose[3:7], local_offset)
        self._move_to_target_point(data, target_translation, hanging_offset, force_direction)

    def _gripper_motion_completed(self, data, cmd: GripperRunningCommand, current_pos: np.ndarray) -> bool:
        if cmd.target_value == GRIPPER_OPEN_CTRL and self.gripper_driver_joint is not None:
            joint_pos = float(np.asarray(self.gripper_driver_joint.get_dof_pos(data), dtype=np.float32)[0])
            return joint_pos <= GRIPPER_OPEN_POSITION_THRESHOLD
        error = float(np.linalg.norm(cmd.reference_pos - current_pos))
        return error < GRIPPER_SETTLE_POSITION_THRESHOLD

    def _is_at_waypoint(self, data, waypoint: np.ndarray) -> bool:
        base_pose = np.asarray(self.model.get_link(ROBOT_BASE_LINK).get_pose(data), dtype=np.float32)
        delta = np.asarray(waypoint, dtype=np.float32) - base_pose[:2]
        threshold = self.transporter.reach_position_threshold
        return abs(float(delta[0])) < threshold and abs(float(delta[1])) < threshold

    def _has_reached_waypoint(self, waypoint: np.ndarray) -> bool:
        if self.transporter.last_reached_waypoint is None:
            return False
        return bool(
            np.allclose(
                np.asarray(self.transporter.last_reached_waypoint, dtype=np.float32),
                np.asarray(waypoint, dtype=np.float32),
                atol=1e-4,
            )
        )

    def _forbidden_zone_modify(self, base_pose: np.ndarray, waypoints: list[np.ndarray]) -> None:
        min_height = float(base_pose[2] + 0.3)
        x_offset_car = 0.5
        y_offset_car = 0.5
        x_offset_arm_base = 0.2
        y_offset_arm_base = 0.3
        # Keep the final target pose exact. `generate_track()` appends the final
        # target twice at the end, so only adjust intermediate waypoints here.
        protected_tail = 2 if len(waypoints) >= 2 else 0
        adjustable_waypoints = waypoints[1:-protected_tail] if protected_tail else waypoints[1:]
        for waypoint in adjustable_waypoints:
            local_waypoint = inverse_transform_point(base_pose, waypoint[:3])
            if (
                waypoint[2] < min_height
                and abs(float(local_waypoint[0] + 0.28)) < x_offset_car
                and abs(float(local_waypoint[1])) < y_offset_car
            ):
                waypoint[2] = min_height
            if abs(float(local_waypoint[0])) < x_offset_arm_base and abs(float(local_waypoint[1])) < y_offset_arm_base:
                waypoint[0] = float(base_pose[0] - x_offset_arm_base)

    def _command_navigation_target(self, waypoints: list[list[float]]) -> np.ndarray | None:
        if not waypoints:
            return None
        return np.asarray(waypoints[-1], dtype=np.float32)

    def _same_navigation_target(self, target: np.ndarray) -> bool:
        return bool(np.allclose(np.asarray(target, dtype=np.float32), self.current_navigation_target, atol=1e-4))
