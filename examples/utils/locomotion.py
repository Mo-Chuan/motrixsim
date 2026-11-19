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

"""Shared physics scheduling and episode data ownership for locomotion examples."""

from motrixsim import SceneData


class LocomotionSimulation:
    """Advance physics with optional policy initialization and control-start hooks."""

    def __init__(self, model, policy, control_dt: float = 0.02):
        self.model = model
        self.policy = policy
        self.n_ctrl = max(1, round(control_dt / model.options.timestep))
        self.step_count = 0
        self.data = SceneData(model)
        if hasattr(policy, "initialize"):
            policy.initialize(self.data)

    def reset(self) -> None:
        """Create episode data and restore policy-specific state before the next step."""
        self.data = SceneData(self.model)
        if hasattr(self.policy, "initialize"):
            self.policy.initialize(self.data)
        elif hasattr(self.policy, "reset"):
            self.policy.reset()
        self.step_count = 0

    def step(self, command) -> None:
        """Advance one physics tick, preserving the existing end-of-interval policy step."""
        if self.step_count % self.n_ctrl == 0 and hasattr(self.policy, "pre_control_step"):
            self.policy.pre_control_step(self.data, command)
        if hasattr(self.policy, "pre_physics_step"):
            self.policy.pre_physics_step(self.data)
        self.model.step(self.data)
        self.step_count += 1
        if self.step_count % self.n_ctrl == 0 and self.policy.step(self.data, command):
            self.reset()
