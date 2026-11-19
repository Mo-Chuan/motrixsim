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

import numpy as np

from motrixsim import real_dtype, ui


def depth_camera_image(depth, max_depth=6.0):
    """Display-only grayscale: near is bright, far is dark, misses are blue."""
    depth = np.asarray(depth)
    if depth.ndim != 2 or not np.isfinite(max_depth) or max_depth <= 0:
        raise ValueError("expected a 2D depth image and a finite positive display range")
    valid = np.isfinite(depth) & (depth >= 0)
    intensity = np.zeros(depth.shape, dtype=np.uint8)
    intensity[valid] = (255 * (1 - np.clip(depth[valid] / max_depth, 0, 1))).astype(np.uint8)
    pixels = np.empty((*depth.shape, 4), dtype=np.uint8)
    pixels[..., :3] = intensity[..., None]
    pixels[~valid, :3] = (24, 40, 80)
    pixels[..., 3] = 255
    return pixels


class DepthCameraPanel:
    """Single-instance camera frame preview; the sensor itself is renderer independent."""

    def __init__(self, sensor_name, width, height, max_depth=6.0):
        self.sensor_name = sensor_name
        self.shape = (height, width)
        self.max_depth = max_depth
        self.values = np.empty(width * height, dtype=real_dtype())
        self.image = None
        with ui.View(title="Go2 physics depth camera") as self.view:
            with ui.Column():
                ui.Text("Optical-axis depth | near bright, far dark | blue = miss")
                self.slot = ui.Image(width=384, height=216)
                ui.Text(f"Display range: 0–{max_depth:g} m | headless collision raycast")

    def bind(self, render):
        self.image = render.create_image(depth_camera_image(np.full(self.shape, -1.0), self.max_depth))
        self.slot.set_image(self.image)

    def update(self, render, model, data):
        model.get_sensor_value(self.sensor_name, data, out=self.values)
        self.image.pixels = depth_camera_image(self.values.reshape(self.shape), self.max_depth)
