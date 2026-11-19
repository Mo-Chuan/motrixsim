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

from motrixsim import ui
from utils.lidar_bev import BEV_MODES, LidarBevPreview, lidar_bev_image


class LidarBevPanel:
    """Reusable single-instance UI; an optional site/link pair supplies the moving yaw frame."""

    def __init__(self, sensor_name, *, site=None, base_link=None):
        self.site = site
        self.base_link = base_link
        self.preview = LidarBevPreview(sensor_name)
        self.image = None
        with ui.View(title="LiDAR bird's-eye view") as self.view:
            with ui.Column():
                ui.Text("XY hits | forward/+X right, left/+Y up | green arrow = forward")
                self.mode = ui.Select("Color", BEV_MODES, value="Distance")
                ui.Text("Distance: blue 0 / green 3 / red 6 world units (clamped)")
                ui.Text("Height: blue -0.2 / green 0.65 / red 1.5 relative to the view origin")
                ui.Text("World normal: RGB=(normal+1)/2; low upward surfaces dimmed")
                self.slot = ui.Image(width=384, height=384)
                ui.Text("Rings 1/3/6 | highest hit per pixel | not an occupancy map")
                self.status_frame = ui.Frame(build_fn=lambda: ui.Text(self.preview.status))
                ui.Text("Time offsets include misses and may repeat across scans.")
                if site is not None:
                    ui.Text("Current mount-centered, base-yaw aligned; historical hits, no motion compensation.")

    def bind(self, render):
        self.image = render.create_image(lidar_bev_image(np.empty(0), "Distance"))
        self.slot.set_image(self.image)

    def update(self, render, model, data):
        origin = (0.0, 0.0, 0.0)
        yaw = 0.0
        if self.site is not None:
            origin = self.site.get_pose(data)[:3]
            qx, qy, qz, qw = self.base_link.get_pose(data)[3:]
            yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        if self.preview.update(model, data, self.image, self.mode.value, origin=origin, yaw=yaw):
            render.rebuild(self.status_frame)
