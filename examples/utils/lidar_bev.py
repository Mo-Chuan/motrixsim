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

import numpy as np


def scan_heatmap(values: np.ndarray) -> np.ndarray:
    """Map normalized scalars to blue (low), green, red (high)."""
    values = np.clip(values, 0.0, 1.0)
    return (255 * np.stack([values, 1 - np.abs(2 * values - 1), 1 - values], axis=-1)).astype(np.uint8)


BEV_SIZE = 384
BEV_EXTENT = 6.5
BEV_MODES = ("Distance", "Height", "World normal")


def lidar_bev_image(values: np.ndarray, mode: str, *, origin=(0.0, 0.0, 0.0), yaw=0.0) -> np.ndarray:
    """Project full-field stride-8 LiDAR hits onto XY; +X right, +Y up.

    The input must include distance, point, time_offset, and normal fields in that order.
    """
    if mode not in BEV_MODES:
        raise ValueError(f"unknown BEV color mode: {mode}")
    image = np.full((BEV_SIZE, BEV_SIZE, 3), [15, 20, 28], dtype=np.uint8)
    scale = (BEV_SIZE - 1) / (2 * BEV_EXTENT)
    center = (BEV_SIZE - 1) / 2
    rows, cols = np.indices((BEV_SIZE, BEV_SIZE))
    radius = np.hypot(cols - center, rows - center)
    for distance in (1, 3, 6):
        image[np.abs(radius - distance * scale) < 0.6] = [65, 76, 90]
    image[:, BEV_SIZE // 2] = [35, 45, 58]
    image[BEV_SIZE // 2, :] = [35, 45, 58]

    beams = values.reshape(-1, 8)
    hit = (beams[:, 0] >= 0) & np.isfinite(beams).all(axis=1)
    beams = beams[hit]
    points = beams[:, 1:4] - np.asarray(origin)
    c_yaw, s_yaw = np.cos(yaw), np.sin(yaw)
    points[:, :2] = points[:, :2] @ np.array([[c_yaw, -s_yaw], [s_yaw, c_yaw]])
    inside = (np.abs(points[:, :2]) <= BEV_EXTENT).all(axis=1)
    beams, points = beams[inside], points[inside]
    if len(beams):
        if mode == "Distance":
            colors = scan_heatmap(beams[:, 0] / 6.0)
        elif mode == "Height":
            colors = scan_heatmap((points[:, 2] + 0.2) / 1.7)
        else:
            colors = (255 * np.clip((beams[:, 5:8] + 1) / 2, 0, 1)).astype(np.uint8)
        # Ground-like upward surfaces are dimmed, not treated as free space.
        ground = (beams[:, 7] > 0.8) & (points[:, 2] < 0.3)
        colors[ground] = (colors[ground].astype(np.float32) * 0.3).astype(np.uint8)
        x = np.rint(center + points[:, 0] * scale).astype(int)
        y = np.rint(center - points[:, 1] * scale).astype(int)
        # Sort low to high, then explicitly select the final return in each cell.
        order = np.argsort(points[:, 2], kind="stable")
        cells = y[order] * BEV_SIZE + x[order]
        _, reverse_indices = np.unique(cells[::-1], return_index=True)
        selected = order[len(order) - 1 - reverse_indices]
        image[y[selected], x[selected]] = colors[selected]
    # Green center marker and +X forward arrow (mount orientation is fixed in this example).
    c = BEV_SIZE // 2
    image[c - 3 : c + 4, c - 3 : c + 4] = [0, 255, 90]
    image[c - 1 : c + 2, c + 4 : c + 18] = [0, 255, 90]
    for delta in range(6):
        image[c - delta, c + 18 - delta] = [0, 255, 90]
        image[c + delta, c + 18 - delta] = [0, 255, 90]
    return image


class LidarBevPreview:
    """Retain one image and the latest published FullScan for color-mode changes."""

    def __init__(self, sensor_name="rotating_lidar"):
        self.sensor_name = sensor_name
        self.frame = None
        self.scan_seq = 0
        self.mode = "Distance"
        self.values = None
        self.status = "Waiting for first FullScan"

    def update(self, model, data, image, mode: str, *, origin=(0.0, 0.0, 0.0), yaw=0.0) -> bool:
        status = model.get_lidar_status(self.sensor_name, data)
        seq = int(status.scan_seq.item())
        new_scan = bool(status.ready.item()) and seq != self.scan_seq
        if new_scan:
            self.values = model.get_sensor_value(self.sensor_name, data).copy()
            self.scan_seq = seq
            beams = self.values.reshape(-1, 8)
            self.status = (
                f"FullScan #{seq} | hits={np.count_nonzero(beams[:, 0] >= 0)}/{len(beams)} | "
                f"acquisition={beams[:, 4].min():.3f}..{beams[:, 4].max():.3f}s from scan start"
            )
        frame = (*origin, yaw)
        changed = new_scan or mode != self.mode or (self.frame is not None and frame != self.frame)
        self.frame = frame
        if changed:
            image.pixels = lidar_bev_image(
                self.values if self.values is not None else np.empty(0), mode, origin=origin, yaw=yaw
            )
            self.mode = mode
        return new_scan
