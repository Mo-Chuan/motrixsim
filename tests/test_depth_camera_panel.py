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

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "examples"))

from control.robot_locomotion import (
    GO2_DEPTH_HEIGHT,
    GO2_DEPTH_SENSOR,
    GO2_DEPTH_WIDTH,
    attach_go2_depth_camera,
    create_argument_parser,
)
from utils.depth_camera_panel import DepthCameraPanel, depth_camera_image
from utils.robot import Go2Robot

from motrixsim import SceneData, msd


def test_depth_display_misses_range_and_orientation():
    pixels = depth_camera_image(np.array([[-1, 0, 3, 6, np.inf]]))
    assert pixels.shape == (1, 5, 4)
    np.testing.assert_array_equal(pixels[0, 0], [24, 40, 80, 255])
    np.testing.assert_array_equal(pixels[0, 1], [255, 255, 255, 255])
    np.testing.assert_array_equal(pixels[0, 3], [0, 0, 0, 255])
    np.testing.assert_array_equal(pixels[0, 4], pixels[0, 0])
    with pytest.raises(ValueError):
        depth_camera_image(np.zeros((2, 2)), 0)


def test_go2_depth_mount_and_headless_read():
    assert create_argument_parser().parse_args(["--depth-camera"]).depth_camera
    world = msd.from_file(Go2Robot.mjcf_path)
    attach_go2_depth_camera(world)
    model = world.build()
    data = SceneData(model)
    model.step(data)
    values = model.get_sensor_value(GO2_DEPTH_SENSOR, data)
    assert values.shape == (GO2_DEPTH_WIDTH * GO2_DEPTH_HEIGHT,)
    assert np.isfinite(values).all()
    # The bare robot asset has no environment surface in front of the camera.
    assert np.all((values == -1) | (values >= 0))


def test_panel_reuses_image_and_preallocated_payload():
    panel = DepthCameraPanel("depth", 2, 1)
    images = []

    def create_image(pixels):
        image = SimpleNamespace(pixels=pixels)
        images.append(image)
        return image

    render = SimpleNamespace(create_image=create_image)
    assigned = []
    panel.slot = SimpleNamespace(set_image=assigned.append)
    panel.bind(render)
    assert assigned == images
    payload = np.array([-1, -1])
    reads = []

    def read(name, data, out):
        assert name == "depth"
        reads.append(out)
        out[:] = payload
        return out

    model = SimpleNamespace(get_sensor_value=read)
    panel.update(render, model, None)
    np.testing.assert_array_equal(images[0].pixels, depth_camera_image(payload.reshape(1, 2)))
    payload[:] = [0, 3]
    panel.update(render, model, None)
    panel.update(render, model, None)
    assert len(images) == 1
    assert len(reads) == 3 and all(out is panel.values for out in reads)
    np.testing.assert_array_equal(images[0].pixels, depth_camera_image(np.array([[0, 3]])))
