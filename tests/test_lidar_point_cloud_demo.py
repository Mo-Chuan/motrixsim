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

from sensors.lidar_point_cloud_demo import (
    ROTATING_LIDAR_BODY_NAME,
    ROTATING_LIDAR_SENSOR_NAME,
    SOLID_STATE_LIDAR_SENSOR_NAME,
    build_lidar_debug_world,
    print_lidar_stats,
)
from utils.lidar_bev import BEV_EXTENT, BEV_SIZE, LidarBevPreview, lidar_bev_image

from motrixsim import SceneData


def test_lidar_mount_body_uses_geometry_inertial_mass():
    world = build_lidar_debug_world()
    body = next(body for body in world.hierarchy.bodies if body.link.name == ROTATING_LIDAR_BODY_NAME)

    assert len(body.link.geoms) == 1
    assert body.link.geoms[0].inertial.mass == pytest.approx(0.01)


@pytest.mark.parametrize("stride", [4, 8])
def test_lidar_stats_use_stored_distance_including_world_origin(stride, capsys):
    values = np.zeros((3, stride), dtype=np.float32)
    # Zero world point is a real hit. Miss point data must not influence validity.
    values[:, 0] = [2.0, -1.0, 5.0]
    if stride > 1:
        values[1, 1:4] = 100.0
    if stride == 8:
        values[:, 4] = [0.01, 0.09, 0.05]
        values[0, 5:8] = [0, 0, 1]
    model = SimpleNamespace(
        get_lidar_status=lambda name, data: SimpleNamespace(ready=np.array(True), scan_seq=np.array(7)),
        get_sensor_value=lambda name, data: values.ravel(),
    )
    name = ROTATING_LIDAR_SENSOR_NAME if stride == 8 else SOLID_STATE_LIDAR_SENSOR_NAME
    assert print_lidar_stats(model, None, name) == 7

    output = capsys.readouterr().out
    assert "hits=2/3, min=2.000, max=5.000" in output
    assert "first_world_point=[0.0, 0.0, 0.0]" in output
    if stride == 8:
        assert "acquisition=[0.0100, 0.0900]s" in output
        assert "first_world_normal=[0.0, 0.0, 1.0]" in output


def test_lidar_stats_without_hits(capsys):
    model = SimpleNamespace(
        get_lidar_status=lambda name, data: SimpleNamespace(ready=np.array(True), scan_seq=np.array(1)),
        get_sensor_value=lambda name, data: np.array([-1.0, 0.0, 0.0, 0.0]),
    )

    print_lidar_stats(model, None, "test")

    assert "hits=0/1, min=nan, max=nan" in capsys.readouterr().out


@pytest.mark.parametrize("ready,seq,previous", [(False, 0, 0), (True, 7, 7)])
def test_lidar_stats_skip_unpublished_and_repeated_frames(ready, seq, previous, capsys):
    def unexpected_payload_read(name, data):
        pytest.fail("unchanged/unpublished scans must not read payload")

    model = SimpleNamespace(
        get_lidar_status=lambda name, data: SimpleNamespace(ready=np.array(ready), scan_seq=np.array(seq)),
        get_sensor_value=unexpected_payload_read,
    )
    assert print_lidar_stats(model, None, ROTATING_LIDAR_SENSOR_NAME, previous) == previous
    assert not capsys.readouterr().out


def test_example_full_scan_fields_headless(capsys):
    model = build_lidar_debug_world().build()
    data = SceneData(model)
    assert print_lidar_stats(model, data, ROTATING_LIDAR_SENSOR_NAME) == 0
    assert not capsys.readouterr().out
    for _ in range(int(np.ceil(0.11 / model.options.timestep))):
        model.step(data)
    seq = print_lidar_stats(model, data, ROTATING_LIDAR_SENSOR_NAME)
    assert seq >= 1
    output = capsys.readouterr().out
    assert "acquisition=" in output
    assert "first_world_normal=" in output
    beams = model.get_sensor_value(ROTATING_LIDAR_SENSOR_NAME, data).reshape(-1, 8)
    assert np.ptp(beams[:, 4]) > 0
    hits = beams[:, 0] >= 0
    np.testing.assert_allclose(np.linalg.norm(beams[hits, 5:8], axis=1), 1, atol=1e-5)
    assert print_lidar_stats(model, data, ROTATING_LIDAR_SENSOR_NAME, seq) == seq
    assert not capsys.readouterr().out
    assert print_lidar_stats(model, data, SOLID_STATE_LIDAR_SENSOR_NAME) > 0
    assert model.get_sensor_value(SOLID_STATE_LIDAR_SENSOR_NAME, data).size == 64 * 32 * 4


@pytest.mark.parametrize(
    "mode,expected", [("Distance", [127, 255, 127]), ("Height", [255, 0, 0]), ("World normal", [255, 127, 127])]
)
def test_bev_projection_and_highest_return(mode, expected):
    beams = np.array(
        [[3, 2, 1, 0.1, 0.02, 0, 0, 1], [3, 2, 1, 1.5, 0.05, 1, 0, 0], [-1, -2, -1, 1, 0.08, 0, 0, 1]], dtype=np.float32
    )
    image = lidar_bev_image(beams.ravel(), mode)
    baseline = lidar_bev_image(np.empty(0), mode)
    assert image.shape == (BEV_SIZE, BEV_SIZE, 3) and image.dtype == np.uint8 and image.flags.c_contiguous
    c = (BEV_SIZE - 1) / 2
    scale = (BEV_SIZE - 1) / (2 * BEV_EXTENT)
    np.testing.assert_array_equal(image[round(c - scale), round(c + 2 * scale)], expected)
    np.testing.assert_array_equal(
        image[round(c + scale), round(c - 2 * scale)], baseline[round(c + scale), round(c - 2 * scale)]
    )


def test_bev_ground_dimming_and_invalid_mode():
    beams = np.array([[0, 0, 0, 0, 0, 0, 0, 1], [2, 2, 1, 0, 0, 0, 0, 1]], dtype=np.float32)
    image = lidar_bev_image(beams.ravel(), "World normal")
    c = (BEV_SIZE - 1) / 2
    scale = (BEV_SIZE - 1) / (2 * BEV_EXTENT)
    np.testing.assert_array_equal(image[round(c - scale), round(c + 2 * scale)], [38, 38, 76])
    with pytest.raises(ValueError):
        lidar_bev_image(beams.ravel(), "unsupported")


def test_bev_new_frames_and_cached_recoloring():
    status = SimpleNamespace(ready=np.array(False), scan_seq=np.array(0))
    reads = []
    model = SimpleNamespace(
        get_lidar_status=lambda name, data: status,
        get_sensor_value=lambda name, data: reads.append(name) or np.array([2, 2, 1, 1, 0.01, 0, 0, 1]),
    )
    image = SimpleNamespace(pixels=None)
    preview = LidarBevPreview()
    assert not preview.update(model, None, image, "Distance")
    assert not reads and image.pixels is None
    status.ready = np.array(True)
    status.scan_seq = np.array(1)
    assert preview.update(model, None, image, "Distance")
    assert len(reads) == 1
    first = image.pixels
    assert not preview.update(model, None, image, "Distance") and image.pixels is first
    assert not preview.update(model, None, image, "Height")
    assert image.pixels is not first and len(reads) == 1
    assert "FullScan #1" in preview.status and "acquisition=" in preview.status


def test_bev_translation_yaw_and_height_reference():
    beams = np.array([[3, 4, 7, 1.5, 0.01, 1, 0, 0]], dtype=np.float32)
    # Relative world XY=(0,2), yaw=90 degrees -> forward/local XY=(2,0).
    image = lidar_bev_image(beams.ravel(), "Height", origin=(4, 5, 0), yaw=np.pi / 2)
    c = (BEV_SIZE - 1) / 2
    scale = (BEV_SIZE - 1) / (2 * BEV_EXTENT)
    np.testing.assert_array_equal(image[round(c), round(c + 2 * scale)], [255, 0, 0])


def test_bev_motion_reprojects_cached_copy_without_payload_read():
    values = np.array([2, 2, 1, 1, 0.01, 0, 0, 1], dtype=np.float32)
    reads = []
    model = SimpleNamespace(
        get_lidar_status=lambda name, data: SimpleNamespace(ready=np.array(True), scan_seq=np.array(1)),
        get_sensor_value=lambda name, data: reads.append(name) or values,
    )
    preview = LidarBevPreview("go2_lidar")
    image = SimpleNamespace(pixels=None)
    preview.update(model, None, image, "Distance")
    first = image.pixels.copy()
    values[:] = 0  # Stored frame must not alias live sensor data.
    assert not preview.update(model, None, image, "Distance", origin=(1, 0, 0))
    assert reads == ["go2_lidar"]
    assert not np.array_equal(first, image.pixels)
    assert preview.values[0] == 2


@pytest.mark.parametrize("num_envs", [1, 3])
def test_benchmark_validates_distance_point_width(num_envs):
    from bench.lidar import BenchmarkCase, validate_lidars

    case = BenchmarkCase("test", "test", num_lidars=2, num_envs=num_envs)
    model = SimpleNamespace(get_sensor_value=lambda name, data: np.zeros(num_envs * 5 * 4))
    validate_lidars(model, None, case, raycasts_per_lidar=5)

    model.get_sensor_value = lambda name, data: np.zeros(num_envs * 5 * 3)
    with pytest.raises(RuntimeError, match=r"distance \+ point"):
        validate_lidars(model, None, case, raycasts_per_lidar=5)
