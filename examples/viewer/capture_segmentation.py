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

from motrixsim import SceneData, msd, run, step, ui
from motrixsim.render import Layout, RenderApp

# Camera slots built into the scene: index 0 renders colorful, index 1 renders id
MODE_CAMERAS = {"colorful": 0, "id": 1}


def lookat_quat(pos, target, up=(0.0, 0.0, 1.0)):
    """Quaternion (x, y, z, w) orienting a -Z-forward camera from pos toward target."""
    pos, target, up = np.array(pos, float), np.array(target, float), np.array(up, float)
    forward = target - pos
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    rotation = np.stack([right, up, -forward], axis=1)
    trace = np.trace(rotation)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        return np.array([
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        ])
    i = int(np.argmax(np.diag(rotation)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + rotation[i, i] - rotation[j, j] - rotation[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[3] = (rotation[k, j] - rotation[j, k]) / s
    q[j] = (rotation[j, i] + rotation[i, j]) / s
    q[k] = (rotation[k, i] + rotation[i, k]) / s
    return q


CAMERA_POS = [1.2, -1.2, 0.7]
CAMERA_TARGET = [0.0, 0.0, 0.2]


def build_scene():
    """Build the go1 scene with two identical cameras in different segmentation modes."""
    world = msd.from_file("examples/assets/go1/scene.xml")
    orientation = list(lookat_quat(CAMERA_POS, CAMERA_TARGET))
    # tag: camera render target
    camera_kwargs = dict(
        position=CAMERA_POS,
        orientation=orientation,
        resolution=[320, 240],
        render_target=msd.RenderTarget.Image,
    )
    # endtag

    # tag: segmentation camera
    colorful_cam = msd.Camera(name="seg_colorful", **camera_kwargs)
    colorful_cam.segmentation = msd.Segmentation.Colorful
    world.hierarchy.cameras.append(colorful_cam)
    # endtag

    # Segmentation is configured per camera before build(), so a second camera
    # with the id mode is added alongside instead of rebuilding the scene.
    # tag: id color camera
    id_cam = msd.Camera(name="seg_id", **camera_kwargs)
    id_cam.segmentation = msd.Segmentation.Id
    world.hierarchy.cameras.append(id_cam)
    # endtag
    return world.build()


# Bright, easily distinguishable colors for id display
PALETTE = np.array(
    [
        [0, 0, 0],  # unused (background is forced to black)
        [230, 57, 70],
        [66, 135, 245],
        [42, 157, 143],
        [255, 183, 3],
        [156, 76, 219],
        [255, 127, 80],
        [0, 191, 99],
    ],
    dtype=np.uint8,
)


def to_display_pixels(pixels):
    """Convert a captured frame to displayable RGB.

    Colorful captures are RGBA. Id captures hold small per-geom indices
    (id = geom_id + 1, 0 = background), which would look almost black if
    shown raw, so each distinct id is mapped to a distinct palette color.
    """
    if pixels.ndim == 2 or pixels.shape[-1] == 1:
        ids = pixels.astype(np.int64).squeeze()
        rgb = PALETTE[ids % len(PALETTE)]
        rgb[ids == 0] = 0  # keep the background black
        return rgb.astype(np.uint8)
    if pixels.shape[-1] == 4:
        pixels = pixels[..., :3]
    return np.ascontiguousarray(pixels)


def main():
    # Panel for switching the segmentation mode at runtime; each mode binds to
    # its own pre-built camera, so switching never rebuilds the scene.
    with ui.View(title="Segmentation") as panel:
        mode_state = ui.RadioGroup("Mode", options=list(MODE_CAMERAS), value="colorful")
    blueprint = ui.Horizontal(
        [
            ui.Viewport(camera=ui.CameraRef.system(), title="Scene"),
            panel,
        ],
        shares=[4, 1],  # keep the control panel compact
    )

    # Create render window for visualization
    with RenderApp(blueprint=blueprint) as render:
        model = build_scene()
        render.launch(model)
        data = SceneData(model)
        # Settle the robot into its authored "home" pose (default spawn is prone)
        if model.keyframes:
            model.keyframes[0].apply(data)

        # Display the captured image as an on-screen widget instead of saving it to disk
        display = None
        capture_interval = 30
        frame = 0

        def phys_step():
            step(model, data)

        def render_step():
            nonlocal display, frame
            frame += 1
            if frame % capture_interval == 0:
                camera_index = MODE_CAMERAS[mode_state.value]
                task = render.get_camera(camera_index).capture()
                render.sync(data, wait=True)
                image = task.take_image()
                assert image is not None
                if display is None:
                    display = render.create_image(to_display_pixels(image.pixels))
                    render.widgets.create_image_widget(
                        display,
                        layout=Layout(right=10, top=10, width=320, height=240),
                        target_camera=model.cameras.system_camera,
                    )
                else:
                    display.pixels = to_display_pixels(image.pixels)
            else:
                render.sync(data)

        run.render_loop(model.options.timestep, 60, phys_step, render_step)


if __name__ == "__main__":
    main()
