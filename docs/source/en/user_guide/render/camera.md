# 📷 Camera

## System Camera

When you use the renderer feature, MotrixSim automatically creates a system camera. The system camera accepts user mouse operations to enable movement, zooming, and other functions.

```{video} /_static/videos/store_motrixsim.mp4
:poster: /_static/images/poster/store_motrixsim.jpg
:caption: System camera operation effects
:nocontrols:
:autoplay:
:playsinline:
:muted:
:loop:
:width: 100%
```

Operation Instructions:

-   Press and drag left mouse button: Rotate camera around focal point
-   Press and drag right mouse button: Move focal point (red circle indicates current focal point)
-   Mouse wheel: Zoom in/out (cannot zoom in further when reaching focal point)

The system camera supports partial configurations from [mjcf/visual/global](https://mujoco.readthedocs.io/en/stable/XMLreference.html#visual-global), such as:

-   orthographic: Whether to use orthographic projection
-   fovy: Camera field of view
-   azimuth: Initial azimuth angle (rotation around z-axis) of system camera
-   elevation: Initial elevation angle (pitch) of system camera

The system camera can be enabled/disabled.

```
system_camera = render.system_camera # get system camera
system_camera.active = True # enable camera
system_camera.active = False # disable camera
```

After `render.launch(...)`, you can set the system camera view with orbit/look-at
parameters. This controls the renderer's system camera, not a model-defined MJCF
camera. `azimuth` and `elevation` are in degrees.

```python
render.system_camera.set_view(
    lookat=[0.0, 0.0, 0.75],
    distance=6.0,
    elevation=-20.0,
    azimuth=90.0,
)
```

### Near/Far Planes and Output Modes

The system camera is configured on the MSD layer before `build()` through
`world.system_camera`; the built model is immutable. Explicit clipping planes must be
configured in pairs and satisfy `znear > 0` and `zfar > znear`; by default both planes
are derived automatically from the scene extent.

```python
sys_cam.znear = near
sys_cam.zfar = far
```

The system camera also supports depth-only and segmentation output, following the
same semantics as scene cameras. The mode takes effect on the `"image"` render
target used for offscreen capture; a windowed system camera starts in the
configured mode and can still be switched at runtime from the viewport toolbar.

```python
if mode == "depth":
    sys_cam.depth_only = True
elif mode == "seg-colorful":
    sys_cam.segmentation = mx.msd.Segmentation.Colorful
elif mode == "seg-id":
    sys_cam.segmentation = mx.msd.Segmentation.Id
```

## Scene Camera

In addition to the system camera, you can configure additional Camera tags in MJCF files. We refer to these as scene cameras. Scene cameras can provide additional viewing angles for visualization.

### Custom Additional Cameras

Taking [go1.xml](../../../../examples/assets/go1/go1_mjx_fullcollisions.xml) as an example, we define cameras in MJCF as follows:

```xml
<body name="trunk" pos="0 0 0.445" childclass="go1">
    <camera name="track" pos="0.846 -1.3 0.316" xyaxes="0.866 0.500 0.000 -0.171 0.296 0.940" mode="track"/>
    <camera name="top" pos="-1 0 1" xyaxes="0 -1 0 0.7 0 0.7" mode="track"/>
    <camera name="side" pos="0 -2 1" xyaxes="1 0 0 0 1 2" mode="track"/>
    <camera name="back" pos="-2.4 0 0.8" target="trunk" mode="targetbody"/>
</body>
```

This means we've defined 4 additional cameras on the go1's trunk. The camera position and orientation are specified by pos and xyaxes, while mode specifies the camera's movement mode.

Similarly, the scene camera can also be enabled/disabled.

```
camera_index = 0 # The first scene camera index
scene_camera = render.get_camera(camera_index) # get scene camera
scene_camera.active = True # enable camera
scene_camera.active = False # disable camera
```

### Switching a viewport camera

Each `ui.Viewport` has its own camera binding. Declare the initial camera in the layout blueprint; use `ui.CameraRef.system()` for the system camera or `ui.CameraRef(camera.index)` for a scene camera. The following snippet assumes that `model` and its `SceneData` instance `data` have already been created and that the model contains the Go1 cameras shown above.

```python
from motrixsim import ui
from motrixsim.render import RenderApp

view = ui.Viewport(camera=ui.CameraRef(model.cameras["back"].index), title="Robot")
with RenderApp(blueprint=view) as render:
    render.launch(model)
    render.set_viewport_camera(view, model.cameras["side"])
    render.sync(data)
```

Use the same `view` object that was included in this RenderApp's blueprint. `set_viewport_camera(view, camera)` changes that specific viewport; a hidden, live viewport can also be changed. An invalid or closed target is not redirected to another viewport.

For keyboard shortcuts, use `render.set_active_viewport_camera(camera)` to change the viewport that last received input. Click the intended viewport before pressing the shortcut. The target is resolved when the renderer executes the queued command; an unavailable target produces an Editor log error instead of selecting another viewport.

Both setters accept `None` to select the system camera. Continue calling `render.sync(data)` in the render loop to deliver queued changes. Use blueprint bindings for startup rather than relying on a prior viewport interaction.

| Task | API |
| --- | --- |
| Choose the initial camera | `ui.Viewport(camera=ui.CameraRef(...))` |
| Change a specific viewport | `render.set_viewport_camera(view, camera)` |
| Change the last interacted viewport | `render.set_active_viewport_camera(camera)` |
| Return to the system camera | Pass `None` to either setter |

The complete [two-viewport example](../../../../examples/viewer/viewport_camera_targets.py) demonstrates both switching methods. Camera overlays remain attached to their target camera; see [CameraViewport](camera_viewport.md).

```python
cameras = model.cameras # Get all cameras (note: system camera not included here)
preview_cameras = [None,*cameras] # None indicates system camera
```

Switch cameras using keyboard events:

```{literalinclude} ../../../../examples/control/go1.py
:language: python
:dedent:
:start-after: "# tag: switch camera"
:end-before:  "# endtag"

```

```{video} /_static/videos/switch_camera.mp4
:caption: Switch the interactive viewport camera using left/right arrow keys
:nocontrols:
:autoplay:
:playsinline:
:muted:
:loop:
:width: 100%
```

### RGBD Sensors

MotrixSim supports converting scene cameras into RGBD vision sensors, allowing you to obtain both RGB images and depth images. You can use the following interface to render a scene camera to an offline image:

```{literalinclude} ../../../../examples/control/go1.py
:language: python
:dedent:
:start-after: "# tag: camera render target"
:end-before:  "# endtag"

```

By default, the camera uses RGB rendering mode. If you want the camera to only render depth images, you can set the camera's depth_only property to True:

```{literalinclude} ../../../../examples/control/go1.py
:language: python
:dedent:
:start-after: "# tag: depth camera"
:end-before:  "# endtag"

```

```{note}
The values stored in the depth map are in NDC (normalized device coordinates) space. If you want to convert them to actual distances from the camera in 3D space, you can use the following formula:

view_z = camera.near_plane / depth

```

The real-time effects of the RGBD camera will be displayed in the Camera panel on the left:

```{video} /_static/videos/rgbd_camera.mp4
:caption: The top-right corner displays real-time RGB and depth camera effects
:nocontrols:
:autoplay:
:playsinline:
:muted:
:loop:
:width: 100%
```

### Segmentation

MotrixSim supports converting scene cameras into Segmentation vision sensors, allowing you to obtain a semantic segmentation map of geometric objects in the scene. Similarly, we need to configure the camera's output image:

```{literalinclude} ../../../../examples/viewer/capture_segmentation.py
:language: python
:dedent:
:start-after: "# tag: camera render target"
:end-before:  "# endtag"

```

By default, the camera uses RGB rendering mode. If you want the camera to render only the segmentation map, you can set the camera's segmentation property to colorful:

```{literalinclude} ../../../../examples/viewer/capture_segmentation.py
:language: python
:dedent:
:start-after: "# tag: segmentation camera"
:end-before:  "# endtag"

```

The output image in this mode is similar to:

```{figure} /_static/images/capture_segmentation.png
:width: 100%
```

If you want to obtain the ID image of the geometric objects, you can set the camera's segmentation property to id:

```{literalinclude} ../../../../examples/viewer/capture_segmentation.py
:language: python
:dedent:
:start-after: "# tag: id color camera"
:end-before:  "# endtag"

```

The output image in this mode is similar to:

```{figure} /_static/images/capture_id_color.png
:width: 100%
```

It is an R single-channel image.

To distinguish it from the background during rendering, the value 0 is used as the background. Therefore, for a geometric object with geom_id = 0, its rendered id is 1, that is, id = geom_id + 1.

```{note}

To output images in a special format, you must set the camera's render target to an image before building the model, for example `camera.render_target = msd.RenderTarget.Image` together with `camera.resolution = [320, 240]`.

The current output image format is R8Unorm rather than an sRGB format, so the image data is stored as exact values.

```

### Reading Camera Image Data

To read camera image data, you need to obtain the RenderCamera object from RenderApp and then perform a capture operation.

```{literalinclude} ../../../../examples/control/go1.py
:language: python
:dedent:
:start-after: "# tag: camera capture"
:end-before:  "# endtag"
```

Note that our capture operation is asynchronous, so the [`RenderCamera.capture`] method will return a CaptureTask object. You need to keep this CaptureTask object and continuously check its status until it is complete. Once finished, you can obtain the image data through the task.take_image() method.

You can refer to the API documentation for more details:

-   [`RenderCamera`]
-   [`CaptureTask`]
-   [`Image`]

[`RenderCamera`]: motrixsim.render.RenderCamera
[`CaptureTask`]: motrixsim.render.CaptureTask
[`Image`]: motrixsim.render.Image
[`RenderCamera.capture`]: motrixsim.render.RenderCamera.capture
