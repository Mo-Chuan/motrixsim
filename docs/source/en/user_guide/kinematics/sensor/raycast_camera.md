# Raycast Camera

{bdg-success-line}`CPU: fully supported` {bdg-danger-line}`GPU: not supported`

`RaycastCamera` queries collision geometry with rays to produce depth, hit points, and surface normals. It does not require a renderer and supports headless and batched simulation.

The sensor supports two projections: pinhole rays share one optical origin; orthographic rays are parallel and start on a regular grid, useful for terrain or height sampling. Both projections share range, output fields, frame updates, and collision filtering.

## Configure a pinhole camera

Add the following configuration inside the MJCF `<mujoco>` element:

```xml
<worldbody>
  <site name="camera_mount" pos="0 0 0.5"/>
  <geom type="box" pos="0 0 2" size="2 2 0.1"/>
</worldbody>
<sensor>
  <raycastcamera name="camera" site="camera_mount" projection="pinhole"
    width="48" height="27" intrinsics="34 34 24 13.5"
    near="0.05" far="10" update_period="0.03333333333333333"
    exclude="parentbody" returns="depth"/>
</sensor>
```

`raycastcamera` is a MotrixSim MJCF extension. Select the projection with `projection="pinhole"` or `projection="orthographic"`.

## Coordinate frame and pinhole rays

The camera uses the mounting site's complete position and rotation. The optical frame is right-handed: +X points to image right, +Y to image down, and +Z forward.

Pixels are row-major, with index `v * width + u` and pixel center `(u + 0.5, v + 0.5)`. Each pinhole ray starts at the optical origin and has the unnormalized local direction:

```text
[(u+0.5-cx)/fx, (v+0.5-cy)/fy, 1]
```

The ray query uses the normalized direction. `intrinsics="fx fy cx cy"` uses pixel units: `fx` and `fy` must be positive and finite, while `cx` and `cy` must be finite. `width` and `height` must be positive integers.

## Orthographic projection

### Configure downward sampling

Add the following configuration inside the MJCF `<mujoco>` element:

```xml
<worldbody>
  <site name="scanner_mount" pos="0 0 3" quat="0 1 0 0"/>
  <geom type="box" pos="0 0 2" size="2 2 0.1"/>
</worldbody>
<sensor>
  <raycastcamera name="height" site="scanner_mount" projection="orthographic"
    width="8" height="5" spacing="0.4 0.4" near="0" far="8"
    update_period="0" exclude="parentbody"
    returns="depth point normal"/>
</sensor>
```

`projection="orthographic"` requires `spacing="sx sy"` and cannot be combined with pinhole `intrinsics`. `width` and `height` must be positive integers, and both spacing values must be positive and finite.

This example rotates the mounting site 180° about X, pointing the camera's local +Z downward in world coordinates. The camera uses the site's complete position and rotation; change the site's orientation to change the sampling direction.

### Grid layout

Pixel `(u, v)` has its ray origin on the camera's local Z=0 plane:

```text
[(u-(width-1)/2)*sx, (v-(height-1)/2)*sy, 0]
```

The grid is centered on the site origin along local X and Y, and all rays point along local +Z. `sx` and `sy` are the distances between adjacent column and row origins, respectively.

Images are row-major, with pixel index `v * width + u`. For optical-axis directions, see “Coordinate frame and pinhole rays” above.

## Range

For both projections, `near` and `far` limit **optical-axis depth**, measured along local +Z. The limits are inclusive and must satisfy `0 <= near < far`, with finite positive `far`.

-   The query finds the nearest surface before testing its depth. A surface closer than `near` still occludes objects behind it; the query does not skip it.
-   After exclusion and group filtering, a nearest hit at exactly zero ray distance makes that pixel invalid. This includes occupied origins inside a solid and zero-distance surface-origin hits. The query does not recover an exit or skip the collider to see objects behind it.
-   Invalidity is per pixel: an orthographic camera can have only some ray origins inside colliders. Strictly positive ray distances are not rejected with an epsilon.

## Output fields

Select output fields with `returns`. The default is `depth` only. At least one of `depth` or `distance` must be enabled.

| Field      | Dimensions | Meaning                                                                                    |
| ---------- | ---------- | ------------------------------------------------------------------------------------------ |
| `depth`    | 1          | Optical-axis depth: ray distance multiplied by the Z component of the local unit direction |
| `distance` | 1          | Ray distance from that pixel's ray origin to the hit point                                 |
| `point`    | 3          | Hit point in world coordinates                                                             |
| `normal`   | 3          | Surface normal in world coordinates                                                        |

Fields always follow the table order, with disabled fields omitted. The order of field names in MJCF does not affect output order. The number of floats per ray, `stride`, is the sum of the enabled fields' dimensions.

Pinhole rays start at the camera's optical origin, so `distance` is also Euclidean distance from that origin. Orthographic pixels have distinct ray origins, and both distance fields equal optical-axis depth; `distance` is **not** Euclidean distance from the camera center.

On an invalid pixel or a miss, distances are `-1` and points and normals are zero. Test distance `>= 0` for validity, rather than checking whether the point is zero: the world origin can be a valid hit point.

## Read sensor data

### Read a pinhole depth image

`model.get_sensor_value()` returns an array with shape `(*data.shape, width * height * stride)`. Reshape it to `(*data.shape, height, width, stride)` using the configured dimensions and output layout. Batched and indexed `SceneData` dimensions are preserved.

The following assumes `model` has loaded the pinhole MJCF configuration above. Since only one depth field is enabled, the final dimension of size 1 can be omitted:

```python
import numpy as np
from motrixsim import SceneData

data = SceneData(model, batch=(8,))
buffer = np.empty_like(model.get_sensor_value("camera", data))
model.step(data)
model.get_sensor_value("camera", data, out=buffer)
image = buffer.reshape(*data.shape, 27, 48)
valid = image >= 0
```

Reuse `buffer` when reading in a loop to avoid allocating a new output array each time.

### Read depth and surface height

The following assumes `model` has loaded the orthographic MJCF configuration above. Depth, hit point, and normal are enabled, giving 7 floats per ray:

```python
import numpy as np
from motrixsim import SceneData

data = SceneData(model, batch=(8,))
buffer = np.empty_like(model.get_sensor_value("height", data))
model.step(data)
model.get_sensor_value("height", data, out=buffer)
rays = buffer.reshape(*data.shape, 5, 8, 7)
depth = rays[..., 0]
points = rays[..., 1:4]
normals = rays[..., 4:7]
valid = depth >= 0
surface_height = np.where(valid, points[..., 2], np.nan)
```

`depth` measures distance from each grid origin along the sampling direction; it is not surface height in world coordinates. Since `points` uses world coordinates, `points[..., 2]` gives the hit surface's world Z coordinate. Exclude miss pixels before using these values.

For depth alone, set `returns` to `depth`. With orthographic projection, `distance` equals the same depth, not distance from the camera center. See “Output fields” above for the complete field definitions.

## Frame updates

`update_period` is measured in simulation seconds and must be nonnegative and finite.

-   `update_period=0`: capture one complete frame every physics step.
-   Positive period: if no frame has been captured, the first physics step captures it immediately. Later captures follow accumulated simulation time, holding the previous frame between updates. If `forward_kinematic()` has already captured the first frame, subsequent steps follow the period rather than unconditionally capturing again.
-   A step captures at most one frame. It does not reconstruct historical frames, perform rolling scans, or interpolate substep poses.

During a physics step, capture uses the camera and scene poses **before integration**. The current camera pose after the step returns may therefore differ from the cached image's acquisition pose. A low-frequency image may also predate the final state of the current step. Do not directly pair cached depth with the current post-step pose for coordinate transforms.

Before the first capture, outputs contain miss values; these initial values are not a measurement. `model.get_sensor_value("camera", data)` only reads the cached complete frame and does not trigger a capture.

### Refresh without advancing physics

`model.forward_kinematic(data)` updates kinematics without advancing simulation time or changing DoF positions and velocities. Camera capture depends on whether an image has been acquired since initialization or reset:

| Situation                             | Camera behavior                                        |
| ------------------------------------- | ------------------------------------------------------ |
| Fresh `SceneData`, before any capture | Capture the first frame at the current pose            |
| After reset, before any new capture   | Capture the first frame at the current post-reset pose |
| An image already exists               | Hold the image without bypassing `update_period`       |

After FK captures the first frame, subsequent physics steps accumulate one sampling period before updating again. Repeated FK calls neither advance nor restart that period. For LiDAR behavior, see [Explicit Forward Kinematics](./lidar.md#explicit-forward-kinematics).

### Acquire an image after reset

An ordinary `data.reset(model)` does not capture a new image immediately. Reading at this point may still return the pre-reset cache, which must not be used as an observation of the new episode. The first `model.forward_kinematic(data)` call or physics step after reset captures an image from the new state and restarts sampling timing.

Use `data.reset(model, forward_kinematic=True)`, or call `model.forward_kinematic(data)` after resetting, to acquire the reset-state image without advancing simulation time.

The following continues the batched pinhole example above and resets only the selected environments:

```python
reset_mask = np.array([True, False, True, False, False, False, False, False])
selected = data[reset_mask]
selected.reset(model, forward_kinematic=True)
reset_images = model.get_sensor_value("camera", selected).reshape(*selected.shape, 27, 48)
```

Reset and FK act only on the environments in the supplied `SceneData` view. Other environments retain their images and sampling schedules. The same rules apply to pinhole and orthographic cameras.

## Collision filtering

`exclude` controls self-filtering for the mounting body:

| Value                  | Meaning                                                        |
| ---------------------- | -------------------------------------------------------------- |
| `none`                 | Do not exclude the mounting body                               |
| `parentbody` (default) | Exclude colliders belonging to the site's body                 |
| `parentsubtree`        | Exclude colliders belonging to the site's body and its subtree |

Optional MJCF `groups="0 2"` restricts queries to the specified collision geometry groups. Group IDs must be integers in `0..=255`; negative or larger IDs are rejected. An in-range ID is valid even when no geometry uses it. Omit the whitelist to apply no group restriction; an empty whitelist (`groups=""`) includes no colliders. This filtering is independent of contact settings `contype` and `conaffinity`.
