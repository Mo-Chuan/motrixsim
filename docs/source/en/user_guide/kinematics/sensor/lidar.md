# Lidar Sensor

{bdg-success-line}`CPU: fully supported` {bdg-danger-line}`GPU: not supported`

MotrixSim Lidar is a multi-beam ranging sensor that performs batched raycasts against physics
collision geometry. A site defines its mounting pose, and the sensor publishes a world-frame point
cloud for obstacle avoidance, terrain perception, localization and mapping, and reinforcement
learning observations.

```{video} /_static/videos/lidar_sensor.mp4
:nocontrols:
:autoplay:
:playsinline:
:muted:
:loop:
:width: 100%
```

The colored lines and points show real-time Lidar hits on the floor, walls, spheres, capsules, and
obstacles around the robot. Point-cloud visualization is display-only and does not participate in
sensor computation.

## Quick Start

A Lidar consists of three parts:

1. `body/site` defines the mounting frame;
2. `asset/lidar` defines a reusable model and scan geometry;
3. `sensor/lidar` attaches a sensor instance to the site and references the profile.

The following configuration creates a 1024×64, 360-degree Grid Lidar. Omitting `hz` produces one
complete static snapshot per physics step.

```xml
<mujoco model="lidar_example">
  <asset>
    <lidar name="grid_64"
      cutoff="25"
      pattern="grid"
      hscan="1024" vscan="64"
      hrange="-180 180" vrange="-15 15"/>
  </asset>

  <worldbody>
    <body name="robot">
      <site name="lidar_site"
        pos="0.25 0 0.20"
        quat="0.5 0.5 0.5 0.5"
        size="0.01"/>
    </body>
  </worldbody>

  <sensor>
    <lidar name="roof_lidar"
      site="lidar_site"
      asset="grid_64"
      exclude="parentsubtree"/>
  </sensor>
</mujoco>
```

Each `step()` advances the Lidar. `get_sensor_value()` returns the same flat numeric sensor buffer
as other sensors. The default beam layout is `[distance, point_x, point_y, point_z]` (stride 4).

```python
from motrixsim import SceneData, load_model

HSCAN = 1024
VSCAN = 64

model = load_model("lidar_scene.xml")
data = SceneData(model)

model.step(data)
values = model.get_sensor_value("roof_lidar", data)
beams = values.reshape(*data.shape, HSCAN, VSCAN, 4)
distances = beams[..., 0]
points = beams[..., 1:4]

# Distance is authoritative; a hit at the world origin is valid.
hit_mask = distances >= 0.0
hit_points = points[hit_mask]
print(f"hits: {hit_points.shape[0]}")
```

## MJCF Parameter Overview

Model and scan parameters belong to `asset/lidar`; instance identity, mounting, and runtime scan
frequency and output selection belong to `sensor/lidar`. Do not put profile parameters on a sensor instance.

### `asset/lidar`

| Attribute    | Applies to  | Default / constraint                                                | Description                                           |
| ------------ | ----------- | ------------------------------------------------------------------- | ----------------------------------------------------- |
| `name`       | All         | Required and globally unique                                        | Profile name referenced by `sensor/lidar.asset`       |
| `cutoff`     | All         | Default `100`; positive and finite                                  | Maximum range in meters                               |
| `pattern`    | All         | Default `grid`; `grid` / `rings`                                    | Scan geometry type                                    |
| `hscan`      | Grid, Rings | Default `1`; positive integer; mutually exclusive with `reportrate` | Horizontal columns in one complete scan               |
| `vscan`      | Grid        | Default `1`; positive integer                                       | Vertical beams in each column                         |
| `hrange`     | Grid, Rings | Default `0 0`; two finite values                                    | Horizontal start and end angles in degrees            |
| `vrange`     | Grid        | Default `0 0`; two finite values                                    | Vertical start and end angles in degrees              |
| `elevations` | Rings       | Non-empty array                                                     | Per-ring elevation table in degrees                   |
| `reportrate` | Grid, Rings | Optional, positive and finite; requires sensor `hz`                 | Horizontal columns per second, used to derive `hscan` |
| `hzrange`    | Grid, Rings | Optional; two positive, non-decreasing values                       | Supported inclusive complete-scan frequency range     |

Grid uses `vscan`/`vrange`, while Rings uses `elevations`; these pattern-specific attributes cannot
be mixed.

### `sensor/lidar`

| Attribute | Default / constraint                                | Description                                                               |
| --------- | --------------------------------------------------- | ------------------------------------------------------------------------- |
| `name`    | Optional                                            | Sensor name passed to `get_sensor_value()`                                |
| `asset`   | Required                                            | References one `asset/lidar.name`                                         |
| `site`    | Required                                            | Site defining ray origins and the local frame                             |
| `exclude` | Default `parentbody`                                | `none` / `parentbody` / `parentsubtree` self-filter policy                |
| `returns` | Default `distance point`; `distance` required       | Output field set: `distance`, `point`, `time_offset`, `normal`            |
| `hz`      | Optional, positive and finite; `1 / hz >= timestep` | Omit for a complete snapshot every step; set to enable in-frame spreading |

If the profile defines `hzrange`, the sensor's `hz` must fall inside that closed range.

## Coordinate Frame and Output Layout

Lidar uses the site's local frame:

-   `+Z`: forward;
-   `+X`: right;
-   `+Y`: up;
-   `hrange`: horizontal scan range around the up axis, in degrees;
-   `vrange` / `elevations`: elevation relative to the horizontal plane, in degrees.

Both Grid and Rings are ordered with horizontal angle first, so their outputs have stable reshape
layouts:

| Pattern | Numeric output shape                            |
| ------- | ----------------------------------------------- |
| Grid    | `(*data.shape, hscan, vscan, stride)`           |
| Rings   | `(*data.shape, hscan, len(elevations), stride)` |

Configure output fields on the **sensor instance**, not on `asset/lidar`:

```xml
<sensor>
  <lidar name="roof_lidar" site="lidar_site" asset="grid_64"
    returns="distance point time_offset normal"/>
</sensor>
```

The list selects a set; token order never changes the fixed field order below. Unknown or duplicate
tokens and a missing `distance` are errors. There is no point-only compatibility mode.

| Field         | Width | Meaning                                      | Miss value                |
| ------------- | ----- | -------------------------------------------- | ------------------------- |
| `distance`    | 1     | Acquisition-time distance along the unit ray | `-1`                      |
| `point`       | 3     | Acquisition-time world position              | All zero                  |
| `time_offset` | 1     | Seconds from the FullScan start              | Actual acquisition offset |
| `normal`      | 3     | World-space surface normal                   | All zero                  |

Disabled fields are omitted. `stride = 1 + 3*point + time_offset + 3*normal`, so the default is 4
and all fields use 8 values per beam. Only `distance >= 0` identifies a hit; a zero world point
can be valid. Before the first publication, distances are `-1` and other fields are zero.

Use model metadata rather than hard-coded offsets for configurable layouts:

```python
lidar = next(sensor for sensor in model.low.low_lidar_sensors if sensor.name == "roof_lidar")
layout = lidar.returns
beams = values.reshape(*data.shape, lidar.beam_count, layout.stride)
distances = beams[..., layout.distance_offset]
if layout.point_offset is not None:
    points = beams[..., layout.point_offset:layout.point_offset + 3]
```

`model.low.low_lidar_sensors` is a list with `.name`, `.beam_count`, `.hz`, and `.returns` metadata.
The returns layout exposes `.stride`, `.distance_offset`, `.point_offset`, `.time_offset_offset`,
and `.normal_offset`; disabled optional offsets are `None`.

## Scan Patterns

### Grid

Grid samples horizontal and vertical angles uniformly. It is useful for solid-state approximations,
depth perception, and quick scene setup.

| Attribute | Meaning                         |
| --------- | ------------------------------- |
| `hscan`   | Horizontal columns in one frame |
| `vscan`   | Vertical beams in each column   |
| `hrange`  | Horizontal start and end angles |
| `vrange`  | Vertical start and end angles   |
| `cutoff`  | Maximum range in meters         |

### Rings

Rings uses an explicit elevation table per channel. It fits rotating Lidars such as Velodyne and
Ouster whose vertical beams are not uniformly spaced.

```xml
<asset>
  <lidar name="rings_8"
    cutoff="100"
    pattern="rings"
    elevations="-15 1 -13 3 -11 5 -9 7"
    hscan="1800"
    hrange="-180 180"/>
</asset>
```

Pattern-specific attributes cannot be mixed: Rings uses `elevations`, not `vscan` or `vrange`. See
the complete {ref}`MJCF asset/lidar reference <asset-lidar>`.

## Static Snapshots and `hz`

`sensor/lidar.hz` controls scan timing:

-   **Omit `hz`**: emit the complete pattern every physics step using one site pose. This produces
    distortion-free static snapshots and is suitable for RL tasks that require a complete
    observation every step.
-   **Set `hz="10"`**: spread one frame over `0.1s` and emit only the current phase's beams in each
    physics step. Hits use each step's site pose and accumulate in world coordinates. The completed
    frame is published at the end of the period, so a moving carrier exhibits realistic scan-motion
    distortion.

```xml
<sensor>
  <lidar name="roof_lidar"
    site="lidar_site"
    asset="rotating_profile"
    hz="10"
    exclude="parentsubtree"/>
</sensor>
```

The configuration must satisfy `1 / hz >= physics timestep`. For a profile that uses `reportrate`,
the model builder derives the actual horizontal resolution as
`hscan = round(reportrate / hz)`.

## Publication Status and Acquisition Time

`model.get_lidar_status("roof_lidar", data)` returns an object with `.ready` (boolean ndarray) and
`.scan_seq` (`uint64` ndarray), both shaped exactly as `data.shape`, including scalar arrays for
unbatched data. Fresh `SceneData` or full instance reinitialization restores `ready=False`, `scan_seq=0`.

-   Static scans publish a complete frame and increment the sequence every physics step.
-   Rolling scans increment the sequence only after a complete frame has been acquired. After a publication, `ready` stays true until reset, and the complete frame is held until the next publication.
-   Ordinary Python `reset()` / `reset_world()` immediately make `ready=False` for the selected environments, while retaining the cached readings and `scan_seq`. The cache is not a valid observation of the new episode. Scanning restarts without mixing unfinished pre-reset beams; sequence numbers do not reset. Environments that were not reset are unaffected.

These status values are not part of the per-beam numeric output.

### Explicit Forward Kinematics

`model.forward_kinematic(data)` does not advance simulation time:

-   **Static LiDAR**: capture the complete pattern at the current site pose, set every `time_offset` to zero, set `ready=True`, and increment `scan_seq`.
-   **Rolling LiDAR**: do not capture or publish a frame, or advance scanning. Without a reset, subsequent physics steps continue the existing scan. After reset, discard the unfinished scan and restart from the beginning; `ready` stays false until physics steps complete a full new scan.

An ordinary reset performs no ray queries. `data.reset(model, forward_kinematic=True)` refreshes static LiDAR immediately, but rolling LiDAR still waits for physics steps to complete a new scan according to `hz`. Its cached pre-reset values remain readable but invalid (`ready=False`); reading them never triggers capture.
Reset and FK affect only environments in the supplied `SceneData` view. For RaycastCamera behavior, see [camera lifecycle](./raycast_camera.md#refresh-without-advancing-physics).

### Per-beam acquisition time

During physics stepping, static `time_offset` is zero. Rolling beams record the actual acquisition **step end**, not ideal
firing phases: `elapsed + dt` for the current scan tail, and `elapsed + dt - period` for a wrap head
belonging to the next scan. All beams acquired in a step share its pose and acquisition time; no
substep interpolation is performed. Step quantization can make tail offsets slightly exceed the
scan period. Misses record the same acquisition time as hits.

## Self-Filtering

`exclude` prevents the Lidar from scanning its own mount or robot:

| Value           | Behavior                                                                        |
| --------------- | ------------------------------------------------------------------------------- |
| `none`          | Do not exclude any self collider                                                |
| `parentbody`    | Exclude the site's parent body; this is the default                             |
| `parentsubtree` | Exclude the parent body and all descendants; recommended for articulated robots |

Lidar hits physics colliders, not visual-only geoms. Changing rendering materials does not change
range measurements.

## Using Bundled Real-Model Profiles

The `examples/assets/lidar-profiles/` directory provides starter profiles for HESAI XT32 and Ouster
OS0/OS1/OS2. Include its entry-point
[`catalog.xml`](../../../../../examples/assets/lidar-profiles/catalog.xml) in MJCF and reference a
profile by name:

```xml
<include file="lidar-profiles/catalog.xml"/>

<sensor>
  <lidar name="roof_lidar"
    site="lidar_site"
    asset="hesai_xt32_sd10"
    hz="10"
    exclude="parentsubtree"/>
</sensor>
```

Include paths are resolved relative to the current MJCF file. See
[`robot_locomotion.py`](../../../../../examples/control/robot_locomotion.py) for a complete mounting
example.

## Point-Cloud Visualization

The renderer reads declared offsets and uses distance to identify hits. Static distance-only scans
can reconstruct points from the beam pattern and acquisition pose. Rolling distance-only scans lack
historical beam poses and are warned about and skipped; enable `point` to visualize rolling scans.

The renderer does not stream Lidar point clouds by default. Enable it explicitly for debugging with
`set_lidar_point_vis(True)`:

```python
from motrixsim.render import RenderApp

with RenderApp() as render:
    render.launch(model)
    render.opt.set_lidar_point_vis(True)

    while not render.is_closed:
        model.step(data)
        render.sync(data)
```

See [`lidar_point_cloud_demo.py`](../../../../../examples/sensors/lidar_point_cloud_demo.py) for the
complete programmatic scene and visualization code. The example shows a 60° × 20° Solid State-style
static Grid alongside a 360°, 10 Hz rotating Rings Lidar with an explicit 16-channel elevation
table and in-frame spreading. Both Lidars use inline MJCF configuration; the latter's mount moves
to demonstrate scan-motion distortion.

## Performance Reference

**Historical point-only measurements:** the chart and table below were measured with the old
3-value point-only output, not the new default distance + point payload. They are not new
measurements and must not be used as stride-4 performance claims.

That end-to-end Python benchmark used the playground scene. Each Lidar uses a 1024×64 Grid
and omits `hz`, so it emits 65,536 rays per step. The test calls `model.step()` one step at a time and
excludes model loading, building, and rendering. It ran with 32 logical CPUs and automatic Rayon
thread selection; values are medians of three rounds.

```{figure} /_static/images/lidar_raycast_benchmark.svg
:alt: Raycast throughput bar chart for three Lidar scale cases
:width: 100%

Historical point-only end-to-end Lidar raycast throughput
```

| Case                             | Total rays/step | Median throughput | Wall time/step |
| -------------------------------- | --------------: | ----------------: | -------------: |
| 1 environment × 1 Lidar          |          65,536 | 63.495 Mraycast/s |       1.032 ms |
| 1 environment × 128 Lidars       |       8,388,608 | 78.324 Mraycast/s |     107.102 ms |
| 1024 environments × 1 Lidar each |      67,108,864 | 80.656 Mraycast/s |     832.036 ms |

Throughput is calculated as:

```text
total rays = hscan × vscan × lidars × environments × timed steps
throughput = total rays / wall-clock time
```

`80.656 Mraycast/s` means an aggregate throughput of about 80.656 million raycasts per second across
all environments. The measurement includes the complete physics pipeline seen by a user calling
`step()`; it is not an isolated raycast-kernel peak.

Run the benchmark with the **current default stride-4 output** using the command below; new results
are not directly comparable to the historical point-only figures without noting the layout change:

```bash
cd motrixsim-python/motrixsim-docs
uv run python examples/bench/lidar.py --case all
```
