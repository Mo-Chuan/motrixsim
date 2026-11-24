# 📚 Example Programs

:::{tip}
For models and code, see the [MotrixSim Docs](https://github.com/Motphys/motrixsim-docs) repository.

Before running the examples, please refer to {ref}`Path 2: Clone the Repository and Run Examples <install-from-repository>` to complete the environment setup.
:::

**On Linux and Windows**, run `uv run examples/{category}/{example_name}.py` from the repository root to launch an example; for examples whose **Arguments** column is non-empty, append the listed arguments. Shared examples that support GPU execution accept a `--gpu` suffix.

**On MacOS**, examples that use RenderApp (most visual examples, such as robotic arm control and robot locomotion) replace `uv run` with `uv run mxpython`, for example:

```bash
uv run mxpython examples/{category}/{example_name}.py
```

Examples that do not use RenderApp (physics simulation only) behave the same as other platforms: run them with `uv run`.

## Getting Started

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **Preview**
  - **Example**
  - **Arguments**
  - **Description**
* - ![falling_ball](/_static/images/examples/thumbs/falling_ball.jpg)
  - [`falling_ball.py`](../../../../examples/getting_started/falling_ball.py)
  - —
  - A ball falls under gravity, demonstrating how to create a `model` and `data`.
* - ![load_from_str](/_static/images/examples/thumbs/load_from_str.jpg)
  - [`load_from_str.py`](../../../../examples/getting_started/load_from_str.py)
  - —
  - Load an MJCF model from a string, demonstrating how to create a scene directly from an XML string.
* - ![hello_usd](/_static/images/examples/thumbs/hello_usd.jpg)
  - [`hello_usd.py`](../../../../examples/physics/hello_usd.py)
  - —
  - Load a scene from a USD file (cobotta_pro_1300 arm), demonstrating the same `load_model` entry point as MJCF.
```

## Physics

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **Preview**
  - **Example**
  - **Arguments**
  - **Description**
* - ![actuator](/_static/images/examples/thumbs/actuator.jpg)
  - [`actuator.py`](../../../../examples/physics/actuator.py)
  - —
  - Retrieve and configure `actuator` parameters.
* - ![body](/_static/images/examples/thumbs/body.jpg)
  - [`body.py`](../../../../examples/physics/body.py) · [GPU](../../../../examples/gpu/body.py)
  - —
  - Usage of `body`-related APIs; here, `body` specifically refers to the root world body.
* - ![joint](/_static/images/examples/thumbs/joint.jpg)
  - [`joint.py`](../../../../examples/physics/joint.py) · [GPU](../../../../examples/gpu/joint.py)
  - —
  - Usage of `joint`-related APIs, including reading and writing `dof_position` and `dof_velocity`.
* - ![link](/_static/images/examples/thumbs/link.jpg)
  - [`link.py`](../../../../examples/physics/link.py)
  - —
  - Usage of `link`-related APIs.
* - ![options](/_static/images/examples/thumbs/options.jpg)
  - [`options.py`](../../../../examples/physics/options.py)
  - —
  - Configure simulator parameters using `options`.
* - ![site_and_sensor](/_static/images/examples/thumbs/site_and_sensor.jpg)
  - [`site_and_sensor.py`](../../../../examples/physics/site_and_sensor.py)
  - —
  - Usage of `site` and `sensor`-related APIs.
* - ![friction](/_static/images/examples/thumbs/friction.jpg)
  - [`friction.py`](../../../../examples/physics/friction.py)
  - —
  - Scene demonstrating friction configuration.
* - ![geom](/_static/images/examples/thumbs/geom.jpg)
  - [`geom.py`](../../../../examples/physics/geom.py)
  - —
  - Usage of geometry-related APIs, showing how to access and query geometry position, velocity, and other information.
* - ![hfield](/_static/images/examples/thumbs/hfield.jpg)
  - [`hfield.py`](../../../../examples/physics/hfield.py)
  - —
  - Usage of height field APIs, demonstrating how to access terrain height data and perform statistical analysis.
* - ![combine_msd](/_static/images/examples/thumbs/combine_msd.jpg)
  - [`combine_msd.py`](../../../../examples/physics/combine_msd.py)
  - —
  - Combine multiple MSD models, demonstrating how to use the `Scene.attach()` method to attach multiple models together, supporting transforms and namespace prefixes.
* - ![adhesion](/_static/images/examples/thumbs/adhesion.jpg)
  - [`adhesion.py`](../../../../examples/physics/adhesion.py)
  - —
  - A robotic arm with adhesion actuator.
* - ![external_force](/_static/images/examples/thumbs/external_force.jpg)
  - [`external_force.py`](../../../../examples/physics/external_force.py)
  - —
  - External force and torque application example, demonstrating how to use `add_external_force` and `add_external_torque` APIs to apply forces and torques to links in local coordinate frames. Includes three phases: center-of-mass thrust, Z-axis torque, and offset-point force.
* - ![lidar_point_cloud_demo](/_static/images/examples/thumbs/lidar_point_cloud_demo.jpg)
  - [`lidar_point_cloud_demo.py`](../../../../examples/sensors/lidar_point_cloud_demo.py)
  - —
  - Lidar point cloud demo, showing the configuration of rotating (16-channel ring) and solid-state grid lidar sensors with point cloud visualization.
* - ![touch_sensor_demo](/_static/images/examples/thumbs/touch_sensor_demo.jpg)
  - [`touch_sensor_demo.py`](../../../../examples/sensors/touch_sensor_demo.py)
  - —
  - Touch sensor demo, showing how to read touch sensor data to detect contact between the robot end-effector and environmental objects, with force feedback visualization.
* - ![sdf_collision](/_static/images/examples/thumbs/sdf_collision.jpg)
  - [`sdf_collision.py`](../../../../examples/physics/sdf_collision.py)
  - —
  - SDF collision demo with two meshing gears: tooth contacts are solved by the SDF narrow phase, a velocity actuator spins the driving gear, and detected contacts (points and normals) are drawn with gizmos. See the [SDF collision documentation](../kinematics/geometry/sdf.md).
* - ![equality_transmission_demo](/_static/images/examples/thumbs/equality_transmission_demo.jpg)
  - [`equality_transmission_demo.py`](../../../../examples/control/equality_transmission_demo.py)
  - —
  - Equality constraint demo with two scenes: a welded flange platform and a joint transmission gallery, showing drivetrains coupled by equality/joint rows.
* - ![strandbeest](/_static/images/examples/thumbs/strandbeest.jpg)
  - [`strandbeest.py`](../../../../examples/physics/strandbeest.py)
  - `--suspended` suspended view · `--headless` measurement mode (remaining flags are for benchmark output)
  - Single-crank Strandbeest multi-leg walking mechanism, showing locomotion driven by equality loop-closure constraints.
```

## Control

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **Preview**
  - **Example**
  - **Arguments**
  - **Description**
* - ![keyboard_car](/_static/images/examples/thumbs/keyboard_car.jpg)
  - [`keyboard_car.py`](../../../../examples/control/keyboard_car.py)
  - —
  - Control a cart using the keyboard; demonstrates keyboard event handling. W/S to move forward/backward, A/D to turn.
* - ![car](/_static/images/examples/thumbs/car.jpg)
  - [`car.py`](../../../../examples/control/car.py)
  - —
  - Full remote-controlled car driving example, showing steering/drive/brake actuator control, heading hold, and speed ramping, with a custom UI layout.
* - ![mouse_click](/_static/images/examples/thumbs/mouse_click.jpg)
  - [`mouse_click.py`](../../../../examples/control/mouse_click.py) · [GPU](../../../../examples/gpu/mouse_click.py)
  - —
  - Move a ball by clicking on the ground with the mouse; demonstrates mouse event handling.
* - ![inverse kinematics](/_static/images/examples/thumbs/ik.jpg)
  - [`ik.py`](../../../../examples/control/ik.py)
  - —
  - Demonstrates how to use the built-in IK module in MotrixSim for inverse kinematics solving.
* - ![local_arm](/_static/images/examples/thumbs/local_arm.jpg)
  - [`local_arm.py`](../../../../examples/control/local_arm.py)
  - —
  - A robotic arm composed of simple geometries and `joints`.
* - ![robotic_arm](/_static/images/examples/thumbs/robotic_arm.jpg)
  - [`robotic_arm.py`](../../../../examples/control/robotic_arm.py)
  - —
  - The Stanford robotic arm uses a sequence of movement commands to pick and place a ball.
* - ![apple_grasp](/_static/images/examples/thumbs/apple_grasp.jpg)
  - [`apple_grasp.py`](../../../../examples/control/apple_grasp.py)
  - —
  - Replay a stem pinch, lift, placement, fruit-body grasp, and second lift. Automatically plays the 40-second sequence, with pause and resume controls. Results are printed to the terminal.
* - ![robot_locomotion](/_static/images/poster/go2_keyboard_control.jpg)
  - [`robot_locomotion.py`](../../../../examples/control/robot_locomotion.py)
  - `--robot go1|go2|g1|microduck` (default go2)
    `--scene playground|parlour|plane|stairs` (default playground; microduck only supports its bundled plane)
    `--lidar <profile>` (g1 / go1 / go2 only)
  - Keyboard control for quadrupeds and humanoids: arrow keys and WASD control walking and turning. Microduck uses a bundled ONNX walking policy and resets automatically when its trunk touches the floor.
* - ![g1_motion_tracking](/_static/images/poster/g1_motion_tracking.png)
  - [`g1_motion_tracking.py`](../../../../examples/control/g1_motion_tracking.py)
  - —
  - G1 humanoid motion-tracking playback example. It loads the bundled reference motion and ONNX actor for single-environment playback.
* - ![rm65_open_cabinet](/_static/images/examples/thumbs/rm65_open_cabinet.jpg)
  - [`rm65_open_cabinet.py`](../../../../examples/control/rm65_open_cabinet.py)
  - —
  - RM65 cabinet-opening policy example using an ONNX policy. [Demo video](/_static/videos/rm65_open_cabinet.mp4). Press `R` to reset and `ESC` to exit.
* - ![shadow_hand_repose](/_static/images/examples/thumbs/shadow_hand_repose.jpg)
  - [`shadow_hand_repose.py`](../../../../examples/control/shadow_hand_repose.py)
  - —
  - Shadow Hand cube reorientation policy example using an ONNX policy. [Demo video](/_static/videos/shadow_hand_repose.mp4). Press `R` to reset and `ESC` to exit.
* - ![osc](/_static/images/examples/thumbs/osc.jpg)
  - [`osc.py`](../../../../examples/control/osc.py)
  - —
  - Operational Space Control (OSC) interactive example, demonstrating how to use `OscSolver` with `IkChain` to control end-effector position and orientation via computed torques. Arrow keys and WASD to move target, Q/E to rotate, R to reset.
* - ![go1](/_static/images/poster/go1.jpg)
  - [`go1.py`](../../../../examples/control/go1.py) · [GPU](../../../../examples/gpu/go1.py)
  - —
  - Random motion of the Go1 quadruped robot, demonstrating how to integrate a neural network and use `.onnx` files.
* - ![go1_multi_task](/_static/images/examples/thumbs/go1_multi_task.jpg)
  - [`go1_multi_task.py`](../../../../examples/control/go1_multi_task.py)
  - —
  - Go1 quadruped multi-task policy example, supporting walking, handstand, footstand, and getup recovery mode switching. WASD to move, U for handstand, I for footstand, O for recovery, P to reset.
```

## Parallel Simulation

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **Preview**
  - **Example**
  - **Arguments**
  - **Description**
* - ![parallelsim](/_static/images/examples/thumbs/parallelsim.jpg)
  - [`parallelsim.py`](../../../../examples/parallel/parallelsim.py) · [GPU](../../../../examples/gpu/parallelsim.py)
  - —
  - Multi-environment parallel simulation; the GPU version generates batched random controls with a Warp kernel.
```

## Randomization

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **Preview**
  - **Example**
  - **Arguments**
  - **Description**
* - ![randomize_mass](/_static/images/examples/thumbs/randomize_mass.jpg)
  - [`mass.py`](../../../../examples/randomize/mass.py)
  - —
  - Randomize rigid body mass from 0.1 kg to 500 kg across 16 parallel instances, demonstrating how mass affects falling dynamics.
* - ![randomize_friction](/_static/images/examples/thumbs/randomize_friction.jpg)
  - [`friction.py`](../../../../examples/randomize/friction.py)
  - —
  - Randomize per-geom friction coefficients from 0.01 to 2.0, demonstrating the transition from sliding to static grip on a 30° ramp.
* - ![randomize_armature](/_static/images/examples/thumbs/randomize_armature.jpg)
  - [`armature.py`](../../../../examples/randomize/armature.py)
  - —
  - Randomize joint armature (virtual rotor inertia), showing how armature affects the natural frequency of a torsion-spring-driven arm.
* - ![randomize_frictionloss](/_static/images/examples/thumbs/randomize_frictionloss.jpg)
  - [`frictionloss.py`](../../../../examples/randomize/frictionloss.py)
  - —
  - Randomize joint frictionloss (Coulomb dry friction) from 0 to 200 N·m, demonstrating how friction can freeze a pendulum at different angles.
* - ![randomize_com](/_static/images/examples/thumbs/randomize_com.jpg)
  - [`com.py`](../../../../examples/randomize/com.py)
  - —
  - Randomize center of mass position within the cylinder cross-section, showing how COM offset affects rolling behavior.
* - ![inertia](/_static/images/examples/thumbs/inertia.jpg)
  - [`inertia.py`](../../../../examples/randomize/inertia.py)
  - —
  - Randomize the complete link-local inertia tensor about the center of mass, showing how hinge-axis inertia changes a torsion-spring arm's oscillation period while mass and COM remain fixed.
* - ![randomize_actuator_kp_kd](/_static/images/examples/thumbs/randomize_actuator_kp_kd.jpg)
  - [`actuator_kp_kd.py`](../../../../examples/randomize/actuator_kp_kd.py)
  - —
  - Randomize position actuator PD gains in a 4×4 grid with kp (10–120 N·m/rad) varying by row and kd (1–20 N·m·s/rad) by column, visualizing PD servo dynamics.
* - ![randomize_geom_size](/_static/images/examples/thumbs/randomize_geom_size.jpg)
  - [`geom_size.py`](../../../../examples/randomize/geom_size.py)
  - —
  - Randomize collision and visual sizes of five geometry types (sphere, capsule, box, cylinder, ellipsoid), showing how size affects settling height and contact behavior.
* - ![geom_shape](/_static/images/examples/thumbs/geom_shape.jpg)
  - [`geom_shape.py`](../../../../examples/randomize/geom_shape.py)
  - —
  - Randomize primitive and compile-time Mesh selections across 16 parallel instances, synchronizing effective collision and visual geometry while exposing Mesh variant metadata.
* - ![randomize_gravity_direction](/_static/images/examples/thumbs/randomize_gravity_direction.jpg)
  - [`gravity_direction.py`](../../../../examples/randomize/gravity_direction.py)
  - —
  - Randomize gravity direction by assigning each of 16 parallel instances a different gravity vector inside a transparent box, demonstrating per-instance gravity override with `set_gravity_override`.
```

## Viewer

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **Preview**
  - **Example**
  - **Arguments**
  - **Description**
* - ![camera_control](/_static/images/examples/thumbs/change_camera_state.jpg)
  - [`camera_control.py`](../../../../examples/viewer/camera_control.py)
  - —
  - Camera control API usage, showing how to enable/disable system and scene cameras, and get camera poses.
* - ![camera_viewport](/_static/images/examples/thumbs/camera_viewport.jpg)
  - [`camera_viewport.py`](../../../../examples/viewer/camera_viewport.py)
  - —
  - Camera viewport widgets displaying different camera feeds with interactive controls for switching cameras, resizing, and repositioning viewports.
* - ![capture_segmentation](/_static/images/examples/thumbs/capture_segmentation.jpg)
  - [`capture_segmentation.py`](../../../../examples/viewer/capture_segmentation.py)
  - —
  - Segmentation capture example, showing how to configure a scene camera as a Segmentation sensor to obtain semantic segmentation maps or the single-channel geom-id image. A UI panel switches modes at runtime and captured frames are displayed live in an on-screen image widget. See the [camera documentation](../render/camera.md).
* - ![viewport_camera_targets](/_static/images/examples/thumbs/viewport_camera_targets.jpg)
  - [`viewport_camera_targets.py`](../../../../examples/viewer/viewport_camera_targets.py)
  - —
  - Two-viewport camera binding demo, showing how to switch viewport bindings between system and scene cameras while the widget stays attached to its target. See the [camera documentation](../render/camera.md).
* - ![dynamic_texture_per_instance](/_static/images/examples/thumbs/dynamic_texture_per_instance.jpg)
  - [`dynamic_texture_per_instance.py`](../../../../examples/viewer/dynamic_texture_per_instance.py)
  - —
  - Per-instance dynamic texture demo: four camera-frustum-shaped objects each render their own dynamic texture on screen-plane billboards.
* - ![render_app_frame](/_static/images/examples/thumbs/render_app_frame.jpg)
  - [`render_app_frame.py`](../../../../examples/viewer/render_app_frame.py)
  - —
  - RenderApp frame panel demo, showing how to swap UI controls dynamically when the parameter mode changes, and how to handle `RenderClosedError`.
* - ![render_app_ui](/_static/images/examples/thumbs/render_app_ui.jpg)
  - [`render_app_ui.py`](../../../../examples/viewer/render_app_ui.py)
  - —
  - Custom RenderApp layout demo, showing how to drive a simulation from Python with a custom window UI.
* - ![ui_gallery](/_static/images/examples/thumbs/ui_gallery.jpg)
  - [`ui_gallery.py`](../../../../examples/viewer/ui_gallery.py)
  - —
  - UI Gallery demonstrating layouts, built-in and custom UI Views, callbacks, and View lifecycle operations.
* - ![gizmos](/_static/images/examples/thumbs/gizmos.jpg)
  - [`gizmos.py`](../../../../examples/viewer/gizmos.py)
  - —
  - Usage of the 3D gizmo drawing system, demonstrating how to draw spheres, cubes, capsules, arrows, grids, and other visualization helpers.
* - ![image_widget](/_static/images/examples/thumbs/image_widget.jpg)
  - [`image_widget.py`](../../../../examples/viewer/image_widget.py)
  - —
  - Image widget system usage, demonstrating how to display and interactively manipulate multiple image panels.
* - ![render_settings](/_static/images/examples/thumbs/render_settings.jpg)
  - [`render_settings.py`](../../../../examples/viewer/render_settings.py)
  - —
  - Rendering settings configuration example, demonstrating how to use `RenderSettings` to configure shadows, screen-space ambient occlusion (SSAO), and other rendering effects.
* - ![partial_rendering](/_static/images/examples/thumbs/partial_rendering.jpg)
  - [`partial_rendering.py`](../../../../examples/viewer/partial_rendering.py)
  - —
  - Selective rendering control in batch rendering, showing how to dynamically control visibility of specific scenes in multi-environment parallel simulation. Press A/D keys to toggle partial scene visibility, Q/E keys for all scenes.
* - ![share_lights_between_envs](/_static/images/examples/thumbs/share_lights_between_envs.jpg)
  - [`share_lights_between_envs.py`](../../../../examples/viewer/share_lights_between_envs.py)
  - `--share_lights=False` disables light sharing
  - Light sharing optimization between environments, demonstrating how to share lights in parallel simulation to improve performance.
* - ![ssgi](/_static/images/poster/go2-ssgi.jpg)
  - [`ssgi.py`](../../../../examples/viewer/ssgi.py)
  - —
  - Screen Space Global Illumination (SSGI) rendering example, demonstrating how to use `RenderSettings` to enable high-quality SSGI rendering effects.
* - ![g1_3dgs_locomotion](/_static/images/examples/thumbs/g1_3dgs.jpg)
  - [`g1_3dgs_locomotion.py`](../../../../examples/viewer/g1_3dgs_locomotion.py)
  - —
  - G1 humanoid keyboard control in a 3D Gaussian Splatting scene, with a first-person head camera shown as an on-screen viewport widget, demonstrating multi-camera 3DGS rendering.
```

## Benchmark

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **Preview**
  - **Example**
  - **Arguments**
  - **Description**
* - ![gyroscope](/_static/images/examples/thumbs/gyroscope.jpg)
  - [`gyroscope_mx.py`](../../../../examples/bench/gyroscope/gyroscope_mx.py)
  - —
  - Physical simulation of a gyroscope.
* - ![gyroscope_zero_gravity](/_static/images/examples/thumbs/gyroscope_zero_gravity.jpg)
  - [`gyroscope_zero_gravity_mx.py`](../../../../examples/bench/gyroscope_zero_gravity/gyroscope_zero_gravity_mx.py)
  - —
  - Gyroscope in a zero-gravity environment, demonstrating conservation of angular momentum.
* - ![newton_cradle](/_static/images/examples/thumbs/newton_cradle.jpg)
  - [`newton_cradle_mx.py`](../../../../examples/bench/newton_cradle/newton_cradle_mx.py)
  - —
  - Physical simulation of Newton's cradle.
* - ![grasp_shaking_test](/_static/images/examples/thumbs/grasp_shaking_test.jpg)
  - [`shake_test_mx.py`](../../../../examples/bench/grasp/shake_test_mx.py)
  - `--object cube|ball|bottle` (default cube) · `--noshake` disables shaking · `--record` records video

    Note: run this example as a module with `uv run python -m examples.bench.grasp.shake_test_mx`
  - Grasping and shaking test for the Franka Panda robotic arm, demonstrating how the arm grasps objects and maintains stability.
```
