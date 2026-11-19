# Microduck robot model

Ported from https://github.com/pollen-robotics/microduck_rl
(`src/mjlab_microduck/robot/microduck/robot_walk.xml`, renamed to
`microduck.xml`). Relative to upstream, a named termination capsule
`trunk_collision` was added on `trunk_base` for the humanoid walk task's
fall-termination contact query; everything else is unmodified.

- Upstream code and MJCF: Apache License 2.0
- 3D mesh files (`assets/*.stl`): Creative Commons BY-SA-NC
  (non-commercial), per the upstream README

The Microduck is a ~800 g, ~25 cm bipedal robot by Pollen Robotics with 14
actuated joints (5 per leg, 4 neck/head).

## Keyboard control

From `motrixsim-docs`, run:

```bash
uv run examples/control/robot_locomotion.py --robot microduck
```

Microduck defaults to its bundled plane scene; other scenes and LiDAR are not supported.

Focus the viewer window. Hold W/S or Up/Down to move forward/backward, Left/Right to move
sideways, and A/D to turn, using the shared locomotion keyboard controls.
Releasing the keys stops the command. Opposite keys prioritize forward, left
translation, or left turn. Escape closes the viewer. The camera uses the same follower
configuration as the Go1 keyboard example: 2 m behind and 0.5 m above the
robot's base, with a 45-degree vertical field of view. Native `fixed_local`
position tracking and `look_at_link` rotation tracking follow `trunk_base`,
with both tracking speeds set to 2.

`flat_scene.xml` defines the floor and MotrixLab training friction and solver
settings. The common runner attaches the robot; `scene.xml` also combines these
assets for standalone model loading. The example restores the standing pose after trunk-floor contact,
with no time limit on an episode. The standing joint pose is defined in `MicroduckRobot.DEFAULT_POSE`;
the base pose comes from `microduck.xml`.

## Policy

`policy.onnx` was exported from MotrixLab's `microduck-walk-flat` FastSAC run
`26-09-08_11-06-02-856936`. It accepts float32 `obs` of shape `(batch, 55)` and
returns `actions` of shape `(batch, 14)`. CPU ONNX Runtime is sufficient;
MotrixLab, Torch and the training checkpoint are not runtime dependencies.

The graph includes learned observation normalization and actor action-range
mapping. Do not apply these again. The example preserves the training residual
bounds: for each actuator, the larger absolute difference between its
`ctrlrange` endpoints and default angle, divided by `0.5`. With the bundled
model's `ctrlrange="-10.0 10.0"`, the symmetric bounds are approximately
±20 to ±20.916 and match the ONNX actor scale. ONNX already bounds normal
outputs through tanh and that scale, so the outer clip normally leaves them
unchanged. This preserves the training action-space contract; it does not
guarantee joint targets stay within joint angle limits.

The example sends `default_angles + 0.5 * action` to the position actuators.
The previous-action observation stores the clipped residual, not the resulting
joint target.

Observation order: body angular velocity scaled by 0.25, projected gravity,
three velocity commands, 14 joint position offsets, 14 joint velocities
scaled by 0.05, 14 previous actions, two phase sines, two phase cosines.
Joint position/velocity observation noise and reset phase sampling are
retained from the source environment. Physics runs at 200 Hz, control at
50 Hz, and the two-leg gait period is 0.5 seconds.

Linear command limits are 0.4 m/s forward and 0.3 m/s sideways; yaw command
limit is 1 rad/s. The policy scales the shared keyboard inputs by
`[0.4, 0.6, 0.5]` before observation and phase updates to retain these limits.
The policy joint order is pinned by name in `MicroduckRobot.DEFAULT_POSE`.
Keep this model, pose and policy together when replacing assets.

Run the headless control-contract check from `motrixsim-docs` with
`uv run python tests/test_microduck_control.py`.
