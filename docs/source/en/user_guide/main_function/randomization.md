# 🎲 Parameter Randomization

{bdg-success-line}`CPU: fully supported` {bdg-warning-line}`GPU: partially supported`

MotrixSim provides Domain Randomization capabilities, allowing you to set different physical parameters for each instance in multi-environment parallel simulations. This is essential for Sim-to-Real transfer and reinforcement learning training.

## Supported Randomization Parameters Overview

| Property          | Object       | Setter                           | Getter                        | Data Shape                  | CPU | GPU |
| :---------------- | :----------- | :------------------------------- | :---------------------------- | :-------------------------- | :-: | :-: |
| Mass              | `Link`       | `set_mass_override`              | `get_mass_override`           | `(batch,)`                  | ✅  | ✅  |
| Center of Mass    | `Link`       | `set_center_of_mass_override`    | `get_center_of_mass_override` | `(batch, 3)`                | ✅  | ✅  |
| Inertia Tensor    | `Link`       | `set_inertia_override`           | `get_inertia_override`        | `(batch, 6)`                | ✅  | —   |
| Principal Inertia | `Link`       | `set_principal_inertia_override` | —                             | `(batch, 3)` + `(batch, 4)` | ✅  | —   |
| Friction          | `Geom`       | `set_friction_override`          | `get_friction_override`       | `(batch, 3)`                | ✅  | ✅  |
| Geom Size         | `Geom*`      | `set_size_override`              | `get_size_override`           | `(batch, N)`                | ✅  | —   |
| Geom Active       | `Geom`       | `set_active_override`            | `get_active_override`         | `(batch,)`                  | ✅  | —   |
| Geom Pose         | `Geom`       | `set_pose_override`              | `get_pose_override`           | `(batch, 3)` + `(batch, 4)` | ✅  | —   |
| Armature          | `Joint`      | `set_armature_override`          | `get_armature_override`       | `(batch,)`                  | ✅  | ❌  |
| Frictionloss      | `Joint`      | `set_frictionloss_override`      | `get_frictionloss_override`   | `(batch,)`                  | ✅  | ❌  |
| Kp Gain           | `Actuator`   | `set_kp_override`                | `get_kp_override`             | `(batch,)`                  | ✅  | ✅  |
| Damping (Kd)      | `Actuator`   | `set_damping_override`           | `get_kd_override`             | `(batch,)`                  | ✅  | ✅  |
| Gravity           | `SceneModel` | `set_gravity_override`           | `get_gravity_override`        | `(batch, 3)`                | ✅  | ❌  |

```{note}
All override methods act on `SceneData` and do not modify the `SceneModel` itself. This allows the same model to use different parameters across different instances.
```

```{note}
The GPU backend currently supports mass, center of mass, geometry friction, actuator Kp, and actuator damping overrides. Gravity, armature, and friction-loss overrides are CPU-only; workflows that depend on them should use the CPU backend. A "—" means GPU support is not yet documented for that row.

Actuator Kp and damping overrides only apply to position actuators targeting a joint or floating_base. Other actuator types such as Motor and Velocity, as well as tendon / link targets, raise `ValueError`.
```

## Core Concept

Parameter randomization is implemented through the **Override API**. Overrides act on `SceneData` rather than `SceneModel`, meaning different simulation instances of the same model can have different physical parameters without reloading the model.

Basic usage pattern:

```python
# Create batch simulation data
data = mtx.SceneData(model, batch=(16,))

# Get model object
link = model.get_link("body_name")

# Set override (shape includes batch dimension)
mass = np.random.uniform(0.1, 10.0, size=(16,))
link.set_mass_override(data, mass)

# Read override
mass_get = link.get_mass_override(data)
```

## Supported Randomization Parameters

### Link Parameters

#### Mass Randomization

Use `Link.set_mass_override` to modify link mass. Mass, center of mass, and the inertia tensor about the center of mass are independent runtime properties, so changing mass does not implicitly scale the inertia tensor.

```{figure} /_static/images/examples/randomize_mass.jpg
:width: 100%
:align: center
```

```{literalinclude} ../../../../examples/randomize/mass.py
:language: python
:dedent:
:lines: 27-47
```

Full example: [`examples/randomize/mass.py`](../../../../examples/randomize/mass.py)

#### Center of Mass Randomization

Use `Link.set_center_of_mass_override` to modify link center of mass position, affecting rotational behavior and stability.

```{figure} /_static/images/examples/randomize_com.jpg
:width: 100%
:align: center
```

```python
com_offset = np.zeros((16, 3), dtype=np.float32)
com_offset[:, 0] = np.random.uniform(-0.25, 0.25, 16)
link.set_center_of_mass_override(data, com_offset)
```

Full example: [`examples/randomize/com.py`](../../../../examples/randomize/com.py)

#### Inertia Tensor Randomization

Use `Link.set_inertia_override` to set the rotational inertia tensor about the current center of mass in the link local frame. The six components are ordered like MJCF `fullinertia`: `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`. The input shape is `(6,)` for broadcast or `(*data.shape, 6)` for per-instance values. Mass and center of mass remain unchanged.

```python
periods = np.linspace(2.5, 8.0, 16, dtype=np.float32)
omega = 2 * np.pi / periods
axis_inertia = (stiffness + np.sqrt(stiffness**2 - damping**2 * omega**2)) / (2 * omega**2)
inertia = np.zeros((16, 6), dtype=np.float32)
inertia[:, 0] = axis_inertia  # Ixx
inertia[:, 1] = axis_inertia  # Iyy
inertia[:, 2] = axis_inertia  # Izz
link.set_inertia_override(data, inertia)
```

Full example: [`examples/randomize/inertia.py`](../../../../examples/randomize/inertia.py)

#### Principal Inertia Randomization

Use `Link.set_principal_inertia_override` to randomize the inertia tensor through its principal moments and principal axes orientation. This is a convenience over `set_inertia_override`: the inputs are combined atomically into `I = R @ diag(principal_moments) @ R.T` about the current center of mass in the link local frame. `principal_moments` has shape `(3,)` or `(*data.shape, 3)`, and `principal_axes` is an `[x, y, z, w]` quaternion rotating principal-frame axes into the link local frame, with shape `(4,)` or `(*data.shape, 4)`. The quaternion is normalized internally, and each argument may independently be broadcast or per-instance.

```python
moments = np.zeros((16, 3), dtype=np.float32)
moments[:, 0] = axis_inertia
moments[:, 1] = axis_inertia * np.random.uniform(1.0, 1.4, 16)
moments[:, 2] = axis_inertia * np.random.uniform(1.0, 1.4, 16)

angles = np.random.uniform(-np.pi, np.pi, 16).astype(np.float32)
axes = np.zeros((16, 4), dtype=np.float32)
axes[:, 2] = np.sin(angles / 2)
axes[:, 3] = np.cos(angles / 2)

link.set_principal_inertia_override(data, moments, axes)
```

Read back the effective tensor with `get_inertia_override`, which stays the canonical getter: when the principal moments have repeated roots, the principal axes are not unique, so no principal-form getter is provided.

### Geometry Parameters

#### Friction Randomization

Use `Geom.set_friction_override` to modify geometry friction coefficients. The friction is a 3D vector `(slide, spin, roll)`.

```{figure} /_static/images/examples/randomize_friction.jpg
:width: 100%
:align: center
```

```python
geom = model.get_geom("box_geom")
frictions = np.zeros((16, 3), dtype=np.float32)
frictions[:, 0] = np.linspace(0.01, 2.0, 16)  # slide friction
geom.set_friction_override(data, frictions)
```

Full example: [`examples/randomize/friction.py`](../../../../examples/randomize/friction.py)

#### Geometry Size Randomization

Use the `set_size_override` method on geometry subclasses to modify collision shape sizes. Different geometry types have different size dimensions:

```{figure} /_static/images/examples/randomize_geom_size.jpg
:width: 100%
:align: center
```

| Geometry Type   | Size Dims | Meaning             |
| :-------------- | :-------- | :------------------ |
| `GeomSphere`    | 1         | Radius              |
| `GeomCapsule`   | 2         | Radius, half-height |
| `GeomCuboid`    | 3         | X/Y/Z half-extents  |
| `GeomCylinder`  | 2         | Radius, half-height |
| `GeomEllipsoid` | 3         | X/Y/Z semi-axes     |

```python
from motrixsim import GeomSphere

sphere = model.get_geom("sphere")
assert isinstance(sphere, GeomSphere)
radii = np.random.uniform(0.1, 0.8, size=(16, 1)).astype(np.float32)
sphere.set_size_override(data, radii)
```

Full example: [`examples/randomize/geom_size.py`](../../../../examples/randomize/geom_size.py)

#### Geometry Shape Randomization

Use `Geom.set_shape_override` to select a primitive collision and visual shape independently for each batch instance. Primitive ids follow `Shape.types()`. Every array argument (`shape`, `size`, `variant`) must have the same batch dimension as `data`; single-value broadcasting is not accepted. Mesh candidates must be registered on the scene before building the model via `msd.World.mesh_variant_sets` (each a `msd.GeometryMeshVariantSet` holding an ordered `meshes` list), and a geometry references one set by name through `msd.Geometry.mesh_variant_set`; selection then passes the set-local ordinal. `model.mesh_variant_sets` returns each set's ordered candidate asset names (position is the ordinal), and persisted configurations should store asset names and resolve them to ordinals at use time. MJCF does not currently expose a variant declaration syntax.

```python
from motrixsim import DisjointIndices, Shape

# One Shape ordinal per data instance; the length must match the data batch.
shape_ids = np.resize(
    np.array([Shape.Sphere, Shape.Cylinder, Shape.Capsule, Shape.Cuboid], dtype=np.int64),
    16,
)
geom.set_shape_override(data, shape_ids)

# Select a mesh candidate by its set-local ordinal; candidate asset names are
# positionally indexed.
candidates = model.mesh_variant_sets[geom.mesh_variant_set]
geom.set_shape_override(
    data[DisjointIndices([0])],
    np.array([Shape.Mesh], dtype=np.int64),
    variant=np.array([candidates.index("tetra")], dtype=np.int64),
)
```

`RenderApp.sync(data)` updates the visual geometry to match the effective selection. Mesh variants render their source meshes; collision cooking hulls remain an internal physics representation.

#### Geometry active randomization

`Geom.set_active_override` makes a geom absent for selected instances (no collision, no raycast hit, no rendering). It is orthogonal to the shape selection, accepted for every geom (including non-switchable Plane/HField/SDF/mesh geoms), and keeps the shape/size/pose overrides while inactive — re-enabling restores them. Use it for slots that exist in the canonical model but are missing from some instances' asset variants (e.g. headless tools).

```python
# Mixed absence: instance 0 has no head part, instance 1 is normal.
geom.set_active_override(data, np.array([False, True]))
# Scalar broadcast: disable / enable every instance.
geom.set_active_override(data, False)
```

`get_active_override(data)` returns a `(*data.shape,)` boolean array. The flag does not couple to mass properties; combine it with `Link.set_inertia_override` when needed.

Full example: [`examples/randomize/geom_shape.py`](../../../../examples/randomize/geom_shape.py)

#### Geometry pose randomization

`Geom.set_pose_override` replaces the per-instance local `pos`/`quat` (the mounting pose relative to the parent link). The pose is a replacement: it is orthogonal to the shape/size selections, and a mesh candidate's own asset transform still composes on top. `pos` accepts a `(3,)` broadcast or `(*data.shape, 3)`; `quat` is `[i, j, k, w]` order and accepts `(4,)` or `(*data.shape, 4)`, normalized internally. Instances with invalid input (NaN translation or zero-norm quaternion) are silently ignored.

```python
geom.set_pose_override(
    data,
    np.array([[0.0, 0.0, 0.05], [0.0, 0.0, -0.05]], dtype=np.float32),
    np.array([[0, 0, 0, 1], [0, 0, 0, 1]], dtype=np.float32),
)
```

The getters `get_pos_override(data)` / `get_quat_override(data)` return `(*data.shape, 3)` and `(*data.shape, 4)` (`[i, j, k, w]`) respectively, with optional `out=` pre-allocated buffers.

### Joint Parameters

#### Armature Randomization

Use `Joint.set_armature_override` to modify joint armature (virtual rotor inertia), affecting joint dynamic response speed.

```{figure} /_static/images/examples/randomize_armature.jpg
:width: 100%
:align: center
```

```python
joint = model.get_joint("rotor_joint")
armature = np.linspace(0.1, 5.0, 16).astype(np.float32)
joint.set_armature_override(data, armature)
```

Full example: [`examples/randomize/armature.py`](../../../../examples/randomize/armature.py)

#### Joint Frictionloss Randomization

Use `Joint.set_frictionloss_override` to modify Coulomb dry friction at the joint, affecting joint resistance characteristics.

```{figure} /_static/images/examples/randomize_frictionloss.jpg
:width: 100%
:align: center
```

```python
joint = model.get_joint("pendulum_hinge")
frictionloss = np.linspace(0.0, 200.0, 16).astype(np.float32)
joint.set_frictionloss_override(data, frictionloss)
```

Full example: [`examples/randomize/frictionloss.py`](../../../../examples/randomize/frictionloss.py)

### Actuator Parameters

#### Kp/Kd Randomization

Use `Actuator.set_kp_override` and `Actuator.set_damping_override` to modify the proportional and derivative gains of position actuators.

```{figure} /_static/images/examples/randomize_actuator_kp_kd.jpg
:width: 100%
:align: center
```

```python
actuator = model.get_actuator("servo")
kp = np.array([10.0, 30.0, 60.0, 120.0], dtype=np.float32)
kd = np.array([1.0, 3.0, 8.0, 20.0], dtype=np.float32)
actuator.set_kp_override(data, kp)
actuator.set_damping_override(data, kd)
```

Full example: [`examples/randomize/actuator_kp_kd.py`](../../../../examples/randomize/actuator_kp_kd.py)

### Scene Parameters

#### Gravity Direction Randomization

Use `SceneModel.set_gravity_override` to modify the gravity vector for each instance.

```{figure} /_static/images/examples/randomize_gravity_direction.jpg
:width: 100%
:align: center
```

```python
gravity = np.random.randn(16, 3).astype(np.float32)
gravity /= np.linalg.norm(gravity, axis=1, keepdims=True)
gravity *= 9.81
model.set_gravity_override(data, gravity)
```

Full example: [`examples/randomize/gravity_direction.py`](../../../../examples/randomize/gravity_direction.py)
