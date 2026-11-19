# 🎲 参数随机化

{bdg-success-line}`CPU 完全支持` {bdg-warning-line}`GPU 部分支持`

MotrixSim 提供了参数随机化（Domain Randomization）能力，允许您在多环境并行仿真中为每个实例设置不同的物理参数。这对于 Sim-to-Real 迁移和强化学习训练至关重要。

## 支持的随机化参数总览

| 属性         | 对象         | 设置方法                         | 获取方法                      | 数据维度                    | CPU | GPU |
| :----------- | :----------- | :------------------------------- | :---------------------------- | :-------------------------- | :-: | :-: |
| 质量         | `Link`       | `set_mass_override`              | `get_mass_override`           | `(batch,)`                  | ✅  | ✅  |
| 质心         | `Link`       | `set_center_of_mass_override`    | `get_center_of_mass_override` | `(batch, 3)`                | ✅  | ✅  |
| 惯量张量     | `Link`       | `set_inertia_override`           | `get_inertia_override`        | `(batch, 6)`                | ✅  | —   |
| 主惯量+主轴  | `Link`       | `set_principal_inertia_override` | —                             | `(batch, 3)` + `(batch, 4)` | ✅  | —   |
| 摩擦力       | `Geom`       | `set_friction_override`          | `get_friction_override`       | `(batch, 3)`                | ✅  | ✅  |
| 几何体尺寸   | `Geom*`      | `set_size_override`              | `get_size_override`           | `(batch, N)`                | ✅  | —   |
| 几何体启停   | `Geom`       | `set_active_override`            | `get_active_override`         | `(batch,)`                  | ✅  | —   |
| 几何体位姿   | `Geom`       | `set_pose_override`              | `get_pose_override`           | `(batch, 3)` + `(batch, 4)` | ✅  | —   |
| 电枢惯量     | `Joint`      | `set_armature_override`          | `get_armature_override`       | `(batch,)`                  | ✅  | ❌  |
| 关节摩擦损耗 | `Joint`      | `set_frictionloss_override`      | `get_frictionloss_override`   | `(batch,)`                  | ✅  | ❌  |
| 比例增益     | `Actuator`   | `set_kp_override`                | `get_kp_override`             | `(batch,)`                  | ✅  | ✅  |
| 阻尼增益     | `Actuator`   | `set_damping_override`           | `get_kd_override`             | `(batch,)`                  | ✅  | ✅  |
| 重力         | `SceneModel` | `set_gravity_override`           | `get_gravity_override`        | `(batch, 3)`                | ✅  | ❌  |

```{note}
所有 override 方法都作用于 `SceneData`，不会修改 `SceneModel` 本身。这使得同一个模型可以在不同实例中使用不同的参数。
```

```{note}
GPU 后端目前支持质量、质心、几何摩擦、执行器 Kp 和执行器阻尼 override。重力、电枢惯量和关节摩擦损耗仅 CPU 后端支持，依赖这些参数的流程请使用 CPU 后端；表中 "—" 表示 GPU 支持情况尚未单独记录。

执行器 Kp 和阻尼 override 仅适用于目标为 joint 或 floating_base 的 position actuator。Motor、Velocity 等其他执行器类型，以及 tendon / link target，会抛出 `ValueError`。
```

## 核心概念

参数随机化通过 **Override API** 实现。Override 作用于 `SceneData` 而非 `SceneModel`，这意味着同一个模型的不同仿真实例可以拥有不同的物理参数，而无需重新加载模型。

基本使用模式：

```python
# 创建批量仿真数据
data = mtx.SceneData(model, batch=(16,))

# 获取模型对象
link = model.get_link("body_name")

# 设置 override（shape 包含 batch 维度）
mass = np.random.uniform(0.1, 10.0, size=(16,))
link.set_mass_override(data, mass)

# 读取 override
mass_get = link.get_mass_override(data)
```

## 支持的随机化参数

### 连杆参数

#### 质量随机化

通过 `Link.set_mass_override` 修改连杆质量。质量、质心和关于质心的惯量张量是相互独立的运行时属性，因此修改质量不会隐式缩放惯量张量。

```{figure} /_static/images/examples/randomize_mass.jpg
:width: 100%
:align: center
```

```{literalinclude} ../../../../examples/randomize/mass.py
:language: python
:dedent:
:lines: 27-47
```

完整示例：[`examples/randomize/mass.py`](../../../../examples/randomize/mass.py)

#### 质心随机化

通过 `Link.set_center_of_mass_override` 修改连杆质心位置，影响物体的旋转行为和稳定性。

```{figure} /_static/images/examples/randomize_com.jpg
:width: 100%
:align: center
```

```python
com_offset = np.zeros((16, 3), dtype=np.float32)
com_offset[:, 0] = np.random.uniform(-0.25, 0.25, 16)
link.set_center_of_mass_override(data, com_offset)
```

完整示例：[`examples/randomize/com.py`](../../../../examples/randomize/com.py)

#### 惯量张量随机化

通过 `Link.set_inertia_override` 设置 link 局部坐标系中、关于当前质心的转动惯量张量。六个分量采用与 MJCF `fullinertia` 一致的 `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]` 顺序。输入形状为 `(6,)` 时广播到所有实例，为 `(*data.shape, 6)` 时逐实例设置。质量和质心保持不变。

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

完整示例：[`examples/randomize/inertia.py`](../../../../examples/randomize/inertia.py)

#### 主惯量随机化

通过 `Link.set_principal_inertia_override` 以主惯量加主轴姿态的形态随机化惯量张量。它是 `set_inertia_override` 的便捷写法：两组输入在一次调用中原子合成为 `I = R @ diag(principal_moments) @ R.T`（关于当前质心、表达在 link 局部坐标系）。`principal_moments` 形状为 `(3,)` 或 `(*data.shape, 3)`；`principal_axes` 是把主轴坐标系旋转到 link 局部坐标系的 `[x, y, z, w]` 四元数，形状为 `(4,)` 或 `(*data.shape, 4)`。四元数在内部归一化，两个参数各自独立支持广播或逐实例输入。

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

读取有效张量仍使用 `get_inertia_override`（canonical getter）：主惯量出现重根（例如球对称）时主轴方向不唯一，因此不提供主惯量形态的 getter。

### 几何体参数

#### 摩擦力随机化

通过 `Geom.set_friction_override` 修改几何体的摩擦系数。摩擦系数是一个 3 维向量 `(slide, spin, roll)`。

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

完整示例：[`examples/randomize/friction.py`](../../../../examples/randomize/friction.py)

#### 几何体尺寸随机化

通过各几何体子类的 `set_size_override` 方法修改碰撞体尺寸。不同几何体类型的 size 维度不同：

```{figure} /_static/images/examples/randomize_geom_size.jpg
:width: 100%
:align: center
```

| 几何体类型      | size 维度 | 含义           |
| :-------------- | :-------- | :------------- |
| `GeomSphere`    | 1         | 半径           |
| `GeomCapsule`   | 2         | 半径, 半高     |
| `GeomCuboid`    | 3         | X/Y/Z 方向半长 |
| `GeomCylinder`  | 2         | 半径, 半高     |
| `GeomEllipsoid` | 3         | X/Y/Z 方向半径 |

```python
from motrixsim import GeomSphere

sphere = model.get_geom("sphere")
assert isinstance(sphere, GeomSphere)
radii = np.random.uniform(0.1, 0.8, size=(16, 1)).astype(np.float32)
sphere.set_size_override(data, radii)
```

完整示例：[`examples/randomize/geom_size.py`](../../../../examples/randomize/geom_size.py)

#### 几何体形状随机化

通过 `Geom.set_shape_override` 为每个批量实例独立选择 primitive 碰撞与渲染形状。Primitive id 顺序与 `Shape.types()` 一致。所有数组参数（`shape`、`size`、`variant`）的 batch 维度必须与 `data` 完全一致，不接受单值广播。Mesh 候选必须在构建 Model 前通过 `msd.World.mesh_variant_sets` 注册（每个 set 是一个持有有序 `meshes` 列表的 `msd.GeometryMeshVariantSet`），几何体通过 `msd.Geometry.mesh_variant_set` 按名字引用某个 set，选择时传入 set 内的候选序号；`model.mesh_variant_sets` 返回各 set 的有序候选资产名（位置即序号），持久化配置建议保存资产名、使用时再解析为序号。MJCF 暂不提供 variant 声明语法。

```python
from motrixsim import DisjointIndices, Shape

# 每个数据实例一个 Shape 序号，数组长度必须与 data batch 一致
shape_ids = np.resize(
    np.array([Shape.Sphere, Shape.Cylinder, Shape.Capsule, Shape.Cuboid], dtype=np.int64),
    16,
)
geom.set_shape_override(data, shape_ids)

# 按 set 内序号选择 mesh 候选，候选资产名的位置即序号
candidates = model.mesh_variant_sets[geom.mesh_variant_set]
geom.set_shape_override(
    data[DisjointIndices([0])],
    np.array([Shape.Mesh], dtype=np.int64),
    variant=np.array([candidates.index("tetra")], dtype=np.int64),
)
```

`RenderApp.sync(data)` 会将 visual geometry 同步为 effective selection。Mesh variant 渲染对应的 source Mesh；collision cooking hull 仍只是内部物理表示。

#### 几何体启停随机化

通过 `Geom.set_active_override` 让某个几何体在部分实例上不存在（absent slot）：无碰撞、无 raycast 命中、不渲染。它独立于形状选择，对所有几何体开放（包括不可换形的 Plane/HField/SDF/mesh），禁用期间的 shape/size/pose override 原样保留，重新启用即恢复。适合表达"canonical 模型有该槽位、部分实例的资产变体没有该零件"的场景（如无头工具的 head 槽）。

```python
# 混合禁用：实例 0 无 head 零件，实例 1 正常
geom.set_active_override(data, np.array([False, True]))
# 标量广播：全部实例禁用 / 启用
geom.set_active_override(data, False)
```

`get_active_override(data)` 返回 `(*data.shape,)` 的布尔数组。active 不联动质量属性；需要联动惯量时配合 `Link.set_inertia_override`。

完整示例：[`examples/randomize/geom_shape.py`](../../../../examples/randomize/geom_shape.py)

#### 几何体位姿随机化

通过 `Geom.set_pose_override` 替换每个批量实例的局部 `pos`/`quat`（相对父 link 的安装位姿）。位姿是替换式语义：与形状/尺寸 override 相互独立，mesh 候选自身的 asset 变换仍在其上叠加。`pos` 支持 `(3,)` 广播或 `(*data.shape, 3)`，`quat` 为 `[i, j, k, w]` 顺序、支持 `(4,)` 广播或 `(*data.shape, 4)`，内部归一化；非法输入（NaN/零范数四元数）的实例被静默忽略。

```python
geom.set_pose_override(
    data,
    np.array([[0.0, 0.0, 0.05], [0.0, 0.0, -0.05]], dtype=np.float32),
    np.array([[0, 0, 0, 1], [0, 0, 0, 1]], dtype=np.float32),
)
```

Getter `get_pos_override(data)` / `get_quat_override(data)` 分别返回 `(*data.shape, 3)` 与 `(*data.shape, 4)`（`[i, j, k, w]`），支持 `out=` 预分配缓冲。

### 关节参数

#### 电枢惯量随机化

通过 `Joint.set_armature_override` 修改关节的电枢惯量（虚拟转动惯量），影响关节的动态响应速度。

```{figure} /_static/images/examples/randomize_armature.jpg
:width: 100%
:align: center
```

```python
joint = model.get_joint("rotor_joint")
armature = np.linspace(0.1, 5.0, 16).astype(np.float32)
joint.set_armature_override(data, armature)
```

完整示例：[`examples/randomize/armature.py`](../../../../examples/randomize/armature.py)

#### 关节摩擦损耗随机化

通过 `Joint.set_frictionloss_override` 修改关节的库仑干摩擦，影响关节的阻力特性。

```{figure} /_static/images/examples/randomize_frictionloss.jpg
:width: 100%
:align: center
```

```python
joint = model.get_joint("pendulum_hinge")
frictionloss = np.linspace(0.0, 200.0, 16).astype(np.float32)
joint.set_frictionloss_override(data, frictionloss)
```

完整示例：[`examples/randomize/frictionloss.py`](../../../../examples/randomize/frictionloss.py)

### 驱动器参数

#### Kp/Kd 随机化

通过 `Actuator.set_kp_override` 和 `Actuator.set_damping_override` 修改位置驱动器的比例增益和阻尼增益。

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

完整示例：[`examples/randomize/actuator_kp_kd.py`](../../../../examples/randomize/actuator_kp_kd.py)

### 场景参数

#### 重力方向随机化

通过 `SceneModel.set_gravity_override` 修改每个实例的重力向量。

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

完整示例：[`examples/randomize/gravity_direction.py`](../../../../examples/randomize/gravity_direction.py)
