# Raycast Camera

{bdg-success-line}`CPU 完全支持` {bdg-danger-line}`GPU 不支持`

`RaycastCamera` 通过射线查询碰撞几何，输出深度、命中点和表面法线。它不依赖渲染器，可在无图形界面和批量仿真中使用。

传感器支持两种投影：针孔投影从同一光学原点发射射线；正交投影从规则网格发射平行射线，适合地形或高度采样。两种投影共用量程、输出字段、帧更新和碰撞过滤规则。

## 配置针孔相机

在 MJCF 的 `<mujoco>` 元素中添加以下配置：

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

`raycastcamera` 是 MotrixSim 的 MJCF 扩展，通过 `projection="pinhole"` 或 `projection="orthographic"` 选择投影。

## 坐标系与针孔射线

相机使用挂载 site 的完整位置和旋转。光学坐标系为右手系：+X 指向图像右方、+Y 指向图像下方、+Z 指向前方。

像素按行优先排列，索引为 `v * width + u`，像素中心为 `(u + 0.5, v + 0.5)`。针孔射线从光学原点出发，局部未归一化方向为：

```text
[(u+0.5-cx)/fx, (v+0.5-cy)/fy, 1]
```

射线查询使用归一化后的方向。`intrinsics="fx fy cx cy"` 以像素为单位，其中 `fx`、`fy` 必须为正有限值，`cx`、`cy` 必须有限；`width`、`height` 必须为正整数。

## 正交投影

### 配置向下采样

在 MJCF 的 `<mujoco>` 元素中添加以下配置：

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

`projection="orthographic"` 需要 `spacing="sx sy"`，不能同时提供针孔参数 `intrinsics`。`width`、`height` 必须为正整数；两个间距必须为正有限值。

此例将挂载 site 绕 X 轴旋转 180°，使相机局部 +Z 指向世界坐标系下方。相机使用 site 的完整位置和旋转；改变采样方向时，直接调整 site 的朝向。

### 网格布局

像素 `(u, v)` 的射线起点位于相机局部 Z=0 平面：

```text
[(u-(width-1)/2)*sx, (v-(height-1)/2)*sy, 0]
```

网格在局部 X、Y 方向以 site 原点为中心，所有射线沿局部 +Z 发射。`sx`、`sy` 分别是相邻列、相邻行起点之间的距离。

图像按行优先排列，像素索引为 `v * width + u`。光学坐标轴的方向见本章“坐标系与针孔射线”一节。

## 量程

两种投影的 `near`、`far` 都限定沿局部 +Z 方向的**光轴深度**，包含端点，必须满足 `0 <= near < far`，且 `far` 为有限正值。

-   先查询最近表面，再判断是否在量程内。位于 `near` 之前的表面会遮挡后方物体，不会被跳过。
-   应用 exclude 和组过滤后，最近命中的射线距离若精确为零，该像素视为 invalid，包括实体内部占据命中和表面起点的零距离命中；不恢复出口，也不跳过该碰撞体查询后方物体。
-   有效性逐像素判断：正交相机可能只有部分像素的射线起点位于碰撞体内。严格正距离命中不会因 epsilon 容差被额外拒绝。

## 输出字段

使用 `returns` 选择输出字段。默认只输出 `depth`，且至少需要启用 `depth` 或 `distance` 其中之一。

| 字段       | 维数 | 含义                                          |
| ---------- | ---- | --------------------------------------------- |
| `depth`    | 1    | 光轴深度，即射线距离乘以局部单位方向的 Z 分量 |
| `distance` | 1    | 从该像素的射线起点到命中点的射线距离          |
| `point`    | 3    | 世界坐标系中的命中点                          |
| `normal`   | 3    | 世界坐标系中的表面法线                        |

字段始终按表中顺序排列，禁用字段省略；MJCF 中字段名称的书写顺序不影响输出顺序。每条射线的浮点数数量 `stride` 等于启用字段的维数之和。

针孔投影的射线起点是相机光学原点，因此 `distance` 也是到相机原点的欧氏距离。正交投影中，每个像素有不同的射线起点，两个距离字段都等于光轴深度；`distance` **不是**到相机中心的欧氏距离。

像素 invalid 或未命中时，距离为 `-1`，点和法线为零。使用距离 `>= 0` 判断是否命中，不要根据命中点是否为零判断：世界原点也可能是有效命中点。

## 读取数据

### 读取针孔深度图像

`model.get_sensor_value()` 返回形状为 `(*data.shape, width * height * stride)` 的数组。按配置尺寸和字段布局重塑为 `(*data.shape, height, width, stride)`；批量或索引后的 `SceneData` 维度会保留。

下面假设 `model` 已加载上述针孔 MJCF 配置。由于只启用一个深度字段，可省略最后一个大小为 1 的维度：

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

循环读取时可复用 `buffer`，避免每次分配输出数组。

### 读取深度与表面高度

下面假设 `model` 已加载正交 MJCF 配置。配置启用了深度、命中点和法线，每条射线占 7 个浮点数：

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

`depth` 是从每个网格起点沿采样方向测得的深度，不是世界坐标系中的表面高度。`points` 使用世界坐标，因此 `points[..., 2]` 才是命中表面的世界 Z 坐标；读取时应先排除未命中像素。

如果只需要深度，可将 `returns` 设为 `depth`。正交投影下，`distance` 也等于该深度，而不是命中点到相机中心的距离。完整字段定义见本章“输出字段”一节。

## 帧更新

`update_period` 的单位是模拟秒，必须为非负有限值。

-   `update_period=0`：每个物理 step 采集一整帧。
-   正周期：尚未采样时，第一个物理 step 立即采集首帧；之后按累计模拟时间更新，未到更新时刻则保留上一帧。如果已通过 `forward_kinematic()` 生成首帧，后续 step 按周期更新，不会无条件重复采集。
-   一个 step 最多采集一帧，不追补多张历史帧，也不进行滚动扫描或子步位姿插值。

物理 step 中的采样使用**该步积分前**的相机和场景位姿。因此，step 返回后的当前相机位姿不一定对应缓存深度图的采样位姿；低频相机的图像也不一定对应本次 step 的最终状态。将深度图用于坐标变换时，不要直接用 step 返回后的当前位姿配对。

首次采集前，输出采用未命中值，但这些初值不代表一次测量。`model.get_sensor_value("camera", data)` 只读取缓存的完整帧，不会触发采样。

### 不推进物理的刷新

`model.forward_kinematic(data)` 更新运动学，不推进模拟时间，也不改变 DoF 位置和速度。相机是否采样取决于是否已有本次初始化或 reset 后的图像：

| 情况                        | 相机行为                         |
| --------------------------- | -------------------------------- |
| 新建 `SceneData` 后尚未采样 | 按当前位姿采集首帧               |
| reset 后尚未采样            | 按 reset 后的当前位姿采集首帧    |
| 已有图像                    | 保留原图，不绕过 `update_period` |

FK 生成首帧后，后续物理 step 累计一个采样周期才再次更新。重复调用 FK 不会推进或重启该周期。LiDAR 的行为见[显式 Forward Kinematics](./lidar.md#显式-forward-kinematics)。

### Reset 后获取图像

普通 `data.reset(model)` 不立即计算新图像；此时读取可能仍得到 reset 前的缓存，不能将其作为新一轮仿真的观测。reset 后第一次调用 `model.forward_kinematic(data)` 或执行物理 step 时，会生成新状态下的图像，并重新开始采样计时。

要在不推进模拟时间的情况下取得 reset 状态图像，可以使用 `data.reset(model, forward_kinematic=True)`，或先 reset 再调用 `model.forward_kinematic(data)`。

下面接续上面的批量针孔示例，仅重置选中的环境：

```python
reset_mask = np.array([True, False, True, False, False, False, False, False])
selected = data[reset_mask]
selected.reset(model, forward_kinematic=True)
reset_images = model.get_sensor_value("camera", selected).reshape(*selected.shape, 27, 48)
```

reset 和 FK 只作用于传入 `SceneData` 视图所选的环境；其他环境保留原图像和采样周期。针孔和正交相机遵循相同规则。

## 碰撞过滤

`exclude` 控制挂载物体的自过滤：

| 值                   | 含义                               |
| -------------------- | ---------------------------------- |
| `none`               | 不排除挂载物体                     |
| `parentbody`（默认） | 排除 site 所属物体的碰撞体         |
| `parentsubtree`      | 排除 site 所属物体及其子树的碰撞体 |

可选的 MJCF `groups="0 2"`只查询指定碰撞几何组。组号必须为 `0..=255` 范围内的整数，负数或超出上界的组号会报错；范围内的组号即使没有几何使用也合法。省略白名单时不按组限制；空白名单（`groups=""`）不查询任何碰撞体。该过滤规则独立于接触配置中的 `contype`、`conaffinity`。
