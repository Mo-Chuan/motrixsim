# 🔍 CPU 与 GPU API 差异

本页面聚焦 CPU 和 GPU 版本的 API 差异及迁移注意事项，不是自动生成的接口清单。两个版本公开 API 高度一致，但底层实现差异显著；完整接口请参阅 [GPU API 参考](../../api_reference/gpu/index.md)。

## 导入和初始化

### 包导入差异

::::{tab-set}
:::{tab-item} CPU 版本

```python
import motrixsim as mx
```

:::

:::{tab-item} GPU 版本

```python
import motrixsim.gpu as mx
```

:::
::::

两边的核心 import 都只需要一行。CPU 版本的数据数组位于主机内存，类型为 `numpy.ndarray`；GPU 版本的数据数组位于 GPU 显存，类型为 `warp.array`。**当用户代码需要直接操作这些数组时**，CPU 侧使用 `numpy`，GPU 侧使用 `warp`（例如编写自定义 kernel）。这是两种数组类型各自的配套依赖，而不是切换仿真版本带来的额外要求。

### 模型加载

CPU 和 GPU 版本都支持导入 MuJoCo 模型格式（MJCF）和统一机器人描述格式（Unified Robot Description Format，URDF）。两者使用相同的模型加载函数和返回类型：

| 功能         | GPU 版本          | CPU 版本          | 说明             |
| ------------ | ----------------- | ----------------- | ---------------- |
| **函数名**   | `mx.load_model()` | `mx.load_model()` | API 名称一致     |
| **返回类型** | `SceneModel`      | `SceneModel`      | 返回类型名称一致 |
| **支持格式** | MJCF, URDF        | MJCF, URDF        | 支持格式相同     |

```python
# GPU
model = mx.load_model("scene.xml")    # mx = motrixsim.gpu

# CPU
model = mx.load_model("scene.xml")    # mx = motrixsim
```

GPU 版本的 `load_model()` / `load_mjcf_str()` 还接受两个容量参数：

-   `max_contact_constraints`：接触约束行容量的下限。
-   `max_contacts`：接触点和热启动缓冲容量。模型构建完成后，约束行容量会被提升到不低于接触点容量，因此提高 `max_contacts` 也可能增加约束行预算和显存占用。

(data-container-label)=

## 数据容器

### 数据对象创建

CPU 和 GPU 版本都使用 `SceneData` 保存运行时状态和控制输入。一个 world 是同一模型的一个独立仿真实例，拥有自己的关节位置、速度等状态；实例总数用 `num_worlds` 表示。

表中的 0D 和 1D 描述 `batch` 参数自身的维度，而不是状态数组的最终秩：

-   `batch=()`（0D）表示单个 world，即 `num_worlds=1`。
-   `batch=(N,)`（1D）表示 N 个 world，即 `num_worlds=N`。

两种形式暴露的 `SceneData.shape` 不同：CPU 保留输入 batch 形状，因此默认是 `()`；GPU 始终保留 world 维，因此默认是 `(1,)`。

| 特性         | CPU 版本                      | GPU 版本                        |
| ------------ | ----------------------------- | ------------------------------- |
| **类名**     | `SceneData`                   | `SceneData`                     |
| **批量维度** | 0D/1D：`()` 或 `(N,)`         | 0D/1D：`()` / `(N,)` / 标量 `N` |
| **API 形式** | `SceneData(model, batch=...)` | `SceneData(model, batch=...)`   |

```{warning}
CPU 和 GPU 版本都只支持 **0D 或 1D** 批量：`batch=()`（单个 world）或 `batch=(N,)`。传入多维形状（如 `batch=(M, N)`）会报错。CPU 版本要求使用序列形式；GPU 版本还额外接受标量 `batch=N`。
```

:::{warning}
**`batch` 参数对 GPU 版本性能影响显著：**

-   **过小**（如 `batch < 256`）：GPU 并行度不足，吞吐量反而可能不如 CPU
-   **过大**：配置的 batch 所需内存超过可用 GPU 显存时会抛出 `RuntimeError: CUDA out of memory`

GPU 显存由当前进程和同一张 GPU 上的其它任务共享。可容纳的 batch 大小取决于 GPU 总显存，以及模型、接触约束、训练临时分配和其它任务当前占用的显存；配置 batch 时需要为后续任务留出余量。具体性能与批量大小的关系见[多环境并行仿真](../main_function/parallel.md)中的实测基准。
:::

::::{tab-set}
:::{tab-item} CPU 版本

```python
# 单环境
data = mx.SceneData(model)

# 批量环境
data = mx.SceneData(model, batch=(1000,))
```

:::

:::{tab-item} GPU 版本

```python
# 单环境
data = mx.SceneData(model)

# 批量环境
data = mx.SceneData(model, batch=(1000,))
```

:::
::::

### 数据数组类型

以 `data = mx.SceneData(model)` 为例，`data` 保存仿真运行时的状态和控制输入：

-   `data.dof_pos` 保存关节的广义位置。
-   `data.dof_vel` 保存关节的广义速度。
-   `data.actuator_ctrls` 保存传给各执行器的控制值。

这些字段都是浮点数组。CPU 版本使用 `numpy.ndarray` 并存储在主机内存；GPU 版本使用 `warp.array` 并存储在 GPU 显存。下表以 `data.dof_pos` 为例，对比两个版本的存储位置、形状和调用方式：

| 属性         | CPU 版本                                 | GPU 版本                                     | 说明               |
| ------------ | ---------------------------------------- | -------------------------------------------- | ------------------ |
| **数组类型** | `numpy.ndarray`                          | `warp.array`                                 | 数据接口不同       |
| **属性访问** | `data.dof_pos`                           | `data.dof_pos`                               | 字段名称一致       |
| **形状**     | `(batch..., dof)`；`batch=()` → `(dof,)` | `(num_worlds, dof)`；`batch=()` → `(1, dof)` | GPU 保留 world 维  |
| **存储位置** | 主机内存                                 | GPU 显存                                     | 存储介质不同       |
| **同步需求** | 无需显式同步                             | 需要显式同步                                 | 执行模型不同       |
| **类型转换** | 直接使用 NumPy API                       | 转回主机时需调用 `.numpy()`                  | 产生额外性能开销   |

当 `batch=()` 时，CPU 字段没有 batch 维（`dof_pos.shape == (num_dof,)`），GPU 字段仍保留前导 world 维（`dof_pos.shape == (1, num_dof)`）。

:::{warning}
**数组类型注意事项**:

-   NumPy 函数不能直接操作 GPU 的 `warp.array`；某个高层 API 是否接受 NumPy 输入需要查看该接口说明
-   **推荐做法**：自定义计算用 warp kernel 在 GPU 上完成，避免 GPU 设备到主机（D2H）的数据传输
-   只有在结果最终要离开 GPU（保存、调试、可视化等）时才用 `.numpy()` 拷回主机内存
-   频繁的主机与 GPU 设备数据传输会增加传输开销，并削弱 CUDA Graph 带来的加速效果

:::

## 数据操作

### 读取数据

:::::{tab-set}
::::{tab-item} GPU 版本（推荐：纯 GPU 处理）

```python
import warp as wp

# 访问warp数组
positions = data.dof_pos  # warp.array
velocities = data.dof_vel  # warp.array

# ✅ 推荐做法：在GPU上直接处理数据，避免CPU-GPU传输
@wp.kernel
def compute_mean_vel(
    velocities: wp.array2d(dtype=float),
    mean_out: wp.array(dtype=float),
):
    env_idx = wp.tid()
    # 在GPU上计算每个环境的平均速度
    n_dof = velocities.shape[1]
    s = float(0.0)
    for i in range(n_dof):
        s += velocities[env_idx, i]
    mean_out[env_idx] = s / float(n_dof)

mean_out = wp.zeros(batch_size, dtype=float, device="cuda")
wp.launch(compute_mean_vel, dim=batch_size, inputs=[velocities], outputs=[mean_out])
# mean_out 仍然在GPU上，可继续被后续kernel使用
```

::::

::::{tab-item} CPU 版本

```python
# 直接访问numpy数组
positions = data.dof_pos  # numpy.ndarray
velocities = data.dof_vel  # numpy.ndarray

# 数组操作
first_pos = positions[0]  # 第一个环境的位置
mean_vel = velocities.mean(axis=1)  # 每个 world 的平均速度
```

::::

::::{tab-item} GPU 版本（仅在最终读取/调试时）

```python
# ⚠️ 仅在确实需要CPU侧数据时使用（如训练结束保存结果、调试观察）
# 频繁调用会引入CPU-GPU同步开销，严重拖慢训练
wp.synchronize()
pos_np = positions.numpy()  # GPU -> CPU 拷贝
velocities_np = velocities.numpy()
first_pos = pos_np[0]
mean_vel = velocities_np.mean(axis=1)
```

::::
:::::

### 设置数据

::::{tab-set}
:::{tab-item} CPU 版本

```python
import numpy as np

# 直接numpy操作 - 零开销
data.dof_pos[0] = 1.0  # ✅ 设置单个值
data.dof_pos[:, 0] = np.ones(batch_size)  # ✅ 设置整列

# 批量设置 - 内存复制
new_positions = np.random.random((batch_size, num_dof))
data.dof_pos[:] = new_positions  # ✅ 直接赋值
```

:::

:::{tab-item} GPU 版本

```python
import warp as wp
import numpy as np

# ❌ 在 GPU 版本中，不能在 Warp kernel 外用 [] 直接给 warp.array 赋值：
#    data.dof_pos[0] = 1.0          → TypeError
#    data.dof_pos[:, 0] = ...       → TypeError

# 路径 A：数据天生在 CPU 端（随机初始化、从配置/磁盘读入）——用 set_dof_pos 整块写入。
#   会触发一次主机到 GPU 设备（H2D）传输。
new_positions = np.random.random((batch_size, num_dof))
data.set_dof_pos(new_positions)

# 路径 B：数据需要每步生成（训练循环中的目标位置、随机扰动）——
#   直接用 warp kernel 在 GPU 上写入 data.dof_pos，不经过主机内存，避免 H2D 传输。
wp.launch(my_init_kernel, dim=batch_size, outputs=[data.dof_pos])
```

:::
::::

:::{warning}
**GPU 数据写入要点**:

-   Warp kernel 外不能用 `[]` 直接给 `warp.array` 赋值，例如 `a[i] = x` / `a[:, j] = ...` 都会抛出 `TypeError`
-   数据本来就在 CPU 端时：用 `data.set_dof_pos(numpy)` / `set_dof_vel` / `set_actuator_ctrls` 等公开 setter
-   数据需要每步生成时：用 warp kernel 在 GPU 上直接写入对应字段，避免主机到 GPU 设备的数据传输

:::

## 仿真执行

### 步进函数

`mx.step(model, data)` 将仿真向前推进一个时间步，并更新 `data` 中的关节位置、速度等状态。CPU 和 GPU 版本使用相同的函数名，参数名称和含义也一致：

| 功能         | CPU 版本        | GPU 版本                                               | 说明                     |
| ------------ | --------------- | ------------------------------------------------------ | ------------------------ |
| **函数名**   | `step`          | `step`                                                 | 名称一致                 |
| **参数**     | `(model, data)` | `(model, data)`                                        | 参数名称与含义一致       |
| **执行方式** | 同步            | 异步（可用 `wp.synchronize()` 等待）                   | 执行同步性存在差异       |

```python
mx.step(model, data)    # GPU
mx.step(model, data)    # CPU
```

### 性能优化

::::{tab-set}
:::{tab-item} CPU 版本

```python
# CPU版本无需特殊优化
for i in range(1000):
    mx.step(model, data)
```

:::

:::{tab-item} GPU 版本

```python
# GPU版本支持CUDA Graph优化
model.capture_step(data)   # 预录制低层物理 Graph
mx.step(model, data)       # 建立高层数据 Graph，并推进一次
wp.synchronize()

for i in range(1000):
    mx.step(model, data)   # 复用已录制的 Graph

# 需要时显式同步
wp.synchronize()
```

:::
::::

## 组件访问

### 执行器控制

GPU 版本的 `actuator.set_ctrl(data, ctrl)` 接受 `warp.array`，也接受形状为 `(num_worlds,)` 的 NumPy 数组；单 world 还接受 `float`。NumPy 输入会触发主机到 GPU 设备的数据传输，因此在性能关键路径上优先使用 GPU 数组。

::::{tab-set}
:::{tab-item} CPU 版本

```python
# 获取执行器
actuator = model.get_actuator(0)

# 设置控制值
ctrl = np.ones(batch_size) * 0.5
actuator.set_ctrl(data, ctrl)

# 读取控制值
current_ctrl = actuator.get_ctrl(data)
```

:::

:::{tab-item} GPU 版本

```python
# 获取执行器(API相同)
actuator = model.get_actuator(0)

# 设置控制值：性能关键路径优先使用 GPU 数组
ctrl = wp.full((batch_size,), 0.5, device="cuda")
actuator.set_ctrl(data, ctrl)

# 也支持 NumPy 数组（形状为 (num_worlds,)），但会产生 H2D 传输
# actuator.set_ctrl(data, np.full(batch_size, 0.5, dtype=np.float32))

# 读取控制值
current_ctrl = actuator.get_ctrl(data)  # 返回warp.array
```

:::
::::

### 传感器数据

| 功能         | CPU 版本                             | GPU 版本                             | 差异         |
| ------------ | ------------------------------------ | ------------------------------------ | ------------ |
| **访问方式** | `model.get_sensor_value(name, data)` | `model.get_sensor_value(name, data)` | API 相同     |
| **返回类型** | `numpy.ndarray`                      | `warp.array`                         | 数组类型不同 |
| **批量处理** | ✅                                   | ✅                                   | 都支持批量处理 |

## 内存与显存数据交换

### 主机内存与 GPU 显存数据交换

`numpy.ndarray` 通常位于主机内存，`warp.array` 通常位于 GPU 显存。需要在两者之间传递数据时，可参考以下示例：

```python
# GPU数组转numpy(GPU -> CPU)
gpu_array = data.dof_pos  # warp.array
cpu_array = gpu_array.numpy()  # 转换为numpy

# numpy 转 GPU 数组（CPU → GPU）
cpu_data = np.random.random((batch_size, num_dof))
gpu_data = wp.array(cpu_data, device="cuda")

# 或者通过 SceneData 的公开 setter 直接写入仿真状态字段
data.set_dof_pos(cpu_data)
```

(common-pitfalls-label)=

## 常见陷阱和最佳实践

### 1. 数组类型不匹配

GPU 高层 API 接受的输入类型需要查看具体接口。例如 `SceneData.set_dof_pos()` 和 `Actuator.set_ctrl()` 可以接收 NumPy 输入并完成 H2D 传输，而 Warp kernel 或明确要求 `warp.array` 的接口需要设备数组。

```python
# ❌ 错误：Warp kernel 的参数要求 warp.array
cpu_data = np.ones(100)
wp.launch(my_kernel, dim=100, inputs=[cpu_data])  # TypeError: warp.array expected

# ✅ 正确：先转换为 warp.array
gpu_data = wp.array(cpu_data, device="cuda")
wp.launch(my_kernel, dim=100, inputs=[gpu_data])
```

### 2. 频繁 D2H 拷回的性能问题

```python
# ⚠️ .numpy() 本身就会触发 GPU 设备到主机（D2H）同步，等待 GPU 跑完——数据不会"过期"，
#     但每次 step 都做这件事会让 step 之间无法并行调度，CUDA Graph 也失去收益
for i in range(100):
    mx.step(model, data)
    result = data.dof_pos.numpy()  # 每步同步，慢

# ✅ 让 step 之间保持异步，只在确实需要 CPU 数据时同步一次
for i in range(100):
    mx.step(model, data)
wp.synchronize()                   # 一次性同步
final_result = data.dof_pos.numpy()
```

### 3. 错误的性能期望

```{note}
**认知误区**：GPU 并非在所有场景下都比 CPU 快——小批量下 GPU 并行度不足，反而可能不如 CPU。具体性能与 batch 的关系见 {ref}`实测性能基准 <gpu-benchmark>`。
```

参考 {ref}`数据容器说明 <data-container-label>` 了解更多批量配置细节。

🔗 **相关文档**:

-   [多环境并行仿真](../main_function/parallel.md) - GPU 并行仿真入门
