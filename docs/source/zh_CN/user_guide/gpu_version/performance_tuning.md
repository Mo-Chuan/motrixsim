(performance-optimization-label)=

# ⚡ 性能优化

## 核心优化：CUDA Graph 加速

`mx.step()` 内置了 CUDA Graph 逻辑，无需用户手动管理。**首次调用 `step()` 时会自动录制所需的低层物理 Graph 和高层数据 Graph，并执行一次仿真步进**；后续每次调用直接复用已录制的 Graph，无需再次录制。

### 基础用法：直接调用 step

```python
# 直接循环调用 step 即可，Graph 由内部自动管理
for i in range(1000):
    mx.step(model, data)
    # 第一次：录制 Graph + 执行一次真正的 step（开销较大）
    # 后续：复用 Graph
```

### 进阶用法：区分 Graph 预录制与完整预热

`capture_step()` 只预录制低层物理 Graph。它不执行仿真步进，也不会建立高层数据 Graph；高层数据 Graph 会在第一次完整 `step()` 中延迟建立。因此，性能计时前仍应完成一次 `step()` 并同步。

```python
import time
import warp as wp

# 在训练/性能测量循环之前，提前录制低层物理 Graph
print("🔧 预录制 CUDA Graph...")
model.capture_step(data)  # 只录制低层 Graph，不推进仿真状态

# 完成高层 Graph 的首次捕获，并推进一次仿真
mx.step(model, data)
wp.synchronize()

# 从这里开始计时；后续 step 复用已录制的低层和高层 Graph
start_time = time.time()
for i in range(1000):
    mx.step(model, data)

wp.synchronize()  # 等待 GPU 完成
total_time = time.time() - start_time
sps = (1000 * batch_size) / total_time
print(f"📊 性能: {sps:,.0f} steps/second")
```

:::{warning}
**CUDA Graph 使用注意事项:**

⚠️ **Graph 固定后不可改变的属性**:

-   batch_size
-   模型结构
-   捕获配置和绑定缓冲区

修改捕获配置或替换缓冲区后，调用 `model.invalidate_bindings(data)` 使旧绑定失效。向现有缓冲区写入新内容不需要重新捕获；模型拓扑或 solver layout 发生变化时，应重建匹配的 `SceneData`。

⚠️ **不要在循环中调用 `.numpy()`**:

-   GPU 设备到主机（D2H）的数据传输会与 GPU 默认流同步，削弱 Graph 带来的流水线收益
-   若步进过程中确实需要读取 GPU 数据（如 RL 训练计算 reward），优先把这部分计算搬到 GPU 上用 warp kernel 实现，详见 [数据传输优化](#数据传输优化)

⚠️ **首次 step 开销**:

-   录制时间通常远高于普通 step
-   `capture_step()` 可以提前承担低层 Graph 的录制开销，但不能替代首次完整 `step()`

ℹ️ **如需调试**: 使用 `step_no_capture()` 方法可绕过 Graph 直接执行（不享受 Graph 加速），便于查看中间结果。
:::

(数据传输优化)=

## 数据传输优化

每次 `.numpy()` 都会触发 GPU↔CPU 同步，破坏 CUDA Graph 的流水线收益。如果训练循环每步都需要 GPU 上的数据（比如 RL 中每步计算 reward），更好的做法是把**那部分计算也搬到 GPU 上**——用 warp kernel 直接在 GPU 上读取 `data` 字段（`dof_pos` / `dof_vel` / `actuator_ctrls` 等），结果保留在 GPU 上供下一次 step 或后续 kernel 使用，避免 CPU 介入。

### 在 GPU 上直接计算

并行仿真示例使用 Warp kernel 在 GPU 上生成随机控制量，并直接写入选中环境的 `data.actuator_ctrls`。随机数生成和控制量写入均在显存中完成。完整文件见 [`examples/gpu/parallelsim.py`](../../../../examples/gpu/parallelsim.py)：

```{literalinclude} ../../../../examples/gpu/parallelsim.py
:language: python
:pyobject: randomize_controls
```

写 warp kernel 的语法详见 [Warp 官方文档](https://nvidia.github.io/warp/)。

### `.numpy()` 何时是合理的

并非所有 `.numpy()` 都要避免——以下场景必须或合理：

-   **最终结果读取**：训练/仿真结束后导出数据
-   **调试与可视化**：检查 NaN、打印中间状态
-   **必要的 CPU 侧消费**：把数据喂给只能在 CPU 上跑的库（某些 PyTorch 模型、绘图工具等）

要避免的是**盲目地每步 `.numpy()`**——尤其是当那部分计算其实可以用 warp kernel 在 GPU 上完成时。

`max_contact_constraints` 是 `load_model()` / `load_mjcf_str()` 的可选参数，用于设置 GPU 上**预分配的接触约束缓冲区大小下限**。该参数仅 GPU 后端需要，CPU 后端走另一套接触构造路径。

```python
# 默认：自动估算
model = mx.load_model("scene.xml")

# 显式提高接触约束容量下限
model = mx.load_model("scene.xml", max_contact_constraints=1024)
```

`max_contacts` 是配套参数，用于提高预分配的接触点 / 热启动容量。模型构建完成后会保证**约束行容量不低于接触点容量**，因此提高 `max_contacts` 也可能提高约束行预算和显存占用：

```python
model = mx.load_model("scene.xml", max_contact_constraints=1024, max_contacts=2048)
```

### 为什么需要这个参数

接触检测在 GPU 上并行进行，**实际产生的接触数量在 GPU 端动态生成**——CPU 端只有付出同步代价才能得知。而每步都从 GPU 拷回这个计数来调整缓冲区大小，会引入 GPU↔CPU 同步、打断 GPU 异步流水线。因此 MotrixSim 的策略是：加载模型时基于碰撞几何配置在 CPU 端做静态估算，**一次性预分配固定大小缓冲区**，仿真运行期间不再调整。

但是，**静态估算无法保证是真实最大值的上界**——它综合了几个启发式估算的结果，对于大多数常见场景够用，但在以下情形下可能低估：

-   大量物体堆叠或紧密接触（抓取、堆垛、地形交互等）
-   场景包含高度场（HField），且地形分辨率高
-   柔性接触下多接触面同时激活

当实际产生的接触超出预分配容量时，超出部分的接触会被**丢弃**，可能导致：

-   物体穿透（应有的接触约束未生效）
-   物理行为异常
-   仿真结果不可复现

### 使用建议

-   **默认值覆盖大多数标准场景**：通常不需手动设置；但默认估算**不保证是真实最大值的上界**，仍可能在堆叠/复杂场景下不够
-   **如何判断默认值是否不够**：仿真过程中如果实际接触数超出预分配，会在控制台打印

    > `Error: Constraint overflow detected! Consider increasing the max_contact_constraint.`

    看到这个警告，就需要显式传入一个更大的 `max_contact_constraints`

-   **该参数是"下限"**：传入的值会与自动估算值取 `max`，因此**只能向上调整**，无法用它减小预分配
-   **避免过大**：缓冲区按 `batch` 与接触点、约束行容量分配；任一容量设置过大都会显著挤占显存

## 性能优化清单

在排查性能问题时，按以下顺序检查：

| 检查项                          | 影响       | 排查方法                                                                                               |
| ------------------------------- | ---------- | ------------------------------------------------------------------------------------------------------ |
| 是否绕开了 CUDA Graph           | ⭐⭐⭐⭐⭐ | 确认用的是 `step()` 而非 `step_no_capture()`；step 之间不要做让 Graph 失效的操作（如改 batch、改模型） |
| 循环内是否有 `.numpy()`         | ⭐⭐⭐⭐⭐ | 频繁拷回 CPU 会触发同步、破坏 Graph 流水线，用 warp kernel 在 GPU 上算                                 |
| batch_size 是否足够大           | ⭐⭐⭐⭐   | 小批量（<256）GPU 并行度不足，需要根据自己的硬件实测                                                   |
| step 外的 Python 逻辑是否过重   | ⭐⭐⭐     | step 之间的纯 Python 逻辑会限制 GPU 利用率；尽量把额外计算放进 warp kernel                             |
| 是否频繁创建新 GPU 数组         | ⭐⭐⭐     | 复用预分配的 `wp.array`，避免每步新分配                                                                |
| 是否盲目调用 `wp.synchronize()` | ⭐⭐⭐     | 仅在确实需要 CPU 数据时同步；GPU 异步执行能让 CPU 提前调度下一帧                                       |

🔗 **相关文档**:

-   [多环境并行仿真](../main_function/parallel.md) - GPU 并行仿真入门
-   [CPU 与 GPU API 差异](api_differences.md) - API 差异详解
