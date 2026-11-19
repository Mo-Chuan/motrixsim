# ​🔀 多环境并行仿真

{bdg-success-line}`CPU 完全支持` {bdg-success-line}`GPU 完全支持`

MotrixSim 内置了多环境并行仿真能力：CPU 后端将实例自动分配到多个核心，GPU 后端可在单卡上并行仿真数千到上万个环境，常用于大规模强化学习训练。两个后端共用同一套 batch 接口，本章通过 CPU 和 GPU 两个示例演示 100 个 tidybot 实例的并行仿真。

## 创建多实例

在构造 `SceneData` 时传入一个 `batch` 参数即可创建多实例：

```python
size = 10
batch_size = size * size
data = mx.SceneData(model, batch=(batch_size,))
```

### CPU 示例

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:language: python
:dedent:
:start-after: "# tag::init batch data"
:end-before:  "# end::init batch data"
```

上面的例子创建了 10×10=100 个 tidybot 模型实例。为了可视化方便，启动渲染器时为每个实例设置了不同的偏移位置，让其分布在一个平面网格上；这些实例会被自动分配到 CPU 的多个核心上进行并行仿真。

```{note}
render_offset 仅作用于渲染画面；物理实例仍位于原始位置。
```

运行命令：

```sh
uv run examples/parallel/parallelsim.py
```

### GPU 示例

GPU 示例同样创建 100 个 tidybot 实例，并预录制 CUDA Graph：

```sh
uv run --extra gpu python examples/gpu/parallelsim.py
```

键盘操作与 CPU 相同：A 随机设置全部实例的控制量，S 随机设置第一个实例，D 随机设置奇数索引实例，R 恢复全部实例的初始状态。GPU 示例通过自定义 Warp kernel 在显存中生成随机控制量，仅写入选中的实例；每次随机操作递增种子，以产生可复现的随机序列。

## 访问多实例数据

批量模式下，模型的状态数据以批量形式存储在 `SceneData` 中。所有 Data 相关的数据字段都会自动增加一个 Batch 维度，例如位置数据 `dof_pos` 的形状为 `(batch_size, num_dof_pos)`：

```python
batch = (1000,)
data = mtx.SceneData(model, batch=batch)
assert data.dof_vel.shape == (*batch, model.num_dof_vel)
```

数据设置接口同样需要增加一个 Batch 维度，但赋值方式因后端而异：

**CPU 后端**直接使用属性赋值：

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:dedent:
:start-after: "# tag:: set actuator ctrl in batch"
:end-before:  "# end:: set actuator ctrl in batch"
```

**GPU 后端**的数组位于显存中，类型为 `warp.array`（CPU 为 `numpy.ndarray`）。可以在 Warp kernel 中直接修改控制数组，也可以通过 `set_actuator_ctrls` 等方法上传主机端数据：

```python
batch_size = 2000
data = mx.SceneData(model, batch=(batch_size,))
assert data.dof_pos.shape == (batch_size, model.num_dof_pos)

# batch=() 等价于单个 world，与 CPU API 一致

ctrl_values = wp.full((batch_size,), 1.0, device="cuda")
actuator = model.get_actuator(0)
actuator.set_ctrl(data, ctrl_values)
```

两个后端的完整 API 差异见 [CPU 与 GPU API 差异](../gpu_version/api_differences.md)。

## 索引（仅 CPU 后端）

批量模式下，CPU 后端可以通过索引访问部分实例数据。

### 1. 访问单个实例

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:dedent:
:start-after: "# tag:: set actuator ctrl in single"
:end-before:  "# end:: set actuator ctrl in single"
```

上面的代码通过 `data[0]` 访问了第一个实例的数据。返回的 `single_data` 是一个新的 SceneData 对象，其 shape 为 `()`，表示标量实例数据。该实例数据与原始的 `data` 共享内存，因此对 `single_data` 的修改会直接反映到原始的 `data` 上。

### 2. 通过 mask 访问

SceneData 也支持通过 `NDArray[bool]` 进行索引，返回新的 SceneData 将只包含 mask 中 index 为 True 的那些实例：

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:dedent:
:start-after: "# tag:: set actuator ctrl in mask"
:end-before:  "# end:: set actuator ctrl in mask"
```

```{note}
SceneData 索引功能目前仅 CPU 后端支持。GPU 示例在 Warp kernel 中根据环境索引选择要更新的实例，并直接写入控制数组。
```

## 性能与基准测试

(gpu-benchmark)=

### GPU：batch 大小与实测基准

`batch` 是 GPU 性能的关键参数：

-   **过小**（`batch < 256` 左右）：GPU 并行度不足，吞吐量反而可能不如 CPU
-   **过大**：超出 GPU 显存容量，抛出 `RuntimeError: CUDA out of memory`

**实测基准**（Go2 场景，无 keyframe；CPU 为 Ryzen 9 9950X，GPU 为 RTX 5090；5 轮，每轮预热 100 步、计时 3000 步）：

| 批量 (worlds) | GPU 单步耗时 |   GPU 吞吐量   | CPU 单步耗时 |   CPU 吞吐量   | GPU 相对 CPU |
| :-----------: | :----------: | :------------: | :----------: | :------------: | :----------: |
|      128      |   0.276 ms   | 0.46 M steps/s |   0.135 ms   | 0.95 M steps/s | **0.49x** ⚠️ |
|      512      |   0.276 ms   | 1.86 M steps/s |   0.479 ms   | 1.07 M steps/s | **1.73x** ✅ |
|      1024     |   0.289 ms   | 3.54 M steps/s |   0.925 ms   | 1.11 M steps/s | **3.20x** ✅ |
|      2048     |   0.337 ms   | 6.08 M steps/s |   1.828 ms   | 1.12 M steps/s | **5.43x** ✅ |
|      4096     |   0.487 ms   | 8.41 M steps/s |   3.793 ms   | 1.08 M steps/s | **7.79x** ✅ |

```{note}
吞吐量取 5 轮中位数；表中单步耗时由该中位数换算。GPU 计时在每步后同步，CPU 计时包含完整 `step` 调用；模型加载、预热、渲染和逐步状态回读不计入耗时。

表中 GPU/CPU 为吞吐量比值。基准数据来自上述硬件和 Go2 场景，GPU 型号、模型复杂度、Warp/驱动版本都会影响实际表现。显存上限随模型复杂度和 GPU 型号变化，建议在自己的硬件上跑一次性能测试。接触约束缓冲区容量的调优见[性能优化 - 接触约束容量](../gpu_version/performance_tuning.md#contact-constraint-capacity)。
```

### CPU：parallel_bench 脚本

在 examples 目录下提供了测试多实例并行仿真性能的脚本：

```bash
uv run examples/parallel/parallel_bench.py --file examples/assets/go1/scene.xml
```

会得到类似下面的输出：

```
Summary: 3000 steps for 1024 instances in 2.824 seconds
Average: 1087683.152 steps/second
```

上述测试的 CPU 型号为 `AMD Ryzen™ 9 9950X × 32`，即在该硬件上可以达到每秒超过 100 万步的仿真速度。
