# ​🔀 Multi-Environment Parallel Simulation

{bdg-success-line}`CPU: fully supported` {bdg-success-line}`GPU: fully supported`

MotrixSim has built-in multi-environment parallel simulation: the CPU backend distributes instances across cores automatically, while the GPU backend runs thousands to tens of thousands of environments on a single card — typical for large-scale reinforcement learning training. Both backends share the same batch interface. This chapter demonstrates parallel simulation of 100 tidybot instances with separate CPU and GPU examples.

## Creating Multiple Instances

Pass a `batch` parameter when constructing `SceneData`:

```python
size = 10
batch_size = size * size
data = mx.SceneData(model, batch=(batch_size,))
```

### CPU Example

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:language: python
:dedent:
:start-after: "# tag::init batch data"
:end-before:  "# end::init batch data"
```

This example creates 10x10=100 tidybot model instances. For visualization, each instance gets a different render offset so they are arranged in a grid on a plane; the instances are automatically distributed across CPU cores.

```{note}
render_offset applies to the rendered view only; the physics instances stay at their original positions.
```

Run with:

```sh
uv run examples/parallel/parallelsim.py
```

### GPU Example

The GPU example also creates 100 tidybot instances and captures the CUDA Graph:

```sh
uv run --extra gpu python examples/gpu/parallelsim.py
```

Keyboard controls match the CPU backend: A randomizes controls for all instances, S randomizes the first instance, D randomizes odd-indexed instances, and R restores all instances to their initial state. A custom Warp kernel generates random controls in GPU memory and writes only the selected instances. The seed increments after each randomization action to produce a reproducible random sequence.

## Accessing Multi-Instance Data

In batch mode, model state data is stored in `SceneData` as batches. All Data-related fields automatically gain a batch dimension. For example, `dof_pos` has shape `(batch_size, num_dof_pos)`:

```python
batch = (1000,)
data = mtx.SceneData(model, batch=batch)
assert data.dof_vel.shape == (*batch, model.num_dof_vel)
```

Data-setting interfaces likewise require the batch dimension, but the assignment style differs per backend:

**CPU backend** uses property assignment:

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:dedent:
:start-after: "# tag:: set actuator ctrl in batch"
:end-before:  "# end:: set actuator ctrl in batch"
```

**GPU backend** arrays live in GPU VRAM with type `warp.array` (`numpy.ndarray` on CPU). Control arrays can be modified directly in a Warp kernel, or populated from host data using methods such as `set_actuator_ctrls`:

```python
batch_size = 2000
data = mx.SceneData(model, batch=(batch_size,))
assert data.dof_pos.shape == (batch_size, model.num_dof_pos)

# batch=() is treated as a single world, matching the CPU API

ctrl_values = wp.full((batch_size,), 1.0, device="cuda")
actuator = model.get_actuator(0)
actuator.set_ctrl(data, ctrl_values)
```

For the complete list, see [CPU vs. GPU API Differences](../gpu_version/api_differences.md).

## Indexing (CPU Backend Only)

On the CPU backend, you can access subsets of instance data through indexing.

### 1. Accessing a Single Instance

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:dedent:
:start-after: "# tag:: set actuator ctrl in single"
:end-before:  "# end:: set actuator ctrl in single"
```

This accesses the first instance via `data[0]`. The returned `single_data` is a new SceneData object with shape `()`, indicating scalar instance data. It shares memory with the original `data`, so modifications are reflected in `data`.

### 2. Accessing via Mask

SceneData also supports indexing with `NDArray[bool]`, returning a new SceneData containing only the instances where the mask is True:

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:dedent:
:start-after: "# tag:: set actuator ctrl in mask"
:end-before:  "# end:: set actuator ctrl in mask"
```

```{note}
SceneData indexing is currently CPU-only. The GPU example selects instances by their world index inside a Warp kernel and writes directly to the control array.
```

## Performance and Benchmarks

(gpu-benchmark)=

### GPU: Batch Size and Measured Benchmark

`batch` is the critical GPU performance knob:

-   **Too small** (`batch < ~256`): GPU parallelism is under-utilized; throughput can actually be lower than the CPU version
-   **Too large**: exceeds GPU VRAM, raising `RuntimeError: CUDA out of memory`

**Measured benchmark** (Go2 scene, no keyframe; CPU: Ryzen 9 9950X, GPU: RTX 5090; 5 rounds, 100 warmup steps and 3000 timed steps per round):

| Batch (worlds) | GPU step time | GPU throughput | CPU step time | CPU throughput |  GPU vs CPU  |
| :------------: | :-----------: | :------------: | :-----------: | :------------: | :----------: |
|      128       |   0.276 ms    | 0.46 M steps/s |   0.135 ms    | 0.95 M steps/s | **0.49x** ⚠️ |
|      512       |   0.276 ms    | 1.86 M steps/s |   0.479 ms    | 1.07 M steps/s | **1.73x** ✅ |
|      1024      |   0.289 ms    | 3.54 M steps/s |   0.925 ms    | 1.11 M steps/s | **3.20x** ✅ |
|      2048      |   0.337 ms    | 6.08 M steps/s |   1.828 ms    | 1.12 M steps/s | **5.43x** ✅ |
|      4096      |   0.487 ms    | 8.41 M steps/s |   3.793 ms    | 1.08 M steps/s | **7.79x** ✅ |

```{note}
Throughput uses the median of 5 rounds; the step times in the table are derived from that median. GPU timing synchronizes after every step, while CPU timing includes complete `step` calls. Model loading, warmup, rendering, and per-step state readback are excluded.

The GPU/CPU column is a throughput ratio. These numbers apply only to the hardware and Go2 scene above; GPU model, model complexity, and Warp/driver version affect actual performance. The VRAM ceiling depends on model complexity and GPU model — run your own measurement on your own hardware. For contact-constraint buffer capacity tuning, see [Performance Tuning — contact constraint capacity](../gpu_version/performance_tuning.md#contact-constraint-capacity).
```

### CPU: parallel_bench Script

A benchmark script measures multi-instance parallel simulation performance:

```bash
uv run examples/parallel/parallel_bench.py --file examples/assets/go1/scene.xml
```

Output looks like:

```
Summary: 3000 steps for 1024 instances in 2.824 seconds
Average: 1087683.152 steps/second
```

The test CPU was an `AMD Ryzen™ 9 9950X × 32`: over 1,000,000 simulation steps per second on this hardware.
