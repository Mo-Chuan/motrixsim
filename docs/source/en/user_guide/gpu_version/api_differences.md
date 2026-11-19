# 🔍 CPU vs. GPU API Differences

This page focuses on the API differences and migration considerations between the CPU and GPU versions; it is not an auto-generated interface listing. The two backends share most of their public API surface, but the underlying implementations differ significantly. For the complete interface listing, see the [GPU API Reference](../../api_reference/gpu/index.md).

## Imports and Initialization

### Package imports

::::{tab-set}
:::{tab-item} CPU version

```python
import motrixsim as mx
```

:::

:::{tab-item} GPU version

```python
import motrixsim.gpu as mx
```

:::
::::

The core import is one line in both cases. CPU data arrays live in host memory and use `numpy.ndarray`; GPU data arrays live in GPU memory and use `warp.array`. **When user code needs to operate on these arrays directly**, the CPU side uses `numpy` and the GPU side uses `warp` (for example, to write custom kernels). These are dependencies of the respective array types, not extra requirements introduced by choosing a simulation backend.

### Model loading

Both the CPU and GPU versions support MuJoCo native models (MJCF) and the Unified Robot Description Format (URDF). They use the same model-loading function and return the same model type:

| Feature           | GPU version       | CPU version       | Note                    |
| ----------------- | ----------------- | ----------------- | ----------------------- |
| **Function name** | `mx.load_model()` | `mx.load_model()` | Same API name           |
| **Return type**   | `SceneModel`      | `SceneModel`      | Same return type name   |
| **Formats**       | MJCF, URDF        | MJCF, URDF        | Same supported formats  |

```python
# GPU
model = mx.load_model("scene.xml")    # mx = motrixsim.gpu

# CPU
model = mx.load_model("scene.xml")    # mx = motrixsim
```

In the GPU version, `load_model()` / `load_mjcf_str()` also accept two capacity arguments:

-   `max_contact_constraints`: the lower bound for contact-constraint row capacity.
-   `max_contacts`: contact-point and warm-start buffer capacity. After model construction, constraint-row capacity is raised to at least the contact-point capacity, so increasing `max_contacts` can also increase the constraint-row budget and VRAM usage.

(data-container-label)=

## Data Containers

### Creating data objects

Both versions use `SceneData` to store runtime state and control inputs. A world is one independent simulation instance of the same model, with its own joint positions, velocities, and other state. The total number of instances is `num_worlds`.

The 0D/1D labels describe the dimensions of the `batch` argument, not the final rank of state arrays:

-   `batch=()` (0D) creates one world, so `num_worlds=1`.
-   `batch=(N,)` (1D) creates N worlds, so `num_worlds=N`.

The exposed `SceneData.shape` differs: CPU preserves the input batch shape and defaults to `()`, while GPU always keeps a world dimension and defaults to `(1,)`.

| Feature              | CPU version                   | GPU version                       |
| -------------------- | ----------------------------- | --------------------------------- |
| **Class name**       | `SceneData`                   | `SceneData`                       |
| **Batch dimensions** | 0D/1D: `()` or `(N,)`         | 0D/1D: `()` / `(N,)` / scalar `N` |
| **API shape**        | `SceneData(model, batch=...)` | `SceneData(model, batch=...)`     |

```{warning}
The CPU and GPU versions both limit batch shapes to **0D or 1D**: `batch=()` (single world) or `batch=(N,)`. Passing a multi-dimensional shape (e.g. `batch=(M, N)`) will raise. The CPU version requires the sequence form; the GPU version additionally accepts a scalar `batch=N`.
```

:::{warning}
**`batch` strongly affects GPU performance:**

-   **Too small** (e.g. `batch < 256`): under-utilizes GPU parallelism; throughput can be lower than CPU
-   **Too large**: the memory required for the configured batch exceeds available GPU memory and raises `RuntimeError: CUDA out of memory`

GPU memory is shared by the current process and other tasks running on the same GPU. The batch size that fits therefore depends on total GPU memory and the memory currently used by the model, contact constraints, temporary training allocations, and other tasks. Leave headroom for workloads that will run after this one. For the actual relationship between batch size and performance, see the benchmark in [Multi-Environment Parallel Simulation](../main_function/parallel.md).
:::

::::{tab-set}
:::{tab-item} CPU version

```python
# Single environment
data = mx.SceneData(model)

# Batched environments
data = mx.SceneData(model, batch=(1000,))
```

:::

:::{tab-item} GPU version

```python
# Single environment
data = mx.SceneData(model)

# Batched environments
data = mx.SceneData(model, batch=(1000,))
```

:::
::::

### Data array type

For example, `data = mx.SceneData(model)` stores simulation runtime state and control inputs:

-   `data.dof_pos` stores generalized joint positions.
-   `data.dof_vel` stores generalized joint velocities.
-   `data.actuator_ctrls` stores the control values sent to the actuators.

These fields are floating-point arrays. The CPU version uses `numpy.ndarray` in host memory; the GPU version uses `warp.array` in GPU memory. The table below uses `data.dof_pos` to compare storage, shape, and calling conventions:

| Property             | CPU version                              | GPU version                                  | Note                      |
| -------------------- | ---------------------------------------- | -------------------------------------------- | ------------------------- |
| **Array type**       | `numpy.ndarray`                          | `warp.array`                                 | Different data interfaces |
| **Attribute access** | `data.dof_pos`                           | `data.dof_pos`                               | Same field name           |
| **Shape**            | `(batch..., dof)`; `batch=()` → `(dof,)` | `(num_worlds, dof)`; `batch=()` → `(1, dof)` | GPU keeps the world dim   |
| **Storage location** | Host memory                              | GPU memory                                   | Different storage medium  |
| **Sync requirement** | No explicit sync                         | Explicit sync required                       | Different execution model |
| **Type conversion**  | Use NumPy APIs directly                  | Call `.numpy()` to return to host            | Adds performance overhead |

With `batch=()`, CPU fields have no batch dimension (`dof_pos.shape == (num_dof,)`), while GPU fields keep a leading world dimension (`dof_pos.shape == (1, num_dof)`).

:::{warning}
**Array type caveats:**

-   NumPy functions cannot operate directly on GPU `warp.array`; check each high-level API to see whether it accepts NumPy input
-   **Recommended**: write custom computations as Warp kernels on the GPU to avoid device-to-host (D2H) transfers
-   Only call `.numpy()` to copy back to host memory when the result really needs to leave the GPU (saving, debugging, visualization, etc.)
-   Frequent host/device transfers add transfer overhead and reduce the speedup from CUDA Graphs

:::

## Data Operations

### Reading data

:::::{tab-set}
::::{tab-item} GPU version (recommended: pure GPU processing)

```python
import warp as wp

# Access the warp array
positions = data.dof_pos  # warp.array
velocities = data.dof_vel  # warp.array

# ✅ Recommended: process data on the GPU directly, no CPU-GPU transfer
@wp.kernel
def compute_mean_vel(
    velocities: wp.array2d(dtype=float),
    mean_out: wp.array(dtype=float),
):
    env_idx = wp.tid()
    # Compute mean velocity per environment on the GPU
    n_dof = velocities.shape[1]
    s = float(0.0)
    for i in range(n_dof):
        s += velocities[env_idx, i]
    mean_out[env_idx] = s / float(n_dof)

mean_out = wp.zeros(batch_size, dtype=float, device="cuda")
wp.launch(compute_mean_vel, dim=batch_size, inputs=[velocities], outputs=[mean_out])
# mean_out stays on the GPU and can feed into the next kernel
```

::::

::::{tab-item} CPU version

```python
# Access numpy array directly
positions = data.dof_pos  # numpy.ndarray
velocities = data.dof_vel  # numpy.ndarray

# Array operations
first_pos = positions[0]  # First environment's position
mean_vel = velocities.mean(axis=1)  # Mean velocity per world
```

::::

::::{tab-item} GPU version (only for final readout / debugging)

```python
# ⚠️ Only use when you really need CPU-side data (final result, debugging)
# Calling this frequently introduces CPU-GPU sync and significantly slows training
wp.synchronize()
pos_np = positions.numpy()  # GPU -> CPU copy
velocities_np = velocities.numpy()
first_pos = pos_np[0]
mean_vel = velocities_np.mean(axis=1)
```

::::
:::::

### Setting data

::::{tab-set}
:::{tab-item} CPU version

```python
import numpy as np

# Direct numpy ops — zero overhead
data.dof_pos[0] = 1.0  # ✅ Set a single value
data.dof_pos[:, 0] = np.ones(batch_size)  # ✅ Set an entire column

# Batched set — memory copy
new_positions = np.random.random((batch_size, num_dof))
data.dof_pos[:] = new_positions  # ✅ Direct assignment
```

:::

:::{tab-item} GPU version

```python
import warp as wp
import numpy as np

# ❌ Outside a Warp kernel, assigning to warp.array with [] is unsupported:
#    data.dof_pos[0] = 1.0          → TypeError
#    data.dof_pos[:, 0] = ...       → TypeError

# Path A: data originates on the CPU (random init, read from config / disk) —
#         use set_dof_pos to bulk-write. This triggers one host-to-device (H2D) copy.
new_positions = np.random.random((batch_size, num_dof))
data.set_dof_pos(new_positions)

# Path B: data is generated each step (target pose / random perturbation in a training loop) —
#         write directly to data.dof_pos with a Warp kernel; no host detour and no H2D copy.
wp.launch(my_init_kernel, dim=batch_size, outputs=[data.dof_pos])
```

:::
::::

:::{warning}
**GPU data-write summary:**

-   Outside a Warp kernel, you cannot assign to `warp.array` with `[]`; `a[i] = x` / `a[:, j] = ...` both raise `TypeError`
-   When the data is already on the CPU: use `data.set_dof_pos(numpy)` / `set_dof_vel` / `set_actuator_ctrls` and the other public setters
-   When the data is generated per step: write to the corresponding field directly with a Warp kernel to avoid host-to-device transfers

:::

## Simulation Execution

### Step function

`mx.step(model, data)` advances the simulation by one time step and updates joint positions, velocities, and other state in `data`. The CPU and GPU versions use the same function name, parameter names, and parameter meanings:

| Aspect            | CPU version     | GPU version                                                      | Note                         |
| ----------------- | --------------- | ---------------------------------------------------------------- | ---------------------------- |
| **Function name** | `step`          | `step`                                                           | Same name                    |
| **Parameters**    | `(model, data)` | `(model, data)`                                                  | Same names and meanings      |
| **Execution**     | Synchronous     | Asynchronous (use `wp.synchronize()` to wait)                    | Synchronization differs      |

```python
mx.step(model, data)    # GPU
mx.step(model, data)    # CPU
```

### Performance tuning

::::{tab-set}
:::{tab-item} CPU version

```python
# CPU version needs no special tuning
for i in range(1000):
    mx.step(model, data)
```

:::

:::{tab-item} GPU version

```python
# GPU version benefits from CUDA Graph
model.capture_step(data)   # pre-record the low-level physics Graph
mx.step(model, data)       # capture the high-level Graph and advance once
wp.synchronize()

for i in range(1000):
    mx.step(model, data)   # replay the recorded Graphs

# Sync when you actually need the data
wp.synchronize()
```

:::
::::

## Component Access

### Actuator control

In the GPU version, `actuator.set_ctrl(data, ctrl)` accepts `warp.array` and NumPy arrays with shape `(num_worlds,)`; for a single world it also accepts `float`. NumPy input triggers a host-to-device transfer, so prefer GPU arrays on performance-critical paths.

::::{tab-set}
:::{tab-item} CPU version

```python
# Get actuator
actuator = model.get_actuator(0)

# Set control values
ctrl = np.ones(batch_size) * 0.5
actuator.set_ctrl(data, ctrl)

# Read control values
current_ctrl = actuator.get_ctrl(data)
```

:::

:::{tab-item} GPU version

```python
# Get actuator (same API)
actuator = model.get_actuator(0)

# Set control values: prefer GPU arrays on performance-critical paths
ctrl = wp.full((batch_size,), 0.5, device="cuda")
actuator.set_ctrl(data, ctrl)

# NumPy input is also accepted with shape (num_worlds,), but incurs an H2D copy
# actuator.set_ctrl(data, np.full(batch_size, 0.5, dtype=np.float32))

# Read control values
current_ctrl = actuator.get_ctrl(data)  # returns warp.array
```

:::
::::

### Sensor data

| Feature     | CPU version                          | GPU version                          | Difference           |
| ----------- | ------------------------------------ | ------------------------------------ | -------------------- |
| **Access**  | `model.get_sensor_value(name, data)` | `model.get_sensor_value(name, data)` | Same API             |
| **Return**  | `numpy.ndarray`                      | `warp.array`                         | Different array type |
| **Batched** | ✅                                   | ✅                                   | Both support batching |

## Host Memory and GPU Memory Exchange

### Exchanging host-memory and GPU-memory data

`numpy.ndarray` data normally resides in host memory, while `warp.array` data normally resides in GPU memory. Use the following pattern when data must move between the two:

```python
# GPU array to numpy (GPU -> CPU)
gpu_array = data.dof_pos  # warp.array
cpu_array = gpu_array.numpy()  # convert to numpy

# numpy to GPU array (CPU -> GPU)
cpu_data = np.random.random((batch_size, num_dof))
gpu_data = wp.array(cpu_data, device="cuda")

# Or write into the simulation state fields directly via the public setters on SceneData
data.set_dof_pos(cpu_data)
```

(common-pitfalls-label)=

## Common Pitfalls and Best Practices

### 1. Array type mismatch

Check the input contract of each high-level GPU API. For example, `SceneData.set_dof_pos()` and `Actuator.set_ctrl()` accept NumPy input and perform an H2D copy, while Warp kernels and APIs that explicitly require `warp.array` need device arrays.

```python
# ❌ Wrong: this Warp kernel parameter requires a warp.array
cpu_data = np.ones(100)
wp.launch(my_kernel, dim=100, inputs=[cpu_data])  # TypeError: warp.array expected

# ✅ Correct: convert to a warp array first
gpu_data = wp.array(cpu_data, device="cuda")
wp.launch(my_kernel, dim=100, inputs=[gpu_data])
```

### 2. Frequent D2H copies hurting performance

```python
# ⚠️ .numpy() triggers a device-to-host (D2H) sync waiting for the GPU — data isn't stale,
#     but doing this every step prevents step-level overlap and destroys the CUDA Graph benefit
for i in range(100):
    mx.step(model, data)
    result = data.dof_pos.numpy()  # syncs every step — slow

# ✅ Keep steps async, sync only once when you actually need CPU data
for i in range(100):
    mx.step(model, data)
wp.synchronize()                   # one-shot sync
final_result = data.dof_pos.numpy()
```

### 3. Unrealistic performance expectations

```{note}
**Misconception**: GPU is not always faster than CPU — at small batches GPU parallelism is under-utilized and CPU can be faster. For the relationship between batch and performance see {ref}`measured benchmark <gpu-benchmark>`.
```

See {ref}`data container notes <data-container-label>` for more on batch configuration.

🔗 **Related docs**:

-   [Multi-Environment Parallel Simulation](../main_function/parallel.md) - GPU parallel simulation guide
