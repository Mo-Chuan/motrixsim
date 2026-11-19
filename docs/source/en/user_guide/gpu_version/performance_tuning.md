(performance-optimization-label)=

# ⚡ Performance Tuning

## Core Optimization: CUDA Graph Acceleration

`mx.step()` has CUDA Graph logic built in — no manual management required. **The first call to `step()` records the required low-level physics Graph and high-level data Graph, and runs one simulation step**; every subsequent call simply replays the recorded Graphs.

### Basic usage: call step directly

```python
# Just loop step in the obvious way — the Graph is managed internally
for i in range(1000):
    mx.step(model, data)
    # First time: records the Graph + runs one real step (significant cost)
    # After that: replays the Graph
```

### Advanced usage: distinguish Graph pre-recording from full warmup

`capture_step()` only pre-records the low-level physics Graph. It does not execute a simulation step or create the high-level data Graph; that Graph is captured lazily during the first complete `step()`. Complete one `step()` and synchronize before starting performance measurements.

```python
import time
import warp as wp

# Pre-record the low-level physics Graph before the training / measurement loop
print("🔧 Pre-recording the CUDA Graph...")
model.capture_step(data)  # records the low-level Graph only; simulation state does not advance

# Capture the high-level Graph and advance the simulation once
mx.step(model, data)
wp.synchronize()

# Start timing here; later steps replay both recorded Graphs
start_time = time.time()
for i in range(1000):
    mx.step(model, data)

wp.synchronize()  # wait for GPU to finish
total_time = time.time() - start_time
sps = (1000 * batch_size) / total_time
print(f"📊 Performance: {sps:,.0f} steps/second")
```

:::{warning}
**CUDA Graph caveats:**

⚠️ **Properties frozen once the Graph is recorded:**

-   batch_size
-   model structure
-   capture configuration and bound buffers

After changing capture configuration or replacing bound buffers, call `model.invalidate_bindings(data)`. Writing new contents into existing buffers does not require another capture. Rebuild matching `SceneData` when model topology or solver layout changes.

⚠️ **Do not call `.numpy()` inside the loop:**

-   Device-to-host (D2H) copies synchronize with the default GPU stream and reduce the Graph's pipeline benefit
-   If you genuinely need GPU data inside the step loop (e.g. computing reward in RL), prefer moving that computation onto the GPU via a Warp kernel — see [Data Transfer Optimization](#data-transfer-optimization)

⚠️ **First-step cost:**

-   Recording is typically much slower than a normal step
-   `capture_step()` can move low-level Graph recording off the critical path, but it does not replace the first complete `step()`

ℹ️ **For debugging**: `step_no_capture()` bypasses the Graph and executes directly (no Graph speedup), making it easier to inspect intermediate state.
:::

(data-transfer-optimization)=

## Data Transfer Optimization

Every `.numpy()` triggers a GPU↔CPU sync and breaks the CUDA Graph pipeline. If your training loop needs GPU-side data every step (e.g. computing reward each step in RL), the better approach is to **move that computation onto the GPU too** — write a Warp kernel that reads `data` fields (`dof_pos` / `dof_vel` / `actuator_ctrls` etc.) directly on the GPU, and keep the result on the GPU for the next step or next kernel, avoiding the CPU round-trip.

### Compute on GPU directly

The parallel simulation example uses a Warp kernel to generate random controls on the GPU and write directly to `data.actuator_ctrls` for selected worlds. Both random number generation and control updates stay on the GPU. Full source: [`examples/gpu/parallelsim.py`](../../../../examples/gpu/parallelsim.py).

```{literalinclude} ../../../../examples/gpu/parallelsim.py
:language: python
:pyobject: randomize_controls
```

For Warp kernel syntax, see the [Warp official docs](https://nvidia.github.io/warp/).

### When `.numpy()` is fine

Not every `.numpy()` call is bad — some are mandatory or perfectly reasonable:

-   **Final result readout**: exporting data after training/simulation completes
-   **Debugging and visualization**: inspecting NaNs, printing intermediate state
-   **Necessary CPU-side consumption**: feeding data into libraries that only run on CPU (some PyTorch models, plotting tools, etc.)

What to avoid is **blindly `.numpy()`-ing on every step** — especially when that computation could just as well be done with a Warp kernel on the GPU.

`max_contact_constraints` is an optional argument of `load_model()` / `load_mjcf_str()` that sets a **lower bound** on the pre-allocated contact-constraint buffer size on GPU. Only the GPU backend needs it; the CPU backend uses a different contact construction path.

```python
# Default: auto-estimated
model = mx.load_model("scene.xml")

# Explicitly raise the lower bound
model = mx.load_model("scene.xml", max_contact_constraints=1024)
```

`max_contacts` is a companion option that raises the pre-allocated contact-point / warm-start capacity. After model construction, the implementation ensures that **constraint-row capacity is at least as large as contact-point capacity**, so increasing `max_contacts` can also raise the constraint-row budget and VRAM usage:

```python
model = mx.load_model("scene.xml", max_contact_constraints=1024, max_contacts=2048)
```

### Why this parameter exists

Contact detection runs in parallel on the GPU, and **the actual contact count is generated GPU-side at runtime** — the CPU only learns it via a sync. Copying that count back every step to resize the buffer would force a GPU↔CPU sync and break the GPU's async pipeline. MotrixSim instead estimates statically on the CPU at load time based on the collision geometry, **pre-allocates a fixed-size buffer once**, and never resizes during simulation.

However, **the static estimate is not guaranteed to be an upper bound on the true maximum** — it combines several heuristics, and while it covers most common scenes, it can under-estimate in:

-   Lots of stacked or tightly-packed bodies (grasping, stacking, terrain interaction)
-   Scenes with a high-resolution height field (HField)
-   Soft contact where many contact surfaces are active simultaneously

When the actual contact count exceeds the pre-allocated capacity, **the excess contacts are dropped**, which can cause:

-   Body penetration (legitimate contact constraints are missing)
-   Anomalous physical behavior
-   Non-reproducible simulation results

### Usage guidance

-   **The default covers most standard scenes**: usually no manual setting is needed; the default estimate, however, **is not guaranteed to be an upper bound on the real max**, and can still fall short in stacking / complex scenes
-   **How to tell when the default is insufficient**: at simulation time, if the actual contact count overflows the pre-allocated buffer, the console prints

    > `Error: Constraint overflow detected! Consider increasing the max_contact_constraint.`

    When you see this, pass a larger `max_contact_constraints` explicitly.

-   **This is a lower bound**: the value you pass is `max`'ed with the auto-estimate, so **it can only push the allocation up, not down**
-   **Don't over-allocate**: buffers are sized from `batch` plus contact-point and constraint-row capacities; setting either too large consumes significant VRAM

## Performance Tuning Checklist

When investigating performance issues, walk down this list:

| Check                                              | Impact     | How to investigate                                                                                                                           |
| -------------------------------------------------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| Are you bypassing the CUDA Graph?                  | ⭐⭐⭐⭐⭐ | Confirm you use `step()` not `step_no_capture()`; between steps, avoid operations that invalidate the Graph (changing batch, swapping model) |
| Any `.numpy()` calls inside the loop?              | ⭐⭐⭐⭐⭐ | Frequent D2H copies sync the stream and break the Graph pipeline — compute on GPU via a Warp kernel                                          |
| Is `batch_size` large enough?                      | ⭐⭐⭐⭐   | Small batches (<256) under-utilize GPU parallelism; benchmark on your own hardware                                                           |
| Is per-step Python logic outside `step` too heavy? | ⭐⭐⭐     | Pure-Python work between steps caps GPU utilization; push extra computation into a Warp kernel                                               |
| Allocating new GPU arrays per step?                | ⭐⭐⭐     | Reuse pre-allocated `wp.array` instances; avoid per-step allocation                                                                          |
| Calling `wp.synchronize()` blindly?                | ⭐⭐⭐     | Only sync when you actually need CPU-side data; async execution lets the CPU schedule the next frame ahead                                   |

🔗 **Related docs**:

-   [Multi-Environment Parallel Simulation](../main_function/parallel.md) - GPU parallel simulation guide
-   [CPU vs. GPU API Differences](api_differences.md) - API differences in detail
