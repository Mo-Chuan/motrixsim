# 🚀 GPU API Quick Reference

The GPU module (**motrixsim.gpu**) provides large-scale parallel physics simulation built on NVIDIA Warp, supporting NVIDIA GPUs (Windows / Linux).

```{important}
The public API of the GPU version closely mirrors the CPU version (`motrixsim`). The main differences are:
- Data resides on GPU memory (`warp.array` instead of `numpy.ndarray`)
- Built-in CUDA Graph acceleration
- Only supports 0D/1D batching (`batch=()` or `batch=(N,)`); a scalar `batch=N` is also accepted

For detailed differences, see [CPU vs. GPU API Differences](../../user_guide/gpu_version/api_differences.md).
```

## 📋 Top-Level API - [`motrixsim.gpu`](motrixsim_gpu.md)

| Category       | API                 | Description                 |
| -------------- | ------------------- | --------------------------- |
| **Model load** | `load_model(path)`  | Load MJCF/URDF model        |
| **Model load** | `load_mjcf_str(s)`  | Load from MJCF string       |
| **Step**       | `step(model, data)` | Advance one simulation step |

`load_model` / `load_mjcf_str` also accept `max_contact_constraints` and `max_contacts` to raise the pre-allocated GPU contact/warm-start buffers.

### 🔧 Core Objects

| Object       | Type   | Description                                        |
| ------------ | ------ | -------------------------------------------------- |
| `SceneModel` | Model  | GPU simulation world (same name as CPU version)    |
| `SceneData`  | Data   | GPU simulation state (`warp.array`, requires sync) |
| `Options`    | Config | Simulation parameters                              |
| `SolverType` | Enum   | Solver type                                        |

### 🎛️ Per-Environment Overrides

| API                                           | Description                     |
| --------------------------------------------- | ------------------------------- |
| `Geom.set_friction_override(data, ...)`       | Per-world sliding friction      |
| `Link.set_mass_override(data, ...)`           | Per-world body mass             |
| `Link.set_center_of_mass_override(data, ...)` | Per-world center-of-mass offset |
| `Actuator.set_kp_override(data, ...)`         | Per-world position gain         |
| `Actuator.set_damping_override(data, ...)`    | Per-world actuator damping      |

### 🎮 GPU-Specific Methods

| API                                | Description                                      |
| ---------------------------------- | ------------------------------------------------ |
| `SceneModel.capture_step(data)`    | Pre-record CUDA Graph (avoid first-step stutter) |
| `SceneModel.step_no_capture(data)` | Bypass Graph execution (for debugging)           |

### 🎨 Rendering

`motrixsim.gpu.render.RenderApp` is the CPU `motrixsim.render.RenderApp` with a GPU-data bridge, so `launch`, `sync`, gizmos, input, image widgets, and image buffers match the CPU version. `create_camera_viewport` currently expects the CPU `Camera` type, not the GPU `Camera` wrapper.

### 🔍 Named Access

Named access APIs (`get_body`, `get_link`, `get_joint`, `get_actuator`, etc.) work the same as in the CPU version. See the [CPU Quick Reference](../api_quick_reference.md) for details.

## 📚 Detailed Documentation

```{seealso}
- [Full GPU API](motrixsim_gpu.md) - Auto-generated API reference
- [GPU Version Guide](../../user_guide/gpu_version/index.md) - Tutorials and performance tuning
- [CPU vs. GPU API Differences](../../user_guide/gpu_version/api_differences.md) - CPU vs GPU comparison
```
