# 🚀 GPU API 快速参考

GPU 模块（**motrixsim.gpu**）基于 NVIDIA Warp 提供大规模并行物理仿真能力，支持 NVIDIA GPU（Windows / Linux）。

```{important}
GPU 版本的公开 API 与 CPU 版本（`motrixsim`）高度一致，主要差异在于：
- 数据存储在 GPU 显存中（`warp.array` 而非 `numpy.ndarray`）
- 内置 CUDA Graph 加速
- 仅支持 0D/1D 批量（`batch=()` 或 `batch=(N,)`），也接受标量 `batch=N`

详细差异请参考 [CPU 与 GPU API 差异](../../user_guide/gpu_version/api_differences.md)。
```

## 📋 顶层 API - [`motrixsim.gpu`](motrixsim_gpu.md)

| 功能类别     | API                 | 说明                |
| ------------ | ------------------- | ------------------- |
| **模型加载** | `load_model(path)`  | 加载 MJCF/URDF 模型 |
| **模型加载** | `load_mjcf_str(s)`  | 从 MJCF 字符串加载  |
| **仿真步进** | `step(model, data)` | 执行一个仿真步      |

`load_model` / `load_mjcf_str` 还接受 `max_contact_constraints` 和 `max_contacts`，用于提高 GPU 端预分配的接触/热启动缓冲容量。

### 🔧 核心对象

| 对象         | 类型 | 用途描述                                      |
| ------------ | ---- | --------------------------------------------- |
| `SceneModel` | 模型 | GPU 仿真世界描述（与 CPU 版本同名）           |
| `SceneData`  | 数据 | GPU 仿真状态（`warp.array` 数据，需显式同步） |
| `Options`    | 配置 | 仿真参数设置                                  |
| `SolverType` | 枚举 | 求解器类型                                    |

### 🎛️ Per-Environment Override

| API                                           | 说明                    |
| --------------------------------------------- | ----------------------- |
| `Geom.set_friction_override(data, ...)`       | 每个 world 的滑动摩擦   |
| `Link.set_mass_override(data, ...)`           | 每个 world 的刚体质量   |
| `Link.set_center_of_mass_override(data, ...)` | 每个 world 的质心偏移   |
| `Actuator.set_kp_override(data, ...)`         | 每个 world 的位置增益   |
| `Actuator.set_damping_override(data, ...)`    | 每个 world 的执行器阻尼 |

### 🎮 GPU 特有方法

| API                                | 说明                                      |
| ---------------------------------- | ----------------------------------------- |
| `SceneModel.capture_step(data)`    | 提前录制 CUDA Graph（避免首次 step 卡顿） |
| `SceneModel.step_no_capture(data)` | 绕过 Graph 执行（用于调试）               |

### 🎨 渲染

`motrixsim.gpu.render.RenderApp` 就是带 GPU 数据桥接的 CPU `motrixsim.render.RenderApp`，因此 `launch`、`sync`、gizmos、输入、图像 widget 和图像缓冲都与 CPU 版本一致。`create_camera_viewport` 目前需要 CPU `Camera` 类型，不能直接传 GPU `Camera` 包装类。

### 🔍 模型组件访问

API 与 CPU 版本一致（`get_body`、`get_link`、`get_joint`、`get_actuator` 等），具体请参考 [CPU 快速参考](../api_quick_reference.md) 的对应章节。

## 📚 详细文档

```{seealso}
- [GPU 版本完整 API](motrixsim_gpu.md) - 通过 automodule 生成的完整 API 文档
- [GPU 版本指南](../../user_guide/gpu_version/index.md) - 使用教程和性能优化
- [CPU 与 GPU API 差异](../../user_guide/gpu_version/api_differences.md) - CPU vs GPU API 对比
```
