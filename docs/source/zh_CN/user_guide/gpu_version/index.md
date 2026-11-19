# 🔍 GPU 版本概览

GPU 版本专为大规模并行仿真优化，基于 NVIDIA Warp 实现 GPU 加速，常用于强化学习的大规模并行训练。安装与系统要求的完整说明见[统一安装指南](../getting_started/installation.md)。

```{important}
GPU 版本目前仅支持 **NVIDIA GPU**（Windows / Linux，需支持 CUDA 12.9+ 的驱动）；AMD / Intel GPU 与 macOS 不支持，这些环境请使用 CPU 版本。
```

## 核心差异概览

| 特性         | CPU 版本        | GPU 版本                  |
| ------------ | --------------- | ------------------------- |
| **计算后端** | CPU 多线程      | GPU 大规模并行            |
| **数组类型** | `numpy.ndarray` | `warp.array`              |
| **存储位置** | 主机内存        | GPU 显存                  |
| **开发调试** | 友好            | 受限（需要熟悉 GPU 调试） |

## 完整 GPU 指南

-   [安装 Python SDK](../getting_started/installation.md) — CPU / GPU 安装与验证
-   [多环境并行仿真](../main_function/parallel.md) — CPU / GPU 并行、batch 性能与容量配置
-   [性能优化](performance_tuning.md) — CUDA Graph、数据传输
-   [示例程序](../overview/examples.md) — CPU / GPU 示例与运行方式
-   [CPU 与 GPU API 差异](api_differences.md) — API 对比与常见陷阱
