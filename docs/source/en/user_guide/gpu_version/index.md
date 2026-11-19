# 🔍 GPU Version Overview

The GPU version is optimized for large-scale parallel simulation, powered by NVIDIA Warp, and is typical for large-scale RL training. For the full installation and system requirements, see the [unified installation guide](../getting_started/installation.md).

```{important}
The GPU version currently supports **NVIDIA GPUs only** (Windows / Linux, with a CUDA 12.9+ capable driver); AMD / Intel GPUs and macOS are not supported — use the CPU version on those systems.
```

## Key Differences at a Glance

| Feature             | CPU version         | GPU version                                       |
| ------------------- | ------------------- | ------------------------------------------------- |
| **Compute backend** | CPU multi-threading | GPU massive parallelism                           |
| **Array type**      | `numpy.ndarray`     | `warp.array`                                      |
| **Storage location** | Host memory        | GPU VRAM                                          |
| **Dev / debug**     | Friendly            | Limited (requires familiarity with GPU debugging) |

## Full GPU Guide

-   [Install Python SDK](../getting_started/installation.md) — CPU / GPU installation and verification
-   [Multi-Environment Parallel Simulation](../main_function/parallel.md) — CPU / GPU parallelism, batch performance, and capacity settings
-   [Performance Tuning](performance_tuning.md) — CUDA Graph, data transfer
-   [Example Programs](../overview/examples.md) — CPU / GPU examples and run commands
-   [CPU vs. GPU API Differences](api_differences.md) — API comparison and common pitfalls
