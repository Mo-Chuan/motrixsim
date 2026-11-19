# MotrixSim

MotrixSim 是一个高性能的多体动力学物理仿真引擎，为机器人仿真设计。它提供一个高效、稳定的物理仿真平台，适用于机器人控制算法开发、强化学习环境构建、工业实时仿真以及教学科研等场景。

```{video} _static/videos/motrixsim-prom-10m.mp4
:autoplay:
:playsinline:
:muted:
:loop:
:width: 80%
```

## 主要特性

-   **物理仿真**：完整的刚体动力学、碰撞检测与接触求解，支持复杂多体系统建模
-   **多环境并行仿真**：单进程内并行仿真数千个模型实例，面向大规模强化学习训练与批量评估（[文档](user_guide/main_function/parallel.md)）
-   **GPU 加速**：1.0 版本提供 GPU 版本，面向大规模并行仿真优化（[文档](user_guide/gpu_version/index.md)）
-   **约束求解器**：自研的约束模型和求解器，提供高效、稳定的多体动力学计算
-   **高性能 CPU 内核**：基于 Rust 开发，兼顾计算性能与内存安全
-   **Python API**：简洁易用的接口，便于快速开发和原型制作（[快速入门](user_guide/getting_started/hello_motrixsim.md)）
-   **机器人仿真支持**：高度兼容 MJCF 模型格式，内置逆运动学（[IK](user_guide/main_function/ik.md)）、传感器、渲染等机器人仿真常用功能

## MotrixSim 生态与研究

-   **MotrixLab**：基于 MotrixSim 构建的机器人强化学习框架。提供环境注册、训练配置、RL 框架适配、评估和回放工具，覆盖四足、人形、机械臂、灵巧手等机器人任务。仓库：[Motphys/MotrixLab](https://github.com/Motphys/MotrixLab)，文档：[motrixlab.readthedocs.io](https://motrixlab.readthedocs.io/)。
-   **GS-Playground**：由清华 AIR DISCOVER 实验室、Motphys 及合作者共同开发的高保真具身仿真平台。它将 MotrixSim 的并行物理仿真与 3D Gaussian Splatting 渲染相结合，面向视觉导航、操作、运动控制和 Sim2Real 研究。相关工作已被 RSS 2026 接收。仓库：[discoverse-dev/gs_playground](https://github.com/discoverse-dev/gs_playground)，论文：[arXiv:2604.25459](https://arxiv.org/abs/2604.25459)，项目主页：[gsplayground.github.io](https://gsplayground.github.io/)。
-   **UniLab**：一个 CPU 仿真 / GPU 学习的异构机器人强化学习系统。MotrixSim 作为 CPU 批量物理后端集成到统一训练运行时中，支持大规模 rollout 和跨平台训练。仓库：[unilabsim/UniLab](https://github.com/unilabsim/UniLab)，论文：[arXiv:2605.30313](https://arxiv.org/abs/2605.30313)。

```{toctree}
:maxdepth: 1

user_guide/index
```

```{toctree}
:maxdepth: 1

api_reference/index
user_guide/getting_started/mjcf_reference
```
