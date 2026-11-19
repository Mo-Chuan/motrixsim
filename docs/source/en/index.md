# MotrixSim

MotrixSim is a high-performance multibody dynamics simulation engine designed for robotics. It provides an efficient and stable physics simulation platform for developing robot control algorithms, building reinforcement learning environments, real-time industrial simulation, education, and research.

```{video} _static/videos/motrixsim-prom-10m.mp4
:autoplay:
:playsinline:
:muted:
:loop:
:width: 80%
```

## Key Features

-   **Physics Simulation**: Complete rigid body dynamics, collision detection, and contact solving, with support for modeling complex multibody systems
-   **Parallel Simulation of Multiple Environments**: Simulates thousands of model instances in parallel within a single process, for large-scale RL training and batch evaluation ([docs](user_guide/main_function/parallel.md))
-   **GPU Acceleration**: Version 1.0 ships a GPU build optimized for massively parallel simulation ([docs](user_guide/gpu_version/index.md))
-   **Constraint Solver**: A proprietary constraint model and solver delivering efficient and stable multibody dynamics computation
-   **High-Performance CPU Core**: Developed in Rust, combining computational performance with memory safety
-   **Python API**: Intuitive interface for rapid development and prototyping ([quick start](user_guide/getting_started/hello_motrixsim.md))
-   **Robotics Support**: High compatibility with the MJCF model format, with built-in features commonly used in robot simulation such as inverse kinematics ([IK](user_guide/main_function/ik.md)), sensors, and rendering

## MotrixSim Ecosystem and Research

-   **MotrixLab**: A robot reinforcement learning framework built on MotrixSim. It provides environment registration, training configuration, RL framework adapters, evaluation, and replay tools for quadrupeds, humanoids, robotic arms, dexterous hands, and related robot tasks. Repository: [Motphys/MotrixLab](https://github.com/Motphys/MotrixLab), documentation: [motrixlab.readthedocs.io](https://motrixlab.readthedocs.io/).
-   **GS-Playground**: A high-fidelity embodied simulation platform developed by Tsinghua AIR DISCOVER Lab, Motphys, and collaborators. It combines MotrixSim parallel physics simulation with 3D Gaussian Splatting rendering for visual navigation, manipulation, locomotion, and Sim2Real research. The work has been accepted by RSS 2026. Repository: [discoverse-dev/gs_playground](https://github.com/discoverse-dev/gs_playground), paper: [arXiv:2604.25459](https://arxiv.org/abs/2604.25459), project page: [gsplayground.github.io](https://gsplayground.github.io/).
-   **UniLab**: A heterogeneous CPU-simulation / GPU-learning system for robot reinforcement learning. MotrixSim is integrated as a CPU-batched physics backend in the unified training runtime, supporting large-scale rollouts and cross-platform training. Repository: [unilabsim/UniLab](https://github.com/unilabsim/UniLab), paper: [arXiv:2605.30313](https://arxiv.org/abs/2605.30313).

```{toctree}
:maxdepth: 1

user_guide/index
```

```{toctree}
:maxdepth: 1

api_reference/index
user_guide/getting_started/mjcf_reference
```
