# User Guide

If you are new to MotrixSim, start with [installation](getting_started/installation.md) and the [quick start](getting_started/hello_motrixsim.md). Then read about [models](main_function/scene_model.md) and [data](main_function/scene_data.md) to understand the basic components of a simulation.

- **Modeling and Physics Simulation**: Load and build models, manage state, and configure physics parameters and control algorithms.
- **Visualization and Interaction**: Inspect models, display simulation state, configure cameras, and capture images.
- **Examples and Practice**: Set up the example environment and find complete programs and simulation cases.

To use the GPU backend, first read the [GPU overview](gpu_version/index.md) and complete GPU installation and verification in the unified installation guide; for GPU-vs-CPU API differences, see the [API differences page](gpu_version/api_differences.md) under GPU Version. For interface signatures, parameters, and return values, see the [Python API Reference](../api_reference/index.md).

```{toctree}
:caption: Getting Started
:maxdepth: 1

getting_started/installation
getting_started/hello_motrixsim
getting_started/motrixsim_web
getting_started/agent_skill
```

```{toctree}
:caption: GPU Version
:maxdepth: 1

gpu_version/index
gpu_version/api_differences
gpu_version/performance_tuning
```

```{toctree}
:caption: Examples and Practice
:maxdepth: 1

overview/examples
overview/case_comparison
```

```{toctree}
:caption: Modeling and Physics Simulation
:maxdepth: 1

main_function/scene_model
main_function/scene_data
main_function/model_building
main_function/options
main_function/acd
main_function/ik
main_function/osc
main_function/parallel
main_function/randomization
getting_started/usd_reference
```

```{toctree}
:caption: Visualization and Interaction
:maxdepth: 1

main_function/managed_viewer
main_function/render
render/camera
render/widgets
render/advanced
render/3dgs
render/headless_cpu
```
