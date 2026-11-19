# 用户指南

第一次使用时，请从[安装 Python SDK](getting_started/installation.md)和[快速入门](getting_started/hello_motrixsim.md)开始，再阅读[模型](main_function/scene_model.md)与[数据](main_function/scene_data.md)，了解仿真的基本组成。

- **建模与物理仿真**：加载与构建模型、管理状态、配置物理参数和控制算法。
- **可视化与交互**：检查模型、显示仿真状态、配置相机并采集图像。
- **示例与实践**：准备示例运行环境，查找完整程序和仿真用例。

如果需要使用 GPU，请先阅读[GPU 版本概览](gpu_version/index.md)并完成统一安装指南中的 GPU 安装与验证；GPU 与 CPU 的 API 差异见 GPU 版本分类下的[API 差异页](gpu_version/api_differences.md)。具体接口的签名、参数和返回值请查阅 [Python API Reference](../api_reference/index.md)。

```{toctree}
:caption: 开始使用
:maxdepth: 1

getting_started/installation
getting_started/hello_motrixsim
getting_started/motrixsim_web
getting_started/agent_skill
```

```{toctree}
:caption: GPU 版本
:maxdepth: 1

gpu_version/index
gpu_version/api_differences
gpu_version/performance_tuning
```

```{toctree}
:caption: 示例与实践
:maxdepth: 1

overview/examples
overview/case_comparison
```

```{toctree}
:caption: 建模与物理仿真
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
:caption: 可视化与交互
:maxdepth: 1

main_function/managed_viewer
main_function/render
render/camera
render/widgets
render/advanced
render/3dgs
render/headless_cpu
```
