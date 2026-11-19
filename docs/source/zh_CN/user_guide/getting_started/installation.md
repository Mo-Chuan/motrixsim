# 🛠️ 安装 Python SDK

按照以下步骤选择安装路径，在自己的项目中使用 MotrixSim，或运行仓库附带的示例。

## 系统要求

(installation-gpu-requirements)=

CPU / GPU 仿真是两种物理计算后端；渲染器和交互式查看器用于显示与图像采集，可以按需与仿真后端配合使用。

| 环境条件 | CPU 仿真 | GPU 仿真 | 渲染器 / 交互式查看器 |
| --- | --- | --- | --- |
| **Python** | 3.10–3.13 | 3.10–3.13 | 3.10–3.13 |
| **Windows x86_64** | ✅ | ✅ | ✅ |
| **Linux x86_64** | ✅ | ✅ | ✅ |
| **macOS Apple Silicon（arm64）** | ✅ | ❌ | ✅；RenderApp 窗口程序需通过 `mxpython` 启动 |
| **硬件要求** | 无需 GPU | NVIDIA GPU；暂不支持 AMD / Intel GPU | 可用的图形设备；无 GPU 时可使用 CPU 软渲染¹ |
| **驱动要求** | 无 GPU 驱动要求 | 支持 CUDA 12.9 或更新版本的 NVIDIA 驱动 | 对应图形设备的驱动；CPU 软渲染需软件图形驱动 |
| **安装依赖** | `motrixsim` | `motrixsim[gpu]`，同时包含 CPU 支持 | 随 `motrixsim` 安装，无需额外的渲染依赖组 |

¹ [CPU 软渲染](../render/headless_cpu.md)适合离线图像采集，不建议用于实时交互。

CPU 仿真可以配合 GPU 渲染。

**检查 GPU 驱动**：在终端运行 `nvidia-smi`，确认 GPU 被识别并检查驱动支持的 CUDA 版本。完整兼容要求见 [GPU 运行环境要求](https://nvidia.github.io/warp/stable/user_guide/compatibility.html#runtime-requirements)。

## 选择安装路径

| 使用目标 | 安装路径 |
| --- | --- |
| 在自己的新项目或已有项目中使用 SDK | {ref}`路径一：从 PyPI 安装 <install-from-pypi>` |
| 运行仓库附带的模型和示例程序 | {ref}`路径二：克隆仓库运行示例 <install-from-repository>` |

两条路径均使用 [uv](https://docs.astral.sh/uv/getting-started/installation/) 管理 Python 环境。

(install-from-pypi)=

## 路径一：从 PyPI 安装

### 1. 准备项目

已有项目在项目根目录执行后续命令。新项目先运行：

```bash
uv init --python 3.13 motrixsim-demo
cd motrixsim-demo
```

### 2. 安装 SDK

根据所需的仿真后端选择命令：

| 仿真后端 | 安装命令 | Python 导入 |
| --- | --- | --- |
| CPU | `uv add motrixsim` | `import motrixsim` |
| CPU + GPU | `uv add "motrixsim[gpu]"` | `import motrixsim` 或 `import motrixsim.gpu` |

已有 SDK 的项目也可通过 GPU 安装命令添加 GPU 支持。

::::{dropdown} 使用其他包管理器

在对应的项目环境中执行：

| 工具 | CPU | CPU + GPU |
| --- | --- | --- |
| pip 23.0+ | `python -m pip install motrixsim` | `python -m pip install "motrixsim[gpu]"` |
| pdm | `pdm add motrixsim` | `pdm add "motrixsim[gpu]"` |
| poetry | `poetry add motrixsim` | `poetry add "motrixsim[gpu]"` |

后续验证命令中的 `uv run python`，在已激活的 pip 虚拟环境中使用 `python`，在 pdm / poetry 项目中分别使用 `pdm run python` / `poetry run python`。

::::

### 3. 验证安装

在项目根目录运行：

```bash
uv run python -c "import motrixsim; print('motrixsim import OK')"
```

输出 `motrixsim import OK` 表示 SDK 已成功加载。

(installation-verify-gpu)=

安装了 GPU 依赖时，再检查 GPU 组件：

```bash
uv run python -c "import motrixsim.gpu; print('motrixsim.gpu import OK')"
```

输出 `motrixsim.gpu import OK` 表示 GPU 组件已成功加载。

接下来阅读[快速入门](hello_motrixsim.md)，准备模型并编写第一个仿真程序。

(install-from-repository)=

## 路径二：克隆仓库运行示例

仓库提供模型资产、示例程序和配套依赖，适合直接体验与修改示例。

### 1. 准备工具并克隆仓库

安装 [Git](https://git-scm.com/downloads)、[Git LFS](https://git-lfs.com/) 和 [uv](https://docs.astral.sh/uv/getting-started/installation/)，然后执行：

```bash
git lfs install
git clone https://github.com/Motphys/motrixsim.git
cd motrixsim
git lfs pull
```

后续命令均在 `motrixsim` 仓库根目录执行。

:::{dropdown} 中国大陆：通过国内站点克隆并配置镜像源（可选）
:animate: fade-in
:color: warning

如果依赖下载较慢，可从 ModelScope 镜像克隆，并在 `pyproject.toml` 中配置国内镜像源：

1.  从镜像克隆仓库：

    ```bash
    git clone https://www.modelscope.cn/datasets/motphysdevelopers/motrixsim-docs.git
    cd motrixsim-docs
    git lfs pull
    ```

2.  修改 `pyproject.toml`，添加国内镜像源：

    ```toml
    [[tool.uv.index]]
    name = "mirror"
    # 请填写您选择的国内镜像源，例如：
    # 清华源: "https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple"
    url = ""
    ```

:::

### 2. 安装示例依赖

uv 根据仓库配置准备 Python 环境、SDK 和示例所需依赖。按仿真后端选择命令：

| 仿真后端 | 安装命令 |
| --- | --- |
| CPU | `uv sync` |
| CPU + GPU | `uv sync --extra gpu` |

### 3. 运行第一个示例

加载 Spot 机器人并打开仿真窗口：

```bash
uv run python examples/getting_started/hello_motrixsim.py
```

使用 GPU 后端时运行：

```bash
uv run --extra gpu python examples/getting_started/hello_motrixsim.py --gpu
```

运行后应看到显示 Spot 机器人场景的窗口，程序持续推进物理仿真并同步画面。可在终端按 `Ctrl+C` 停止程序。

macOS Apple Silicon 上，窗口示例使用 `mxpython` 启动：

```bash
uv run mxpython examples/getting_started/hello_motrixsim.py
```

接下来浏览[示例程序](../overview/examples.md)，选择感兴趣的任务。

## 常见问题

| 现象 | 检查与处理 |
| --- | --- |
| 找不到匹配的安装包 | 检查 Python 版本和系统架构是否满足系统要求；仓库示例使用仓库配置指定的 Python 版本 |
| `ModuleNotFoundError` | 在安装依赖的项目目录中运行命令；GPU 模块对应 `motrixsim[gpu]` 或仓库的 `gpu` extra |
| 示例模型或网格加载失败 | 在仓库根目录运行示例，并执行 `git lfs pull` 获取模型资产 |
| CUDA 运行错误 | 使用 `nvidia-smi` 检查 GPU 识别情况和驱动支持的 CUDA 版本 |
