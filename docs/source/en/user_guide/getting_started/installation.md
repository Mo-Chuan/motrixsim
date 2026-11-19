# 🛠️ Install Python SDK

Choose an installation path to use MotrixSim in your own project or run the examples included in the repository.

## System Requirements

(installation-gpu-requirements)=

CPU and GPU simulation are physics backends. The renderer and interactive viewer provide visualization and image capture and can be used alongside a simulation backend as needed.

| Environment | CPU simulation | GPU simulation | Renderer / interactive viewer |
| --- | --- | --- | --- |
| **Python** | 3.10–3.13 | 3.10–3.13 | 3.10–3.13 |
| **Windows x86_64** | ✅ | ✅ | ✅ |
| **Linux x86_64** | ✅ | ✅ | ✅ |
| **macOS Apple Silicon (arm64)** | ✅ | ❌ | ✅; launch RenderApp windowed programs with `mxpython` |
| **Hardware** | No GPU required | NVIDIA GPU; AMD / Intel GPUs currently unsupported | Available graphics device; CPU software rendering is an option without a GPU¹ |
| **Driver** | No GPU driver required | NVIDIA driver supporting CUDA 12.9 or newer | Driver for the graphics device; software rendering requires a software graphics driver |
| **Dependency** | `motrixsim` | `motrixsim[gpu]`, including CPU support | Included with `motrixsim`; no additional rendering extra required |

¹ [CPU software rendering](../render/headless_cpu.md) is suitable for offline image capture, not recommended for real-time interaction.

CPU simulation can also be combined with GPU rendering.

**Check the GPU driver**: Run `nvidia-smi` in a terminal to confirm GPU detection and check the CUDA version supported by the driver. See the [GPU runtime requirements](https://nvidia.github.io/warp/stable/user_guide/compatibility.html#runtime-requirements) for full compatibility details.

## Choose an Installation Path

| Your goal | Installation path |
| --- | --- |
| Use the SDK in a new or existing project | {ref}`Path 1: Install from PyPI <install-from-pypi>` |
| Run the models and example programs included in the repository | {ref}`Path 2: Clone the Repository and Run Examples <install-from-repository>` |

Both paths use [uv](https://docs.astral.sh/uv/getting-started/installation/) to manage the Python environment.

(install-from-pypi)=

## Path 1: Install from PyPI

### 1. Prepare Your Project

For an existing project, run subsequent commands from its root directory. For a new project, first run:

```bash
uv init --python 3.13 motrixsim-demo
cd motrixsim-demo
```

### 2. Install the SDK

Choose a command for your simulation backend:

| Simulation backend | Installation command | Python import |
| --- | --- | --- |
| CPU | `uv add motrixsim` | `import motrixsim` |
| CPU + GPU | `uv add "motrixsim[gpu]"` | `import motrixsim` or `import motrixsim.gpu` |

The GPU installation command also adds GPU support to projects that already use the SDK.

::::{dropdown} Other package managers

Run the appropriate command in your project environment:

| Tool | CPU | CPU + GPU |
| --- | --- | --- |
| pip 23.0+ | `python -m pip install motrixsim` | `python -m pip install "motrixsim[gpu]"` |
| pdm | `pdm add motrixsim` | `pdm add "motrixsim[gpu]"` |
| poetry | `poetry add motrixsim` | `poetry add "motrixsim[gpu]"` |

In the verification commands below, replace `uv run python` with `python` in an activated pip virtual environment, or with `pdm run python` / `poetry run python` in the corresponding project.

::::

### 3. Verify Installation

From your project root, run:

```bash
uv run python -c "import motrixsim; print('motrixsim import OK')"
```

The output `motrixsim import OK` confirms that the SDK loaded successfully.

(installation-verify-gpu)=

For a GPU installation, also check the GPU components:

```bash
uv run python -c "import motrixsim.gpu; print('motrixsim.gpu import OK')"
```

The output `motrixsim.gpu import OK` confirms that the GPU components loaded successfully.

Continue with the [quick start](hello_motrixsim.md) to prepare a model and write your first simulation program.

(install-from-repository)=

## Path 2: Clone the Repository and Run Examples

The repository includes model assets, example programs, and their dependencies, ready for running and modifying examples.

### 1. Prepare Tools and Clone the Repository

Install [Git](https://git-scm.com/downloads), [Git LFS](https://git-lfs.com/), and [uv](https://docs.astral.sh/uv/getting-started/installation/), then run:

```bash
git lfs install
git clone https://github.com/Motphys/motrixsim.git
cd motrixsim
git lfs pull
```

Run all subsequent commands from the `motrixsim` repository root.

:::{dropdown} China mainland: clone from a mirror and configure a package index (optional)

If downloads are slow, clone from the ModelScope mirror and configure a package index in `pyproject.toml`:

1.  Clone from the mirror:

    ```bash
    git clone https://www.modelscope.cn/datasets/motphysdevelopers/motrixsim-docs.git
    cd motrixsim-docs
    git lfs pull
    ```

2.  Add an index to `pyproject.toml`:

    ```toml
    [[tool.uv.index]]
    name = "mirror"
    # For example, the Tsinghua mirror:
    # "https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple"
    url = ""
    ```

:::

### 2. Install Example Dependencies

uv prepares the Python environment, SDK, and example dependencies using the repository configuration. Choose a command for your simulation backend:

| Simulation backend | Installation command |
| --- | --- |
| CPU | `uv sync` |
| CPU + GPU | `uv sync --extra gpu` |

### 3. Run Your First Example

Load the Spot robot and open a simulation window:

```bash
uv run python examples/getting_started/hello_motrixsim.py
```

To use the GPU backend, run:

```bash
uv run --extra gpu python examples/getting_started/hello_motrixsim.py --gpu
```

A window should display the Spot robot scene while the program steps physics and updates the view. Press `Ctrl+C` in the terminal to stop the program.

On macOS Apple Silicon, launch windowed examples with `mxpython`:

```bash
uv run mxpython examples/getting_started/hello_motrixsim.py
```

Next, browse the [example programs](../overview/examples.md) and choose a task to explore.

## Troubleshooting

| Symptom | Check and action |
| --- | --- |
| No matching package found | Check the Python version and system architecture against the requirements; repository examples use the Python version specified by the repository configuration |
| `ModuleNotFoundError` | Run commands from the project where dependencies were installed; GPU components require `motrixsim[gpu]` or the repository's `gpu` extra |
| Example model or mesh fails to load | Run examples from the repository root and use `git lfs pull` to fetch model assets |
| CUDA execution error | Use `nvidia-smi` to check GPU detection and the CUDA version supported by the driver |
