# 机械臂抓苹果示例资产

本目录包含抓苹果动作回放使用的场景、网格、纹理和关节目标序列。程序入口和显示控制均位于 `examples/control/apple_grasp.py`。

## 环境与运行

使用仓库根目录的 `.python-version`、`pyproject.toml` 和 `uv.lock` 管理 Python 与依赖版本，本示例不单独维护环境。以下命令在仓库根目录执行：

```bash
uv sync --locked
uv run examples/control/apple_grasp.py
```

默认打开窗口并自动播放“夹苹果梗、提起、放回、松开、抓果身、再次提起”的 40 秒动作序列。点击 Pause 暂停、Play 继续；关闭窗口会提前结束回放。在 Apple Silicon macOS 上，图形模式使用 `uv run mxpython examples/control/apple_grasp.py`。

无窗口检查、缩短回放时间或保存运行报告：

```bash
uv run examples/control/apple_grasp.py --headless --seconds 2
uv run examples/control/apple_grasp.py --headless --output
uv run examples/control/apple_grasp.py --headless --output /tmp/apple-grasp/run
uv run examples/control/apple_grasp.py --help
```

`--seconds` 范围为 0.002 至 40。默认只在终端输出结果；指定 `--output` 后，将 `summary.json` 和 `records.json` 保存到仓库的 `examples/output/apple_grasp/`，不受启动目录影响。也可以通过 `--output DIRECTORY` 指定其他位置。`examples/output/` 统一存放示例运行产物，已被 Git 忽略；自定义目录的版本管理由使用者自行处理。

完整回放退出码 0 表示模拟与物理检查均通过，2 表示回放完成但抓取检查失败，1 表示提前结束或运行错误。命令行参数错误由参数解析器以退出码 2 报告。短时回放仅验证执行流程，不证明完整抓取成功。

## 资产内容

- `scene.xml`：双臂、灵巧手、桌面和苹果场景，包含苹果初始姿态、碰撞掩码、位置执行器及接触传感器。程序负责回放关节目标，并在第 23 秒切换右手抓取增益。
- `meshes/`：机械臂、灵巧手、桌面和苹果网格。
- `textures/`：材质贴图和环境照明。
- `objects/`：苹果预制体、碰撞网格和来源信息。
- `data/joint_targets.npz`：40 秒关节目标序列。
- `LICENSE-*`、`NOTICE-*`：各组件的许可和声明。

网格、纹理、环境贴图及动作数据沿用仓库的 Git LFS 规则。
