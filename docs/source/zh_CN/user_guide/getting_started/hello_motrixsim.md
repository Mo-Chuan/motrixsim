# 🚀 快速入门：运行第一个仿真

本教程通过一个小球下落实验，介绍创建模型、管理状态、推进物理和显示画面的基本流程。最后修改重力，观察仿真结果的变化。

## 准备

先完成[安装](installation.md)，然后在自己的项目根目录或克隆的仓库根目录操作。下面的场景直接写在程序中，包含地面和一个小球。

## 1. 创建程序

在项目根目录创建 `first_simulation.py`，写入以下代码：

```python
import sys
import time

if "--gpu" in sys.argv:
    import motrixsim.gpu as mx
else:
    import motrixsim as mx

from motrixsim.render import RenderApp, RenderClosedError

SCENE = """<mujoco>
  <option timestep="0.02" gravity="0 0 -9.81"/>
  <worldbody>
    <light type="directional" pos="0 -2 3" dir="0.45 0.35 -0.83"
           diffuse="1 1 1" intensity="2732" castshadow="true"/>
    <geom type="plane" size="3 3 0.1" rgba="0.7 0.7 0.7 1"/>
    <body name="ball" pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1" rgba="0.2 0.5 0.9 1"/>
    </body>
  </worldbody>
</mujoco>"""

model = mx.load_mjcf_str(SCENE)
data = mx.SceneData(model)

with RenderApp() as render:
    render.launch(model)
    try:
        while True:
            mx.step(model, data)
            render.sync(data)
            time.sleep(0.02)
    except RenderClosedError:
        pass
```

场景使用 MJCF XML 描述。这里先认识四项配置：

| 配置 | 含义 |
| --- | --- |
| `timestep="0.02"` | 每次物理步进推进 0.02 秒 |
| `gravity="0 0 -9.81"` | 沿世界坐标系负 Z 方向施加重力加速度，单位为 m/s² |
| `body pos="0 0 1"` | 将小球初始中心放在地面上方 1 米处 |
| `freejoint` | 让小球能够自由平移和旋转 |

球体的 `size="0.1"` 表示半径为 0.1 米，`mass="1"` 表示质量为 1 千克。

## 2. 运行并观察

在保存程序的目录运行：

```bash
uv run python first_simulation.py
```

macOS Apple Silicon 使用：

```bash
uv run mxpython first_simulation.py
```

窗口打开后，蓝色小球从地面上方下落，与地面接触后逐渐停留在地面附近。关闭窗口结束程序。再次执行命令，可从初始位置重新观察下落过程。

```{figure} /_static/images/quickstart_ball_gravity.jpg
:alt: 实际运行窗口中，蓝色小球下落后停留在地面上
:width: 90%
:align: center

重力为 −9.81 m/s² 时，小球下落并与地面接触。
```

安装了 GPU 组件时，同一个程序可以使用 GPU 后端运行：

```bash
uv run python first_simulation.py --gpu
```

通过仓库安装路径使用 GPU 时，运行 `uv run --extra gpu python first_simulation.py --gpu`。

## 3. 理解仿真循环

程序按照以下顺序工作，CPU 和 GPU 后端共用这一流程：

| 步骤 | 代码 | 作用 |
| --- | --- | --- |
| 创建模型 | `mx.load_mjcf_str(SCENE)` | 将场景描述加载为模型，包含几何体、质量、连接关系和物理参数 |
| 创建状态 | `mx.SceneData(model)` | 创建位置、速度等随仿真变化的数据 |
| 推进物理 | `mx.step(model, data)` | 按模型的时间步长更新状态 |
| 同步画面 | `render.sync(data)` | 将当前状态交给渲染器显示 |

循环末尾的 `time.sleep(0.02)` 在每步后短暂等待，便于观察下落过程。关闭窗口时，程序捕获 `RenderClosedError` 并退出渲染上下文。

## 4. 修改重力，比较结果

关闭窗口，将场景中的重力改为零：

```xml
<option timestep="0.02" gravity="0 0 0"/>
```

保存并重新运行。小球应保持在初始位置。将重力恢复为 `0 0 -9.81`，再次运行，小球会重新下落。

```{figure} /_static/images/quickstart_ball_zero_gravity.jpg
:alt: 实际运行窗口中，零重力下的蓝色小球悬停在地面上方
:width: 90%
:align: center

重力为零时，小球保持在初始高度。
```

这个实验展示了：场景参数决定物理行为，仿真循环持续更新状态，渲染器显示更新后的结果。

## 下一步

- [模型](../main_function/scene_model.md)：学习加载外部模型与访问组件。
- [数据](../main_function/scene_data.md)：学习读取和设置仿真状态。
- [示例程序](../overview/examples.md)：运行机器人、关节、传感器等完整示例。
