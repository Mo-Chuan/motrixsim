# 🎮 交互式查看器

{bdg-success-line}`CPU 完全支持` {bdg-danger-line}`GPU 不支持`

## 概述

交互式查看器用于打开模型、检查结构、调整关节和执行器，以及播放物理仿真。使用 `mxview` 命令或 [`viewer.launch()`] 启动后，查看器会管理物理步进、渲染和界面交互。

```{important}
交互式查看器是阻塞的。调用 `viewer.launch()` 后，你的 Python 代码不会继续执行，直到查看器窗口关闭。如果需要在窗口打开期间继续执行 Python 代码，例如运行控制算法或其他并发任务，请使用 [`RenderApp`](motrixsim.render.RenderApp)，并在 Python 中管理物理步进和渲染同步。
```

如果需要在 Python 中逐步执行控制算法、处理自定义 UI 回调或同时渲染多个仿真实例，请使用 [RenderApp](render.md)。

```{figure} /_static/images/web_viewer/current/master-overview.jpg
:alt: 查看器的中央视口、底部 Files 和右侧场景对象与属性面板
:width: 100%

查看器的主要区域：三维视口、文件浏览器和对象检查面板。可用文件来源会随运行方式变化；桌面查看器可以浏览本地目录。
```

## 使用场景

交互式查看器适合测试和调试模型、教学演示，以及模型原型开发。使用内建面板即可播放仿真、检查对象和调整参数，适合不需要自行管理运行循环的交互式工具。

### 主要特性

- 自动管理物理步进和渲染同步。
- 提供播放、暂停、单步、重置和速度控制。
- 支持空参数、仅模型、模型和初始数据三种调用方式。
- 支持拖入模型文件，也可以从文件面板加载模型。
- 使用默认面板布局，或通过 `blueprint` 组合内建面板。

## 命令行接口

安装 `motrixsim` 后，在项目环境中运行：

```bash
uv run mxview examples/assets/boston_dynamics_spot/scene_arm.xml
```

也可以不指定文件，打开空查看器后拖入模型，或从 `Files` 面板中选择文件。

```bash
# 打开空查看器
uv run mxview

# 加载模型后立即播放
uv run mxview examples/assets/boston_dynamics_spot/scene_arm.xml --play

# 预览独立的 OBJ / STL 网格
uv run mxview path/to/part.stl
```

### CLI 参数

| 参数 | 用途 |
| --- | --- |
| `model` | 可选的位置参数，指定模型或独立 OBJ / STL 网格的路径 |
| `--play` | 加载后立即运行仿真；省略时从编辑状态开始 |

独立网格会被包装为用于显示的最小场景，适合检查网格外观。`mxview` 会加载文件、创建初始场景数据，再启动交互式查看器。激活项目虚拟环境后，也可以直接运行 `mxview`。

Python 接口支持传入模型和初始状态。

## 使用模式

### 1. 空会话

```python
import motrixsim as mx

mx.viewer.launch()
```

不传入模型时，打开空查看器。可以在窗口中选择文件或拖入模型资源；此时传入的 `data` 不会被使用。

### 2. 仅模型

```python
import motrixsim as mx

model = mx.load_model("examples/assets/boston_dynamics_spot/scene_arm.xml")
mx.viewer.launch(model)
```

查看器会为模型创建内部仿真数据。需要启动后立即播放时，传入 `play=True`：

```python
mx.viewer.launch(model, play=True)
```

### 3. 模型和数据

```python
import motrixsim as mx

model = mx.load_model("examples/assets/boston_dynamics_spot/scene_arm.xml")
data = mx.SceneData(model)

# 在打开窗口前，先执行一些物理步。
for _ in range(20):
    mx.step(model, data)

mx.viewer.launch(model, data)
```

查看器使用 `data` 中的状态创建内部副本，再管理自己的仿真循环。需要从 Python 持续读取或更新同一份数据时，请使用 {ref}`RenderApp 的同步循环 <render-sync>`。

传入初始数据适合设置位置、速度或控制输入，也适合从预先计算的仿真状态开始查看。

### 预仿真示例

上面的循环在打开查看器前执行 20 个物理步。可以用这种方式预计算派生量，或让初始接触状态先演化一段时间。20 步不保证模型已经达到平衡，需要根据模型检查状态。

## 完整示例

下面的示例加载带机械臂的 Spot 模型，创建 `SceneData`，执行 20 个物理步，再调用 `viewer.launch(model, data)`：

```{literalinclude} ../../../../examples/viewer/interactive_viewer.py
:language: python
:start-at: import motrixsim as mx
```

## 内置控件

查看器使用可停靠的面板布局。同一区域中的多个面板以标签页组织，点击标题可以切换；拖动分隔线可以调整宽度和高度。


| 区域 | 用途 |
| --- | --- |
| 左侧仿真工具栏 | 播放、暂停、单步、停止、重置和重载 |
| `Viewport` | 观察三维场景、操作相机和选择对象 |
| `Scene Hierarchy` | 浏览模型层级，选择对象和切换可见性 |
| `Inspector` | 按关节、执行器和传感器等类别检查运行数据 |
| `Favorites` | 集中查看已收藏的运行对象 |
| `Properties` | 查看选中对象的属性、运行状态和关联对象 |
| `Options` | 设置物理、渲染、光照和环境选项 |
| `Files` | 浏览目录、搜索和筛选文件，双击文件名加载模型 |
| `Log` | 查看桌面查看器的加载和运行消息 |

窗口较窄时，可使用标签栏的箭头查看其他标签，或增加面板宽度。`Files` 中的目录树、文件列表和详情栏也可以分别调整宽度。

### 播放和暂停

- 点击三角形播放按钮开始仿真；运行时，该按钮切换为暂停按钮。
- 暂停后使用单步按钮向前执行一帧。
- 方形停止按钮会停止并重置仿真，返回编辑状态。
- 重置按钮恢复初始状态；重载按钮重新读取模型文件和资源。

快捷键作用于当前接收输入的视口或面板。`Space` 用于切换播放和暂停，`F10` 用于单步，`Ctrl+E` 用于重置。点击目标视口或面板后，它才会接收输入；正在搜索框或可编辑字段中输入时，快捷键可能被该控件占用。

### 选择对象和检查属性

在 `Scene Hierarchy` 中点击对象，或在视口中单击物体，查看器会显示对应高亮，并更新 `Properties`。也可以从 `Inspector` 中点击关节或执行器名称，直接定位其属性。

```{figure} /_static/images/web_viewer/current/master-properties.jpg
:alt: 从 Inspector 选择关节后，视口高亮对应部位，Properties 显示关节类型、范围和运行状态
:width: 100%

对象选择与视口高亮、属性面板联动。暂停仿真后更便于检查和修改可编辑字段。
```

`Inspector` 中的星形按钮用于收藏对象。经常检查的关节或执行器可加入 `Favorites`，减少在完整列表中查找的次数。

### 物理和显示选项

`Options` 提供 `Physics`、`Rendering` 和 `Lighting / Environment` 分类。物理设置包括关键帧、重力和执行器等选项；显示相关设置还可以从视口左上角的齿轮菜单进入。

相机操作：

- 鼠标左键拖动：围绕焦点旋转。
- 鼠标右键拖动：平移视角。
- 滚轮：缩放。
- 启用物理拖拽时，按住 `Ctrl` 并用鼠标左键拖动物体。

## 自定义面板布局

使用 `motrixsim.ui` 声明需要显示的内建面板，通过 `blueprint` 传给 [`viewer.launch()`]。

```python
import motrixsim as mx
from motrixsim import ui

model = mx.load_model("examples/assets/boston_dynamics_spot/scene_arm.xml")
viewport = ui.Viewport(camera=ui.CameraRef.system(), title="Simulation")
objects = ui.Tabs([ui.SceneTree(), ui.Inspector(), ui.Favorites()])
settings = ui.Tabs([ui.Properties(), ui.Options()])

blueprint = ui.Horizontal(
    [
        ui.Vertical([viewport, ui.Log()], shares=[4, 1]),
        ui.Vertical([objects, settings]),
    ],
    shares=[3, 1],
)
mx.viewer.launch(model, blueprint=blueprint)
```

`Horizontal` 和 `Vertical` 划分区域，`Tabs` 将多个面板放在同一组标签页中。`shares` 按子项顺序指定相对比例，不要求加起来等于 1。上面的布局给视口列和右侧面板列分配了 3:1 的宽度。

自定义布局中的系统相机视口使用 `camera=ui.CameraRef.system()` 绑定系统相机。每个 View 和布局只能移交一次；再次启动查看器时，请重新声明。

```{note}
交互式查看器支持上述内建面板的布局。需要 Python 控件回调、动态 Frame 或图片绑定时，请使用 {ref}`RenderApp 的自定义 UI <render-custom-ui>`。
```

## 与 RenderApp 的对比

| 需求 | 交互式查看器 | RenderApp |
| --- | --- | --- |
| 执行模式 | 阻塞到窗口关闭 | Python 管理自己的运行循环 |
| 快速打开并检查模型 | 直接使用 `mxview` 或 `viewer.launch()` | 需要编写加载和同步代码 |
| 物理循环 | 查看器自动管理 | Python 调用 `step()` |
| 渲染同步 | 自动处理 | Python 调用 `sync()` |
| 关节、执行器和对象检查 | 内建 `Inspector`、`Properties` 等面板 | 从 Python 读取数据，按需要声明自定义控件 |
| 面板布局 | 可组合内建查看器面板 | 可组合视口、Options、Log 和自定义 View |
| Python 控制算法和 UI 回调 | 使用 RenderApp | 在自己的循环中处理 |
| 多实例或无头渲染 | 使用 RenderApp | 支持 |

### 何时使用

以下场景适合使用交互式查看器：

- 测试或调试模型，检查关节、执行器和物体状态。
- 创建教学演示，使用内建控件播放、暂停和重置仿真。
- 利用内建面板构建简单的交互式工具。
- 用少量代码打开模型，让查看器管理物理步进和渲染。

以下场景适合使用 [RenderApp](render.md)：

- 将仿真接入现有应用程序或控制算法。
- 自行控制物理步进、渲染频率和同步时机。
- 构建需要自定义运行流程的生产应用程序。
- 添加自定义 UI、处理控件回调或绘制调试信息。
- 运行无头仿真，并按需采集渲染图像。

## 最佳实践

### 模型加载

`load_model()` 接受绝对路径或相对于当前工作目录的路径。在脚本中，可以用 `pathlib` 相对于脚本位置构造路径，避免从不同目录启动时找不到资源：

```python
from pathlib import Path
import motrixsim as mx

model_path = Path(__file__).parent / "assets" / "model.xml"
model = mx.load_model(str(model_path))
```

### 数据初始化

`SceneData` 的 `qpos` 和 `qvel` 分别保存位置和速度。示例中的 `initial_positions` 和 `initial_velocities` 由调用方提供，维度须分别与模型的 `qpos`、`qvel` 一致：

```python
data = mx.SceneData(model)
data.qpos[:] = initial_positions
data.qvel[:] = initial_velocities

for _ in range(10):
    mx.step(model, data)

mx.viewer.launch(model, data)
```

## 限制和注意事项

- `viewer.launch()` 会阻塞，直到窗口关闭；需要在 Python 中继续执行循环时，使用 RenderApp。
- 一次只能有一个交互式查看器处于活动状态。
- 传入的 `SceneData` 用作初始状态；交互式查看器不接收批量数据实例。
- 需要从磁盘重新读取模型修改时，使用重载；只恢复仿真状态时，使用重置。

## 相关文档

- [渲染器](render.md)
- [相机](../render/camera.md)
- [Widget 组件](../render/widgets.md)
- [网页仿真器](../getting_started/motrixsim_web.md)

### API 参考

- [`viewer.launch()`]
- [`SceneModel`](motrixsim.SceneModel)
- [`SceneData`](motrixsim.SceneData)
- [`step()`](motrixsim.step)

[`viewer.launch()`]: motrixsim.viewer.launch
