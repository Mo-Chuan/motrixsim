# 🎨 渲染器

## 概述

[`RenderApp`] 将 Python 中的仿真状态显示在窗口中，并将按键、按钮等操作传回 Python，供控制程序处理。

RenderApp 提供模型加载、渲染同步、自定义 UI、图像显示、调试绘制和多实例渲染功能。物理步进由 Python 代码控制，窗口在独立线程中渲染，两者通过 `sync()` 交换数据。

只需要打开模型、使用内建检查面板并播放仿真时，可以直接使用[交互式查看器](managed_viewer.md)。

## 启动与加载

`load_model()` 加载模型，`SceneData` 保存仿真状态，`RenderApp.launch()` 将模型加载到渲染器。

以下示例通过 `run.render_loop()` 调用物理步进和渲染同步。模型路径相对于仓库根目录：

```python
import motrixsim as mx
from motrixsim import run
from motrixsim.render import RenderApp, RenderClosedError

model = mx.load_model("examples/assets/boston_dynamics_spot/scene.xml")
data = mx.SceneData(model)

with RenderApp() as renderer:
    renderer.launch(model)
    try:
        run.render_loop(
            model.options.timestep,
            60,
            lambda: mx.step(model, data),
            lambda: renderer.sync(data),
        )
    except RenderClosedError:
        pass
```

运行后，窗口会显示机器人，Python 按模型时间步调用 `mx.step()`，并以 60 Hz 调用 [`render.sync(data)`] 更新画面。接入控制算法时，将计算控制量的代码放在 `mx.step()` 之前。

示例中的 `try` 和 `with` 让程序在关闭窗口后正常退出。

## 界面与显示

三维视口 `Viewport` 支持左键拖动旋转、右键拖动平移和滚轮缩放。操作作用于鼠标所在的视口。

点击视口左上角的相机选择器可以切换相机，齿轮菜单中可以调整显示模式、可见性和辅助标记。

调节光照、查看日志和添加按钮都通过面板完成。将所需面板组合为 `blueprint`，再传给 RenderApp。后面的自定义 UI 示例会创建这样的布局。

| 面板或控件 | RenderApp 中的用途 |
| --- | --- |
| `ui.Viewport()` | 显示系统相机或指定模型相机的画面 |
| `ui.Options()` | 调整 `Rendering` 和 `Lighting / Environment` 选项 |
| `ui.Log()` | 查看日志 |
| `ui.View(...)` | 声明自己的按钮、开关、文本、输入框等控件 |
| `ui.Image(...)` | 显示图片或相机采集结果 |

模型层级、关节和属性检查面板由[交互式查看器](managed_viewer.md)提供。RenderApp 中的控制面板由 Python 代码创建。

暂停、单步和重置也由 Python 循环控制；RenderApp 的 `Options` 用于调整显示，不提供 `Physics` 分类。

(render-sync)=
## 同步

### 理解主线程与渲染线程的同步

Python 主线程负责物理步进，独立的渲染线程负责窗口和画面。调用 [`render.sync(data)`] 时，主线程向渲染线程发送物体位姿、绘制指令等帧数据，同时接收窗口输入、UI 事件和相机数据。

```{figure} /_static/images/render_thread_model.jpg
:alt: Python 主线程通过 Frame 管道发送帧数据，渲染线程通过 Event 管道返回输入事件和相机数据
:width: 100%

主线程与渲染线程通过 Frame 和 Event 两条管道交换数据。
```

主线程执行 Python 代码，负责计算控制量和物理步进。通常在若干次物理步进后调用一次 `render.sync(data)`，完成两项同步工作：

1. 向 **Frame 管道**发送当前场景的帧数据。管道容量有限，已满时会等待渲染线程接收。
2. 从 **Event 管道**接收窗口输入、UI 事件和相机数据，供 Python 代码读取和处理。

渲染线程负责创建窗口并执行渲染循环，主要处理两项工作：

1. 从 Frame 管道接收帧数据，更新场景并提交给 GPU 渲染。没有新帧数据时，跳过场景状态的更新。
2. 处理窗口操作、UI 交互和相机数据请求，将结果通过 Event 管道传回主线程。

物理步进与画面更新可以使用不同频率。例如，物理仿真每秒执行 1000 步，画面每秒更新 60 次。按显示帧率调用 `sync()` 即可；调用过于频繁时，主线程可能因 Frame 管道已满而等待。

### 暂停期间的同步

`renderer.sync(data)` 在更新画面的同时接收窗口输入。暂停物理步进时仍需持续同步，Python 才能收到恢复操作。

空格键切换暂停状态的代码片段如下。`paused` 在循环外初始化为 `False`：

```python
renderer.sync(data)
if renderer.input.is_key_just_pressed("space"):
    paused = not paused
```

物理步进函数仅在 `paused` 为 `False` 时调用 `mx.step()`，渲染函数持续调用 `sync()`。

## 渲染配置

`RenderSettings` 定义场景加载时的全局渲染配置，通过 `launch()` 的 `render_settings` 参数传入。

| 参数 | 用途 | 默认值 |
| --- | --- | --- |
| `simplify_render_mesh` | 自动简化渲染网格 | `False` |
| `enable_shadow` | 启用阴影 | `True` |
| `enable_ssao` | 启用屏幕空间环境光遮蔽 | `True` |
| `enable_oit` | 启用顺序无关透明渲染 | `True` |

```python
from motrixsim.render import RenderSettings

settings = RenderSettings.quality()
settings.enable_shadow = True
settings.enable_oit = False
renderer.launch(model, render_settings=settings)
```

示例启用阴影并关闭顺序无关透明渲染。对于同时受全局配置和场景描述文件（MJCF 或 MSD）控制的渲染项，两处都启用才会生效。实际效果还取决于渲染设备的支持情况。

完整示例：[render_settings.py](../../../../examples/viewer/render_settings.py)。

(render-custom-ui)=
## 自定义 UI 组件

通过 `RenderApp(blueprint=blueprint)` 将内建面板和自定义 `ui.View` 组合到同一窗口。自定义 UI 支持布局容器、文本、按钮、开关、文本输入、选择控件、动态 Frame 和图片。

保存控件返回的 state 对象，用于读取值、查询交互或更新控件。UI 回调在调用 `render.sync(data)` 的 Python 线程中执行，可以修改 `SceneData`，供下一次物理步使用。不调用 `sync()` 时，待处理的 UI 回调不会执行。

### 布局的所有权与生命周期

`ui.Viewport()` 和 `ui.Options()` 返回 `ui.View`。使用 `ui.Tabs([viewport, controls, ui.Options()])` 将内建面板与自定义面板组合。组合时，View 的声明和内容移入生成的蓝图。嵌套组合也会消费输入的子蓝图。

将最终蓝图传给 `RenderApp(blueprint=...)` 时，应用会消费该蓝图。每个 View 和蓝图只能移交一次，也可以把单个新 View 直接传给 RenderApp 构造函数。保留各 View 对象，供所属应用调用 `hide(view)` 和 `show(view)`，这两种操作保留当前 UI 内容。

即使原应用已经关闭，其他应用也不能复用这些 View。组合或构造失败时，需要重新构建 View 和布局，已消费的输入不会归还。

### 仿真控制面板示例

在 `with ui.View(...)` 块内声明自定义面板的内容。每个 View 需要恰好一个根节点。使用 `ui.Column()` 或 `ui.Row()` 将多个控件组合在同一个根节点下。

`render_app_ui.py` 使用运行开关、单步按钮和重置按钮控制 Python 中的物理步进。

示例将窗口分为左侧视口和右侧工具面板，宽度比例为 3:1。右侧包含 `Controls`、`Options` 和 `Log` 三个标签。在仓库根目录运行示例的命令为：

```bash
uv run python examples/viewer/render_app_ui.py
```

示例中各控件的行为如下：

- **Run simulation** 控制物理步进，取消勾选时暂停。
- **Step once (when paused)** 在暂停时执行一个物理时间步。
- **Reset state** 恢复模型的初始仿真数据，运行开关保持原来的状态。
- **Options** 提供渲染、光照和环境设置，**Log** 显示日志消息。

暂停时仍持续调用 `sync()`，因此相机和工具面板继续响应。单步和重置得到的新状态会在下一次同步时显示。

```{literalinclude} ../../../../examples/viewer/render_app_ui.py
:language: python
:start-at: import motrixsim as mx
```

### 布局比例与相机绑定

`shares` 指定子区域的相对比例。`shares=[3, 1]` 按 3:1 分配宽度，`[2, 1]` 按 2:1 分配。窗口中的分隔线也支持拖动调整。

布局组件分别负责区域划分、标签分组和相机绑定：

- `ui.Horizontal([...], shares=[...])` 按相对宽度划分区域。
- `ui.Vertical([...], shares=[...])` 按相对高度划分区域。
- `ui.Tabs([...])` 将多个面板组合为标签页。
- `ui.Viewport(camera=ui.CameraRef.system())` 将视口绑定到系统相机。
- `ui.Viewport(camera=ui.CameraRef(0))` 将视口绑定到当前应用的第 0 个模型相机。

`shares` 中的数字表示各区域的相对大小，例如 `[3, 1]`。创建视口时，用 `camera=ui.CameraRef.system()` 显示系统相机的画面。

### 控件状态与回调

同一个重置按钮有两种处理方式：在同步后用 `.is_clicked()` 主动查询，或在创建按钮时通过 `on_click` 注册回调。下面分别展示这两种写法。

#### 方式一：同步后查询 state

控件返回的 state 对象用于读取状态和查询操作。以下片段来自 `render_app_ui.py` 的 `Controls` 面板声明：

```{literalinclude} ../../../../examples/viewer/render_app_ui.py
:language: python
:dedent: 4
:start-at: with ui.View(title="Controls") as controls:
:end-before: "    viewport ="
```

`reset = ui.Button("Reset state")` 创建的就是重置按钮，界面上显示为 **Reset state**。将 `controls` 加入 `blueprint`，如上面的完整示例所示，按钮才会出现在窗口中。

示例中的 `physics_step()` 处理物理步进，`render_frame()` 负责同步和查询控件操作。两个函数位于 `main()` 内、`renderer.launch(model)` 之后：

```{literalinclude} ../../../../examples/viewer/render_app_ui.py
:language: python
:dedent: 8
:start-at: def physics_step():
:end-before: "        try:"
```

`run.render_loop()` 分别调度这两个函数：

```python
run.render_loop(model.options.timestep, 60, physics_step, render_frame)
```

勾选 `Run simulation` 时持续运行；取消勾选后，点击单步按钮推进一个物理时间步。重置按钮将 `data` 恢复为初始状态，后续步进和画面更新都会使用这份数据。

其他控件也使用相同方式：通过 `value` 读取当前值，通过 `is_changed()`、`is_submitted()` 等方法判断这一轮是否发生操作。

#### 方式二：通过 callback 处理操作

也可以直接把重置函数交给按钮。点击按钮后，`sync()` 会调用这个函数，无需再查询 `reset.is_clicked()`。

运行、单步和重置也支持回调处理。以下片段对应 `render_app_ui.py` 中的控件声明和两个循环函数，使用时位于 `main()` 内，替换主动查询版本的对应代码。模型加载、`blueprint`、RenderApp 的创建和 `run.render_loop()` 调用保持不变：

```python
running = True

def set_running(value):
    nonlocal running
    running = value

def reset_state():
    nonlocal data
    data = mx.SceneData(model)

def step_once():
    if not running:
        mx.step(model, data)

with ui.View(title="Controls") as controls:
    with ui.Column():
        ui.Checkbox("Run simulation", value=True, on_change=set_running)
        ui.Button("Step once (when paused)", on_click=step_once)
        ui.Button("Reset state", on_click=reset_state)

def physics_step():
    if running:
        mx.step(model, data)

def render_frame():
    renderer.sync(data)  # 在这里接收事件并执行回调
```

`on_click` 回调不接收参数；`on_change` 接收控件的新值，上面的 `set_running(value)` 就用它更新运行状态。文本输入框还可以通过 `on_submit` 处理提交操作。

使用回调也要持续调用 `sync()`，包括暂停期间。

| 操作 | 主动查询 | 注册回调 |
| --- | --- | --- |
| 点击按钮 | `button.is_clicked()` | `on_click=handler` |
| 改变控件值 | `state.is_changed()`，再读取 `state.value` | `on_change=handler`，参数为新值 |
| 提交文本 | `text.is_submitted()`，再读取 `text.value` | `on_submit=handler`，参数为提交的文本 |

主动查询适合将操作集中写在渲染函数中；回调适合将每个操作写成单独的函数。同一个按钮选一种处理方式即可，避免重复执行。

#### 从 Python 更新控件

下拉框内容也可以随任务更新。创建 `Select` 后，保存返回的对象，再用 `set_options()` 更换选项、用 `set_value()` 选择其中一项：

```python
# 在 ui.View 中创建
preset = ui.Select("Preset", options=["Walk", "Stand"], value="Stand")

# 在后续操作中更新
preset.set_options(["Walk", "Stand", "Trot"])
preset.set_value("Trot")
renderer.sync(data)
```

`Select`、`RadioGroup` 和 `ToggleGroup` 返回的 state 都提供 `options`、`value`、`set_options()` 和 `set_value()`。setter 的修改由下一次 `sync()` 统一提交，在后续同步中使 `is_changed()` 返回 `True` 并调用 `on_change`。调用 setter 时不会立即执行回调。

更多控件示例：[ui_gallery.py](../../../../examples/viewer/ui_gallery.py)。

### 按需显示工具面板

`hide()` 隐藏面板，`show()` 恢复显示。以下片段使用控制面板示例中的 `controls` 对象：

```python
renderer.hide(controls)
renderer.sync(data)

renderer.show(controls, select=True)
renderer.sync(data)
```

这里展示了隐藏和恢复两种操作，实际使用时分别放到对应的按键或按钮处理分支中。`select=True` 会同时激活该面板所在的标签。

```python
from motrixsim import ui

with ui.View(title="Details", visible=False) as details:
    ui.Text("Simulation details")
```

将 `details` 加入 `blueprint`。`visible=False` 使面板初始隐藏。调用 `renderer.show(details)` 后，面板会回到保留的停靠位置。

`show()` 和 `hide()` 支持三种选择方式，但不能混用：

- 传入 View 对象。
- 用 `title` 精确匹配面板标题。
- 用 `include` 和 `exclude` 正则表达式匹配完整标题，排除规则优先。

```python
renderer.hide(title="Controls")
renderer.show(include=["Camera.*"], exclude=["Camera Debug"])
renderer.sync(data)
```

所有面板都要在渲染器的构造蓝图中声明。蓝图之外的 ViewId 会被渲染服务记录错误日志并忽略。按标题搜索可以恢复仍存在的隐藏面板。要重新打开已关闭的面板，请将原来的 View 对象传给 `show(view)`。`select=True` 仅支持传入 View 对象。

### 动态 Frame 与局部滚动

`render_app_frame.py` 使用 `Mode` 下拉框切换两组控件。`Position` 显示目标位置，`Velocity` 显示目标速度和限速开关。

`build_parameters()` 声明两组参数，`ui.Frame` 显示当前参数组。Frame 的构建函数每次都要声明一个根节点。使用 `ui.Column()` 或 `ui.Row()` 组合多个控件。模式切换时，示例调用 `renderer.rebuild(frame)` 请求重建 Frame。

在仓库根目录运行：

```bash
uv run python examples/viewer/render_app_frame.py
```

```{literalinclude} ../../../../examples/viewer/render_app_frame.py
:language: python
:start-at: import motrixsim as mx
```

切换到 `Velocity` 后，面板中的位置输入框会替换为速度输入框，并出现限速开关。输入目标值并按 Enter，终端会打印当前模式和值；本例演示参数面板的切换，控制量的计算由具体控制算法实现。

切换模式后，通过新的 `target` 读取输入。本例每次将输入值设为 `"0"`；保留用户输入时，先将值保存到 Python 变量，再传给新控件。

Frame 用于条件分支、控件增删等结构变化。为 `ui.Frame` 声明 builder 后，调用 `RenderApp.rebuild(frame)` 请求在后续同步中重建。重建替换控件后，旧控件对象失效，继续使用会抛出 `ReferenceError`，其回调也不会再执行。保存并使用 builder 新建的控件对象。

示例中的 `ui.ScrollArea(max_height=240)` 将参数区域限制为 240 像素高，内容较多时可在其中滚动。

`ScrollArea` 是可组合的垂直容器。Section 是否滚动由其内部是否嵌套 ScrollArea 决定。`max_height` 必须是有限正数，内容不足时区域沿纵向收缩。区域横向占满父容器可用宽度，滚动条位于最右侧。内容溢出时，只滚动这个局部区域。

### 图像绑定与显示

`ui.Image` 声明显示区域，`renderer.create_image()` 创建图片资源，`set_image()` 将图片绑定到显示区域。

在仓库根目录运行以下示例，将同一张 320×240 的黑色 RGB 图像显示在嵌入区域和独立的 **RGB Preview** 标签中。

```python
import numpy as np
import motrixsim as mx
from motrixsim import run, ui
from motrixsim.render import RenderApp, RenderClosedError

model = mx.load_model("examples/assets/boston_dynamics_spot/scene.xml")
data = mx.SceneData(model)

with ui.View(title="Diagnostics") as diagnostics:
    with ui.Column():
        ui.Text("RGB")
        embedded = ui.Image(width=320, height=240)

preview = ui.Image(title="RGB Preview", toolbar=False)
blueprint = ui.Tabs([diagnostics, preview])

with RenderApp(blueprint=blueprint) as renderer:
    pixels = np.zeros((240, 320, 3), dtype=np.uint8)
    image = renderer.create_image(pixels)
    embedded.set_image(image)
    preview.set_image(image)
    renderer.launch(model)
    try:
        run.render_loop(
            model.options.timestep,
            60,
            lambda: mx.step(model, data),
            lambda: renderer.sync(data),
        )
    except RenderClosedError:
        pass
```

同步后，两个显示区域都会显示这张黑色图像。将 `pixels` 换成算法输出即可显示处理结果；持续更新的完整示例见 [image_widget.py](../../../../examples/viewer/image_widget.py)。

修改 `image.pixels` 后调用 `sync()`，共享同一图片的显示区域都会更新，无需重建 View。`set_image(None)` 可以清空显示。

在 `ui.View` 或 Frame 中调用 `ui.Image(...)` 会添加嵌入图片，在这些上下文之外调用则创建独立面板，默认标题为 `"Image"`。显示区域可以先保持为空，之后再绑定图片。

嵌入图片默认使用原始像素尺寸。只指定一个尺寸时保持原始比例，同时指定宽高时，由 `keep_aspect_ratio` 决定保持比例完整显示（contain）还是拉伸。

独立 Image 默认占满 View，显式指定尺寸时在 View 内居中。仅独立模式支持 `fit_to_size`，默认值为 `True`。设为 `False` 时，图片按原始像素尺寸居中显示，超出面板的内容会被裁剪。

独立图片面板的工具栏提供缩放控件。独立 `ui.Image` 或 `ui.Viewport` 可以通过 `toolbar=False` 初始隐藏工具栏，之后仍可从所属 Tabs 的 **...** 菜单恢复。该参数只声明初始状态，没有对应的 Python 运行期 setter。

相机返回的 CPU 截图也支持共享更新。CPU 截图首次提交时，原 Image 会关联到当前 RenderApp。提交成功后，显示区域只保留图片资源，不会延长 Python Image 或其 RenderApp 的生命周期。

## UI Gallery

下面的 Gallery 展示布局容器、选择控件、动态 Frame 和图片的组合方式：

```{literalinclude} ../../../../examples/viewer/ui_gallery.py
:language: python
:dedent:
:start-after: "# tag::ui_gallery[]"
:end-before: "# end::ui_gallery[]"
```

下面的独立示例演示如何更新选择控件。运行前先创建 `model` 和对应的 `data`：

```python
from motrixsim import ui
from motrixsim.render import RenderApp

with ui.View(title="Settings") as settings:
    preset = ui.Select("Preset", options=["Fast", "Accurate"], value="Fast")

with RenderApp(blueprint=settings) as render:
    render.launch(model)
    preset.set_options(["Draft", "Final"], value="Final")
    render.sync(data)
    assert preset.value == "Final"
```

完整代码见 [ui_gallery.py](../../../../examples/viewer/ui_gallery.py)。图片共享和更新示例见 [image_widget.py](../../../../examples/viewer/image_widget.py)。

## 离屏渲染

批量生成相机数据可以使用无头渲染，直接输出图像。用 `RenderApp(headless=True)` 创建渲染器，调用物理步进和渲染同步，再通过相机捕获接口读取结果。

无头模式不能传入 `blueprint`。详细流程见[相机](../render/camera.md)和[CPU 软渲染](../render/headless_cpu.md)。

## Gizmos 绘制

在机器人旁边画出目标点、方向或坐标轴，可以直观比较控制目标和实际运动。在渲染函数中通过 `renderer.gizmos` 添加这些辅助图形，再调用 `sync()` 显示。

| 方法名             |        释义         |
| :----------------- | :-----------------: |
| draw_sphere        |    绘制线框球形     |
| draw_cuboid        |    绘制线框矩体     |
| draw_cylinder      |   绘制线框圆柱体    |
| draw_capsule       |   绘制线框胶囊体    |
| draw_ray           |      绘制射线       |
| draw_line          |      绘制线段       |
| draw_arrow         |      绘制箭头       |
| draw_rect          |    绘制 2D 矩形     |
| draw_grid          |    绘制 3D 栅格     |
| draw_axes          |     绘制坐标轴      |
| set_draw_collider  | 开启/关闭碰撞体绘制 |
| set_draw_joint     |  开启/关闭关节绘制  |
| set_draw_site      | 开启/关闭参考点绘制 |
| set_collider_color | 设置碰撞体绘制颜色  |
| set_joint_color    |  设置关节绘制颜色   |
| set_line_width     |  设置绘制线段宽度   |
| set_joint_size     |  设置关节绘制大小   |

辅助图形只对当前帧生效。希望某个标记一直显示，就需要在每次 `sync()` 前重新添加，即使它的位置没有变化。

```{literalinclude} ../../../../examples/viewer/gizmos.py
:language: python
:dedent:
:start-after: "# tag::draw_gizmos[]"
:end-before:  "# end::draw_gizmos[]"
```

_完整代码见 [examples/viewer/gizmos.py](../../../../examples/viewer/gizmos.py)_

## IO 输入事件

按住 `w` 控制机器人前进、点击场景选取目标，都需要将窗口输入用于控制计算。通过 [`render.input`] 获取 [`Input`] 对象，在 `sync()` 后查询本轮事件，再更新控制量。

| 方法 | 用途 |
| --- | --- |
| `is_key_just_pressed("space")` | 判断本轮是否刚按下按键，适合切换状态 |
| `is_key_pressed("w")` | 判断按键是否持续按住，适合连续控制 |
| `is_mouse_just_pressed("left")` | 判断本轮是否刚按下鼠标按钮 |
| `mouse_ray()` | 获取鼠标位置对应的三维射线，返回起点和方向共 6 个分量 |

鼠标拾取和键盘控制示例见 [mouse_click.py](../../../../examples/control/mouse_click.py) 和 [keyboard_car.py](../../../../examples/control/keyboard_car.py)。

### 合法键盘输入列表

以下是支持的键盘按键输入列表：

| 按键类型 |           示例按键            |
| :------- | :---------------------------: |
| 字母键   |      `a`, `b`, ..., `z`       |
| 数字键   |      `0`, `1`, ..., `9`       |
| 功能键   |    `f1`, `f2`, ..., `f12`     |
| 特殊键   |    `enter`, `esc`, `space`    |
| 方向键   | `up`, `down`, `left`, `right` |
| 符号键   | `=`, `-` |

以上键名不区分大小写，示例统一使用小写名称。表格中未列出的键位暂不支持。

## 系统相机控制

系统相机围绕焦点运动，焦点始终位于屏幕中心。以下操作作用于鼠标所在的视口：

-   鼠标左键按下并拖动：绕着焦点旋转系统相机
-   鼠标右键按下并拖动：移动焦点（此时显示红圈为焦点）
-   鼠标滚轮：缩放（到焦点位置不可再放大）

更多的相机控制方式请参考 [相机](../render/camera.md)。

## 单模型多实例渲染

[`render.launch(model)`] 的 `batch` 指定渲染实例数，须与批量 `SceneData` 的实例数一致。`render_offset` 指定各实例的显示偏移，用于将场景并排显示。

以下示例创建三个初始状态相同的实例，并沿 Y 轴排列。各实例的数据可以分别修改。

```python
import motrixsim as mx
from motrixsim import run
from motrixsim.render import RenderApp, RenderClosedError

model = mx.load_model("examples/assets/boston_dynamics_spot/scene.xml")
offsets = [[0, 0, 0], [0, 2, 0], [0, -2, 0]]
data = mx.SceneData(model, batch=(3,))

with RenderApp() as renderer:
    renderer.launch(model, batch=3, render_offset=offsets)
    try:
        run.render_loop(
            model.options.timestep,
            60,
            lambda: mx.step(model, data),
            lambda: renderer.sync(data),
        )
    except RenderClosedError:
        pass
```

显示偏移用于排列渲染画面，不改变各实例自身的物理坐标。更多数据操作见 [parallelsim.py](../../../../examples/parallel/parallelsim.py)。

`parallelsim.py` 中的批量加载片段如下：

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:language: python
:dedent:
:start-after: "# tag::init batch data"
:end-before: "# end::init batch data"
```

### 实例可见性

`set_scene_vis()` 按从 0 开始的索引设置实例可见性。以下片段用于已调用 `launch()` 的三实例场景，隐藏第 1、2 号实例后，仅显示第 0 号实例：

```python
renderer.set_scene_vis([1, 2], False)  # 隐藏第 1、2 号实例
renderer.sync(data)

renderer.set_scene_vis([1, 2], True)   # 恢复显示
renderer.sync(data)
```

将隐藏和恢复操作分别放入对应的输入处理分支：隐藏后只显示第 0 号实例，恢复后三组场景重新出现。需要统一切换所有实例时，调用：

```python
renderer.set_all_scene_vis(False)  # 隐藏所有实例
renderer.sync(data)

renderer.set_all_scene_vis(True)   # 显示所有实例
renderer.sync(data)
```

上述操作仅影响渲染可见性，不影响对象的物理仿真。

_完整代码见 [examples/viewer/partial_rendering.py](../../../../examples/viewer/partial_rendering.py)_

## 常见问题

| 现象 | 检查方式 |
| --- | --- |
| 模型显示了，但不运动 | 检查 Python 循环是否调用 `mx.step(model, data)`，以及运行开关是否开启 |
| 暂停后按钮不响应 | 保留渲染循环中的 `sync()`，只暂停物理步进 |
| 布局中没有 Inspector 或 Properties | 这些面板属于交互式查看器，需要使用 `viewer.launch()` |
| 自定义视口没有相机画面 | 为 `ui.Viewport` 显式传入 `camera=ui.CameraRef.system()` 或模型相机 |
| 关窗后出现 RenderClosedError | 在循环外捕获该异常，并用 `with RenderApp()` 释放资源 |

[`RenderApp`]: motrixsim.render.RenderApp
[`load_model`]: scene_model.md
[`render.launch(model)`]: motrixsim.render.RenderApp.launch
[`render.sync(data)`]: motrixsim.render.RenderApp.sync
[`render.input`]: motrixsim.render.RenderApp.input
[`Input`]: motrixsim.render.Input
