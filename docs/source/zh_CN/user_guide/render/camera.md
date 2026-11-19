# 📷 相机（Camera）

## 系统相机

当您使用渲染器功能时，MotrixSim 会自动创建一个系统相机。 系统相机可以接受用户的鼠标操作，实现移动、缩放等等。

```{video} /_static/videos/store_motrixsim.mp4
:poster: /_static/images/poster/store_motrixsim.jpg
:caption: 系统相机操作效果
:nocontrols:
:autoplay:
:playsinline:
:muted:
:loop:
:width: 100%
```

操作说明：

-   鼠标左键按下并拖动：绕着焦点旋转摄像头
-   鼠标右键按下并拖动：移动焦点（此时显示红圈为焦点）
-   鼠标滚轮：缩放（到焦点位置不可再放大）

系统相机支持[mjcf/visual/global](https://mujoco.readthedocs.io/en/stable/XMLreference.html#visual-global)中的部分配置，例如:

-   orthographic： 是否采用正交投影
-   fovy： 相机视野
-   azimuth： 初始化下，系统相机绕 z 轴的方位角
-   elevation： 初始化下，系统相机的俯仰角

系统相机可以被开启/关闭：

```
system_camera = render.system_camera # 获取系统相机
system_camera.active = True # 开启相机
system_camera.active = False # 关闭相机
```

在 `render.launch(...)` 之后，可以用 orbit/look-at 参数设置系统相机视角。
该 API 操作的是渲染器的系统相机，不是模型内定义的 MJCF camera。
`azimuth` 和 `elevation` 的单位是度。

```python
render.system_camera.set_view(
    lookat=[0.0, 0.0, 0.75],
    distance=6.0,
    elevation=-20.0,
    azimuth=90.0,
)
```

### 近/远平面与输出模式

系统相机在 `build()` 之前通过 MSD 层的 `world.system_camera` 配置；构建后的 model 不可变。
显式裁剪平面必须成对配置且满足 `znear > 0`、`zfar > znear`；默认情况下两个平面都根据场景范围自动推导。

```python
sys_cam.znear = near
sys_cam.zfar = far
```

系统相机同样支持深度与分割输出，语义与场景相机一致。该配置作用于离屏采集使用的
`"image"` 渲染目标；窗口模式下的系统相机会以配置的模式启动，并且仍可在运行期通过视口工具栏切换。

```python
if mode == "depth":
    sys_cam.depth_only = True
elif mode == "seg-colorful":
    sys_cam.segmentation = mx.msd.Segmentation.Colorful
elif mode == "seg-id":
    sys_cam.segmentation = mx.msd.Segmentation.Id
```

## 场景相机

除了系统相机之外，您可以在 MJCF 文件中配置额外的 Camera 标签。 我们把这种相机称为场景相机。场景相机可以为可视化提供额外的视角

### 自定义额外相机

以[go1.xml](../../../../examples/assets/go1/go1_mjx_fullcollisions.xml)为例，我们在 mjcf 中定义通过如下方式定义了相机:

```xml
<body name="trunk" pos="0 0 0.445" childclass="go1">
    <camera name="track" pos="0.846 -1.3 0.316" xyaxes="0.866 0.500 0.000 -0.171 0.296 0.940" mode="track"/>
    <camera name="top" pos="-1 0 1" xyaxes="0 -1 0 0.7 0 0.7" mode="track"/>
    <camera name="side" pos="0 -2 1" xyaxes="1 0 0 0 1 2" mode="track"/>
    <camera name="back" pos="-2.4 0 0.8" target="trunk" mode="targetbody"/>
</body>
```

这意味着我们在 go1 的 trunk 上定义了 4 个额外的相机。 相机的位置和朝向由 pos 和 xyaxes 指定，mode 指定了相机的运动模式。

同样的，场景相机也可以开启/关闭：

```
camera_index = 0 # 第一个场景相机的索引
scene_camera = render.get_camera(camera_index) # 获取场景相机
scene_camera.active = True # 开启相机
scene_camera.active = False # 关闭相机
```

### 切换视口相机

每个 `ui.Viewport` 都有独立的相机绑定。在布局 blueprint 中声明初始相机：系统相机使用 `ui.CameraRef.system()`，场景相机使用 `ui.CameraRef(camera.index)`。以下片段假设已经创建 `model` 及对应的 `SceneData` 实例 `data`，模型包含上文的 Go1 相机。

```python
from motrixsim import ui
from motrixsim.render import RenderApp

view = ui.Viewport(camera=ui.CameraRef(model.cameras["back"].index), title="Robot")
with RenderApp(blueprint=view) as render:
    render.launch(model)
    render.set_viewport_camera(view, model.cameras["side"])
    render.sync(data)
```

保留并使用加入当前 RenderApp blueprint 的同一个 `view` 对象。`set_viewport_camera(view, camera)` 只修改指定视口，也可修改隐藏但仍存活的视口。目标无效或已关闭时，不会改为操作其他视口。

键盘快捷键使用 `render.set_active_viewport_camera(camera)`，修改最近接收输入的视口。按快捷键前先点击目标视口。渲染器执行排队命令时才确定目标；目标不可用时会在 Editor 日志中报错，不会选择其他视口。

两个 setter 都接受 `None`，表示切回系统相机。在渲染循环中持续调用 `render.sync(data)` 以传递排队的修改。启动时使用 blueprint 绑定，不依赖此前是否与视口交互。

| 操作 | 接口 |
| --- | --- |
| 设置初始相机 | `ui.Viewport(camera=ui.CameraRef(...))` |
| 切换指定视口 | `render.set_viewport_camera(view, camera)` |
| 切换最近交互的视口 | `render.set_active_viewport_camera(camera)` |
| 切回系统相机 | 向任一 setter 传入 `None` |

完整的[双视口样例](../../../../examples/viewer/viewport_camera_targets.py) 演示了两种切换方式。相机叠加控件始终绑定到目标相机，详见 [CameraViewport](camera_viewport.md)。

```python
cameras = model.cameras # 获取所有相机(注意这里不包括系统相机)
preview_cameras = [None,*cameras] # None表示系统相机
```

通过键盘事件切换相机：

```{literalinclude} ../../../../examples/control/go1.py
:language: python
:dedent:
:start-after: "# tag: switch camera"
:end-before:  "# endtag"

```

```{video} /_static/videos/switch_camera.mp4
:caption: 通过左右按键切换交互视口的相机
:nocontrols:
:autoplay:
:playsinline:
:muted:
:loop:
:width: 100%
```

### RGBD 传感器

MotrixSim 支持将场景相机转变为 RGBD 视觉传感器，从而允许您获取相机的 RGB 图像和深度图像。 您可以通过以下接口来让一个场景相机渲染到一个离线图片上:

```{literalinclude} ../../../../examples/control/go1.py
:language: python
:dedent:
:start-after: "# tag: camera render target"
:end-before:  "# endtag"

```

默认情况下，相机采用 RGB 渲染模式。 如果您想让相机只渲染深度图像，可以设置相机的 depth_only 属性为 True:

```{literalinclude} ../../../../examples/control/go1.py
:language: python
:dedent:
:start-after: "# tag: depth camera"
:end-before:  "# endtag"

```

```{note}
深度图里保存的数值是NDC（normalized device coordinates）空间的。 如果您想将其转换为实际3D空间中与摄像机的距离，可以使用如下公式:

view_z = camera.near_plane / depth

```

RGBD 相机的实时效果会在左侧的 Camera 面板中显示：

```{video} /_static/videos/rgbd_camera.mp4
:caption: 右上角展示RGB相机和深度相机实时效果
:nocontrols:
:autoplay:
:playsinline:
:muted:
:loop:
:width: 100%
```

### Segmentation

MotrixSim 支持将场景相机转变为 Segmentation 视觉传感器，从而允许您获取场景中几何物体的语义分割图（Semantic Segmentation Map）。 同样的，我们需要配置相机的输出图片：

```{literalinclude} ../../../../examples/viewer/capture_segmentation.py
:language: python
:dedent:
:start-after: "# tag: camera render target"
:end-before:  "# endtag"

```

默认情况下，相机采用 RGB 渲染模式。 如果您想让相机渲染着色分割图，可以设置相机的 segmentation 属性为 colorful:

```{literalinclude} ../../../../examples/viewer/capture_segmentation.py
:language: python
:dedent:
:start-after: "# tag: segmentation camera"
:end-before:  "# endtag"

```

此模式的输出图像类似于：

```{figure} /_static/images/capture_segmentation.png
:width: 100%
```

如果您想获得几何体的 ID 图像，可以设置相机的 segmentation 属性为 id:

```{literalinclude} ../../../../examples/viewer/capture_id_color.py
:language: python
:dedent:
:start-after: "# tag: id color camera"
:end-before:  "# endtag"

```

此模式的输出图像类似于：

```{figure} /_static/images/capture_id_color.png
:width: 100%
```

它是一张 R 单通道的图像。

为了在渲染时与背景区分开来，0 值被用作于背景。所以对于一个 geom_id =0 的几何物体而言，它的渲染 id 是 1， 即 id = geom_id + 1。

```{note}

特殊格式的图像输出，必须在构建模型前把相机的渲染目标设置为图片，例如 `camera.render_target = msd.RenderTarget.Image` 并配合 `camera.resolution = [320, 240]`。

当前输出图像格式是R8Unorm，并非Srgb图像格式，图像数据即为精确值。

```

### 读取相机图像数据

要读取相机图像数据，您需要从 RenderApp 上获取 RenderCamera 对象，然后执行 capture 操作。

```{literalinclude} ../../../../examples/control/go1.py
:language: python
:dedent:
:start-after: "# tag: camera capture"
:end-before:  "# endtag"
```

注意到，我们的 capture 操作是异步的，因此[`RenderCamera.capture`]方法会返回一个 CaptureTask 对象，您需要保存这个 CaptureTask 对象，并持续去检查它的状态，直到它完成。完成后，您可以通过 task.take_image()方法获取图像数据。

您可以看 Api 文档来获取更多细节:

-   [`RenderCamera`]
-   [`CaptureTask`]
-   [`Image`]

[`RenderCamera`]: motrixsim.render.RenderCamera
[`CaptureTask`]: motrixsim.render.CaptureTask
[`Image`]: motrixsim.render.Image
[`RenderCamera.capture`]: motrixsim.render.RenderCamera.capture
