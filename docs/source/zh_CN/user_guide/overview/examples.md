# 📚 示例程序

:::{tip}
模型和代码详见 [MotrixSim Docs](https://github.com/Motphys/motrixsim-docs) 仓库

在运行示例之前，请先参考 {ref}`路径二：克隆仓库运行示例 <install-from-repository>` 完成环境准备。
:::

**Linux 和 Windows** 上，在仓库根目录执行 `uv run examples/{category}/{example_name}.py` 即可运行对应示例；表格中**参数**列非空的示例，请在命令后追加相应参数。支持 GPU 的共用示例可在命令后追加 `--gpu` 切换后端。

**MacOS** 上，使用 RenderApp 的示例（机械臂控制、机器人运动等大多数可视化示例）需将 `uv run` 替换为 `uv run mxpython`，例如：

```bash
uv run mxpython examples/{category}/{example_name}.py
```

不使用 RenderApp（仅物理仿真计算）的示例与其他平台一致，直接使用 `uv run`。

## 基础入门

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **展示**
  - **示例**
  - **参数**
  - **说明**
* - ![falling_ball](/_static/images/examples/thumbs/falling_ball.jpg)
  - [`falling_ball.py`](../../../../examples/getting_started/falling_ball.py)
  - —
  - 小球在重力作用下下落，展示如何创建`model`与`data`。
* - ![load_from_str](/_static/images/examples/thumbs/load_from_str.jpg)
  - [`load_from_str.py`](../../../../examples/getting_started/load_from_str.py)
  - —
  - 从字符串加载 MJCF 模型，展示如何直接从 XML 字符串创建场景。
* - ![hello_usd](/_static/images/examples/thumbs/hello_usd.jpg)
  - [`hello_usd.py`](../../../../examples/physics/hello_usd.py)
  - —
  - 从 USD 文件加载场景（cobotta_pro_1300 机械臂），展示与 MJCF 相同的 `load_model` 入口。
* - ![slope](/_static/images/examples/thumbs/slope.jpg)
  - [`slope.py`](../../../../examples/getting_started/slope.py)
  - —
  - 方块在斜坡上滚动的物理仿真模型。
```

## 物理仿真

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **展示**
  - **示例**
  - **参数**
  - **说明**
* - ![actuator](/_static/images/examples/thumbs/actuator.jpg)
  - [`actuator.py`](../../../../examples/physics/actuator.py)
  - —
  - 获取和配置`actuator`的参数。
* - ![body](/_static/images/examples/thumbs/body.jpg)
  - [`body.py`](../../../../examples/physics/body.py) · [GPU](../../../../examples/gpu/body.py)
  - —
  - `body`相关 API 的使用，这里的`body`特指根节点的 world body。
* - ![joint](/_static/images/examples/thumbs/joint.jpg)
  - [`joint.py`](../../../../examples/physics/joint.py) · [GPU](../../../../examples/gpu/joint.py)
  - —
  - `joint`相关 API 的使用，包括读写`dof_position`和`dof_velocity`。
* - ![link](/_static/images/examples/thumbs/link.jpg)
  - [`link.py`](../../../../examples/physics/link.py)
  - —
  - `link`相关 API 的使用。
* - ![options](/_static/images/examples/thumbs/options.jpg)
  - [`options.py`](../../../../examples/physics/options.py)
  - —
  - 使用`options`对模拟器进行参数配置。
* - ![site_and_sensor](/_static/images/examples/thumbs/site_and_sensor.jpg)
  - [`site_and_sensor.py`](../../../../examples/physics/site_and_sensor.py)
  - —
  - `site`和`sensor`相关 API 的使用。
* - ![friction](/_static/images/examples/thumbs/friction.jpg)
  - [`friction.py`](../../../../examples/physics/friction.py)
  - —
  - 摩擦力配置的场景。
* - ![geom](/_static/images/examples/thumbs/geom.jpg)
  - [`geom.py`](../../../../examples/physics/geom.py)
  - —
  - 几何体相关 API 的使用，展示如何访问和查询几何体的位置、速度等信息。
* - ![hfield](/_static/images/examples/thumbs/hfield.jpg)
  - [`hfield.py`](../../../../examples/physics/hfield.py)
  - —
  - 高度场 API 的使用，展示如何访问地形高度数据并进行统计分析。
* - ![combine_msd](/_static/images/examples/thumbs/combine_msd.jpg)
  - [`combine_msd.py`](../../../../examples/physics/combine_msd.py)
  - —
  - 组合多个 MSD 模型，展示如何使用 `Scene.attach()` 方法将多个模型附加到一起，支持变换和命名空间前缀。
* - ![adhesion](/_static/images/examples/thumbs/adhesion.jpg)
  - [`adhesion.py`](../../../../examples/physics/adhesion.py)
  - —
  - 使用吸附功能的机械臂。
* - ![external_force](/_static/images/examples/thumbs/external_force.jpg)
  - [`external_force.py`](../../../../examples/physics/external_force.py)
  - —
  - 外力与外力矩施加示例，展示如何使用 `add_external_force` 和 `add_external_torque` API 在局部坐标系中对刚体施加力和力矩。包含质心推力、绕轴力矩、偏移点施力三个阶段的演示。
* - ![lidar_point_cloud_demo](/_static/images/examples/thumbs/lidar_point_cloud_demo.jpg)
  - [`lidar_point_cloud_demo.py`](../../../../examples/sensors/lidar_point_cloud_demo.py)
  - —
  - 雷达点云演示，展示旋转式（16 线环形）与固态网格型 lidar 传感器的配置及点云可视化。
* - ![touch_sensor_demo](/_static/images/examples/thumbs/touch_sensor_demo.jpg)
  - [`touch_sensor_demo.py`](../../../../examples/sensors/touch_sensor_demo.py)
  - —
  - 触觉传感器演示，展示如何读取 touch sensor 数据检测机械臂末端与环境物体的接触，并进行力反馈可视化。
* - ![sdf_collision](/_static/images/examples/thumbs/sdf_collision.jpg)
  - [`sdf_collision.py`](../../../../examples/physics/sdf_collision.py)
  - —
  - 两个啮合齿轮的 SDF 碰撞演示，齿面接触由 SDF narrow phase 求解，速度执行器驱动主动轮，窗口内以 gizmos 绘制接触点与法向。详见 [SDF 碰撞文档](../kinematics/geometry/sdf.md)。
* - ![equality_transmission_demo](/_static/images/examples/thumbs/equality_transmission_demo.jpg)
  - [`equality_transmission_demo.py`](../../../../examples/control/equality_transmission_demo.py)
  - —
  - equality 约束演示，包含焊接法兰平台与关节传动画廊两个场景，展示由 equality/joint 行耦合的传动结构。
* - ![strandbeest](/_static/images/examples/thumbs/strandbeest.jpg)
  - [`strandbeest.py`](../../../../examples/physics/strandbeest.py)
  - `--suspended` 悬空展示 · `--headless` 进入测量模式（其余参数用于基准测量输出）
  - 单曲柄 Strandbeest 多足步行机构，展示 equality 闭环连接约束驱动的行走运动。
```

## 控制

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **展示**
  - **示例**
  - **参数**
  - **说明**
* - ![keyboard_car](/_static/images/examples/thumbs/keyboard_car.jpg)
  - [`keyboard_car.py`](../../../../examples/control/keyboard_car.py)
  - —
  - 使用键盘操控小车移动，展示键盘事件的使用。W/S 前进后退，A/D 转向。
* - ![car](/_static/images/examples/thumbs/car.jpg)
  - [`car.py`](../../../../examples/control/car.py)
  - —
  - 遥控小车完整驾驶示例，展示转向/驱动/刹车执行器控制、航向保持与速度渐变，搭配自定义 UI 布局。
* - ![mouse_click](/_static/images/examples/thumbs/mouse_click.jpg)
  - [`mouse_click.py`](../../../../examples/control/mouse_click.py) · [GPU](../../../../examples/gpu/mouse_click.py)
  - —
  - 使用鼠标点击地面移动小球，展示鼠标事件的使用。
* - ![inverse kinematics](/_static/images/examples/thumbs/ik.jpg)
  - [`ik.py`](../../../../examples/control/ik.py)
  - —
  - 展示如何使用 MotrixSim 内置的 IK 模块进行逆运动学求解。
* - ![local_arm](/_static/images/examples/thumbs/local_arm.jpg)
  - [`local_arm.py`](../../../../examples/control/local_arm.py)
  - —
  - 由简单几何形状和`joint`组成的机械臂。
* - ![robotic_arm](/_static/images/examples/thumbs/robotic_arm.jpg)
  - [`robotic_arm.py`](../../../../examples/control/robotic_arm.py)
  - —
  - 斯坦福机械臂使用移动命令序列进行小球的抓取与摆放。
* - ![apple_grasp](/_static/images/examples/thumbs/apple_grasp.jpg)
  - [`apple_grasp.py`](../../../../examples/control/apple_grasp.py)
  - —
  - 回放夹苹果梗、提起、放回、抓果身并再次提起的动作序列。自动播放 40 秒动作序列，支持暂停和继续，运行结果输出到终端。
* - ![robot_locomotion](/_static/images/poster/go2_keyboard_control.jpg)
  - [`robot_locomotion.py`](../../../../examples/control/robot_locomotion.py)
  - `--robot go1|go2|g1|microduck`（默认 go2）
    `--scene playground|parlour|plane|stairs`（默认 playground，microduck 仅支持内置平地）
    `--lidar <profile>`（仅 g1 / go1 / go2）
  - 机械狗/人形机器人的键盘控制示例，方向键和 WASD 控制行走与转向。microduck 使用内置 ONNX 行走策略，躯干触地后自动复位。
* - ![g1_motion_tracking](/_static/images/poster/g1_motion_tracking.png)
  - [`g1_motion_tracking.py`](../../../../examples/control/g1_motion_tracking.py)
  - —
  - g1 人形机器人的运动跟踪播放示例，加载内置 motion 参考轨迹和 ONNX actor 进行单环境播放。
* - ![rm65_open_cabinet](/_static/images/examples/thumbs/rm65_open_cabinet.jpg)
  - [`rm65_open_cabinet.py`](../../../../examples/control/rm65_open_cabinet.py)
  - —
  - RM65 开抽屉策略示例，展示如何加载 ONNX 策略并控制机械臂完成柜体抽屉操作。[演示视频](/_static/videos/rm65_open_cabinet.mp4)。按 `R` 重置，按 `ESC` 退出。
* - ![shadow_hand_repose](/_static/images/examples/thumbs/shadow_hand_repose.jpg)
  - [`shadow_hand_repose.py`](../../../../examples/control/shadow_hand_repose.py)
  - —
  - Shadow Hand 方块重定向策略示例，展示如何加载 ONNX 策略并控制灵巧手调整方块姿态。[演示视频](/_static/videos/shadow_hand_repose.mp4)。按 `R` 重置，按 `ESC` 退出。
* - ![osc](/_static/images/examples/thumbs/osc.jpg)
  - [`osc.py`](../../../../examples/control/osc.py)
  - —
  - 操作空间控制（OSC）交互示例，展示如何使用 `OscSolver` 配合 `IkChain` 通过计算力矩控制机械臂末端执行器的位置和姿态。方向键和 WASD 控制目标移动，Q/E 旋转，R 重置。
* - ![go1](/_static/images/poster/go1.jpg)
  - [`go1.py`](../../../../examples/control/go1.py) · [GPU](../../../../examples/gpu/go1.py)
  - —
  - go1 机械狗的随机运动，展示如何引入神经网络和使用`.onnx`文件。
* - ![go1_multi_task](/_static/images/examples/thumbs/go1_multi_task.jpg)
  - [`go1_multi_task.py`](../../../../examples/control/go1_multi_task.py)
  - —
  - Go1 机械狗多任务策略示例，支持行走、倒立、脚立、起身恢复等多种运动模式切换。WASD 控制移动，U 倒立，I 脚立，O 恢复，P 重置。
```

## 并行仿真

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **展示**
  - **示例**
  - **参数**
  - **说明**
* - ![parallelsim](/_static/images/examples/thumbs/parallelsim.jpg)
  - [`parallelsim.py`](../../../../examples/parallel/parallelsim.py) · [GPU](../../../../examples/gpu/parallelsim.py)
  - —
  - 多环境并行仿真；GPU 版本使用 Warp kernel 生成批量随机控制量。
```

## 随机化

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **展示**
  - **示例**
  - **参数**
  - **说明**
* - ![randomize_mass](/_static/images/examples/thumbs/randomize_mass.jpg)
  - [`mass.py`](../../../../examples/randomize/mass.py)
  - —
  - 随机化刚体质量，在 16 个并行实例中将质量从 0.1 kg 到 500 kg 变化，展示质量对下落动力学的影响。
* - ![randomize_friction](/_static/images/examples/thumbs/randomize_friction.jpg)
  - [`friction.py`](../../../../examples/randomize/friction.py)
  - —
  - 随机化几何体摩擦系数，在 0.01 到 2.0 范围内变化，展示物体在 30° 斜面上从滑动到静止的过渡。
* - ![randomize_armature](/_static/images/examples/thumbs/randomize_armature.jpg)
  - [`armature.py`](../../../../examples/randomize/armature.py)
  - —
  - 随机化关节转动惯量（虚拟转子惯量），展示 armature 参数对扭转弹簧驱动臂的固有频率的影响。
* - ![randomize_frictionloss](/_static/images/examples/thumbs/randomize_frictionloss.jpg)
  - [`frictionloss.py`](../../../../examples/randomize/frictionloss.py)
  - —
  - 随机化关节干摩擦力（库仑摩擦），从 0 到 200 N·m 变化，展示摩擦力如何使摆锤在不同角度停止。
* - ![randomize_com](/_static/images/examples/thumbs/randomize_com.jpg)
  - [`com.py`](../../../../examples/randomize/com.py)
  - —
  - 随机化质心位置，在圆柱体截面内偏移质心，展示质心偏移对滚动行为的影响。
* - ![inertia](/_static/images/examples/thumbs/inertia.jpg)
  - [`inertia.py`](../../../../examples/randomize/inertia.py)
  - —
  - 随机化 link 局部坐标系中关于质心的完整惯量张量，展示在质量和质心不变时，铰链轴惯量如何改变扭转弹簧臂的振荡周期。
* - ![randomize_actuator_kp_kd](/_static/images/examples/thumbs/randomize_actuator_kp_kd.jpg)
  - [`actuator_kp_kd.py`](../../../../examples/randomize/actuator_kp_kd.py)
  - —
  - 在 4×4 网格中随机化位置执行器的 PD 增益，行方向变化 kp（10–120 N·m/rad），列方向变化 kd（1–20 N·m·s/rad），可视化 PD 伺服动力学。
* - ![randomize_geom_size](/_static/images/examples/thumbs/randomize_geom_size.jpg)
  - [`geom_size.py`](../../../../examples/randomize/geom_size.py)
  - —
  - 随机化五种几何体（球体、胶囊体、盒体、圆柱体、椭球体）的碰撞和视觉尺寸，展示尺寸对沉降高度和接触行为的影响。
* - ![geom_shape](/_static/images/examples/thumbs/geom_shape.jpg)
  - [`geom_shape.py`](../../../../examples/randomize/geom_shape.py)
  - —
  - 在 16 个并行实例中随机选择 primitive 和编译期 Mesh，同时同步 effective collision 与 visual geometry，并展示 Mesh variant metadata。
* - ![randomize_gravity_direction](/_static/images/examples/thumbs/randomize_gravity_direction.jpg)
  - [`gravity_direction.py`](../../../../examples/randomize/gravity_direction.py)
  - —
  - 随机化重力方向，在 16 个并行实例的透明盒子中为每个实例设置不同的重力向量，展示 `set_gravity_override` 的逐实例重力覆盖能力。
```

## 可视化与渲染

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **展示**
  - **示例**
  - **参数**
  - **说明**
* - ![camera_control](/_static/images/examples/thumbs/change_camera_state.jpg)
  - [`camera_control.py`](../../../../examples/viewer/camera_control.py)
  - —
  - 相机控制 API 的使用，展示如何启用/禁用系统相机和场景相机，以及获取相机位姿。
* - ![camera_viewport](/_static/images/examples/thumbs/camera_viewport.jpg)
  - [`camera_viewport.py`](../../../../examples/viewer/camera_viewport.py)
  - —
  - 相机视口组件的使用，展示不同相机画面的显示，以及切换相机、调整大小和位置等交互控制。
* - ![capture_segmentation](/_static/images/examples/thumbs/capture_segmentation.jpg)
  - [`capture_segmentation.py`](../../../../examples/viewer/capture_segmentation.py)
  - —
  - 分割图捕获示例，展示如何将场景相机配置为 Segmentation 视觉传感器，获取语义分割图（colorful 模式）。详见[相机文档](../render/camera.md)。
* - ![capture_id_color](/_static/images/examples/thumbs/capture_id_color.jpg)
  - [`capture_id_color.py`](../../../../examples/viewer/capture_id_color.py)
  - —
  - ID 图捕获示例，展示如何将相机 segmentation 属性设为 id，获取几何体 ID 的单通道图像（id = geom_id + 1）。详见[相机文档](../render/camera.md)。
* - ![viewport_camera_targets](/_static/images/examples/thumbs/viewport_camera_targets.jpg)
  - [`viewport_camera_targets.py`](../../../../examples/viewer/viewport_camera_targets.py)
  - —
  - 双视口相机绑定演示，展示视口与系统/场景相机的绑定切换，相机组件在绑定变化时保持跟随。详见[相机文档](../render/camera.md)。
* - ![dynamic_texture_per_instance](/_static/images/examples/thumbs/dynamic_texture_per_instance.jpg)
  - [`dynamic_texture_per_instance.py`](../../../../examples/viewer/dynamic_texture_per_instance.py)
  - —
  - 逐实例动态纹理演示，在屏幕平面公告板上为 4 个相机视锥形状物体分别渲染各自的动态纹理。
* - ![render_app_frame](/_static/images/examples/thumbs/render_app_frame.jpg)
  - [`render_app_frame.py`](../../../../examples/viewer/render_app_frame.py)
  - —
  - RenderApp 帧面板演示，展示参数模式切换时如何动态替换 UI 控件，以及 `RenderClosedError` 的处理。
* - ![render_app_ui](/_static/images/examples/thumbs/render_app_ui.jpg)
  - [`render_app_ui.py`](../../../../examples/viewer/render_app_ui.py)
  - —
  - 自定义 RenderApp 布局演示，展示用 Python 控制仿真并搭建自定义窗口 UI。
* - ![ui_gallery](/_static/images/examples/thumbs/ui_gallery.jpg)
  - [`ui_gallery.py`](../../../../examples/viewer/ui_gallery.py)
  - —
  - UI Gallery，演示布局、内建与自定义 UI View、回调及 View 生命周期操作。
* - ![gizmos](/_static/images/examples/thumbs/gizmos.jpg)
  - [`gizmos.py`](../../../../examples/viewer/gizmos.py)
  - —
  - 3D 辅助绘图系统的使用，展示如何绘制球体、立方体、胶囊体、箭头、网格等可视化辅助元素。
* - ![image_widget](/_static/images/examples/thumbs/image_widget.jpg)
  - [`image_widget.py`](../../../../examples/viewer/image_widget.py)
  - —
  - 图像组件系统的使用，展示如何显示和交互操作多个图像面板。
* - ![render_settings](/_static/images/examples/thumbs/render_settings.jpg)
  - [`render_settings.py`](../../../../examples/viewer/render_settings.py)
  - —
  - 渲染设置配置示例，展示如何使用 `RenderSettings` 配置阴影、屏幕空间环境光遮蔽（SSAO）等渲染效果。
* - ![partial_rendering](/_static/images/examples/thumbs/partial_rendering.jpg)
  - [`partial_rendering.py`](../../../../examples/viewer/partial_rendering.py)
  - —
  - 批量渲染中的选择性渲染控制，展示如何在多环境并行仿真中动态控制特定场景的可见性。按 A/D 键切换部分场景可见性，Q/E 键控制所有场景。
* - ![share_lights_between_envs](/_static/images/examples/thumbs/share_lights_between_envs.jpg)
  - [`share_lights_between_envs.py`](../../../../examples/viewer/share_lights_between_envs.py)
  - `--share_lights=False` 禁用光源共享
  - 多环境间的光源共享优化，展示如何在并行仿真中共享光源以提升性能。
* - ![ssgi](/_static/images/poster/go2-ssgi.jpg)
  - [`ssgi.py`](../../../../examples/viewer/ssgi.py)
  - —
  - 屏幕空间全局光照（SSGI）渲染示例，展示如何使用 `RenderSettings` 启用高质量 SSGI 渲染效果。
* - ![headless](/_static/images/examples/thumbs/empty.jpg)
  - [`headless.py`](../../../../examples/viewer/headless.py)
  - `--no-wait` 切换异步捕获
  - 无头渲染示例，展示如何使用 `RenderApp(headless=True)` 在无窗口模式下运行仿真并通过系统相机批量截取图像。
* - ![g1_3dgs_locomotion](/_static/images/examples/thumbs/g1_3dgs.jpg)
  - [`g1_3dgs_locomotion.py`](../../../../examples/viewer/g1_3dgs_locomotion.py)
  - —
  - G1 人形机器人在 3DGS 高斯场景中的键盘控制，头部第一人称画面以视口组件叠加显示，展示多相机 3DGS 渲染效果。
```

## 基准测试

```{list-table}
:header-rows: 1
:class: longtable
:widths: 18 22 26 34

* - **展示**
  - **示例**
  - **参数**
  - **说明**
* - ![gyroscope](/_static/images/examples/thumbs/gyroscope.jpg)
  - [`gyroscope_mx.py`](../../../../examples/bench/gyroscope/gyroscope_mx.py)
  - —
  - 陀螺的物理仿真场景。
* - ![gyroscope_zero_gravity](/_static/images/examples/thumbs/gyroscope_zero_gravity.jpg)
  - [`gyroscope_zero_gravity_mx.py`](../../../../examples/bench/gyroscope_zero_gravity/gyroscope_zero_gravity_mx.py)
  - —
  - 零重力环境下的陀螺场景，展示角动量守恒的物理特性。
* - ![newton_cradle](/_static/images/examples/thumbs/newton_cradle.jpg)
  - [`newton_cradle_mx.py`](../../../../examples/bench/newton_cradle/newton_cradle_mx.py)
  - —
  - 牛顿摆的物理仿真场景。
* - ![grasp_shaking_test](/_static/images/examples/thumbs/grasp_shaking_test.jpg)
  - [`shake_test_mx.py`](../../../../examples/bench/grasp/shake_test_mx.py)
  - `--object cube|ball|bottle`（默认 cube） · `--noshake` 关闭抖动 · `--record` 录制视频

    注意：本示例需以模块方式运行 `uv run python -m examples.bench.grasp.shake_test_mx`
  - Franka Panda 机械臂的抓取与抖动测试，展示机械臂如何抓取物体并保持稳定。
```
