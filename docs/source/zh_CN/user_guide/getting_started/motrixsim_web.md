# 🌐 网页仿真器

## 概述

MotrixSim Web Viewer 用于在浏览器中加载模型、运行物理仿真和检查对象状态。页面提供三维视口、文件浏览器，以及关节、执行器和属性检查面板。

## 打开 Viewer

使用支持 WebAssembly 的现代浏览器打开 <https://motrix.motphys.com/>。加载官方示例时，从 **Online** 中选择模型；加载本地模型时，将模型及资源拖入页面。

通过 **Files** 面板的位置识别界面。公开站点目前在左侧显示文件树。下文截图中的新版界面将 **Files** 放在视口下方。

### 在公开站点加载 Spot

1. 在左侧 **Files** 中依次展开 **Online** 和 **boston_dynamics_spot**。
2. 单击 **scene.xml**。场景加载后，中央视口会显示 Spot。
3. 使用左侧工具栏播放、暂停或单步运行仿真。右侧的 **Joint** 和 **Actuator** 是独立面板，显示设置位于左侧 **Scenes**。

## 在新版界面加载官方示例

在页面底部找到 **Files** 面板，按以下顺序打开示例：

1. 展开左侧目录树中的 **Online**。
2. 点击 `boston_dynamics_spot` 文件夹名称。
3. 在中间文件列表中双击 `scene.xml`。

等待 **Loading resources** 进度完成。中央的 **Viewport** 中会出现 Spot 模型。

```{figure} /_static/images/web_viewer/current/master-overview.jpg
:alt: Spot 模型显示在中央视口，底部是 Files，右侧是对象和设置面板
:width: 100%

打开场景后的界面。左侧为仿真工具栏，中央为视口，右侧为对象和设置面板。
```

```{figure} /_static/images/web_viewer/current/master-files.jpg
:alt: Files 左侧选中 Online 下的 boston_dynamics_spot，中间列表显示 scene.xml，右侧显示文件详情
:width: 100%

在中间列表中双击 scene.xml 打开场景。单击文件只会选中它，并在右侧显示详情。
```

如果看不清文件名，拖动目录树与文件列表之间的分隔线，给中间列表留出更多宽度。右侧详情栏也可以缩窄。

## 通过拖拽文件夹加载模型

### 推荐的文件夹结构

浏览器不能直接读取任意本地路径，需要先将模型资源拖入页面。找到包含场景文件的文件夹，确认它也包含场景引用的网格和纹理。例如，一个模型目录可以组织为：

```text
my_robot/
├── scene.xml
├── meshes/
│   └── ...
└── textures/
    └── ...
```

这里的目录结构只是示例。保留模型原有的文件名和相对路径，不要为了匹配示例移动资源。如果 `scene.xml` 引用了 `meshes/` 中的文件，拖入时也要包含这个目录。

### 加载步骤

1. 从文件管理器中将整个模型文件夹拖入 Viewer 页面。例如，拖入上面的 `my_robot` 文件夹。
2. 在 **Files** 左侧展开 **Customize**，进入刚导入的目录。
3. 在中间文件列表中找到场景文件，例如 `scene.xml`，双击文件名打开。单击只会选中文件并显示详情。
4. 等待 **Loading resources** 进度完成，检查 **Viewport** 中是否出现自己的模型。

## `Online` 与 `Customize` 的区别

**Online** 保存 Viewer 提供的官方在线示例。**Customize** 显示拖入当前浏览器会话的本地文件和文件夹，用于加载自己的 MJCF、URDF 及其网格、纹理资源。刷新页面后，如果自定义资源不再显示，重新拖入模型目录。

如果界面显示 **Remote**，它是远程资源入口，是否能加载取决于资源地址及访问条件。拖入的本地模型在 **Customize** 中查找。

如果加载时提示找不到网格或纹理，检查缺失文件是否包含在拖入的目录中，以及场景文件中的相对路径是否与实际目录一致。补齐资源后，重新拖入完整目录并打开场景。

## 从文件浏览器加载场景

在 **Files** 中展开 **Online** 或 **Customize**，点击文件夹名称进入目录，再在中间文件列表中双击场景文件名。场景文件包括模型使用的 `.xml`、`.urdf` 或 `.json` 文件。单击只显示详情，双击才开始加载。

文件较多时，在 **Search** 中输入文件名关键字，查找当前目录中的文件。也可以使用漏斗按钮按模型类型筛选，或点击路径导航返回上级目录。

模型加载后，可以使用下文介绍的播放、暂停和关节检查功能。

## 仿真工具栏

播放和场景控制位于新版界面最左侧的竖向工具栏。

### 播放控制

```{figure} /_static/images/web_viewer/current/master-overview.jpg
:alt: 模型加载后的界面，最左侧工具栏依次显示播放、单步、重置和重载按钮
:width: 100%

最左侧工具栏从上到下为播放、单步、重置和重载。开始运行后，播放按钮变为暂停按钮。
```

1. 点击页面最左侧工具栏中的三角形播放按钮。按钮变为双竖线暂停按钮，仿真开始运行。
2. 点击暂停按钮。仿真停在当前状态，便于观察模型。
3. 点击单步按钮。仿真向前执行一帧，然后保持暂停。模型的变化可能很小，可以结合下文的关节数值检查状态。
4. 点击重置按钮，恢复初始仿真状态。

点击视口使其接收键盘输入后，也可以用 `Space` 切换播放和暂停，用 `F10` 单步，用 `Ctrl+E` 重置。如果浏览器占用了快捷键，使用工具栏按钮。

### 场景控制

方形停止按钮会停止并重置仿真，返回编辑状态。重置按钮恢复初始仿真状态，重载按钮重新读取当前模型及其资源。

界面提供 **Help** 入口时，可从该入口打开 Web Viewer 操作指南。

## 输入指南

### 鼠标操作

在 **Viewport** 中使用以下操作调整视角：

- 按住鼠标左键拖动，绕模型旋转视角。
- 滚动滚轮，拉近或拉远视角。
- 按住鼠标右键拖动，平移视角。

这些操作改变观察位置，模型的仿真状态保持不变。

### 键盘操作

- `Space`：暂停或继续仿真。
- `F10`：单步执行一帧。
- `Ctrl+E`：重置场景。
- `Ctrl+R`：重新加载当前模型。浏览器可能将此快捷键用于刷新网页，发生冲突时使用重载按钮。
- `F11`：在支持的平台上切换全屏。

### 物体拖拽交互

应用启用物理拖拽后，按住 `Ctrl`，用鼠标左键点击并拖动物体，观察它的响应。

```{video} /_static/videos/physics_drag.mp4
:poster: /_static/images/poster/physics_drag.jpg
:caption: 按住 Ctrl 后拖动物体
:playsinline:
:width: 100%
```

### 相机与显示设置

点击视口左上角的相机按钮，例如 **Perspective**，可以查看可用相机。旁边的齿轮按钮打开显示菜单，其中包含 **Camera Settings**、**Visibility** 和 **Gizmos** 等设置。

```{figure} /_static/images/web_viewer/current/master-viewport-menu.jpg
:alt: 视口齿轮菜单显示统计信息、显示模式、相机设置、可见性、辅助标记和调试显示入口
:width: 100%

相机和显示设置位于视口左上角的齿轮菜单中。
```

## 检查关节和对象属性

1. 切换到右侧的 **Inspector** 标签页。
2. 找到 **Joint** 分组，选择一个关节名称。
3. 切换到 **Properties** 标签页，查看该关节的类型、范围和运行数值。

选中关节后，模型对应部位会高亮，属性面板随选择更新。可以用面板中的搜索框查找字段。

```{figure} /_static/images/web_viewer/current/master-inspector.jpg
:alt: Inspector 中列出 Spot 模型的 Joint 和 Actuator 分组
:width: 100%

在 Joint 分组中选择关节。Actuator 分组显示执行器及其控制量。
```

```{figure} /_static/images/web_viewer/current/master-properties.jpg
:alt: 选中的关节在视口中高亮，Properties 显示其类型、范围、位置、速度和加速度
:width: 100%

Properties 显示当前选中关节的属性和状态。
```

点击播放，观察关节的位置或速度数值，再暂停。数值是否变化取决于模型是否运动；模型静止时，数值可能保持不变。

想持续查看这个关节，可以点击对应的星形收藏按钮，再切换到 **Favorites**。收藏的对象会集中显示在这里。

如果右侧标签标题显示不全，使用标签栏两端的箭头找到目标标签，或拖动面板分隔线增加宽度。

## 物理与渲染选项

切换到右侧的 **Options** 标签页，找到 **Physics** 分类。在这里可以查看关键帧、动力学和重力设置，以及执行器开关。

```{figure} /_static/images/web_viewer/current/master-options.jpg
:alt: Options 的 Physics 分类中显示 Keyframes、Dynamics、Gravity 和 Actuators 设置
:width: 100%

在 Physics 中查看当前物理设置。面板内可以滚动。
```

需要调整外观时，在同一面板中打开 **Rendering** 或 **Lighting / Environment**，修改渲染、光照和环境设置。

## 典型使用流程

1. 打开 Web Viewer。
2. 选择官方示例，或拖入完整的本地模型文件夹。
3. 在 **Files** 中双击场景文件名。
4. 使用鼠标观察场景，在检查面板中查看对象状态。
5. 使用左侧工具栏播放、暂停、重置或重载场景。

## 相关内容

- [安装指南](installation.md)
- [Hello MotrixSim](hello_motrixsim.md)
- [交互式查看器](../main_function/managed_viewer.md)
- [渲染器](../main_function/render.md)
