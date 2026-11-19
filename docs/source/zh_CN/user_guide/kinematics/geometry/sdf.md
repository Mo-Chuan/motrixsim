# 有符号距离场（SDF）

{bdg-success-line}`CPU 完全支持`

MotrixSim CPU 版本支持使用有符号距离场（Signed Distance Field，简称 SDF）进行碰撞检测。对于带有凹槽、孔洞或齿形的物体，SDF 可以保留普通凸包近似难以表达的碰撞形状。您可以通过 MJCF 或 USD 启用 SDF，MotrixSim 会在加载模型时根据网格自动生成所需数据。

## 什么是 SDF

SDF 用空间中的点到物体表面的距离来描述形状，同时用距离的正负区分物体内部和外部：物体外部为正，表面为零，内部为负。

设物体占据的闭合区域为 $\Omega \subset \mathbb{R}^3$，其表面为 $\partial\Omega$。对于空间中的点 $\mathbf{x}$，它到物体表面的最短距离定义为：

```{math}
d(\mathbf{x}, \partial\Omega)
= \min_{\mathbf{y}\in\partial\Omega}\|\mathbf{x}-\mathbf{y}\|_2
```

其中，$\mathbf{y}$ 是表面上的点，$\|\mathbf{x}-\mathbf{y}\|_2$ 表示两点之间的直线距离。在此基础上，有符号距离场是一个函数 $\phi:\mathbb{R}^3\to\mathbb{R}$：

```{math}
\phi(\mathbf{x}) =
\begin{cases}
 d(\mathbf{x}, \partial\Omega), & \mathbf{x}\notin\Omega,\\
 0, & \mathbf{x}\in\partial\Omega,\\
 -d(\mathbf{x}, \partial\Omega), & \mathbf{x}\in\operatorname{int}(\Omega).
\end{cases}
```

这里 $\operatorname{int}(\Omega)$ 表示物体内部。因此，$|\phi(\mathbf{x})|$ 表示点到表面的距离，而正负号表示点位于哪一侧。所有满足 $\phi(\mathbf{x})=0$ 的点共同构成物体表面，也称为 SDF 的**零等值面**。

例如，球心为 $\mathbf{c}$、半径为 $r$ 的球体，其 SDF 为：

```{math}
\phi(\mathbf{x}) = \|\mathbf{x}-\mathbf{c}\|_2-r
```

若球的半径为 1 米，距离球心 1.2 米的点，其 SDF 值为 0.2 米，表示它在球外、距离表面 0.2 米；距离球心 0.7 米的点，其 SDF 值为 -0.3 米，表示它在球内、距离表面 0.3 米。

在使用上，您仍然通过网格提供物体的形状和外观。启用 SDF 后，MotrixSim 使用由该网格生成的距离场计算碰撞，模型仍以网格显示，无需另外编写距离函数或准备专门的 SDF 文件。

## 什么时候使用 SDF

普通网格碰撞体通常使用凸包近似。凸包可以理解为包住物体的紧绷外壳，它会填平凹槽、封住孔洞，也可能把齿轮的齿间空隙包在内部。物体外观虽然仍有这些细节，碰撞形状却可能已经不同。

SDF 能够表达这些非凸结构，适合碰撞行为依赖局部形状的场景，例如齿轮啮合、物体进入凹槽，以及物体与带孔零件的接触。使用 SDF 时，无需为了表示这些凹陷而手动将物体拆成多个凸形状。

下面的演示展示小球沿螺旋轨道逐层滚落。轨道使用 SDF 表达凹槽形状，使小球能够沿槽道运动。

```{video} /_static/videos/sdf_trail_ball.mp4
:poster: /_static/images/poster/sdf_trail_ball.jpg
:caption: 小球沿着凹槽轨道螺旋滚落
:playsinline:
:muted:
:width: 100%
```

对于球、盒体等规则形状，直接使用对应的基础几何体即可。SDF 会增加模型加载时的数据准备和内存开销，细小结构的接触效果也需要结合实际模型验证。

## 支持范围

以下碰撞组合适用于 MotrixSim CPU 版本：

| 与 SDF 碰撞的几何体                | 当前支持情况 |
| :--------------------------------- | :----------- |
| SDF                                | 支持         |
| 球体（`sphere`）                   | 支持         |
| 盒体（`box`）                      | 支持         |
| 胶囊体（`capsule`）                | 支持         |
| 圆柱体（`cylinder`）               | 支持         |
| 椭球（`ellipsoid`）                | 支持         |
| 普通网格碰撞体（`mesh`，包括凸包） | 暂不支持     |
| 平面（`plane`）                    | 暂不支持     |
| 高度场（`hfield`）                 | 暂不支持     |

这里的“普通网格碰撞体”指未启用 SDF 的网格碰撞形状。SDF 本身虽然也使用网格资产，但两个都启用了 SDF 的几何体可以相互碰撞。

```{important}
暂不支持的组合不会产生碰撞响应。模型能够加载、外观能够显示，并不代表该组合可以碰撞。

如果两个复杂网格需要相互碰撞，请将两者都配置为 SDF。如果需要地面承托 SDF 物体，可以使用有厚度的静态盒体；盒体只覆盖指定范围，不具备无限平面的效果。
```

## 网格和场景要求

- **网格应围成闭合的实体。** 表面应能明确区分内部和外部。避免破面、缺失面和自相交；只有单层表面、没有厚度的模型不适合作为 SDF 碰撞体。带通孔的实体也可以是闭合网格，只要孔壁与外表面完整相连。
- **保留需要参与碰撞的几何细节。** 凹槽、孔壁和齿形需要体现在网格中，仅靠贴图或材质表现的细节不会参与碰撞。
- **检查尺寸、厚度和间隙。** 网格单位与场景缩放应一致。对于很薄的壁面、很窄的间隙或相对整体尺寸很小的细节，应先验证接触效果；不能仅凭渲染外观判断碰撞精度。
- **检查碰撞双方的配置。** 双方都应启用碰撞，且未被碰撞过滤规则排除；另一方的几何类型需要在上述支持范围内。

## 在 MJCF 中使用

先在 `<asset>` 中声明网格，再在 `<geom>` 上设置 `type="sdf"`，并通过 `mesh` 引用该资产。

下面的例子将一个网格物体配置为 SDF，并用静态盒体作为地面。将 `part.obj` 替换为自己的闭合网格，并根据模型实际尺寸调整物体位置和地面大小。

```xml
<mujoco model="sdf_example">
    <asset>
        <mesh name="part_mesh" file="part.obj"/>
    </asset>

    <worldbody>
        <!-- 盒体顶面位于 z = 0 -->
        <geom name="floor" type="box" size="2 2 0.1" pos="0 0 -0.1"/>

        <body name="part" pos="0 0 1">
            <freejoint/>
            <geom name="part_collision" type="sdf" mesh="part_mesh" mass="1"/>
        </body>
    </worldbody>
</mujoco>
```

`type="sdf"` 决定碰撞表示，`mesh="part_mesh"` 指定形状来源。几何体的尺寸来自网格；如果需要缩放，请设置网格资产的 `scale`，而不是在 SDF 几何体上设置 `size`：

```xml
<mesh name="part_mesh" file="part.obj" scale="0.01 0.01 0.01"/>
```

对于已有的网格几何体，可以将 `<geom type="mesh" mesh="part_mesh"/>` 改为 `<geom type="sdf" mesh="part_mesh"/>`。如果它需要与另一个普通网格碰撞，也要相应调整另一方的碰撞类型。

保存为 `scene.xml` 后，按通常方式加载：

```python
from motrixsim import load_model

model = load_model("scene.xml")
```

## 在 USD 中使用

在需要参与碰撞的 `Mesh` 上应用 `PhysicsCollisionAPI` 和 `PhysicsMeshCollisionAPI`，并将 `physics:approximation` 设置为 `"sdf"`。

下面仅展示已有网格需要添加的碰撞配置。请保留该网格原有的顶点、面索引、材质和变换等数据；已有其他 API 配置时，也应保留。

```usda
def Mesh "PartMesh" (
    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
)
{
    bool physics:collisionEnabled = true
    uniform token physics:approximation = "sdf"

    # 此处保留已有的网格数据。
}
```

这些属性需要设置在实际的 `Mesh` 上。若一个物体由多个网格组成，需要分别配置参与碰撞的网格，并确保它们各自满足网格要求。

SDF 配置只决定碰撞形状。如果物体需要自由运动，仍需按通常方式为所属物体配置 `PhysicsRigidBodyAPI`、质量等刚体属性。

保存 USD 文件后，按通常方式加载：

```python
from motrixsim import load_model

model = load_model("scene.usda")
```

将两个网格都设为 `"sdf"` 后，可以使用 SDF 与 SDF 碰撞。仅设置 `"convexHull"` 或保留默认网格碰撞配置，不会启用 SDF。其他 USD 导入说明见 [USD 支持参考](../../getting_started/usd_reference.md)。

## 运行示例

### 齿轮碰撞

在示例项目目录下运行齿轮碰撞演示：

```bash
uv run examples/physics/sdf_collision.py
```

该示例展示两个齿轮之间的 SDF 碰撞，并可查看接触位置和接触法向。完整代码见 [`examples/physics/sdf_collision.py`](../../../../../examples/physics/sdf_collision.py)。

### 灵巧手抓苹果

抓苹果示例展示 SDF 与 SDF 之间的抓取接触。苹果（包括苹果梗）和灵巧手指腹的碰撞几何体均使用 `type="sdf"`，可以观察手指夹住细梗、提起苹果，以及改为抓住果身时的接触行为。苹果的显示网格与碰撞几何体分开配置，外观由网格渲染，碰撞由 SDF 计算。

在仓库根目录运行：

```bash
uv run examples/control/apple_grasp.py
```

程序打开窗口并自动回放 40 秒动作序列：夹住苹果梗、提起、放回、松开，再抓住果身并提起。窗口支持暂停和继续，终端显示初始化进度，并每秒输出仿真时间、苹果高度以及食指和拇指指腹的接触点数量。回放结束时显示完成或提前停止提示。

场景为食指、拇指的指腹与苹果分别配置了显式接触对：

```{literalinclude} ../../../../../examples/assets/apple_grasp/scene.xml
:language: xml
:start-at: <pair geom1="r_index_finger_pad_collision"
:end-before: <pair geom1="table"
:dedent: 4
```

SDF 描述碰撞形状，接触参数则影响接触响应。

完整代码见 [`examples/control/apple_grasp.py`](../../../../../examples/control/apple_grasp.py)，场景和接触配置见 [`examples/assets/apple_grasp/scene.xml`](../../../../../examples/assets/apple_grasp/scene.xml)。

### USD 配置

USD 配置可以参考 [`examples/assets/usd/sdf_bookshelf_with_book.usda`](../../../../../examples/assets/usd/sdf_bookshelf_with_book.usda)，其中书架和书本的网格均启用了 SDF。
