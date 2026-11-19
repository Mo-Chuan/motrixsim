# Signed Distance Field (SDF)

{bdg-success-line}`CPU: fully supported`

The CPU version of MotrixSim supports collision detection using signed distance fields (SDFs). For objects with grooves, holes, or gear teeth, SDFs can preserve collision shapes that ordinary convex hull approximations cannot capture. You can enable SDF through MJCF or USD, and MotrixSim automatically generates the required data from the mesh when loading the model.

## What Is an SDF?

An SDF describes a shape using the distance from each point in space to the object's surface. The sign distinguishes the inside from the outside: the value is positive outside the object, zero on the surface, and negative inside.

Let $\Omega \subset \mathbb{R}^3$ be the closed region occupied by the object, and let $\partial\Omega$ be its surface. For a point $\mathbf{x}$ in space, the shortest distance to the surface is defined as:

```{math}
d(\mathbf{x}, \partial\Omega)
= \min_{\mathbf{y}\in\partial\Omega}\|\mathbf{x}-\mathbf{y}\|_2
```

Here, $\mathbf{y}$ is a point on the surface, and $\|\mathbf{x}-\mathbf{y}\|_2$ is the straight-line distance between the two points. The signed distance field is then a function $\phi:\mathbb{R}^3\to\mathbb{R}$:

```{math}
\phi(\mathbf{x}) =
\begin{cases}
 d(\mathbf{x}, \partial\Omega), & \mathbf{x}\notin\Omega,\\
 0, & \mathbf{x}\in\partial\Omega,\\
 -d(\mathbf{x}, \partial\Omega), & \mathbf{x}\in\operatorname{int}(\Omega).
\end{cases}
```

Here, $\operatorname{int}(\Omega)$ denotes the interior of the object. Thus, $|\phi(\mathbf{x})|$ gives the distance to the surface, while the sign indicates which side of the surface the point lies on. All points satisfying $\phi(\mathbf{x})=0$ form the object's surface, also called the SDF's **zero level set**.

For example, the SDF of a sphere with center $\mathbf{c}$ and radius $r$ is:

```{math}
\phi(\mathbf{x}) = \|\mathbf{x}-\mathbf{c}\|_2-r
```

For a sphere with a radius of 1 meter, a point 1.2 meters from the center has an SDF value of 0.2 meters: it is outside the sphere, 0.2 meters from the surface. A point 0.7 meters from the center has an SDF value of -0.3 meters: it is inside the sphere, 0.3 meters from the surface.

In practice, you still provide a mesh for the object's shape and appearance. With SDF enabled, MotrixSim uses the distance field generated from that mesh to compute collisions, while continuing to display the model as a mesh. You do not need to write a distance function or prepare a separate SDF file.

## When to Use SDF

Ordinary mesh colliders typically use a convex hull approximation. You can think of a convex hull as a tight shell wrapped around an object: it fills grooves, closes holes, and may enclose the gaps between gear teeth. Although these details remain visible, the collision shape may no longer match them.

An SDF can represent these non-convex structures. It is useful when collision behavior depends on local shape, such as meshing gears, objects entering grooves, or contact with parts that have holes. With SDF, you do not need to manually split an object into multiple convex shapes to represent these concavities.

The following demo shows a ball rolling down a spiral track. The track uses SDF to represent its groove, allowing the ball to follow the channel.

```{video} /_static/videos/sdf_trail_ball.mp4
:poster: /_static/images/poster/sdf_trail_ball.jpg
:caption: A ball rolling down an SDF spiral track
:playsinline:
:muted:
:width: 100%
```

For regular shapes such as spheres and boxes, use the corresponding geometric primitives directly. SDF adds data preparation work and memory overhead when loading a model, and contact behavior around small features should be verified with the actual model.

## Supported Collisions

The following collision combinations apply to the CPU version of MotrixSim:

| Geometry colliding with an SDF                          | Current support         |
| :------------------------------------------------------ | :---------------------- |
| SDF                                                     | Supported               |
| Sphere (`sphere`)                                       | Supported               |
| Box (`box`)                                             | Supported               |
| Capsule (`capsule`)                                     | Supported               |
| Cylinder (`cylinder`)                                   | Supported               |
| Ellipsoid (`ellipsoid`)                                 | Supported               |
| Ordinary mesh collider (`mesh`, including convex hulls) | Not currently supported |
| Plane (`plane`)                                         | Not currently supported |
| Height field (`hfield`)                                 | Not currently supported |

Here, an "ordinary mesh collider" means a mesh collision shape without SDF enabled. Although SDF also uses a mesh asset, two geometries with SDF enabled can collide with each other.

```{important}
Unsupported combinations do not produce a collision response. Being able to load and display a model does not mean that the combination supports collisions.

If two complex meshes need to collide, configure both as SDF. To support an SDF object on a floor, you can use a static box with nonzero thickness. The box covers only its specified extent and does not behave as an infinite plane.
```

## Mesh and Scene Requirements

-   **The mesh should enclose a solid volume.** Its surface should clearly separate the inside from the outside. Avoid broken or missing faces and self-intersections. A single surface layer with no thickness is not suitable as an SDF collider. A solid with a through-hole can still have a closed mesh, provided that the hole walls and outer surface are fully connected.
-   **Include the geometric details needed for collisions.** Grooves, hole walls, and gear teeth must be present in the mesh. Details represented only by textures or materials do not participate in collisions.
-   **Check dimensions, thicknesses, and clearances.** Mesh units and scene scaling should be consistent. For very thin walls, narrow gaps, or features that are small relative to the overall object, verify contact behavior first. Rendered appearance alone does not establish collision accuracy.
-   **Check both colliders' settings.** Collision must be enabled for both objects, and collision filtering must not exclude the pair. The other object's geometry type must be within the supported range above.

## Using SDF in MJCF

Declare a mesh in `<asset>`, then set `type="sdf"` on the `<geom>` and reference the asset through `mesh`.

The following example configures a mesh object as SDF and uses a static box as the floor. Replace `part.obj` with your own closed mesh, and adjust the object's position and floor dimensions to suit the model's actual size.

```xml
<mujoco model="sdf_example">
    <asset>
        <mesh name="part_mesh" file="part.obj"/>
    </asset>

    <worldbody>
        <!-- The top of the box is at z = 0 -->
        <geom name="floor" type="box" size="2 2 0.1" pos="0 0 -0.1"/>

        <body name="part" pos="0 0 1">
            <freejoint/>
            <geom name="part_collision" type="sdf" mesh="part_mesh" mass="1"/>
        </body>
    </worldbody>
</mujoco>
```

`type="sdf"` selects the collision representation, and `mesh="part_mesh"` specifies the source of the shape. The geometry's dimensions come from the mesh. To scale it, set `scale` on the mesh asset rather than setting `size` on the SDF geometry:

```xml
<mesh name="part_mesh" file="part.obj" scale="0.01 0.01 0.01"/>
```

For an existing mesh geometry, you can change `<geom type="mesh" mesh="part_mesh"/>` to `<geom type="sdf" mesh="part_mesh"/>`. If it needs to collide with another ordinary mesh, adjust the other object's collision type accordingly.

Save the file as `scene.xml` and load it as usual:

```python
from motrixsim import load_model

model = load_model("scene.xml")
```

## Using SDF in USD

Apply `PhysicsCollisionAPI` and `PhysicsMeshCollisionAPI` to the `Mesh` that should participate in collisions, and set `physics:approximation` to `"sdf"`.

The following snippet shows only the collision settings to add to an existing mesh. Keep its existing vertices, face indices, materials, transforms, and other data. Preserve any other API settings already applied to it as well.

```usda
def Mesh "PartMesh" (
    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
)
{
    bool physics:collisionEnabled = true
    uniform token physics:approximation = "sdf"

    # Keep the existing mesh data here.
}
```

These properties must be set on the actual `Mesh`. If an object consists of multiple meshes, configure each mesh that participates in collisions and ensure that each meets the mesh requirements.

SDF settings determine only the collision shape. If the object needs to move freely, configure `PhysicsRigidBodyAPI`, mass, and other rigid-body properties on the owning object as usual.

Save the USD file and load it as usual:

```python
from motrixsim import load_model

model = load_model("scene.usda")
```

Setting both meshes to `"sdf"` enables SDF-to-SDF collisions. Setting only `"convexHull"` or keeping the default mesh collision settings does not enable SDF. For other USD import details, see the [USD Support Reference](../../getting_started/usd_reference.md).

## Running the Examples

### Gear Collisions

Run the gear collision demo from the examples project directory:

```bash
uv run examples/physics/sdf_collision.py
```

This example demonstrates SDF collisions between two gears and lets you view contact positions and contact normals. See [`examples/physics/sdf_collision.py`](../../../../../examples/physics/sdf_collision.py) for the complete code.

### Dexterous Hand Grasping an Apple

The apple-grasp example demonstrates SDF-to-SDF contact during manipulation. Both the apple, including its stem, and the finger-pad colliders use `type="sdf"`. You can observe contact as the fingers pinch the narrow stem, lift the apple, and then switch to grasping the fruit body. The apple has separate visual and collision geometries: the mesh provides its appearance, while SDF handles collisions.

Run from the repository root:

```bash
uv run examples/control/apple_grasp.py
```

The script opens a window and automatically replays a 40-second sequence: pinch the stem, lift, place back, release, grasp the fruit body, and lift again. The window supports pause and resume. The terminal shows initialization progress and prints the simulation time, apple height, and contact counts for the index-finger and thumb pads once per second. A final message indicates whether playback completed or stopped early.

The scene defines explicit contact pairs between the apple and the index-finger and thumb pads:

```{literalinclude} ../../../../../examples/assets/apple_grasp/scene.xml
:language: xml
:start-at: <pair geom1="r_index_finger_pad_collision"
:end-before: <pair geom1="table"
:dedent: 4
```

Here, `condim="4"` enables torsional friction around the contact normal in addition to sliding friction. Together with this example's `friction` settings, it reduces wobble while holding the stem. SDF describes the collision shape, while contact parameters affect the contact response. These values are configured for this example; verify them against the dimensions, mass, and controller of other models. See the {ref}`MJCF contact-pair reference <contact-pair-condim>` for parameter definitions.

See [`examples/control/apple_grasp.py`](../../../../../examples/control/apple_grasp.py) for the complete code and [`examples/assets/apple_grasp/scene.xml`](../../../../../examples/assets/apple_grasp/scene.xml) for the scene and contact settings.

### USD Configuration

For a USD configuration example, see [`examples/assets/usd/sdf_bookshelf_with_book.usda`](../../../../../examples/assets/usd/sdf_bookshelf_with_book.usda), where SDF is enabled for both the bookshelf and book meshes.
