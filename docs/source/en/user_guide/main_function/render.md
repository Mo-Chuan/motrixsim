# 🎨 Renderer

## Launching and Loading

[`RenderApp`] is responsible for basic scene rendering. Typically, you create a [`RenderApp`] instance in your code, load a model using [`load_model`], and then call [`render.launch(model)`] to load the model into the renderer.

## Viewport and panels

Inside a **Viewport**, drag with the left mouse button to orbit, drag with the right mouse button to pan, and scroll to zoom. Use the camera selector at the top left of the viewport to switch cameras. Open the gear menu beside it to change the display mode, visibility, or gizmos.

Declare the panels you want to display in the `blueprint` you pass to `RenderApp`.

| Panel declaration | Use |
| --- | --- |
| `ui.Viewport()` | Display the system camera or a model camera |
| `ui.Options()` | Open **Rendering** or **Lighting / Environment** settings |
| `ui.Log()` | Read messages from the renderer |
| `ui.View(...)` | Add custom controls such as buttons and text inputs |
| `ui.Image(...)` | Display an image or a camera capture |

Use the [interactive viewer](managed_viewer.md) for the **Physics** settings and object inspection panels. In RenderApp, the Python loop controls playback, single-stepping, and reset. Add custom controls to the blueprint if users need to trigger those actions from the window.

### Try a simulation control panel

From the repository root, run the [RenderApp UI example](../../../../examples/viewer/render_app_ui.py).

```bash
uv run python examples/viewer/render_app_ui.py
```

The example opens the **Simulation** viewport on the left and the **Controls**, **Options**, and **Log** tabs on the right. The blueprint uses `ui.Horizontal(..., shares=[3, 1])` to give the viewport three times the initial width of the tab group. Drag the divider to resize the two areas.

- Clear **Run simulation** to pause physics. The viewport and panels stay responsive because the example keeps calling `renderer.sync(data)`.
- While paused, click **Step once (when paused)** to advance one physics step.
- Click **Reset state** to restore the initial state. The **Run simulation** checkbox keeps its current setting.

## Synchronization

In motrixsim, the renderer runs in an independent thread, so we need to call [`render.sync(data)`] in the Python main thread to achieve bidirectional synchronization between physics simulation data and the renderer. The render.sync method actually executes the following logic:

-   Sends pose data of all physics objects, user drawing instructions, etc. to the renderer
-   Retrieves window, UI, and other input events as well as camera rendering data from the renderer

The underlying thread model is as follows:

![thread model](/_static/images/render_thread_model.jpg)

-   **Main thread**, i.e., the thread where Python code executes. We typically execute `render.sync(data)` once after several physics steps to synchronize data. sync does two things:
    1. Accepts input events and camera data from the render thread via the Event pipeline
    2. Sends frame data to the Frame pipeline (blocks and waits if the Frame pipeline is full)
-   **Render thread**, responsible for window creation and rendering loop execution. In each loop, it does two things:
    1. Accepts frame data from the main thread via the Frame pipeline and submits it to GPU for rendering (skips if the pipeline is empty)
    2. Processes window, UI, and camera data requests, and sends them to the Event pipeline

## Render Settings

Currently, there is a configuration item `RenderSettings` for global rendering configuration:

| Parameter Name       |                   Explanation                    | Default Value |
| :------------------- | :----------------------------------------------: | :-----------: |
| simplify_render_mesh |      Whether to automatically simplify mesh      |     False     |
| enable_shadow        |         Whether to enable light shadows          |     True      |
| enable_ssao          | Whether to enable Screen Space Ambient Occlusion |     True      |
| enable_oit           | Whether to enable Order-Independent Transparency |     True      |

Specifically, the corresponding rendering effects will only take effect when both global configuration is enabled AND certain rendering items are also enabled in the scene description file (mjcf/msd).

_Example code see [examples/viewer/render_settings.py](../../../../examples/viewer/render_settings.py)_

(render-custom-ui)=
## Custom UI Components

`show()` and `hide()` accept a View object, `title` for an exact displayed title, or `include` / `exclude`
for regular expressions matching the full title. Exclusion takes precedence. These selection
forms cannot be mixed. All panels must be declared in the construction blueprint; there is no
runtime panel creation, and ids outside the blueprint are ignored by the render service with a
logged error. Title searches restore hidden panels that are still open. To reopen a closed panel,
pass its original View object to `show(view)`. To also activate its tab, use
`show(view, select=True)`. The `select` argument requires a View object.

```python
renderer.hide(view)
renderer.show(view, select=True)
renderer.hide(title="Controls")
renderer.show(include=["Camera.*"], exclude=["Camera Debug"])
renderer.sync(data)
```

Keep the objects returned by controls to read their values and call `is_clicked()` or
`is_changed()`, as appropriate. Alternatively, use callbacks to respond to user input.
For structural changes, declare a `ui.Frame` with a builder and request `RenderApp.rebuild(frame)`.
The rebuild runs during a subsequent synchronization. The builder must declare exactly one root
node. Use `ui.Column()` or `ui.Row()` to group multiple controls.

In the [RenderApp Frame example](../../../../examples/viewer/render_app_frame.py), change **Mode**
from **Position** to **Velocity**. The panel replaces **Target position (rad)** with
**Target velocity (rad/s)** and adds **Enable velocity limit**.

For images, call `RenderApp.create_image(...)` and pass the result to a `ui.Image` control's
`set_image()`. The following gallery demonstrates the full API.

`ui.Viewport()` and `ui.Options()` return `ui.View` objects. Combine them with a custom panel
using `ui.Tabs([viewport, controls, ui.Options()])`.

Composition transfers each View's declaration and contents into the resulting blueprint.
Nested composition also consumes its input blueprints.

Pass the final blueprint to `RenderApp(blueprint=...)`, which consumes it. Each View and blueprint
can be transferred only once. You can also pass a single new View directly to the constructor.
Keep the View objects so the owning app can hide and show the corresponding panels.
Hiding a panel preserves its controls and docking position. Create fresh View objects for a new app.

Declare a custom View's contents in a `with ui.View(...)` block. Give the View exactly one root
node. Use `ui.Column()` or `ui.Row()` to group multiple controls under that node.

```python
from motrixsim import ui

with ui.View(title="Details", visible=False) as details:
    ui.Text("Simulation details")
```

Include `details` in the blueprint. With `visible=False`, the panel starts hidden.
Call `render.show(details)` to reveal it at its existing dock location.

If composition or construction fails, build new Views and a new layout before trying again.
Consumed inputs are not restored.

## UI Gallery

Pass a Viewer blueprint to `RenderApp(blueprint=blueprint)` to combine built-in Views with Python-defined
`ui.View` panels. Custom UI provides blueprint containers, text, buttons, toggles, text inputs,
dropdowns, radio groups, toggle groups, dynamic Frames, and images. Each choice control
returns a state with `options`, `value`, `set_options()`, and `set_value()`. Setter updates are submitted
by the next `render.sync(data)`. A later synchronization reports the change through
`is_changed()` and invokes `on_change`. Setters do not invoke callbacks immediately. Use Frame rebuilds
only for structural changes such as conditional branches or adding and removing controls.
After a Frame rebuild replaces a control, using the object returned by the old control raises
`ReferenceError`; retain and use the new control object created during the rebuild instead.

The following example assumes `model` and its `data` have already been created.

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

Use `ui.ScrollArea(max_height=...)` when only one local region should scroll. It is a composable
vertical container, so a Section opts into scrolling by nesting the ScrollArea inside it. The
height must be finite and positive. The region fills its parent's available width so the vertical
scrollbar stays on the right edge, while short content can remain naturally compact vertically.

UI callbacks run during `render.sync(data)` on the calling Python thread. They can update
`SceneData` for the next physics step. Callbacks for controls replaced by a Frame rebuild are no
longer invoked. Without calls to `render.sync(data)`, pending UI callbacks are not executed.

`ui.Image(...)` creates an image display. Inside a `ui.View` or Frame it adds an embedded
image; outside these contexts it creates a standalone panel with the default title `"Image"`.
The display can start empty. Call `RenderApp.create_image(pixels, ...)` and pass the result to
`set_image(image)`, or call `set_image(None)` to clear it. An embedded Image uses its original pixel size by default; one explicit dimension preserves the
source aspect ratio, while two dimensions use contain or stretch according to
`keep_aspect_ratio`. A standalone Image fills the View by default and is centered when fixed
dimensions are supplied. Its standalone-only `fit_to_size` option defaults to `True`; set it to
`False` to center the Image at its original pixel size. Content outside the panel is clipped.
The standalone image toolbar provides Image Scaling controls. Pass `toolbar=False` to a
standalone `ui.Image` or `ui.Viewport` to hide that instance's toolbar initially. It can still be
restored from the owning tab group's `...` menu; this option does not add a Python runtime setter.

Run the following example from the repository root. It displays the same black RGB image in
an embedded area and in the standalone **RGB Preview** tab.

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

preview = ui.Image(title="RGB Preview", fit_to_size=True, toolbar=False)
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

Assign new pixels to `image.pixels` and call `renderer.sync(data)`. All displays using that image
show the updated pixels without rebuilding their Views or calling `set_image()` again. This also
works with CPU images returned by camera capture. On first submission, the original CPU Image
becomes associated with this RenderApp. After submission, the display retains the image resource
without keeping the Python Image or its RenderApp alive.

```{literalinclude} ../../../../examples/viewer/ui_gallery.py
:language: python
:dedent:
:start-after: "# tag::ui_gallery[]"
:end-before:  "# end::ui_gallery[]"
```

_Full code see [examples/viewer/ui_gallery.py](../../../../examples/viewer/ui_gallery.py)_

_Image sharing and updates see
[examples/viewer/image_widget.py](../../../../examples/viewer/image_widget.py)_

## Gizmos Drawing

Gizmos are graphical elements used for debugging assistance. The renderer provides a simple API to draw gizmos.

| Method Name        |               Meaning               |
| :----------------- | :---------------------------------: |
| draw_sphere        |        Draw wireframe sphere        |
| draw_cuboid        |        Draw wireframe cuboid        |
| draw_cylinder      |       Draw wireframe cylinder       |
| draw_capsule       |       Draw wireframe capsule        |
| draw_ray           |              Draw ray               |
| draw_line          |          Draw line segment          |
| draw_arrow         |             Draw arrow              |
| draw_rect          |          Draw 2D rectangle          |
| draw_grid          |            Draw 3D grid             |
| draw_axes          |        Draw coordinate axes         |
| set_draw_collider  |    Turn on/off collider drawing     |
| set_draw_joint     |      Turn on/off joint drawing      |
| set_draw_site      | Turn on/off reference point drawing |
| set_collider_color |     Set collider drawing color      |
| set_joint_color    |       Set joint drawing color       |
| set_line_width     |       Set drawing line width        |
| set_joint_size     |       Set joint drawing size        |

Gizmos use immediate mode, so even if no update is needed, users need to add gizmos during each render synchronization.

```{literalinclude} ../../../../examples/viewer/gizmos.py
:language: python
:dedent:
:start-after: "# tag::draw_gizmos[]"
:end-before:  "# end::draw_gizmos[]"
```

_Full code see [examples/viewer/gizmos.py](../../../../examples/viewer/gizmos.py)_

## IO Input Events

You can get the [`Input`] object through [`render.input`]. The Input object provides a series of methods for querying the renderer's mouse, keyboard, and UI events.

You can understand more detailed usage through the following examples:

-   [example/mouse_click.py](../../../../examples/control/mouse_click.py)
-   [example/keyboard_car.py](../../../../examples/control/keyboard_car.py)

### Valid Keyboard Input List

The following is the list of supported keyboard key inputs:

| Key Type      |         Example Keys          |
| :------------ | :---------------------------: |
| Letter Keys   |      `A`, `B`, ..., `Z`       |
| Function Keys |    `F1`, `F2`, ..., `F12`     |
| Special Keys  |    `Enter`, `Esc`, `Space`    |
| Arrow Keys    | `Up`, `Down`, `Left`, `Right` |

> Note: The above are all case-insensitive, and keys not mentioned in the table are not currently supported.

## System Camera Control

The renderer provides a free system camera control system. Users can control the viewing angle and focus point of the system camera through mouse operations (the focus point is always located at the screen center).

-   Left mouse button press and drag: Rotate the system camera around the focus point
-   Right mouse button press and drag: Move the focus point (shows red circle as focus point)
-   Mouse scroll wheel: Zoom (cannot zoom in further when at focus point position)

For more camera control methods, please refer to [Camera](../render/camera.md).

## Multi-Instance Rendering of a Single Model

[`render.launch(model)`] also supports two optional parameters: batch:int and render_offset:List[:3]. These are used to set the number of instances and the offset positions when rendering multiple instances of a single model.

```{literalinclude} ../../../../examples/parallel/parallelsim.py
:language: python
:dedent:
:start-after: "# tag::init batch data"
:end-before: "# end::init batch data"
```

_See the complete code at [examples/parallel/parallelsim.py](../../../../examples/parallel/parallelsim.py)_

### Instance Visibility

When we perform single model multi-instance rendering, we can configure the rendering visibility of each instance as needed.

```
...
# Create multiple instances
render_offsets = []
batch = 10
for i in range(batch):
    render_offsets.append([i * 2.0, 0, 0])
render.launch(model, batch, render_offsets)
data = SceneData(model, batch=(batch,))

target_scene_indices = [1, 3, 5, 7, 9] # Specify instance indices
render.set_scene_vis(target_scene_indices, False) # Hide target instances
render.set_scene_vis(target_scene_indices, True) # Show target instances
```

It can also be enabled/disabled globally:

```
render.set_all_scene_vis(False) # Hide all instances
render.set_all_scene_vis(True) # Show all instances
```

The above operations only affect rendering visibility and do not affect the object's physics simulation.

_Full code see [examples/viewer/partial_rendering.py](../../../../examples/viewer/partial_rendering.py)_

[`RenderApp`]: motrixsim.render.RenderApp
[`load_model`]: scene_model.md
[`render.launch(model)`]: motrixsim.render.RenderApp.launch
[`render.sync(data)`]: motrixsim.render.RenderApp.sync
[`render.input`]: motrixsim.render.RenderApp.input
[`Input`]: motrixsim.render.Input
