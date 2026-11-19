# 🎮 Interactive Viewer

{bdg-success-line}`CPU: fully supported` {bdg-danger-line}`GPU: not supported`

## Overview

The Interactive Viewer provides a simplified interface for quickly visualizing and interacting with physics simulations. The [`viewer.launch()`] function operates in a blocking mode, meaning it will not return control to your Python code until the viewer window is closed. This blocking behavior enables precise timing of the physics loop and automatic handling of the simulation.

Unlike the lower-level [`RenderApp`] API which requires manual synchronization and loop management, the interactive viewer automatically handles the simulation loop, rendering synchronization, and common UI controls.

```{figure} /_static/images/web_viewer/current/master-overview.jpg
:alt: The central viewport, Files panel at the bottom, and inspection panels on the right
:width: 100%

The viewer has a viewport, a file browser, and object inspection panels. The desktop viewer can also browse local directories.
```

## When to Use

The interactive viewer is designed for scenarios where you want to:

-   **Quickly visualize models**: Test and inspect physics simulations with minimal code
-   **Interactive debugging**: Examine simulation behavior using built-in playback controls
-   **Educational demonstrations**: Focus on physics concepts without rendering complexity
-   **Prototype development**: Rapidly iterate on model designs and behaviors
-   **Simple applications**: When custom rendering logic or non-blocking execution is not required

For production applications requiring fine-grained control over the simulation loop, consider using [`RenderApp`] instead.

### Key Features

-   **Blocking execution**: Runs until the window is closed, supporting precise physics loop timing
-   **Automatic simulation loop**: Physics steps and rendering are handled automatically
-   **Built-in controls**: Play/pause, reset, step-through, and speed control out of the box
-   **Multiple invocation modes**: Supports empty session, model-only, and model with data
-   **File loading**: Drag XML model files directly into the viewer window to load them instantly
-   **Panel layouts**: Use the default layout or arrange built-in panels with a `blueprint`

## Command-Line Interface

Installing `motrixsim` provides the `mxview` command, an out-of-the-box visualization entry point that requires no Python code (with the virtual environment activated you can run `mxview` directly; in a uv-managed environment use `uv run mxview`):

```bash
# Launch with a specific model file
uv run mxview path/to/your/model.xml

# Preview a standalone mesh (OBJ/STL)
uv run mxview path/to/convex_parts/part.stl

# Launch empty viewer (drag and drop model files to load)
uv run mxview

# Start playing immediately after launch
uv run mxview path/to/your/model.xml --play
```

`mxview` is built on the interactive viewer ([`viewer.launch()`]); the physics and render loops are handled internally.

### CLI Arguments

| Argument | Required | Description                                                                                                                                 |
| -------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| `model`  | No       | Path to a model file or standalone OBJ/STL mesh (positional). If not provided, launches an empty viewer where you can drag and drop models. |
| `--play` | No       | Start in the playing (simulation running) state instead of the default paused (editor) state.                                               |

### Examples

```bash
# Launch viewer with a specific model
uv run mxview examples/assets/boston_dynamics_spot/scene_arm.xml

# Preview offline convex decomposition output
uv run mxview path/to/convex_parts/link1_convex.obj

# Launch empty viewer for drag-and-drop interaction
uv run mxview
```

The CLI provides a quick way to visualize models without writing Python code. When a file is provided, it automatically:

-   Loads the specified model, or wraps a standalone OBJ/STL mesh in a minimal display-only scene
-   Creates scene data with initial state
-   Launches the interactive viewer

When you launch `mxview` without arguments, drag a model file into the window to load it. You can also browse to the model in **Files** and double-click the file name.

```{note}
For more control over initial conditions or custom callbacks, use the Python API directly with [`viewer.launch()`].
```

## Usage Modes

The [`viewer.launch()`] function supports three distinct invocation patterns to accommodate different use cases:

### 1. Empty Session (Default Model)

```python
import motrixsim as mx

# Launch with default demonstration model
mx.viewer.launch()
```

Launches an interactive visualization session with a default demonstration model. This is the quickest way to explore the viewer functionality.

**Tip:** You can also drag XML model files directly into the viewer window to load them instantly, making it easy to quickly preview different models without writing code.

### 2. Model Only

```python
import motrixsim as mx

# Load your model
model = mx.load_model("path/to/your/model.xml")

# Launch viewer - will create internal SceneData automatically
mx.viewer.launch(model)
```

Launches a visualization session for the given model. The viewer internally creates its own instance of [`SceneData`] for the simulation.

### 3. Model and Data

```python
import motrixsim as mx
from motrixsim import SceneData

# Load model and create data
model = mx.load_model("path/to/your/model.xml")
data = SceneData(model)

# Configure initial state
data.qpos[:] = initial_positions
data.qvel[:] = initial_velocities

# Launch viewer with initial data
mx.viewer.launch(model, data)
```

Uses the provided [`SceneData`] instance as the initial state for the simulation. The viewer creates an internal copy of the data for its simulation loop. This mode is useful when you need to:

-   Set specific initial conditions (positions, velocities, control inputs)
-   Run stabilization steps before visualization
-   Start the viewer from a particular simulation state

### Pre-Simulation Example

Run some physics steps before launching the viewer to stabilize the initial state:

```{literalinclude} ../../../../examples/viewer/interactive_viewer.py
:language: python
:dedent:
:start-after: "# Use custom model with physics data"
:end-before: "viewer.launch(model, data)"
```

This pre-simulation approach is particularly useful for:

-   Allowing initial contact forces to settle
-   Reaching a stable equilibrium configuration
-   Pre-computing derived quantities before visualization

## Complete Example

Here's a complete example demonstrating the interactive viewer with the Boston Dynamics Spot robot:

```{literalinclude} ../../../../examples/viewer/interactive_viewer.py
:language: python
:dedent:
:start-after: "# Copyright (C)"
```

This example:

1. Loads the Spot robot model with robotic arm (includes actuators)
2. Creates scene data
3. Runs 20 physics steps to stabilize the simulation
4. Launches the interactive viewer

## Built-in controls and panels

Click a tab to switch panels in the same area. Drag a divider to resize the panels.

| Area | Use |
| --- | --- |
| Simulation toolbar on the left | Play, pause, step, stop, reset, or reload the simulation |
| **Viewport** | View the scene, control the camera, and select objects |
| **Scene Hierarchy** | Browse the model hierarchy, select objects, and toggle their visibility |
| **Inspector** | Inspect joints, actuators, and sensors |
| **Favorites** | View objects you starred in Inspector |
| **Properties** | View the selected object's properties, simulation state, and related objects |
| **Options** | Adjust physics, rendering, lighting, and environment settings |
| **Files** | Browse, search, and filter files. Double-click a model file to open it |
| **Log** | View model loading and simulation messages in the desktop viewer |

If the window is too narrow to show every tab, use the arrows in the tab bar or widen the panel.

### Play and pause

-   Click the play button to start the simulation. Click it again to pause.
-   Pause the simulation, then click the step button to advance one frame.
-   Click the square stop button to reset the simulation and return to editing.
-   Use reset to restore the initial state or reload to read the model file and its assets again.

Click the viewport or the panel you want to control to give it keyboard focus. Press `Space` to toggle playback, `F10` to step, or `Ctrl+E` to reset. If focus is in a search field or another editable control, click the viewport first.

### Select and inspect objects

Click an object in **Scene Hierarchy** or **Viewport**. The viewer highlights the object and displays its details in **Properties**. To inspect a joint or actuator, click its name in **Inspector**. Click the star next to an object to collect it in **Favorites**.

```{figure} /_static/images/web_viewer/current/master-properties.jpg
:alt: A selected joint is highlighted in the viewport and inspected in Properties
:width: 100%

Properties shows the selected joint's type, range, and simulation state.
```

### Adjust physics and display options

Open **Options** and select **Physics** to load a keyframe or adjust gravity and actuator settings. Select **Rendering** or **Lighting / Environment** to adjust the display. For settings specific to one viewport, open the gear menu beside its camera selector.

Drag with the left mouse button to orbit and with the right button to pan. Scroll to zoom. If physics drag is enabled, hold `Ctrl` and drag an object with the left mouse button.

## Set a custom panel layout

Build a layout with built-in panels from `motrixsim.ui` and pass it to [`viewer.launch()`] as a `blueprint`. Run this example from the repository root.

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

The viewer opens with the **Simulation** viewport and **Log** on the left and two tab groups on the right. `Horizontal` and `Vertical` split an area, and `Tabs` groups panels. The `shares` values set relative sizes. Here the left column is three times as wide as the right column.

`ui.CameraRef.system()` binds the viewport to the system camera. Create fresh panels and a fresh layout for each call to `viewer.launch()`. Use {ref}`RenderApp custom UI <render-custom-ui>` to add Python controls, callbacks, dynamic frames, or images.

## Comparison with RenderApp

The interactive viewer and [`RenderApp`] serve different use cases:

| Feature             | Interactive Viewer               | RenderApp                      |
| ------------------- | -------------------------------- | ------------------------------ |
| **Execution mode**  | Blocking (runs until closed)     | Non-blocking (manual control)  |
| **Simulation loop** | Automatic                        | Manual (`step()` calls)        |
| **Synchronization** | Automatic                        | Manual (`render.sync()` calls) |
| **Use case**        | Quick visualization, prototyping | Production apps, custom logic  |
| **Control flow**    | Viewer controls flow             | You control flow               |
| **Panel layouts**   | Built-in viewer panels           | Viewports, Options, Log, and custom Views |

### When to Use Each

**Use Interactive Viewer when:**

-   Quickly testing or debugging a model
-   Creating educational demonstrations
-   Building simple interactive tools
-   You want minimal code

**Use RenderApp when:**

-   Integrating with existing application logic
-   Need fine-grained control over rendering timing
-   Building production applications
-   Implementing custom UI or visualization
-   Running headless simulations with optional rendering

## Best Practices

### Model Loading

```python
# Good: Absolute or relative paths
model = mx.load_model("examples/assets/boston_dynamics_spot/scene_arm.xml")

# Good: Use pathlib for cross-platform compatibility
from pathlib import Path
model_path = Path(__file__).parent / "assets" / "model.xml"
model = mx.load_model(str(model_path))
```

### Data Initialization

```python
# Initialize data
data = SceneData(model)

# Set initial configuration
data.qpos[:] = initial_positions
data.qvel[:] = initial_velocities

# Run a few steps to stabilize
for _ in range(10):
    mx.step(model, data)

# Launch viewer
mx.viewer.launch(model, data)
```

## Limitations and Notes

```{warning}
The interactive viewer is blocking - your Python code will not continue until the viewer window is closed. If you need concurrent execution, use [`RenderApp`] instead.
```

**Other considerations:**

-   **Single window**: Only one interactive viewer can be active at a time
-   **No multi-instance rendering**: For batch rendering, use [`RenderApp`]

## Related Documentation

-   [Renderer (RenderApp)](render.md) - Lower-level rendering API with full control
-   [Camera](../render/camera.md) - Camera control and configuration
-   [Widget Components](../render/widgets.md) - Multi-viewport rendering

### API Reference

-   [`viewer.launch()`]
-   [`SceneModel`]
-   [`SceneData`]
-   [`step()`]

[`RenderApp`]: motrixsim.render.RenderApp
[`viewer.launch()`]: motrixsim.viewer.launch
[`SceneModel`]: motrixsim.SceneModel
[`SceneData`]: motrixsim.SceneData
[`step()`]: motrixsim.step
