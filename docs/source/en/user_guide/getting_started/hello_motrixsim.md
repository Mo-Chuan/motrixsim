# 🚀 Quick Start: Run Your First Simulation

Use a falling-ball experiment to learn how to create a model, manage state, step physics, and display the results. Then change gravity and observe how the simulation changes.

## Preparation

Complete [installation](installation.md), then work from your own project root or the cloned repository root. The scene is defined directly in the program and contains a ground plane and a ball.

## 1. Create the Program

Create `first_simulation.py` in your project root and add the following code:

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

The scene uses MJCF XML. Start with these four settings:

| Setting | Meaning |
| --- | --- |
| `timestep="0.02"` | Each physics step advances simulation time by 0.02 seconds |
| `gravity="0 0 -9.81"` | Gravitational acceleration along the negative world Z axis, in m/s² |
| `body pos="0 0 1"` | Places the initial ball center 1 meter above the ground |
| `freejoint` | Allows the ball to translate and rotate freely |

For the sphere, `size="0.1"` sets a radius of 0.1 meters and `mass="1"` sets a mass of 1 kilogram.

## 2. Run and Observe

From the directory containing the program, run:

```bash
uv run python first_simulation.py
```

On macOS Apple Silicon, use:

```bash
uv run mxpython first_simulation.py
```

A window opens with a blue ball falling toward the ground. After contact, it gradually settles near the ground. Close the window to exit. Run the command again to observe the fall from its initial position.

```{figure} /_static/images/quickstart_ball_gravity.jpg
:alt: Actual application window showing the blue ball resting on the ground after falling
:width: 90%
:align: center

With gravity set to −9.81 m/s², the ball falls and contacts the ground.
```

With GPU components installed, run the same program on the GPU backend:

```bash
uv run python first_simulation.py --gpu
```

For the repository installation path, use `uv run --extra gpu python first_simulation.py --gpu`.

## 3. Understand the Simulation Loop

The program follows the same sequence on CPU and GPU:

| Step | Code | Purpose |
| --- | --- | --- |
| Create a model | `mx.load_mjcf_str(SCENE)` | Loads the scene description, including geometry, mass, connections, and physics parameters |
| Create state | `mx.SceneData(model)` | Creates data for changing quantities such as positions and velocities |
| Step physics | `mx.step(model, data)` | Updates state using the model's timestep |
| Update the view | `render.sync(data)` | Sends the current state to the renderer |

The `time.sleep(0.02)` call briefly waits after each step so that the fall is easy to observe. When the window closes, the program catches `RenderClosedError` and exits the rendering context.

## 4. Change Gravity and Compare

Close the window and set gravity to zero in the scene:

```xml
<option timestep="0.02" gravity="0 0 0"/>
```

Save and run the program again. The ball should remain at its initial position. Restore gravity to `0 0 -9.81` and run again to make the ball fall.

```{figure} /_static/images/quickstart_ball_zero_gravity.jpg
:alt: Actual application window showing the blue ball suspended above the ground in zero gravity
:width: 90%
:align: center

With zero gravity, the ball remains at its initial height.
```

This experiment shows how scene parameters determine physical behavior, the simulation loop updates state, and the renderer displays the results.

## Next Steps

- [Models](../main_function/scene_model.md): Load external models and access their components.
- [Data](../main_function/scene_data.md): Read and set simulation state.
- [Example programs](../overview/examples.md): Run complete examples with robots, joints, sensors, and more.
