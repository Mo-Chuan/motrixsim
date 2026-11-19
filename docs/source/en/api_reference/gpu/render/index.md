# 🎨 GPU Render Module

`motrixsim.gpu.render` provides visualization components (`RenderApp`, `RenderSettings`, etc.) paired with GPU simulation. The interface mirrors `motrixsim.render` in name and semantics; see [CPU vs. GPU API Differences](../../../user_guide/gpu_version/api_differences.md) for the deltas.

CPU and GPU share the same renderer type: `motrixsim.gpu.render.RenderApp` **is** `motrixsim.render.RenderApp`; there is no separate GPU renderer. Importing `motrixsim.gpu.render` installs a bridge that reduces the GPU package's `SceneModel` / `SceneData` to the forms the renderer accepts, so GPU code still calls `renderer.launch(model)` and `renderer.sync(data)` directly without caring which backend produced the data.

```{toctree}
render
```
