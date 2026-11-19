# 🎨 GPU 渲染模块

`motrixsim.gpu.render` 提供与 GPU 仿真配套的可视化组件（`RenderApp`、`RenderSettings` 等）。接口与 `motrixsim.render` 同名同义，迁移见 [CPU 与 GPU API 差异](../../../user_guide/gpu_version/api_differences.md)。

CPU 与 GPU 共用同一个渲染器类型：`motrixsim.gpu.render.RenderApp` 就是 `motrixsim.render.RenderApp`，不存在一份单独的 GPU 渲染器。导入 `motrixsim.gpu.render` 之后，GPU 包的 `SceneModel` / `SceneData` 会被自动规约成渲染器接受的形式，因此 GPU 侧照样直接写 `renderer.launch(model)` 与 `renderer.sync(data)`，不需要关心类型来自哪一侧。

```{toctree}
render
```
