# API 参考

MotrixSim 提供 CPU 和 GPU 两套版本，针对不同的使用场景优化。本章节按版本组织 API 文档。

## 📚 版本概览

| 版本    | 包名            | 适用场景                                 |
| ------- | --------------- | ---------------------------------------- |
| **CPU** | `motrixsim`     | 机器人控制、强化学习、可视化调试、IK/OSC |
| **GPU** | `motrixsim.gpu` | 强化学习大规模并行训练                   |

```{toctree}
:caption: CPU 版本
:titlesonly:

api_quick_reference
core/index
msd/index
rendering/index
ik/index
osc/index
```

```{toctree}
:caption: GPU 版本
:titlesonly:

gpu/api_quick_reference
gpu/index
gpu/render/index
```
