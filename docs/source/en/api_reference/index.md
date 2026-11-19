# API Reference

MotrixSim ships both CPU and GPU versions optimized for different scenarios. This section is organized by version.

## 📚 Version Overview

| Version | Package         | Use Cases                                            |
| ------- | --------------- | ---------------------------------------------------- |
| **CPU** | `motrixsim`     | Robot control, RL, visualization & debugging, IK/OSC |
| **GPU** | `motrixsim.gpu` | Massively parallel RL training                       |

```{toctree}
:caption: CPU Version
:titlesonly:

api_quick_reference
core/index
msd/index
rendering/index
ik/index
osc/index
```

```{toctree}
:caption: GPU Version
:titlesonly:

gpu/api_quick_reference
gpu/index
gpu/render/index
```
