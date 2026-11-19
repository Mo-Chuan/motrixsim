# example-writing

为 MotrixSim 编写示例程序并登记到用户文档的 example 界面。

## 适用场景

- 新建 example 脚本（`examples/{category}/*.py`）
- 修改现有 example，或为其补充/更新文档条目
- 在 `user_guide/overview/examples.md` 的表格中登记示例

**两条主线必须一起完成**：写 example 脚本后必须同步更新文档界面；改文档条目前必须确认脚本存在且可运行。

## 硬性约束

- API 细节以 `motrixsim` skill 指向的 motrixsim-docs-md 为准，不凭记忆写 API。
- 只改 `docs/source/zh_CN/` 和 `docs/source/en/`；`docs/source/user_guide/` 是构建时自动生成的，**绝不直接编辑**。
- Python 一律用 `uv run`，不用系统/conda Python。
- 修改 `examples/assets/equality/` 时需同步 motrixsim-python 仓库中的副本（见根 AGENTS.md gotchas）。

## 主线一：编写 example 脚本

1. **选目录**：按演示重点分类，不按涉及的技术分类——
   - `getting_started/` 入门最小示例
   - `physics/` API 演示（body/joint/geom/actuator 等）
   - `control/` 控制、策略、键盘交互
   - `viewer/` 渲染与 UI 组件
   - `randomize/` 域随机化
   - `parallel/`、`gpu/`、`bench/` 并行、GPU 变体、基准测试
2. **脚本结构**（参照 `examples/physics/external_force.py`）：
   - Apache License 文件头（照抄现有示例）
   - 模块 docstring：说明演示内容、阶段（phases）划分与各阶段效果
   - `main()` 函数 + 主循环，按时间分阶段演示，切换阶段时 `print` 提示
3. **资产**：MJCF 模型放 `examples/assets/`，代码中用相对仓库根路径 `"examples/assets/xxx.xml"`。
4. **GPU**：共用示例默认支持 `--gpu` 切换后端；仅当 GPU 实现差异大时才在 `examples/gpu/` 下建独立脚本（此时文档表格需加 GPU 链接）。
5. 先写代码，运行验证后再进入主线二。

## 主线二：登记 example 界面

文档界面在 `docs/source/zh_CN/user_guide/overview/examples.md`（及 `en/` 对应文件），是按 section 分组的 `list-table`。新条目：

1. **截图**：实际运行 example 并截图，**截 MotrixRender 应用窗口**（用 `xwininfo -root -tree` 找窗口 id，`import -window <id>` 截取，不要截整个桌面），存入 `docs/source/_static/images/examples/`，统一 16:9 比例。文件名与脚本同名。演示视频放 `_static/videos/`（可配 poster 图）。
2. **表格行模板**（三列：展示 / 文件 / 简介）——表格缩略图统一用固定尺寸 640×360，放在 `docs/source/_static/images/examples/thumbs/`（jpg）。**仅在表格展示的示例不需要保留全尺寸母版**，直接截 1280×720 以上比例正确的窗口图后缩放即可：`convert shot.png -resize 640x360^ -gravity center -extent 640x360 thumbs/master.jpg`；只有正文页面（如 `randomization.md`、`camera_viewport.md`）引用的示例才把全尺寸图留在 `docs/source/_static/images/examples/`。**examples.md 表格只引用 thumbs/ 路径**；`examples/` 下的全尺寸图留给正文页面引用，两者不要混用：

   ```
   * - ![name](/_static/images/examples/thumbs/name.jpg)
     - [`name.py`](../../../../examples/{category}/name.py)
     - 一句话中文简介，说明演示了什么、有哪些交互。
   ```

   相对链接深度固定为 `../../../../examples/...`（指向仓库的 examples 目录）。有 GPU 独立脚本时在文件列追加 ` · [GPU](../../../../examples/gpu/name.py)`。
3. **归类**：按内容选 section（基础入门 / 物理仿真 / 控制 / 并行仿真 / 随机化 / 可视化与渲染 / 基准测试），插入位置参考同 section 现有条目的惯例。
4. **双语同步**：`zh_CN/` 与 `en/` 的 examples.md 都要改，简介翻译而非复制。链接与图片路径两边一致。

## 验证清单（完成前逐项确认）

- [ ] `uv run python examples/{category}/{name}.py` 实际跑通
  - 注意：macOS 上使用 RenderApp 的示例需 `uv run mxpython` 运行（见文档页说明）
- [ ] 截图已生成并放入 `_static/images/examples/`
- [ ] 表格行已加入正确 section，链接/图片路径正确
- [ ] zh_CN 与 en 两份 examples.md 均已更新
- [ ] `cd docs && uv run make html` 构建无新增 warning
- [ ] `git status` 无无关文件混入（遵循根 AGENTS.md 的 commit hygiene）
