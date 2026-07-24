# PAUS UI 深度对齐模式兼容计划

## Goal Description
在现有 PAUS UI 上增加对 `depth_aligned` 观测模式的兼容，同时保留当前 `rgb_pnp` 流程不受影响。UI 仍然保持一套共享壳子，不拆成两套页面；仅在模式、质量指标、waypoint 表列和预览详情上按 `observation_mode` 动态切换。后端继续复用现有半自动闭环与 session 体系，不重写求解器本体。

## Acceptance Criteria

Following TDD philosophy, each criterion includes positive and negative tests for deterministic verification.

- AC-1: UI 顶部可切换 `rgb_pnp / depth_aligned`，并在状态栏明确显示当前模式。
  - Positive Tests (expected to PASS):
    - 页面能看到一个模式切换控件，当前模式显示为 `rgb_pnp` 或 `depth_aligned`。
    - 切换模式后，UI 不会刷新成另一套页面，而是在同一套壳子内更新模式状态。
    - `execute_motion`、session、节点连接状态等共用状态仍然保持可见。
  - Negative Tests (expected to FAIL):
    - 不存在模式入口，或模式切换后页面整页重载成另一套独立 UI。
    - 模式切换导致原有 `rgb_pnp` 运行路径失效。

- AC-2: 质量指标采用统一 schema，但按模式动态渲染。
  - Positive Tests (expected to PASS):
    - `rgb_pnp` 模式下继续展示 `reprojection_error_px`、`board_margin_px`、`board_angle_deg` 等现有指标。
    - `depth_aligned` 模式下展示 `深度有效率`、`平面对齐误差`、`深度残差`、`有效点数`、`棋盘角度`。
    - 前端不依赖单一硬编码字段，而是根据后端返回的 metric 列表渲染卡片和阈值状态。
  - Negative Tests (expected to FAIL):
    - depth 模式下仍然强制显示 RGB 专属指标作为主质量项。
    - 指标 schema 变动后，UI 需要整页重构才能兼容。

- AC-3: Waypoint 表、预览详情与结果面板按模式动态适配，但保留共用结构。
  - Positive Tests (expected to PASS):
    - Waypoint 表在 RGB 模式与 depth 模式下能展示不同的质量列，但其余通用列保持一致。
    - 预览面板继续显示同一个样本图/详情壳子，只是详情字段按当前模式切换。
    - 结果面板仍然展示统一的求解残差，如 `translation_rms_mm`、`rotation_rms_deg`，不被模式切换破坏。
  - Negative Tests (expected to FAIL):
    - waypoint 表被拆成两套完全独立的表格组件。
    - 预览区不能随模式变化而动态展示当前模式的质量字段。

- AC-4: 后端接口向 UI 透传观测模式和质量 schema，保持旧字段兼容。
  - Positive Tests (expected to PASS):
    - `/api/status` 或相关状态事件包含当前 `observation_mode`。
    - `/api/handeye/waypoints`、`/api/handeye/quality` 或同类接口返回统一的 `metrics` / `columns` / `thresholds` 结构。
    - 旧的 RGB 字段仍然可用，旧 session 不会因为新增模式字段而报错。
  - Negative Tests (expected to FAIL):
    - UI 只能通过临时硬编码知道当前模式，后端不返回任何模式信息。
    - 新增模式字段导致旧 session、旧 report.yaml 或旧 API 直接失效。

- AC-5: 文档、测试和启动链路同步更新。
  - Positive Tests (expected to PASS):
    - README / 启动说明明确写出 `rgb_pnp` 与 `depth_aligned` 的区别和切换位置。
    - 至少有一组测试覆盖模式切换、质量 schema 渲染和旧字段回退。
    - `ui.launch.py` 仍可正常启动，且新增参数不会影响默认 `rgb_pnp` 路径。
  - Negative Tests (expected to FAIL):
    - 只有 UI 改动，没有对应测试或说明。
    - 新模式只能在开发者脑子里切换，用户界面看不到清晰反馈。

## Path Boundaries

### Upper Bound (Maximum Acceptable Scope)
UI 继续保持单页壳子，但把模式、质量 schema、waypoint 列、预览详情和结果摘要全部统一成“按 observation_mode 渲染”的数据驱动结构。后端增加统一状态字段与 metric schema，旧 RGB 流程完全不受影响，depth 模式能被同一套半自动入口使用，且在 session、report、waypoint、preview 中都有清晰回显。

### Lower Bound (Minimum Acceptable Scope)
只实现最小可用兼容：UI 能切换 `rgb_pnp / depth_aligned`，并在顶部和质量区显示不同的指标名；Waypoint 表和预览详情至少能按模式切换显示内容；旧 RGB 路径保持完全可用，depth 模式不要求一次性补齐全部深度专属分析面板。

### Allowed Choices
- Can use:
  - 现有 `paus_ui` 的 FastAPI 后端、`ros_bridge.py`、`session_store.py`、`web_server.py`
  - 现有前端 `app.js / index.html / styles.css`
  - 现有 `ui.launch.py` 参数透传机制
  - 统一的 `metrics[]` / `quality_schema` 数据契约
  - 保留旧 RGB 字段作为兼容层
- Cannot use:
  - 把 RGB 和 depth 拆成两套完全独立的 UI 页面
  - 重写半自动标定求解器
  - 让 UI 直接耦合某一种深度算法实现细节
  - 破坏现有 `rgb_pnp` 的默认启动路径

## Feasibility Hints and Suggestions

### Conceptual Approach
1. 在 `ui.launch.py` 和 `UiRosBridge` 中新增 `observation_mode` 参数，默认仍是 `rgb_pnp`。
2. 在后端状态 payload 中增加 `quality_schema`，统一输出 `metrics[]`、`waypoint_columns[]`、`thresholds[]`。
3. 前端把当前质量条、Waypoint 表和预览详情改成按 schema 渲染，不再硬编码字段名。
4. RGB 模式复用当前 `reprojection_error_px / board_margin_px` 路径；depth 模式展示 `深度有效率 / 平面对齐误差 / 深度残差 / 有效点数 / 棋盘角度`。
5. 结果摘要仍然显示求解残差（`translation_rms_mm / rotation_rms_deg`），因为这属于解算结果而不是观测模式专属质量。

### Relevant References
- `src/paus_bringup/launch/ui.launch.py` - UI 启动参数透传入口
- `src/paus_ui/paus_ui/web_server.py` - FastAPI 路由与前端静态资源入口
- `src/paus_ui/paus_ui/ros_bridge.py` - UI 状态聚合与 ROS2 事件桥
- `src/paus_ui/paus_ui/session_store.py` - session / report / waypoint 读写与质量字段组织
- `src/paus_ui/paus_ui/static/app.js` - 模式切换、指标渲染、Waypoint 表、预览区逻辑
- `src/paus_ui/paus_ui/static/index.html` - 页面结构
- `src/paus_ui/paus_ui/static/styles.css` - 样式与布局
- `src/paus_marker_ros2/paus_marker_ros2/eye_to_hand_calibration_node.py` - 半自动标定节点的 status / sample / report 语义
- `docs/eye_to_hand_semi_auto_depth_plan.md` - 深度标定闭环的背景和约束

## Dependencies and Sequence

### Milestones
1. 接口契约对齐
   - 先确认 `observation_mode`、`quality_schema`、`waypoint_columns` 的字段形状。
   - 统一 RGB 与 depth 的状态事件格式，保留旧字段兼容。
2. UI 动态化
   - 顶部增加模式切换。
   - 实时质量区、Waypoint 表、预览详情按 schema 动态渲染。
3. 启动与文档
   - `ui.launch.py`、README、使用说明同步更新。
   - 默认仍走 `rgb_pnp`，depth 模式需要显式切换。
4. 测试与回归
   - 做模式切换、旧 session 回放、缺失字段回退的测试。
   - 在实验室主机上跑一次 UI 启动和手工检查。

## Task Breakdown

Each task must include exactly one routing tag:
- `coding`: implemented by Claude
- `analyze`: executed via Codex (`/humanize:ask-codex`)

| Task ID | Description | Target AC | Tag (`coding`/`analyze`) | Depends On |
|---------|-------------|-----------|----------------------------|------------|
| task1 | 定义并落地 `observation_mode + quality_schema` 的统一状态契约 | AC-1, AC-2, AC-4 | coding | - |
| task2 | 改造 `ui.launch.py` 和 `ros_bridge.py`，把模式与 schema 透传到 UI | AC-1, AC-4 | coding | task1 |
| task3 | 改造 `app.js`，让质量条、waypoint 表、预览详情按 schema 动态渲染 | AC-2, AC-3 | coding | task1, task2 |
| task4 | 改造 `session_store.py`，让 session/report 回放能够按模式呈现质量字段 | AC-3, AC-4 | coding | task1 |
| task5 | 增补 README 和启动说明，写清 RGB / depth 模式差异与默认行为 | AC-5 | coding | task2, task3 |
| task6 | 设计并补充模式切换与旧字段回退的测试用例 | AC-4, AC-5 | analyze | task1 |

## Claude-Codex Deliberation

### Agreements
- UI 保持单页壳子，不拆成两套页面。
- `rgb_pnp` 继续作为默认路径。
- `depth_aligned` 只是在观测层和质量展示层切换，不重写求解器。
- 结果摘要中的求解残差继续共用，不按模式拆分。

### Resolved Disagreements
- 数据驱动 vs 前端写死：选择数据驱动 schema，因为后续深度指标更容易扩展，也更符合“同壳子多模式”的目标。
- 表格全拆 vs 局部动态：选择局部动态列，不拆表格组件，避免冗余。

### Convergence Status
- Final Status: `converged`

## Pending User Decisions

- DEC-1: `depth_aligned` 第一版是否需要在 UI 中显示额外的参数配置区
  - Claude Position: 第一版先只做模式切换与质量 schema 渲染，把参数区留在后端或高级模式里。
  - Codex Position: 可以先不单独暴露一堆参数，以免 UI 复杂化。
  - Tradeoff Summary: 参数区做得越早，使用越直接；但界面会更拥挤，也更容易把模式切换主线冲散。
  - Decision Status: `DEFERRED_TO_V2`
  - User Decision: 第一版不加阈值配置区，第二版再补。

## Implementation Notes

### Code Style Requirements
- Implementation code and comments must NOT contain plan-specific terminology such as "AC-", "Milestone", "Step", "Phase", or similar workflow markers
- These terms are for plan documentation only, not for the resulting codebase
- Use descriptive, domain-appropriate naming in code instead

## Output File Convention

This plan is written as the main review document for the UI depth-align compatibility feature.
