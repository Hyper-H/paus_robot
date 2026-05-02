# 磕头振荡修复计划

## 目标描述

修复机械臂接近 marker 后在 pre_approach、reorient、final_hover 之间反复上下振荡的问题。修复必须以实验室主机上的 /home/chen_lab/worktrees/paus_robot_handeye 这条最新 eye-to-hand semi-auto 基线为起点，在新的隔离 worktree 和新分支中完成，不能修改现有半自动手眼标定 worktree。

核心行为是：marker 可见时继续按最新 marker 位姿逐帧计算几何目标，但执行阶段只能正常单调推进，不能从 reorient 或 final_hover 回退到 pre_approach。marker 短时丢失时保留 last valid target / final hover / surface normal / stage latch，把它视为可能被机械臂遮挡；长时间丢失时报告 target lost 并停止主动推进，而不是重新发高位 pre_approach 命令。safe_lift 等安全异常仍然必须能优先打断。

## 验收标准

- AC-1: 修复工作在实验室主机的新隔离 worktree 中进行，基线是 codex/eye-to-hand-semi-auto
  - Positive Tests (expected to PASS):
    - git worktree list 显示新开发目录在实验室主机上，而不是现有 /home/chen_lab/worktrees/paus_robot_handeye。
    - 新分支从 codex/eye-to-hand-semi-auto 派生，而不是从 main 派生。
  - Negative Tests (expected to FAIL):
    - 在现有半自动 worktree 中产生代码 diff。
    - 从 main 或旧 tracking 分支重新起步。

- AC-2: 正常 tracking 阶段单调推进，不允许低阶段覆盖高阶段
  - Positive Tests (expected to PASS):
    - 已进入 reorient 后，后续可见 marker 帧即使几何候选重新算出 pre_approach，最终可执行阶段也保持在 reorient 或更高。
    - 已进入 final_hover 后，后续可见 marker 帧即使几何候选重新算出 pre_approach 或 reorient，最终可执行阶段保持 final_hover。
  - Negative Tests (expected to FAIL):
    - control_executed 或 dry-run 决策在 final_hover 后又发布可执行 pre_approach。

- AC-3: 阶段锁存不冻结可见 marker 的位置更新
  - Positive Tests (expected to PASS):
    - marker 仍可见时，进入 reorient 或 final_hover 后，新的 marker 位姿仍能更新 candidate pose 的位置。
    - 锁存只限制 stage regression，不把 candidate pose 固定为旧 marker。
  - Negative Tests (expected to FAIL):
    - marker 可见且位姿明显变化后，candidate pose 仍完全使用旧 final hover 坐标。

- AC-4: target 短时丢失时保留 last valid 状态，不触发回到初始接近位姿
  - Positive Tests (expected to PASS):
    - target age 小于 target_hold_timeout_ms 时，last valid target、last valid final hover、surface normal 和 stage latch 保留。
    - 短时丢失发布 pending/hold 状态，不发 pre_approach 重置命令。
  - Negative Tests (expected to FAIL):
    - 短时丢失清空 last valid target 或 stage latch。
    - 短时丢失后下一次状态更新直接回到 pre_approach。

- AC-5: target 长时间丢失时安全停止追踪，而不是盲目重置
  - Positive Tests (expected to PASS):
    - target age 超过 target_hold_timeout_ms 且未达到 completion 时，发布 tracking_target_lost 和 target_lost_timeout_before_reach。
    - 长时间丢失时不生成新的高位 pre_approach 运动命令。
  - Negative Tests (expected to FAIL):
    - 长时间丢失后仍继续主动推进到新候选点。
    - 长时间丢失路径触发回退式 pre_approach。

- AC-6: 安全异常仍然优先于阶段单调锁存
  - Positive Tests (expected to PASS):
    - safe_lift 候选仍可覆盖 reorient 或 final_hover 锁存。
    - workspace、minimum z、minimum plane clearance 等 rejection 行为保持原有安全语义。
  - Negative Tests (expected to FAIL):
    - 阶段锁存阻止 safe_lift。
    - 阶段锁存把安全 rejection 改成可执行运动。

- AC-7: repeat_distance_threshold_mm 与 stage_switch_buffer_mm 语义分离
  - Positive Tests (expected to PASS):
    - repeat_distance_threshold_mm 继续只用于跳过重复命令。
    - 新增或读取 stage_switch_buffer_mm 用于阶段切换缓冲；未配置时默认兼容现有 repeat 阈值，避免破坏旧配置。
  - Negative Tests (expected to FAIL):
    - 调整 repeat threshold 会隐式改变阶段切换行为，且没有单独配置可以控制。

- AC-8: 只运行非真机测试
  - Positive Tests (expected to PASS):
    - 单元测试直接覆盖 control_logic.py 或控制节点中的纯状态辅助逻辑。
    - 测试使用 mock pose/mock node，不连接 chen_lab@192.168.58.183 的真机运动路径。
  - Negative Tests (expected to FAIL):
    - 测试需要实验室主机执行真机运动。
    - 测试调用真实 MoveJ、MoveL 或 FAIRINO SDK 运动路径。

## 路径边界

### Upper Bound (Maximum Scope)

允许在实验室主机上的新 worktree 中修改 paus_motion_ros2 的控制逻辑、控制节点状态机、默认配置和对应单元测试。可以新增小型 helper，例如 stage order、stage latch、candidate decision normalizer，前提是保持文件边界清晰并兼容现有 completion latch / tracking status。

### Lower Bound (Minimum Scope)

至少要在 node 侧阻止 final_hover -> pre_approach 和 reorient -> pre_approach 的正常回退，并增加测试证明短时 target lost 不会清空 last valid 状态或触发初始接近重置。

### Allowed Choices

- Can use: git worktree、Python unit tests、现有 ROS2 package test 结构、mock pose、纯函数 / 小 helper。
- Can use: 新配置 stage_switch_buffer_mm，默认回退到现有 repeat_distance_threshold_mm。
- Cannot use: 真机运动、实验室主机远程运动命令、重置或回退别人的改动、修改现有 /home/chen_lab/worktrees/paus_robot_handeye。

## Dependencies and Sequence

### Milestones

1. 创建隔离工作区
   - 从 /home/chen_lab/worktrees/paus_robot_handeye 对应的 codex/eye-to-hand-semi-auto 创建新 worktree。
   - 新分支建议命名为 codex/kneel-oscillation-fix 或 feature/kneel-oscillation-fix。
   - 确认原 semi-auto worktree 无新增 diff。

2. 定位并抽象阶段锁存
   - 阅读 control_logic.py 的 candidate stage 计算。
   - 阅读 fairino_control_node.py 的 target callback、status timer、repeat threshold、completion latch 路径。
   - 增加一个小而可测的 stage latch / normalization helper：输入当前 latch、几何候选 stage、安全状态，输出最终可执行 stage。

3. 实现正常阶段单调推进
   - pre_approach < reorient < final_hover。
   - 正常 tracking 下只允许保持或推进，不允许回退。
   - safe_lift 和 rejection 不受普通单调规则阻挡。

4. 实现 target lost 保持语义
   - 短时丢失保留 last valid target / final hover / surface normal / stage latch。
   - 长时间丢失只发布 lost 状态，不主动生成 reset-to-pre-approach 命令。
   - marker 重新可见后，使用新 marker 位姿更新 candidate pose，但 stage latch 仍阻止回退。

5. 拆分阶段缓冲配置
   - 在默认配置中加入 stage_switch_buffer_mm。
   - node 读取该配置，未配置时兼容 repeat_distance_threshold_mm。
   - build_approach_decision(... stage_switch_buffer_mm=...) 不再直接复用 repeat threshold。

6. 增加非真机测试
   - 覆盖 stage monotonicity。
   - 覆盖 marker 可见时位置仍更新。
   - 覆盖短时 target lost 保持状态。
   - 覆盖长时间 target lost 不回退到 pre_approach。
   - 覆盖 safe_lift 优先。

7. 运行验证并汇报
   - 运行相关 Python / unit tests。
   - 不启动真机运动，不连接实验室主机。
   - 汇报修改文件、测试命令、测试结果和剩余风险。

## Implementation Notes

- 代码里不要使用 plan 专用术语；实现命名应贴近现有代码风格，例如 stage_latch、last_executed_stage、candidate_stage。
- 阶段锁存应该在 node 侧处理，因为 control_logic.py 是几何候选计算，不应该持有跨帧状态。
- 如果需要对 candidate pose 做阶段提升，优先使用同一帧最新 marker 算出的 final_hover_pose_mmdeg 或 pre_approach_pose_mmdeg，不要在 marker 可见时强行使用旧 target。
- target lost 的 last valid 路径只能短时保持；超过 hold timeout 后应停止主动推进，避免盲追。
- completion latch、stage gated、repeat command skip 的现有日志字段应尽量保持兼容，便于 UI 和日志分析继续工作。

