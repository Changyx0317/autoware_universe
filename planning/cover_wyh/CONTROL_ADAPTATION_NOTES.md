# Control Adaptation Notes

本文档记录当前控制层相对“最初始基线”（`autoware_universe` 分支与 `origin/main` 的分叉点）到当前代码的差异，用于迁移和复现。

本次对比基线：

- `autoware_universe` merge-base: `6b7caf692`（`origin/main`）
- `autoware_launch` merge-base: `93f9e70d7`（`origin/main`）

## 1) 控制层改动范围总览

从基线到当前，控制层涉及的改动不止纯追踪和三段式，包含：

1. 新增控制包
- `control/autoware_dwa_trajectory_follower`（完整新增）
- `control/autoware_diff_drive_control_adapter`（完整新增）
- `control/autoware_short_goal_pivot_coordinator`（完整新增）

2. 现有包深度改造
- `control/autoware_pure_pursuit`
- `control/autoware_trajectory_follower_node`

3. 控制参数入口变更（launch）
- `autoware_launch/config/control/*` 多文件新增/修改

4. 其他控制子包
- 多个 `CHANGELOG.rst` 有版本/说明更新（不涉及核心控制逻辑执行链）。

## 2) 新增控制包

### 2.1 autoware_dwa_trajectory_follower

新增文件（核心）：

- `control/autoware_dwa_trajectory_follower/CMakeLists.txt`
- `control/autoware_dwa_trajectory_follower/package.xml`
- `control/autoware_dwa_trajectory_follower/config/dwa_trajectory_follower.param.yaml`
- `control/autoware_dwa_trajectory_follower/include/autoware/dwa_trajectory_follower/dwa_trajectory_follower_node.hpp`
- `control/autoware_dwa_trajectory_follower/include/autoware/dwa_trajectory_follower/utils.hpp`
- `control/autoware_dwa_trajectory_follower/src/dwa_trajectory_follower/*.cpp`
- `control/autoware_dwa_trajectory_follower/scripts/override_controller_with_dwa.sh`
- `control/autoware_dwa_trajectory_follower/DWA_NEW_AUTOWARE_SETUP.md`
- `control/autoware_dwa_trajectory_follower/DWA_INTEGRATION_AND_LOGIC.md`

作用：

- 引入 DWA 轨迹跟踪控制器实现与接入说明，可替换/并行于原有跟踪链路进行实验。

### 2.2 autoware_diff_drive_control_adapter

新增文件（核心）：

- `control/autoware_diff_drive_control_adapter/CMakeLists.txt`
- `control/autoware_diff_drive_control_adapter/package.xml`
- `control/autoware_diff_drive_control_adapter/config/diff_drive_control_adapter.param.yaml`
- `control/autoware_diff_drive_control_adapter/launch/diff_drive_control_adapter.launch.xml`
- `control/autoware_diff_drive_control_adapter/src/diff_drive_control_adapter_node.cpp`
- `control/autoware_diff_drive_control_adapter/README.md`

作用：

- 在控制指令语义与差速底盘执行语义之间做适配与桥接。

### 2.3 autoware_short_goal_pivot_coordinator

新增文件（核心）：

- `control/autoware_short_goal_pivot_coordinator/CMakeLists.txt`
- `control/autoware_short_goal_pivot_coordinator/package.xml`
- `control/autoware_short_goal_pivot_coordinator/config/short_goal_pivot_coordinator.param.yaml`
- `control/autoware_short_goal_pivot_coordinator/launch/short_goal_pivot_coordinator.launch.xml`
- `control/autoware_short_goal_pivot_coordinator/src/short_goal_pivot_coordinator_node.cpp`
- `control/autoware_short_goal_pivot_coordinator/README.md`

作用：

- 增加短距离目标附近的原地转向/协调能力，辅助末端控制过程。

## 3) 现有控制包逻辑改动

### 3.1 autoware_pure_pursuit

修改文件：

- `control/autoware_pure_pursuit/config/pure_pursuit.param.yaml`
- `control/autoware_pure_pursuit/include/autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller.hpp`
- `control/autoware_pure_pursuit/src/autoware_pure_pursuit/autoware_pure_pursuit_lateral_controller.cpp`

主要改动：

1. 差速控制语义扩展参数（配置与接口层面）。
2. 纯追踪职责收敛：
- 移除纯追踪内部原地对齐/终点锁死主逻辑分支。
- 保留跟踪主线输出（角速度由曲率与速度关系生成并限幅）。
3. 收敛判据简化：
- `calcIsSteerConverged(...)` 以转角阈值为主，不再耦合终点对齐状态。

### 3.2 autoware_trajectory_follower_node

修改文件：

- `control/autoware_trajectory_follower_node/include/autoware/trajectory_follower_node/controller_node.hpp`
- `control/autoware_trajectory_follower_node/src/controller_node.cpp`
- `control/autoware_trajectory_follower_node/param/trajectory_follower_node.param.yaml`

主要改动：

1. 三段式状态机增强（PRE_ALIGN / TRACK / POST_ALIGN / HOLD）：
- 增强末端切换条件与日志，支持末端原地对齐可观测调试。

2. 终点朝向来源改造：
- 新增订阅 `/planning/mission_planning/goal`。
- `getGoalPose(...)` 使用 mission goal 作为对齐 yaw 来源，不再依赖轨迹末端朝向。

3. 末端窗口参数调整：
- `parking_three_phase.post_align_enter_distance_m`
- `parking_three_phase.post_align_keep_distance_m`

## 4) 控制 launch 参数入口改动（autoware_launch）

修改文件：

- `autoware_launch/config/control/operation_mode_transition_manager/operation_mode_transition_manager.param.yaml`
- `autoware_launch/config/control/preset/dwa_tracking_minimal_preset.yaml`（新增）
- `autoware_launch/config/control/preset/pp_pid_direct_passthrough_preset.yaml`（新增）
- `autoware_launch/config/control/trajectory_follower/dwa_trajectory_follower.param.yaml`（新增）
- `autoware_launch/config/control/trajectory_follower/lateral/pure_pursuit.param.yaml`
- `autoware_launch/config/control/trajectory_follower/longitudinal/pid.param.yaml`
- `autoware_launch/config/control/trajectory_follower/trajectory_follower_node.param.yaml`

作用：

- 补齐 DWA/纯追踪/三段式控制链路在 launch 层的参数入口与预设切换。

## 5) 迁移注意事项

1. 若要复刻当前控制行为，迁移时不要只拷贝单个包，至少需要成组迁移：
- `autoware_pure_pursuit`
- `autoware_trajectory_follower_node`
- `autoware_launch/config/control/*` 对应入口
- （按需求）`autoware_dwa_trajectory_follower`、`autoware_diff_drive_control_adapter`、`autoware_short_goal_pivot_coordinator`

2. 运行时建议优先核验以下参数是否生效：
- `/control/trajectory_follower/controller_node_exe parking_three_phase.*`
- `/control/trajectory_follower/lateral/pure_pursuit enable_heading_alignment_gate`
- `/control/trajectory_follower/lateral/pure_pursuit enable_goal_yaw_alignment_after_stop`

3. 运行时建议核验关键日志：
- `three_phase transition TRACK->POST_ALIGN`
- `three_phase PRE_ALIGN active`
- `three_phase POST_ALIGN active`

