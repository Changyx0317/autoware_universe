# Planning Adaptation Notes (3 Packages)

本文档记录当前差速底盘停车/巡检方案在规划层对以下 3 个包做过的适配：

1. `planning/autoware_freespace_planner`
2. `planning/autoware_freespace_planning_algorithms`
3. `planning/autoware_scenario_selector`

适用范围：用于在新的 Autoware 代码基中迁移同等规划能力时做对照。

## 1) autoware_freespace_planner

### 1.1 修改文件

- `planning/autoware_freespace_planner/config/freespace_planner.param.yaml`
- `planning/autoware_freespace_planner/schema/freespace_planner.schema.json`
- `planning/autoware_freespace_planner/src/autoware_freespace_planner/freespace_planner_node.cpp`

### 1.2 新增/调整参数

位于 `astar.*`：

- `motion_model`：`bicycle` / `diff_drive`
- `enable_direct_three_phase_maneuver`：是否优先尝试“三段式直连机动”
- `max_curvature`：diff-drive 展开时最大曲率
- `allow_in_place_turn`：A* 是否允许原地旋转原语
- `in_place_turn_angle_rad`：原地旋转步长
- `in_place_turn_cost`：原地旋转代价

说明：`freespace_planner.param.yaml` 默认设置为 `motion_model: diff_drive` 且开启 `enable_direct_three_phase_maneuver`。
同时，目标判定容差统一为：
- `lateral_goal_range = 0.2`
- `longitudinal_goal_range = 0.3`
- `angle_goal_range = 360.0`

可直接 `ros2 param set` 的参数路径（规划层）：

- 节点：`/planning/scenario_planning/parking/freespace_planner`
- 参数：`astar.max_curvature`
- 说明：差速模型最大曲率约束。
- 示例：`ros2 param set /planning/scenario_planning/parking/freespace_planner astar.max_curvature 0.7`

- 节点：`/planning/scenario_planning/parking/freespace_planner`
- 参数：`astar.in_place_turn_angle_rad`
- 说明：原地旋转单步角度（弧度）。
- 示例：`ros2 param set /planning/scenario_planning/parking/freespace_planner astar.in_place_turn_angle_rad 0.174532925`

- 节点：`/planning/scenario_planning/parking/freespace_planner`
- 参数：`astar.in_place_turn_cost`
- 说明：原地旋转代价权重。
- 示例：`ros2 param set /planning/scenario_planning/parking/freespace_planner astar.in_place_turn_cost 0.2`

- 节点：`/planning/scenario_planning/parking/freespace_planner`
- 参数：`lateral_goal_range`
- 说明：目标横向容差（m），用于终点判定窗口。
- 示例：`ros2 param set /planning/scenario_planning/parking/freespace_planner lateral_goal_range 0.1`

- 节点：`/planning/scenario_planning/parking/freespace_planner`
- 参数：`longitudinal_goal_range`
- 说明：目标纵向容差（m），用于终点判定窗口。
- 示例：`ros2 param set /planning/scenario_planning/parking/freespace_planner longitudinal_goal_range 0.1`

- 节点：`/planning/scenario_planning/parking/freespace_planner`
- 参数：`th_arrived_distance_m`
- 说明：freespace 节点内部“到达目标点”距离阈值（m）。
- 示例：`ros2 param set /planning/scenario_planning/parking/freespace_planner th_arrived_distance_m 0.1`

### 1.3 主要逻辑改动

1. **Diff-drive 直连三段式优先**（无障碍时）
- 在规划前先尝试“起点对齐 + 直线段 + 终点姿态点”的候选路径。
- 若候选路径无碰撞，则直接下发该轨迹。
- 若被障碍物阻塞，再回退到 A*。

2. **前进/倒车方向选择改为最小初始转角**
- 对同一个目标方向，同时计算车头对齐误差和车尾对齐误差。
- 选误差更小的模式（前进或倒车）作为直连三段式的 heading 参考。

3. **终端点与时标修正**
- 对输出轨迹统一补 `time_from_start`。
- 终端点速度明确为 0。
- 终端点位置对齐 goal，终端朝向保持几何连续（不再把末端朝向硬改到 goal 朝向）。

4. **去掉末端强制补段**
- 删除末端“补一段 approach 拉回 goal”的逻辑，避免末段几何被人为重写导致突变。

5. **调试日志增强**
- 增加“FRONT/REAR 选择结果”和“直连失败回退 A*”日志，方便排查连接点行为。

## 2) autoware_freespace_planning_algorithms

### 2.1 修改文件

- `planning/autoware_freespace_planning_algorithms/include/autoware/freespace_planning_algorithms/astar_search.hpp`
- `planning/autoware_freespace_planning_algorithms/include/autoware/freespace_planning_algorithms/kinematic_diff_drive_model.hpp`（新增）
- `planning/autoware_freespace_planning_algorithms/src/astar_search.cpp`
- `planning/autoware_freespace_planning_algorithms/scripts/bind/astar_search_pybind.cpp`

### 2.2 参数/接口扩展

`AstarParam` 增加 diff-drive 相关字段：

- `motion_model`
- `max_curvature`
- `allow_in_place_turn`
- `in_place_turn_angle`
- `in_place_turn_cost`

并在 pybind 中暴露同名字段，便于脚本侧一致配置。

### 2.3 主要逻辑改动

1. **A* 支持 diff-drive 运动学展开**
- 在 `getNextPose(...)` 中根据 `motion_model` 分支，支持差速运动模型推进。

2. **可选原地旋转扩展**
- `expandInPlaceRotations(...)` 在 `allow_in_place_turn=true` 时启用。
- 将原地旋转作为额外邻接原语加入搜索图，并计入 `in_place_turn_cost`。

3. **曲率离散策略适配**
- 使用 `max_curvature` 控制 diff-drive 展开曲率范围与分辨率。

4. **终点语义回归原始 goal**
- 在 `isGoal()` 中禁用 `setShiftedGoalPose(...)` 的终点替代逻辑。
- 终点判定不再通过 shifted-goal 偏移修正，避免“规划末端不以原始 goal 为终点”的现象。

5. **终点容差策略**
- 规划参数统一为 `lateral_goal_range = 0.2`、`longitudinal_goal_range = 0.3`、`angle_goal_range = 360.0`。
- 语义：位置容差收紧，朝向容差放宽（朝向由控制层末端原地对齐处理）。

## 3) autoware_scenario_selector

### 3.1 修改文件

- `planning/autoware_scenario_selector/launch/dummy_scenario_selector_lane_driving.launch.xml`
- `planning/autoware_scenario_selector/launch/dummy_scenario_selector_parking.launch.xml`

### 3.2 主要改动

1. **dummy scenario 发布命令修正**
- `ros2 topic pub` 增加 `-r 1`（持续发布）。
- 场景消息字符串改为带引号的稳定写法，减少 shell/launch 转义导致的发布异常。

2. **目的**
- 避免仿真联调中 scenario topic 只发一次或格式异常，导致 parking/lane_driving 场景链路不稳定。

## 4) 迁移注意事项

1. 以上 3 个包需成套迁移，单独迁移其中某一个会导致行为不一致。
2. `freespace_planner.param.yaml` 与 `schema` 必须同步迁移，否则新参数可能不生效或被校验拒绝。
3. 若新工程的 launch 使用的是其他 scenario_selector 启动链路，需要把 dummy launch 的修正等价合入对应入口。
4. 迁移后建议先验证：
- `ros2 param get /planning/scenario_planning/parking/freespace_planner astar.motion_model`
- `ros2 param get /planning/scenario_planning/parking/freespace_planner astar.enable_direct_three_phase_maneuver`
- `ros2 param get /planning/scenario_planning/parking/freespace_planner lateral_goal_range`
- `ros2 param get /planning/scenario_planning/parking/freespace_planner longitudinal_goal_range`
- `ros2 param get /planning/scenario_planning/parking/freespace_planner angle_goal_range`
5. 控制层建议配套检查：
- `ros2 param get /control/trajectory_follower/controller_node_exe parking_three_phase.post_align_enter_distance_m`
- `ros2 param get /control/trajectory_follower/controller_node_exe parking_three_phase.post_align_keep_distance_m`

控制层可直接 `ros2 param set` 的参数路径（用于末端原地对齐）：

- 节点：`/control/trajectory_follower/controller_node_exe`
- 参数：`parking_three_phase.enable`
- 说明：三段式状态机总开关（PRE_ALIGN/TRACK/POST_ALIGN/HOLD）。
- 示例：`ros2 param set /control/trajectory_follower/controller_node_exe parking_three_phase.enable true`

- 节点：`/control/trajectory_follower/controller_node_exe`
- 参数：`parking_three_phase.stop_velocity_threshold_mps`
- 说明：判定“停稳”的速度阈值。
- 示例：`ros2 param set /control/trajectory_follower/controller_node_exe parking_three_phase.stop_velocity_threshold_mps 0.1`

- 节点：`/control/trajectory_follower/controller_node_exe`
- 参数：`parking_three_phase.post_align_enter_distance_m`
- 说明：进入末端原地对齐窗口的距离阈值。
- 示例：`ros2 param set /control/trajectory_follower/controller_node_exe parking_three_phase.post_align_enter_distance_m 0.15`

- 节点：`/control/trajectory_follower/controller_node_exe`
- 参数：`parking_three_phase.post_align_keep_distance_m`
- 说明：末端原地对齐保持窗口距离阈值（通常略大于 enter）。
- 示例：`ros2 param set /control/trajectory_follower/controller_node_exe parking_three_phase.post_align_keep_distance_m 0.25`

- 节点：`/control/trajectory_follower/controller_node_exe`
- 参数：`parking_three_phase.post_align_enter_yaw_threshold_rad`
- 说明：进入 POST_ALIGN 的航向误差阈值。
- 示例：`ros2 param set /control/trajectory_follower/controller_node_exe parking_three_phase.post_align_enter_yaw_threshold_rad 0.05`

- 节点：`/control/trajectory_follower/controller_node_exe`
- 参数：`parking_three_phase.post_align_exit_yaw_threshold_rad`
- 说明：退出 POST_ALIGN（进入 HOLD）的航向误差阈值。
- 示例：`ros2 param set /control/trajectory_follower/controller_node_exe parking_three_phase.post_align_exit_yaw_threshold_rad 0.07`

- 节点：`/control/trajectory_follower/controller_node_exe`
- 参数：`parking_three_phase.align_kp`
- 说明：原地对齐角速度比例增益。
- 示例：`ros2 param set /control/trajectory_follower/controller_node_exe parking_three_phase.align_kp 2.0`

- 节点：`/control/trajectory_follower/controller_node_exe`
- 参数：`parking_three_phase.align_min_omega_rps`
- 说明：原地对齐最小角速度，防止末端“差一点不动”。
- 示例：`ros2 param set /control/trajectory_follower/controller_node_exe parking_three_phase.align_min_omega_rps 0.15`

- 节点：`/control/trajectory_follower/controller_node_exe`
- 参数：`parking_three_phase.align_max_omega_rps`
- 说明：原地对齐最大角速度限幅。
- 示例：`ros2 param set /control/trajectory_follower/controller_node_exe parking_three_phase.align_max_omega_rps 0.35`

横向控制职责分离相关参数（建议关闭）：

- 节点：`/control/trajectory_follower/lateral/pure_pursuit`
- 参数：`enable_heading_alignment_gate`
- 说明：纯追踪起步对齐门控，建议关闭，避免与 three-phase 重叠。
- 示例：`ros2 param set /control/trajectory_follower/lateral/pure_pursuit enable_heading_alignment_gate false`

- 节点：`/control/trajectory_follower/lateral/pure_pursuit`
- 参数：`enable_goal_yaw_alignment_after_stop`
- 说明：纯追踪末端朝向对齐，建议关闭，仅由 three-phase 负责末端原地转向。
- 示例：`ros2 param set /control/trajectory_follower/lateral/pure_pursuit enable_goal_yaw_alignment_after_stop false`
