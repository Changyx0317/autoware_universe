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
- 对终端点姿态/速度做收敛友好处理（终点明确零速，减少终端控制切换抖动风险）。

4. **调试日志增强**
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

