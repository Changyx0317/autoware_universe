# autoware_dwa_trajectory_follower 集成与逻辑说明（与当前代码同步）

本文档基于当前 `autoware_dwa_trajectory_follower` 代码（多文件拆分版）编写，重点说明：
- 当前文件结构与职责
- 输入输出与数据流
- 目标/终点/前后进状态机
- DWA 采样与代价函数
- 障碍物处理逻辑
- 参数声明与热更新现状

---

## 1. 文件结构（当前）

### 1.1 包内核心文件
- `include/autoware/dwa_trajectory_follower/dwa_trajectory_follower_node.hpp`
- `include/autoware/dwa_trajectory_follower/utils.hpp`
- `src/dwa_trajectory_follower/main.cpp`
- `src/dwa_trajectory_follower/node_construction.cpp`
- `src/dwa_trajectory_follower/callbacks_and_utils.cpp`
- `src/dwa_trajectory_follower/dwa_solver.cpp`
- `src/dwa_trajectory_follower/timer.cpp`
- `config/dwa_trajectory_follower.param.yaml`

### 1.2 包外接线文件（Autoware 启动链）
- `src/universe/autoware_universe/launch/tier4_control_launch/launch/control.launch.xml`
- `src/universe/autoware_universe/launch/tier4_control_launch/package.xml`
- `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_control_component.launch.xml`
- `src/launcher/autoware_launch/autoware_launch/config/control/trajectory_follower/dwa_trajectory_follower.param.yaml`
- `src/launcher/autoware_launch/autoware_launch/config/control/preset/dwa_tracking_minimal_preset.yaml`

---

## 2. 文件职责映射

### 2.1 `main.cpp`
- ROS2 入口：`init -> create node -> spin -> shutdown`。

### 2.2 `node_construction.cpp`
- 声明参数（`declare_parameter`）。
- 创建订阅器、发布器、定时器。
- 注册动态参数回调 `on_set_parameters()`。

### 2.3 `callbacks_and_utils.cpp`
- 输入回调：轨迹、goal、里程计、预测障碍、检测障碍。
- 目标点提取 `get_target()`。
- 前进/倒车判定 `decide_reverse_mode()`。
- 障碍物源选择与过滤 `get_active_obstacles()`。
- 运动学积分 `simulate_step()`。
- 障碍物净空/碰撞评估 `evaluate_obstacle_metrics()`。
- 控制输出封装：`make_cmd/publish_move/publish_stop/publish_gear/publish_engage`。

### 2.4 `dwa_solver.cpp`
- DWA 求解入口 `run_dwa()`。
- 动态窗口构建、候选采样、前向仿真。
- 约束过滤（推进/碰撞/刹停距离）。
- 代价归一化与总分排序。

### 2.5 `timer.cpp`
- 主状态机 `on_timer()`：
  - 终点判定与终点接管
  - 到点停稳与朝向对齐
  - 近距离新目标“先原地转向再起步”
  - 前后进判定
  - DWA 输出到控制命令

### 2.6 `dwa_trajectory_follower_node.hpp`
- 所有数据结构定义（`Obstacle2D/SimState/DwaCandidate/ObstacleMetrics`）。
- 参数缓存、运行状态、函数声明。

---

## 3. 输入输出与话题

### 3.1 输入订阅
- `/planning/scenario_planning/trajectory` (`autoware_planning_msgs/msg/Trajectory`)
- `/planning/mission_planning/goal` (`geometry_msgs/msg/PoseStamped`)
- `/localization/kinematic_state` (`nav_msgs/msg/Odometry`)
- `/perception/object_recognition/objects` (`autoware_perception_msgs/msg/PredictedObjects`)
- `/perception/object_recognition/detection/objects` (`autoware_perception_msgs/msg/DetectedObjects`)

### 3.2 输出发布
- `/control/trajectory_follower/control_cmd` (`autoware_control_msgs/msg/Control`)
- `/control/command/gear_cmd` (`autoware_vehicle_msgs/msg/GearCommand`)
- `/vehicle/engage` (`autoware_vehicle_msgs/msg/Engage`)
- `/control/trajectory_follower/debug/goal_distance_m` (`std_msgs/msg/Float64`)
- `/control/trajectory_follower/debug/goal_yaw_error_rad` (`std_msgs/msg/Float64`)

---

## 4. 目标与状态机逻辑（当前版本）

## 4.1 控制目标来源
`get_target()` 支持两种模式：
1. `use_direct_goal_mode = true`：直接追 `mission goal`。
2. `use_direct_goal_mode = false`：从轨迹中取“最近点”（当前已取消固定前瞻点追踪）。

注意：
- DWA 追踪目标 `(tx, ty)` 与“终点判定目标”解耦。
- 终点判定优先使用 `latest_goal`，否则使用轨迹末点。

## 4.2 到点接管与姿态对齐
在 `timer.cpp` 中：
- 进入 `terminal_mode_active_` 后，逻辑会保持终点接管，避免 DWA 与对齐逻辑来回切换。
- 到点后流程是“先停稳，再原地转向对齐终点朝向（可选）”。
- 对齐时使用：
  - 方向锁定（避免符号来回翻转）
  - 角速度下限/上限
  - 角速度变化率限制
  - settle 时间（线速度连续小于阈值一段时间才允许进入旋转对齐）

## 4.3 停车保持与解锁
- 到点后可 `hold_stopped_` 保持停车。
- 不再因轨迹/goal“时间戳刷新”自动解锁。
- 只有当新 goal 相对 hold 点发生足够位移或 yaw 变化，才解锁。

## 4.4 近距离新目标“短距原地对齐后起步”
`short_goal_pivot` 逻辑：
- 条件：处于 hold_stopped，收到新目标且目标位移小于阈值。
- 行为：先原地旋转，让车头或车尾对准目标方向（自动比较两者误差选更优），然后恢复行驶。

## 4.5 前进/倒车判定
- 正常由 `decide_reverse_mode()`（带滞回阈值）控制。
- 在直接目标模式下，如果目标明显在车后方，可强制倒车优先。
- 近目标范围内可禁用倒车切换，防止末端来回切挡。

---

## 5. DWA 求解逻辑（`dwa_solver.cpp`）

## 5.1 动态窗口
- `dt_ctrl = 1 / control_rate_hz`
- 速度窗口时间：`v_window_dt = max(dt_ctrl, velocity_window_time_s)`
- 线速度窗口：
  - `v_low = max(min_speed, current_v - a_max * v_window_dt)`
  - `v_high = min(max_speed, max(current_v + a_max * v_window_dt, min_search_speed))`
- 角速度窗口：
  - 围绕 `last_cmd_w_`，受 `dwa_max_yaw_accel_rps2` 和 `dwa_max_yaw_rate_rps` 限制。

## 5.2 候选仿真
- 采样 `(v_abs, w)`。
- 使用 `simulate_step()`（unicycle）做离散积分，预测时域 `dwa_predict_time_s`。

## 5.3 过滤约束
- 推进约束：`progress_reward >= v_abs * predict_time * min_progress_ratio`。
- 碰撞剔除：`metrics.collision == true` 直接丢弃。
- 刹停距离约束：
  - `min_clearance >= calc_braking_distance(v_abs) + braking_distance_margin_m`。

## 5.4 代价函数（当前只有经典加权模型）
当前总分为“越小越优”：
- `goal_dist`
- `goal_heading`
- `speed`
- `obstacle`
- `progress_cost = 1 - progress_norm`
- `omega_cost = |w| / max_yaw_rate`

总分：
`J = w_goal_dist*d + w_goal_heading*h + w_speed*s + w_obstacle*o + w_progress*p + w_omega*omega`

备注：
- `use_matlab_style_cost` 相关功能已移除，当前仅保留上述经典加权方案。

---

## 6. 障碍物处理逻辑（当前）

## 6.1 输入抽象
- 无论 `PredictedObjects` 还是 `DetectedObjects`，都转换为：
  - 中心 `(x, y)`
  - 朝向 `yaw`
  - 尺寸 `length, width`
- 即 `Obstacle2D` 旋转矩形（OBB）模型。

## 6.2 源选择与过滤
- 优先 `predicted_obstacles_`，无则回退 `detected_obstacles_`。
- 再按 `obstacle_consider_range_m` 做距离过滤。

## 6.3 净空与碰撞判定
- 对候选轨迹每个离散点，计算“点到旋转矩形有符号距离”。
- 安全边界：`robot_collision_radius_m + obstacle_inflation_radius_m`。
- `clearance = signed_distance - safety_margin`。
- `clearance <= 0` 视为碰撞。

## 6.4 TTC 与障碍物代价
- `obstacle_cost_mode`: `exp` 或 `inv`。
- 可叠加 TTC 代价（`use_ttc_cost`）。

---

## 7. 控制输出语义

- 本包按差速语义输出：`steering_tire_angle` 字段承载 `yaw rate (w)`。
- `publish_move()` 会根据 `reverse` 强制设置 `longitudinal.velocity` 的符号：
  - 前进为正
  - 倒车为负
- 同时发布对应档位：`DRIVE / REVERSE`。
- `publish_stop()`：
  - 档位置 `PARK`
  - 速度 0
  - 加速度给 `max_decel_mps2_`（负值）

---

## 8. 参数与热更新现状（当前代码）

## 8.1 已声明且可热更新（on_set_parameters 已处理）
示例（非穷尽，按大类）：
- 基础：`control_rate_hz`, `goal_tolerance_m`, `terminal_capture_distance_m`, `use_direct_goal_mode`
- 终点对齐：`use_goal_yaw_alignment`, `goal_yaw_tolerance_rad`, `goal_yaw_align_kp`, `goal_yaw_align_max_w_rps`, `yaw_align_*`
- DWA：`dwa_predict_time_s`, `dwa_dt_s`, `dwa_v_resolution_mps`, `dwa_w_resolution_rps`, `dwa_max_yaw_rate_rps`, `dwa_max_yaw_accel_rps2`, `dwa_max_accel_mps2`
- 代价权重：`w_goal_dist`, `w_goal_heading`, `w_speed`, `w_obstacle`, `w_progress`, `w_omega`
- 障碍相关：`obstacle_inflation_radius_m`, `robot_collision_radius_m`, `obstacle_consider_range_m`, `braking_distance_margin_m`, `braking_reaction_time_s`, `ttc_horizon_s`, `ttc_cost_gain`
- 速度策略：`cruise_speed_mps`, `min_speed_mps`, `max_speed_mps`, `min_progress_speed_mps`, `near_goal_slowdown_distance_m`, `velocity_window_time_s`, `velocity_estimator_alpha`
- 近距离新目标 pivot：`enable_short_goal_pivot_after_stop` 及 `short_goal_pivot_*`

## 8.2 代码内固定值/未开放为参数（当前）
以下变量存在于类中，但当前未在 `declare_parameter` 中暴露：
- `hold_stop_on_goal_`
- `force_engage_`
- `use_velocity_state_estimator_`
- `enforce_output_min_progress_speed_`
- `reject_low_progress_candidate_`
- `enable_cost_normalization_`
- `obstacle_cost_mode_`
- `use_obstacle_avoidance_`
- `enable_braking_distance_check_`
- `use_ttc_cost_`
- `log_cost_debug_`, `log_cost_throttle_ms_`, `log_top_k_`

说明：
- 这部分当前按代码默认值运行，`ros2 param set` 不生效（因为未声明）。
- 如需运行时可调，需要在 `node_construction.cpp` 的 `declare_parameter` 与 `on_set_parameters` 同步接入。

---

## 9. 关键排查命令（实用）

```bash
# 节点是否存在
ros2 node list | grep /control/trajectory_follower/controller_node_exe

# DWA 输出是否连续
ros2 topic hz /control/trajectory_follower/control_cmd
ros2 topic echo /control/trajectory_follower/control_cmd --once

# Gate 前后是否被改写
ros2 topic echo /control/trajectory_follower/control_cmd --once
ros2 topic echo /control/command/control_cmd --once

# 档位闭环
ros2 topic echo /control/command/gear_cmd --once
ros2 topic echo /vehicle/status/gear_status --once

# DWA 调试输出
ros2 topic echo /control/trajectory_follower/debug/goal_distance_m --once
ros2 topic echo /control/trajectory_follower/debug/goal_yaw_error_rad --once

# 障碍物输入
ros2 topic echo /perception/object_recognition/objects --once
ros2 topic echo /perception/object_recognition/detection/objects --once
```

---

## 10. 维护建议（按改动类型）

- 改 DWA 采样/代价：`dwa_solver.cpp`
- 改终点行为/状态机：`timer.cpp`
- 改输入处理/障碍物抽象：`callbacks_and_utils.cpp`
- 增删参数：
  1. `node_construction.cpp` 声明参数
  2. `on_set_parameters()` 加热更新逻辑
  3. YAML 增注释与默认值

