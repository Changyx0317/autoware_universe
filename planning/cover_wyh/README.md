# cover_wyh

一个基于 ROS 2 的巡检 + 推土任务调度节点。

当前版本支持两类任务：

- 巡检任务：在地图上录入多个巡检点，节点按顺序发布巡检目标。
- 推土任务：在地图上用 3 次点击定义一个矩形推土区域，节点按模板生成并下发该区域内的目标点。

执行逻辑如下：

- 默认执行巡检任务。
- 收到推土指令后，切换到推土任务执行矩形区域路径。
- 推土区域执行完成后，自动脱离推土状态，回到进入推土前暂停的巡检点，继续巡检。

## 1. 构建与启动

在工作区根目录执行：

```bash
colcon build --packages-select cover_wyh
source install/setup.bash
ros2 launch cover_wyh coverage.launch.py
```

启动文件位于 `launch/coverage.launch.py`。

参数文件位于 `config/coverage_params.yaml`。

## 2. RViz 使用方式

建议在 RViz 中准备两个工具：

- `2D Goal Pose`：用于录入巡检点
- `Publish Point`：用于录入推土区域

### 2.1 巡检点录入

巡检点通过 `2D Goal Pose` 发布到：

```text
/inspection/goal_pose

# 原始/planning/mission_planning/goal
```

消息类型：

```text
geometry_msgs/msg/PoseStamped
```

注意：

- 节点主要使用巡检点的位置信息 `x/y`。
- 巡检目标朝向会由巡检点序列自动计算。

朝向规则：

- 非最后一个巡检点：朝向指向下一个巡检点。
- 最后一个巡检点且 `loop_goals=true`：朝向指向第一个巡检点。
- 最后一个巡检点且不循环：朝向退化为前一个点到当前点的方向。
- 只有一个巡检点时：朝向退化为当前车辆朝向（若无里程计则为 `0`）。

### 2.2 推土区域录入

推土区域通过 `Publish Point` 发布到：

```text
/bulldoze/clicked_point
```

消息类型：

```text
geometry_msgs/msg/PointStamped
```

每 3 次点击定义 1 个矩形区域：

1. 第 1 点：矩形起点
2. 第 2 点：前进方向和长度
3. 第 3 点：宽度和左右侧

说明：

- 每完成 3 次点击会生成一组推土目标路径。
- 当前 `cover_wyh` 维护单个活动推土区域（新 3 点会覆盖并重建当前推土路径）。

## 3. 任务切换指令

任务切换 topic：

```text
/planning/task_command
```

消息类型：

```text
std_msgs/msg/String
```

### 3.1 切换到推土任务

```bash
ros2 topic pub --once /planning/task_command std_msgs/msg/String "{data: 'bulldoze'}"
```

### 3.2 恢复巡检任务

```bash
ros2 topic pub --once /planning/task_command std_msgs/msg/String "{data: 'inspection'}"
```

### 3.3 清空任务

清空巡检点：

```bash
ros2 topic pub --once /planning/task_command std_msgs/msg/String "{data: 'clear_inspection'}"
```

清空推土区域：

```bash
ros2 topic pub --once /planning/task_command std_msgs/msg/String "{data: 'clear_bulldoze'}"
```

全部清空：

```bash
ros2 topic pub --once /planning/task_command std_msgs/msg/String "{data: 'clear_all'}"
```

## 4. 路由下发接口

该节点通过 Autoware 路由服务下发目标，不直接发布 `/planning/mission_planning/goal`：

- 设定目标服务：

```text
/api/routing/set_route_points
```

- 清除路由服务：

```text
/api/routing/clear_route
```

- 路由状态订阅：

```text
/api/routing/state
```

推土和巡检两种模式都会走同一套路由服务流程。

## 5. 可视化

节点会发布两类可视化数据：

### 5.1 MarkerArray

topic：

```text
/coverage_markers
```

消息类型：

```text
visualization_msgs/msg/MarkerArray
```

显示内容包括：

- 推土矩形边框
- 推土模板路径与目标点
- 巡检路径线与巡检点
- 当前激活目标箭头

### 5.2 PoseArray

topic：

```text
/coverage_pose_array
```

消息类型：

```text
geometry_msgs/msg/PoseArray
```

该 topic 发布当前激活任务（巡检或推土）的目标姿态数组，用于调试。

## 6. 典型使用流程

1. 启动节点
2. 在 RViz 中设置 Fixed Frame 为 `map`
3. 用 `2D Goal Pose` 录入巡检点（`/inspection/goal_pose`）
4. 用 `Publish Point` 每 3 次点击录入推土区域（`/bulldoze/clicked_point`）
5. 默认巡检运行
6. 发送 `bulldoze` 指令切换到推土
7. 推土结束后自动返回推土前的巡检点继续巡检

## 7. 主要参数

参数默认值见 `config/coverage_params.yaml`。

常用参数如下：

- `frame_id`：地图坐标系，默认 `map`
- `inspection_pose_topic`：巡检点输入 topic，默认 `/inspection/goal_pose`
- `bulldoze_clicked_point_topic`：推土区域输入 topic，默认 `/bulldoze/clicked_point`
- `task_command_topic`：任务切换指令 topic，默认 `/planning/task_command`
- `odom_topic`：里程计输入 topic，默认 `/localization/kinematic_state`
- `route_state_topic`：路由状态 topic，默认 `/api/routing/state`
- `set_route_service`：设置路由服务，默认 `/api/routing/set_route_points`
- `clear_route_service`：清路由服务，默认 `/api/routing/clear_route`
- `loop_goals`：巡检点是否循环执行
- `margin`：推土区域边界内缩距离
- `desired_spacing`：推土模板点间距
- `allow_goal_modification`：调用路由服务时是否允许目标修改
- `use_clicked_points`：是否通过 RViz 点击输入数据

## 8. 参数模式说明

默认使用点击录入模式：

```yaml
use_clicked_points: true
```

如果改成：

```yaml
use_clicked_points: false
```

则节点会使用参数中的 `origin_x/origin_y/rect_width/rect_length` 初始化推土区域。
