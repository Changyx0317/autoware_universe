# cover_wyh

一个基于 ROS 2 的巡检 + 推土任务规划节点。

当前版本支持两类任务：

- 巡检任务：在地图上录入多个巡检点，节点按顺序循环发布巡检目标。
- 推土任务：在地图上用 3 次点击定义一个矩形推土区域，节点会按模板生成该区域内的目标点。

执行逻辑如下：

- 默认执行巡检任务。
- 收到推土指令后，切换到距离车辆当前位置最近的推土区域执行。
- 推土区域执行完成后，自动回到距离当前位置最近的巡检点，继续巡检。

## 1. 构建与启动

在工作区根目录执行：

```bash
colcon build --packages-select cover_wyh
source install/setup.bash
ros2 launch cover_wyh coverage.launch.py
```

启动文件位于 [launch/coverage.launch.py](/home/wyh/cover_wyh/launch/coverage.launch.py)。

参数文件位于 [config/coverage_params.yaml](/home/wyh/cover_wyh/config/coverage_params.yaml)。

## 2. RViz 使用方式

建议在 RViz 中准备两个工具：

- `2D Goal Pose`：用于录入巡检点
- `Publish Point`：用于录入推土区域

### 2.1 巡检点录入

巡检点通过 `2D Goal Pose` 发布到：

```text
/inspection/goal_pose
```

消息类型：

```text
geometry_msgs/msg/PoseStamped
```

注意：

- 节点只使用巡检点的位置信息 `x/y`
- 你在 RViz 中拖出来的朝向不会直接作为最终目标朝向
- 巡检目标真正发布时，姿态由前后两个巡检点的连线自动计算

朝向规则：

- 非最后一个巡检点：朝向指向下一个巡检点
- 最后一个巡检点且 `loop_goals=true`：朝向指向第一个巡检点
- 最后一个巡检点且不循环：朝向退化为前一个点到当前点的方向
- 只有一个巡检点时：朝向退化为当前车辆朝向，如果没有里程计则为 `0`

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

- 每完成 3 次点击，就会新增 1 个推土区域
- 支持定义多个推土区域
- 各推土区域都会保留，不会覆盖之前的区域

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

收到指令后，节点会：

- 读取当前车辆位置
- 在所有推土区域中选择最近的一个
- 从该推土区域的第一个模板点开始执行

### 3.2 恢复巡检任务

```bash
ros2 topic pub --once /planning/task_command std_msgs/msg/String "{data: 'inspection'}"
```

收到该指令后，节点会回到距离当前位置最近的巡检点继续执行。

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

## 4. 目标输出

节点发布统一目标到：

```text
/planning/mission_planning/goal
```

消息类型：

```text
geometry_msgs/msg/PoseStamped
```

巡检模式和推土模式都会通过这个 topic 输出目标点。

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

- 巡检路径线
- 巡检点
- 所有推土矩形边框
- 各推土区域的模板路径
- 各推土区域的目标点
- 当前激活目标箭头

在 RViz 中添加一个 `MarkerArray` Display，topic 设为 `/coverage_markers` 即可。

### 5.2 PoseArray

topic：

```text
/coverage_pose_array
```

消息类型：

```text
geometry_msgs/msg/PoseArray
```

这个 topic 发布当前激活任务序列的姿态数组，主要用于调试。

在 RViz 中可以额外添加一个 `PoseArray` Display，topic 设为 `/coverage_pose_array`。

通常情况下，只看 `/coverage_markers` 就够用了。

## 6. 典型使用流程

1. 启动节点
2. 在 RViz 中加载地图，Fixed Frame 设为 `map`
3. 用 `2D Goal Pose` 在地图上依次录入多个巡检点，topic 设为 `/inspection/goal_pose`
4. 用 `Publish Point` 在地图上每 3 次点击定义一个推土区域，topic 设为 `/bulldoze/clicked_point`
5. 节点默认开始巡检
6. 需要推土时，发送 `bulldoze` 指令
7. 推土完成后，节点自动回到最近巡检点继续巡检

## 7. 主要参数

参数文件默认值见 [config/coverage_params.yaml](/home/wyh/cover_wyh/config/coverage_params.yaml)。

常用参数如下：

- `frame_id`：地图坐标系，默认 `map`
- `inspection_pose_topic`：巡检点输入 topic，默认 `/inspection/goal_pose`
- `bulldoze_clicked_point_topic`：推土区域输入 topic，默认 `/bulldoze/clicked_point`
- `task_command_topic`：任务切换指令 topic，默认 `/planning/task_command`
- `goal_topic`：目标输出 topic，默认 `/planning/mission_planning/goal`
- `odom_topic`：里程计输入 topic，默认 `/localization/kinematic_state`
- `margin`：推土区域边界内缩距离
- `desired_spacing`：推土模板点间距
- `goal_reach_tolerance_m`：位置到点容差
- `goal_yaw_tolerance_rad`：航向到点容差
- `stop_speed_tolerance_mps`：停车速度阈值
- `require_goal_yaw_alignment`：是否要求到点时航向也满足阈值
- `require_stop_at_goal`：是否要求到点时车速低于停车阈值
- `goal_settle_time_s`：到点稳定等待时间
- `republish_goal_interval_s`：兼容保留参数，当前版本不再周期性重发当前 goal
- `loop_goals`：巡检点是否循环执行

Autoware 联调时，通常建议先使用下面这组更宽松的参数，避免车辆经过中间点但因为没有完全停车或没有精确对正航向而无法切换下一个目标：

```yaml
goal_reach_tolerance_m: 0.5
require_goal_yaw_alignment: false
require_stop_at_goal: false
goal_settle_time_s: 0.0
```

## 8. 参数模式说明

默认使用点击录入模式：

```yaml
use_clicked_points: true
```

如果改成：

```yaml
use_clicked_points: false
```

节点会从参数 `origin_x`、`origin_y`、`rect_width`、`rect_length` 读取一个推土矩形并生成模板点。

注意：

- 该模式下主要用于预置一个推土区域
- 该模式下不会创建 RViz 点选输入订阅
- 默认只会加载参数给出的一个推土区域
- 如果需要巡检点输入，建议保持 `use_clicked_points: true`

## 9. 当前限制

- 巡检点录入时，`2D Goal Pose` 的输入姿态不会被直接使用
- 推土指令会选择最近推土区域，但执行时仍从该区域模板路径的第一个点开始
- 当前没有为 RViz 单独提供自定义插件，依赖标准 `2D Goal Pose` 和 `Publish Point` 工具
