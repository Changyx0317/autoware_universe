# autoware_simple_planning_simulator 差速改动记录

本文档记录本次在 `autoware_simple_planning_simulator` 内直接接入差速模型（DIFF_DRIVE_VEL）的最小改动，具体到文件与代码点。

## 1. 新增文件

### 1.1 差速模型头文件
- 文件：`include/autoware/simple_planning_simulator/vehicle_model/sim_model_diff_drive_vel.hpp`
- 行数：56
- 作用：声明 `SimModelDiffDriveVel`，定义差速状态/输入维度：
  - 状态 `dim_x=3`: `x, y, yaw`
  - 输入 `dim_u=2`: `vx_des, wz_des`

### 1.2 差速模型实现文件
- 文件：`src/simple_planning_simulator/vehicle_model/sim_model_diff_drive_vel.cpp`
- 行数：77
- 作用：实现差速运动学：
  - `x_dot = vx * cos(yaw)`
  - `y_dot = vx * sin(yaw)`
  - `yaw_dot = wz`
- 额外逻辑：
  - 使用 `max_yaw_rate_rps` 对角速度做限幅
  - `getSteer()` 固定返回 `0.0`（差速无转向轮角）

## 2. 现有文件改动

### 2.1 CMake 接入新模型源文件
- 文件：`CMakeLists.txt`
- 代码点：`#L25`
- 改动：在 `ament_auto_add_library(...)` 中新增：
  - `src/simple_planning_simulator/vehicle_model/sim_model_diff_drive_vel.cpp`

### 2.2 模型聚合头接入新模型
- 文件：`include/autoware/simple_planning_simulator/vehicle_model/sim_model.hpp`
- 代码点：`#L24`
- 改动：新增 include：
  - `#include "autoware/simple_planning_simulator/vehicle_model/sim_model_diff_drive_vel.hpp"`

### 2.3 VehicleModelType 新增枚举值
- 文件：`include/autoware/simple_planning_simulator/simple_planning_simulator_core.hpp`
- 代码点：`#L229`
- 改动：新增：
  - `DIFF_DRIVE_VEL = 13`

### 2.4 Core: 新增参数 + 模型分支
- 文件：`src/simple_planning_simulator/simple_planning_simulator_core.cpp`
- 代码点：
  - `#L244` 新增参数：`diff_max_yaw_rate_rps`
  - `#L258-#L260` 新增模型选择分支：
    - 当 `vehicle_model_type_str == "DIFF_DRIVE_VEL"` 时，实例化 `SimModelDiffDriveVel`

### 2.5 Core: 输入分支接入 DIFF_DRIVE_VEL
- 文件：`src/simple_planning_simulator/simple_planning_simulator_core.cpp`
- 代码点：`#L636`
- 改动：`set_input(const Control & cmd, ...)` 中把 `DIFF_DRIVE_VEL` 纳入 `input << vel, steer` 分支
- 当前语义：
  - `cmd.longitudinal.velocity` -> `vx`
  - `cmd.lateral.steering_tire_angle` -> `wz`

### 2.6 Core: 初始状态分支接入 DIFF_DRIVE_VEL
- 文件：`src/simple_planning_simulator/simple_planning_simulator_core.cpp`
- 代码点：`#L737`
- 改动：`set_initial_state(...)` 中把 `DIFF_DRIVE_VEL` 纳入 `state << x, y, yaw` 分支

## 3. 参数文件策略（已合并为单文件）

- 文件：`param/simple_planning_simulator_default.param.yaml`
- 关键参数：
  - `vehicle_model_type: "DIFF_DRIVE_VEL"`
  - `diff_max_yaw_rate_rps: 1.2`
- 说明：
  - 已将差速参数合并进官方默认参数文件
  - 已删除独立文件 `simple_planning_simulator_diff_drive.param.yaml`

## 4. 编译验证

- 已执行：
  - `colcon build --packages-select autoware_simple_planning_simulator`
- 结果：编译通过（仅有既有的 header install 提示，无新增编译错误）

## 5. 使用方式（默认即差速）

`simple_planning_simulator.launch.py` 的默认参数文件本来就指向：
- `param/simple_planning_simulator_default.param.yaml`

由于该文件已改为差速配置，因此不需要额外覆盖参数文件路径。

## 6. 将“差速模型”改为默认模式（仅改本包，具体到文件与行）

说明：以下行号基于当前仓库版本（2026-03-25）。后续若文件前部新增内容，行号会顺延。

### 6.1 方案：直接把 simple_planning_simulator 的默认参数改成差速

1) 修改仿真器默认参数文件  
- 文件：`/home/chang/autoware/src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/param/simple_planning_simulator_default.param.yaml`
- 行：`#L5`
- 现状：
  - `vehicle_model_type: "DELAY_STEER_ACC_GEARED"`
- 修改为：
  - `vehicle_model_type: "DIFF_DRIVE_VEL"`

2) 在同文件增加差速参数  
- 文件：同上
- 建议插入位置：`#L17` 后
- 新增：
  - `diff_max_yaw_rate_rps: 1.2`

3) 该方案影响范围  
- 文件：`/home/chang/autoware/src/universe/autoware_universe/simulator/autoware_simple_planning_simulator/launch/simple_planning_simulator.launch.py`
- 行：`#L160-#L164`
- 说明：这里把 `simulator_model_param_file` 默认值设为 `simple_planning_simulator_default.param.yaml`。  
  所以你改了 default 参数文件，就等于所有未显式覆盖该参数的启动都默认差速。

4) 整套 `planning_simulator.launch.xml` 启动时的同步修改（必须）  
- 文件：`/home/chang/autoware/src/launcher/autoware_launch/vehicle/sample_vehicle_launch/sample_vehicle_description/config/simulator_model.param.yaml`
- 行：`#L5`
- 现状：
  - `vehicle_model_type: "DELAY_STEER_ACC_GEARED"`
- 修改为：
  - `vehicle_model_type: "DIFF_DRIVE_VEL"`
- 说明：
  - 整套启动时，上层会把该文件作为 simulator 参数传入，覆盖本包 default 参数。  
  - 因此若只改本包 default、不改该文件，整套启动仍可能是轮式模型。

## 7. 启动命令（单文件默认差速）

命令：

```bash
source /opt/ros/humble/setup.bash
source /home/chang/autoware/install/setup.bash

ros2 launch autoware_simple_planning_simulator simple_planning_simulator.launch.py \
  vehicle_info_param_file:=/home/chang/autoware/install/sample_vehicle_description/share/sample_vehicle_description/config/vehicle_info.param.yaml \
  input_ackermann_control_command:=/control/command/control_cmd \
  initial_engage_state:=true \
  motion_publish_mode:=full_motion
```

说明：按“最小化修改”原则，已删除外部新增的 `diff_vehicle_launch`，不再提供“通过新增车辆包切换”的路径。
