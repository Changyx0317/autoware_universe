# 在新的 Autoware 中配置 DWA（实操指南）

本文说明如何把 `autoware_dwa_trajectory_follower` 迁移到一个新的 Autoware，并让系统可稳定运行。

## 1. 目标与方案

本文仅保留集成式方案：把 DWA 接入控制链，使用 `control_module_preset:=dwa_tracking_minimal` 启动。

## 2. 必备文件（DWA 包内）

确保以下文件存在并可编译：

- `src/universe/autoware_universe/control/autoware_dwa_trajectory_follower`
- `config/dwa_trajectory_follower.param.yaml`

## 3. 集成式（推荐）

### 3.1 需要在新 Autoware 中准备的外部文件

1. `autoware_launch/config/control/preset/dwa_tracking_minimal_preset.yaml`
2. `autoware_launch/config/control/trajectory_follower/dwa_trajectory_follower.param.yaml`
3. `autoware_launch/launch/components/tier4_control_component.launch.xml`  
需要把 `dwa_trajectory_follower_param_path` 传给 `tier4_control_launch`。
4. `autoware_launch/launch/planning_simulator.launch.xml`  
需要支持 `control_module_preset==dwa_tracking_minimal` 时仿真器输入用  
`/control/trajectory_follower/control_cmd`。
5. `tier4_control_launch/launch/control.launch.xml`  
需要有 `trajectory_follower_mode == dwa_trajectory_follower` 分支并启动 DWA 节点。
6. `tier4_control_launch/package.xml`  
需要 `exec_depend` 包含 `autoware_dwa_trajectory_follower`。

### 3.1.1 这 6 个文件的具体改动（含源码）

1) `autoware_launch/config/control/preset/dwa_tracking_minimal_preset.yaml`  
用途：新增 DWA 控制预设，让 launch 一键切换到 DWA 控制器。

```yaml
launch:
  - arg:
      name: trajectory_follower_mode
      default: dwa_trajectory_follower
```

2) `autoware_launch/config/control/trajectory_follower/dwa_trajectory_follower.param.yaml`  
用途：新增 DWA 参数文件，并由 launch 传入 DWA 节点。该文件来自 DWA 包配置并放到 autoware_launch 下供统一管理。

```yaml
/**:
  ros__parameters:
    control_rate_hz: 50.0
    goal_tolerance_m: 0.1

```

3) `autoware_launch/launch/components/tier4_control_component.launch.xml`  
用途：把 DWA 参数路径传给 `tier4_control_launch`。

```xml
<arg name="dwa_trajectory_follower_param_path"
     value="$(find-pkg-share autoware_launch)/config/control/trajectory_follower/dwa_trajectory_follower.param.yaml"/>
```

4) `autoware_launch/launch/planning_simulator.launch.xml`  
用途：在仿真中，当使用 `dwa_tracking_minimal` 时，把模拟器控制输入切到 DWA 输出。

```xml
<let
  name="simulator_input_ackermann_control_command"
  value="/control/trajectory_follower/control_cmd"
  if="$(eval '&quot;$(var control_module_preset)&quot;==&quot;dwa_tracking_minimal&quot;')"
/>
<let
  name="simulator_input_ackermann_control_command"
  value="/control/command/control_cmd"
  unless="$(eval '&quot;$(var control_module_preset)&quot;==&quot;dwa_tracking_minimal&quot;')"
/>
...
<arg name="input_ackermann_control_command" value="$(var simulator_input_ackermann_control_command)"/>
```

5) `tier4_control_launch/launch/control.launch.xml`  
用途：在控制 launch 中新增 DWA 分支，并声明 DWA 参数路径入参。

```xml
<arg name="dwa_trajectory_follower_param_path"/>
...
<group if="$(eval &quot;'$(var trajectory_follower_mode)' == 'dwa_trajectory_follower'&quot;)">
  <node pkg="autoware_dwa_trajectory_follower" exec="dwa_trajectory_follower_node"
        name="controller_node_exe" namespace="trajectory_follower" output="screen">
    <param from="$(var dwa_trajectory_follower_param_path)"/>
    <param from="$(var vehicle_param_file)"/>
  </node>
</group>
```

6) `tier4_control_launch/package.xml`  
用途：补依赖，确保控制 launch 能找到 DWA 包。

```xml
<exec_depend>autoware_dwa_trajectory_follower</exec_depend>
```

### 3.2 编译

```bash
cd /home/chang/autoware
source /opt/ros/humble/setup.bash
colcon build --packages-select autoware_launch tier4_control_launch autoware_dwa_trajectory_follower
source /home/chang/autoware/install/setup.bash
```

### 3.3 启动

```bash
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=/home/chang/autoware_map/sample-map-planning \
  control_module_preset:=dwa_tracking_minimal \
  launch_planning:=true \
  launch_control:=true \
  rviz:=true
```

### 3.4 验证

```bash
ros2 node list | grep /control/trajectory_follower/controller_node_exe
ros2 topic hz /control/trajectory_follower/control_cmd
ros2 topic echo /control/trajectory_follower/control_cmd --once
```

## 4. 实车建议

实车建议使用方案 1（集成式），保留 gate 与操作模式链路，不建议长期绕过 gate。

## 5. 常见问题

1. 报错：`dwa_tracking_minimal_preset.yaml` 不存在  
说明外部集成文件未补齐，按“方案 1”补文件并重编译。

2. DWA 有输出但车不动  
先看：

```bash
ros2 topic echo /control/trajectory_follower/control_cmd --once
ros2 topic echo /control/command/control_cmd --once
```

若前者非零后者为零，说明被 gate 拦截（模式/engage/emergency）。

3. `controller_node_exe` 不存在  
检查 `trajectory_follower_mode` 是否切到 `dwa_trajectory_follower`，以及 `control_module_preset` 是否为 `dwa_tracking_minimal`。
