# autoware_short_goal_pivot_coordinator

Minimal coordinator node that adds a near-goal pivot stage on top of an existing trajectory follower command stream.

## What it does

- Subscribes trajectory, odometry, and an input control command.
- Runs a tiny state machine:
  - `TRACK` (pass-through input command)
  - `PIVOT_ALIGN` (set `v=0`, output yaw-rate command in `steering_tire_angle`)
- Publishes coordinated control command.

## Topics

- Input control cmd: `/control/trajectory_follower/control_cmd`
- Output control cmd: `/control/trajectory_follower/control_cmd_pivot`
- Trajectory: `/planning/scenario_planning/trajectory`
- Ego odometry: `/localization/kinematic_state`

## Run

```bash
ros2 launch autoware_short_goal_pivot_coordinator short_goal_pivot_coordinator.launch.xml
```

## Important

This minimal node assumes your control chain uses diff-style semantics where `lateral.steering_tire_angle` can carry yaw-rate during pivot stage.
If your chain is pure Ackermann semantic, this will not realize true in-place rotation.
