# autoware_diff_drive_control_adapter

This node converts Autoware `autoware_control_msgs/msg/Control` command into diff-drive command semantics:

- `v = control.longitudinal.velocity`
- `w = control.lateral.steering_tire_angle` (interpreted as yaw rate in rad/s)

Outputs:

- `geometry_msgs/msg/TwistStamped` on `output_twist_stamped_topic`
- optional `geometry_msgs/msg/Twist` on `output_twist_topic`
- optional wheel angular speed `std_msgs/msg/Float64`:
  - left: `output_left_wheel_speed_topic`
  - right: `output_right_wheel_speed_topic`

Wheel speed model:

- `v_l = v - 0.5 * tread * w`
- `v_r = v + 0.5 * tread * w`
- `wl = v_l / radius`
- `wr = v_r / radius`

