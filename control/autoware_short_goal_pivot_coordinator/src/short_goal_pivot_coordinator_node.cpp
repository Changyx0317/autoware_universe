#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

namespace
{
double normalize_angle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}
}  // namespace

class ShortGoalPivotCoordinator : public rclcpp::Node
{
public:
  ShortGoalPivotCoordinator() : Node("short_goal_pivot_coordinator")
  {
    input_control_cmd_topic_ =
      declare_parameter<std::string>("input_control_cmd_topic", "/control/trajectory_follower/control_cmd");
    output_control_cmd_topic_ = declare_parameter<std::string>(
      "output_control_cmd_topic", "/control/trajectory_follower/control_cmd_pivot");
    trajectory_topic_ =
      declare_parameter<std::string>("trajectory_topic", "/planning/scenario_planning/trajectory");
    kinematic_state_topic_ =
      declare_parameter<std::string>("kinematic_state_topic", "/localization/kinematic_state");

    enable_short_goal_pivot_ = declare_parameter<bool>("enable_short_goal_pivot", true);
    short_goal_pivot_trigger_distance_m_ =
      declare_parameter<double>("short_goal_pivot_trigger_distance_m", 2.0);
    pivot_enter_yaw_error_rad_ = declare_parameter<double>("pivot_enter_yaw_error_rad", 0.6);
    pivot_exit_yaw_error_rad_ = declare_parameter<double>("pivot_exit_yaw_error_rad", 0.15);
    pivot_kp_ = declare_parameter<double>("pivot_kp", 1.0);
    pivot_max_w_radps_ = declare_parameter<double>("pivot_max_w_radps", 0.4);
    pivot_timeout_s_ = declare_parameter<double>("pivot_timeout_s", 5.0);

    stop_speed_threshold_mps_ = declare_parameter<double>("stop_speed_threshold_mps", 0.03);
    stop_yaw_rate_threshold_rps_ = declare_parameter<double>("stop_yaw_rate_threshold_rps", 0.1);
    stop_settle_time_s_ = declare_parameter<double>("stop_settle_time_s", 0.3);

    pub_control_ = create_publisher<autoware_control_msgs::msg::Control>(output_control_cmd_topic_, 10);
    pub_goal_distance_ = create_publisher<std_msgs::msg::Float64>("~/debug/goal_distance_m", 10);
    pub_goal_yaw_error_ = create_publisher<std_msgs::msg::Float64>("~/debug/goal_yaw_error_rad", 10);
    pub_state_ = create_publisher<std_msgs::msg::Int32>("~/debug/state", 10);

    sub_control_ = create_subscription<autoware_control_msgs::msg::Control>(
      input_control_cmd_topic_, 10,
      std::bind(&ShortGoalPivotCoordinator::on_control_cmd, this, std::placeholders::_1));
    sub_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
      trajectory_topic_, 10,
      std::bind(&ShortGoalPivotCoordinator::on_trajectory, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      kinematic_state_topic_, 10,
      std::bind(&ShortGoalPivotCoordinator::on_odom, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "short_goal_pivot_coordinator started: in='%s' out='%s'", input_control_cmd_topic_.c_str(), output_control_cmd_topic_.c_str());
  }

private:
  enum class State : int32_t { TRACK = 0, PIVOT_ALIGN = 1 };

  void on_trajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
  {
    last_traj_ = msg;
  }

  void on_odom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    last_odom_ = msg;

    const double v = std::abs(msg->twist.twist.linear.x);
    const double w = std::abs(msg->twist.twist.angular.z);
    const bool stop_now = (v <= stop_speed_threshold_mps_) && (w <= stop_yaw_rate_threshold_rps_);
    const rclcpp::Time now = now_ros();

    if (stop_now) {
      if (!stop_candidate_active_) {
        stop_candidate_active_ = true;
        stop_candidate_start_ = now;
      }
    } else {
      stop_candidate_active_ = false;
    }
  }

  void on_control_cmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg)
  {
    if (!last_odom_ || !last_traj_ || last_traj_->points.empty()) {
      pub_control_->publish(*msg);
      return;
    }

    const auto & goal = last_traj_->points.back().pose.position;
    const auto & ego_pose = last_odom_->pose.pose;
    const double ego_yaw = yaw_from_quaternion(ego_pose.orientation);

    const double dx = goal.x - ego_pose.position.x;
    const double dy = goal.y - ego_pose.position.y;
    const double goal_dist = std::hypot(dx, dy);
    const double yaw_to_goal = std::atan2(dy, dx);
    const double yaw_err = normalize_angle(yaw_to_goal - ego_yaw);

    publish_debug(goal_dist, yaw_err);

    const rclcpp::Time now = now_ros();
    const bool is_stopped = stop_candidate_active_ &&
                            ((now - stop_candidate_start_).seconds() >= stop_settle_time_s_);

    if (state_ == State::TRACK) {
      if (
        enable_short_goal_pivot_ && is_stopped &&
        (goal_dist <= short_goal_pivot_trigger_distance_m_) &&
        (std::abs(yaw_err) >= pivot_enter_yaw_error_rad_))
      {
        state_ = State::PIVOT_ALIGN;
        pivot_enter_time_ = now;
      }
    } else {
      const bool yaw_done = std::abs(yaw_err) <= pivot_exit_yaw_error_rad_;
      const bool timeout = (now - pivot_enter_time_).seconds() >= pivot_timeout_s_;
      if (yaw_done || timeout) {
        state_ = State::TRACK;
      }
    }

    autoware_control_msgs::msg::Control out = *msg;
    if (state_ == State::PIVOT_ALIGN) {
      const double w_cmd = std::clamp(pivot_kp_ * yaw_err, -pivot_max_w_radps_, pivot_max_w_radps_);
      out.longitudinal.velocity = 0.0F;
      out.longitudinal.acceleration = 0.0F;
      // NOTE: this assumes diff-style semantic chain where steering_tire_angle carries yaw-rate command.
      out.lateral.steering_tire_angle = static_cast<float>(w_cmd);
      out.lateral.steering_tire_rotation_rate = 0.0F;
      out.lateral.is_defined_steering_tire_rotation_rate = false;
    }

    pub_state();
    pub_control_->publish(out);
  }

  rclcpp::Time now_ros() { return this->get_clock()->now(); }

  void publish_debug(const double goal_dist, const double yaw_err)
  {
    std_msgs::msg::Float64 dist_msg;
    dist_msg.data = goal_dist;
    pub_goal_distance_->publish(dist_msg);

    std_msgs::msg::Float64 yaw_msg;
    yaw_msg.data = yaw_err;
    pub_goal_yaw_error_->publish(yaw_msg);
  }

  void pub_state()
  {
    std_msgs::msg::Int32 s;
    s.data = static_cast<int32_t>(state_);
    pub_state_->publish(s);
  }

  std::string input_control_cmd_topic_;
  std::string output_control_cmd_topic_;
  std::string trajectory_topic_;
  std::string kinematic_state_topic_;

  bool enable_short_goal_pivot_{};
  double short_goal_pivot_trigger_distance_m_{};
  double pivot_enter_yaw_error_rad_{};
  double pivot_exit_yaw_error_rad_{};
  double pivot_kp_{};
  double pivot_max_w_radps_{};
  double pivot_timeout_s_{};
  double stop_speed_threshold_mps_{};
  double stop_yaw_rate_threshold_rps_{};
  double stop_settle_time_s_{};

  State state_{State::TRACK};
  rclcpp::Time pivot_enter_time_{0, 0, RCL_ROS_TIME};

  bool stop_candidate_active_{false};
  rclcpp::Time stop_candidate_start_{0, 0, RCL_ROS_TIME};

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr last_traj_;
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_;

  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr pub_control_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_goal_distance_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_goal_yaw_error_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_state_;

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_control_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr sub_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShortGoalPivotCoordinator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
