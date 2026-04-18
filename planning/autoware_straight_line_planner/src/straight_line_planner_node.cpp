#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

class StraightLinePlannerNode : public rclcpp::Node
{
public:
  StraightLinePlannerNode()
  : Node("straight_line_planner_node")
  {
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/planning/mission_planning/goal");
    fallback_goal_topic_ = declare_parameter<std::string>(
      "fallback_goal_topic", "/planning/mission_planning/echo_back_goal_pose");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/localization/kinematic_state");
    output_trajectory_topic_ = declare_parameter<std::string>(
      "output_trajectory_topic", "/planning/scenario_planning/trajectory");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    point_interval_m_ = declare_parameter<double>("point_interval_m", 0.5);
    min_num_points_ = declare_parameter<int>("min_num_points", 5);

    target_speed_mps_ = declare_parameter<double>("target_speed_mps", 1.0);
    reverse_target_speed_mps_ = declare_parameter<double>("reverse_target_speed_mps", 1.0);
    stop_at_goal_ = declare_parameter<bool>("stop_at_goal", true);
    enable_reverse_when_goal_behind_ =
      declare_parameter<bool>("enable_reverse_when_goal_behind", true);
    reverse_dot_threshold_ = declare_parameter<double>("reverse_dot_threshold", 0.0);

    trajectory_pub_ = create_publisher<autoware_planning_msgs::msg::Trajectory>(
      output_trajectory_topic_, rclcpp::QoS{1});

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS{10},
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        latest_odom_ = *msg;
        // Handle goal-first race: if goal arrived before first odom, rebuild as soon as odom is ready.
        if (latest_goal_ && !latest_trajectory_) {
          rebuild_trajectory();
        }
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::QoS{1},
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        on_goal(*msg, "goal_topic");
      });

    fallback_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      fallback_goal_topic_, rclcpp::QoS{1},
      [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        on_goal(*msg, "fallback_goal_topic");
      });

    const double period_sec = 1.0 / std::max(1e-3, publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_sec),
      [this]() {
        if (latest_trajectory_) {
          latest_trajectory_->header.stamp = now();
          trajectory_pub_->publish(*latest_trajectory_);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "No trajectory to publish yet. Waiting for odom + goal.");
        }
      });

    RCLCPP_INFO(
      get_logger(),
      "StraightLinePlanner started. goal_topic=%s, odom_topic=%s, output=%s, speed=%.3f m/s",
      goal_topic_.c_str(), odom_topic_.c_str(), output_trajectory_topic_.c_str(), target_speed_mps_);
  }

private:
  void on_goal(const geometry_msgs::msg::PoseStamped & msg, const char * source)
  {
    latest_goal_ = msg;
    RCLCPP_INFO(
      get_logger(), "Received goal from %s: frame=%s (%.2f, %.2f, %.2f)", source,
      msg.header.frame_id.c_str(), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    rebuild_trajectory();
  }

  static geometry_msgs::msg::Quaternion create_quat_from_yaw(const double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    q.normalize();
    return tf2::toMsg(q);
  }

  void rebuild_trajectory()
  {
    if (!latest_goal_) {
      return;
    }
    if (!latest_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Goal received but odometry is not ready. Waiting for odom...");
      return;
    }

    const auto & start = latest_odom_->pose.pose;
    const auto & goal = latest_goal_->pose;

    const double dx = goal.position.x - start.position.x;
    const double dy = goal.position.y - start.position.y;
    const double dz = goal.position.z - start.position.z;
    const double dist = std::hypot(dx, dy);

    const double interval = std::max(0.01, point_interval_m_);
    const int min_points = std::max(3, min_num_points_);
    const int n_segments = std::max(min_points - 1, static_cast<int>(std::ceil(dist / interval)));

    const double yaw_line = (dist > 1e-6) ? std::atan2(dy, dx) : 0.0;
    const double current_yaw = tf2::getYaw(start.orientation);
    const double heading_dot =
      std::cos(current_yaw) * std::cos(yaw_line) + std::sin(current_yaw) * std::sin(yaw_line);
    const bool use_reverse =
      enable_reverse_when_goal_behind_ && dist > 1e-6 && heading_dot < reverse_dot_threshold_;
    const double speed_cmd =
      use_reverse ? -std::abs(reverse_target_speed_mps_) : std::abs(target_speed_mps_);
    // In reverse, keep vehicle heading opposite to trajectory geometric direction.
    const double heading_yaw = use_reverse ? (yaw_line + M_PI) : yaw_line;

    autoware_planning_msgs::msg::Trajectory traj;
    traj.header.stamp = now();
    if (!latest_goal_->header.frame_id.empty()) {
      traj.header.frame_id = latest_goal_->header.frame_id;
    } else if (!latest_odom_->header.frame_id.empty()) {
      traj.header.frame_id = latest_odom_->header.frame_id;
    } else {
      traj.header.frame_id = "map";
    }
    traj.points.reserve(static_cast<size_t>(n_segments + 1));

    for (int i = 0; i <= n_segments; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n_segments);

      autoware_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x = start.position.x + t * dx;
      p.pose.position.y = start.position.y + t * dy;
      p.pose.position.z = start.position.z + t * dz;
      p.pose.orientation = create_quat_from_yaw(heading_yaw);

      p.longitudinal_velocity_mps = static_cast<float>(speed_cmd);
      p.lateral_velocity_mps = 0.0F;
      p.acceleration_mps2 = 0.0F;
      p.heading_rate_rps = 0.0F;
      p.front_wheel_angle_rad = 0.0F;
      p.rear_wheel_angle_rad = 0.0F;

      traj.points.push_back(p);
    }

    if (!traj.points.empty()) {
      traj.points.back().pose.position = goal.position;
      // Keep the final point orientation as the user-requested goal yaw so that
      // controller-side final yaw alignment works for both forward and reverse.
      traj.points.back().pose.orientation = goal.orientation;
      if (stop_at_goal_) {
        traj.points.back().longitudinal_velocity_mps = 0.0F;
      }
    }

    latest_trajectory_ = traj;
    RCLCPP_INFO(
      get_logger(),
      "Published straight trajectory: points=%zu, dist=%.2f m, speed=%.2f m/s, reverse=%s "
      "(dot=%.3f)",
      latest_trajectory_->points.size(), dist, speed_cmd, use_reverse ? "true" : "false",
      heading_dot);
  }

  std::string goal_topic_;
  std::string fallback_goal_topic_;
  std::string odom_topic_;
  std::string output_trajectory_topic_;

  double publish_rate_hz_{};
  double point_interval_m_{};
  int min_num_points_{};

  double target_speed_mps_{};
  double reverse_target_speed_mps_{};
  bool stop_at_goal_{};
  bool enable_reverse_when_goal_behind_{};
  double reverse_dot_threshold_{};

  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fallback_goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<nav_msgs::msg::Odometry> latest_odom_;
  std::optional<geometry_msgs::msg::PoseStamped> latest_goal_;
  std::optional<autoware_planning_msgs::msg::Trajectory> latest_trajectory_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StraightLinePlannerNode>());
  rclcpp::shutdown();
  return 0;
}
