#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <future>

#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"
#include "autoware_adapi_v1_msgs/srv/set_route_points.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

using autoware_adapi_v1_msgs::msg::RouteState;
using autoware_adapi_v1_msgs::srv::ClearRoute;
using autoware_adapi_v1_msgs::srv::SetRoutePoints;
using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;

enum class TaskMode
{
  Inspection,
  Bulldoze
};

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

Vec2 makeVec2(double x, double y)
{
  return Vec2{x, y};
}

Vec2 operator+(const Vec2 & a, const Vec2 & b)
{
  return Vec2{a.x + b.x, a.y + b.y};
}

Vec2 operator-(const Vec2 & a, const Vec2 & b)
{
  return Vec2{a.x - b.x, a.y - b.y};
}

Vec2 operator*(const Vec2 & a, double scale)
{
  return Vec2{a.x * scale, a.y * scale};
}

double dot(const Vec2 & a, const Vec2 & b)
{
  return a.x * b.x + a.y * b.y;
}

double norm(const Vec2 & a)
{
  return std::hypot(a.x, a.y);
}

Vec2 normalize(const Vec2 & a)
{
  const double length = norm(a);
  if (length <= 1e-9) {
    throw std::invalid_argument("vector length must be > 0");
  }
  return a * (1.0 / length);
}

Vec2 leftNormal(const Vec2 & a)
{
  return Vec2{-a.y, a.x};
}

geometry_msgs::msg::Point makePoint(double x, double y, double z = 0.0)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

geometry_msgs::msg::Point makePoint(const Vec2 & point, double z = 0.0)
{
  return makePoint(point.x, point.y, z);
}

geometry_msgs::msg::Pose makePose(double x, double y, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = std::sin(yaw * 0.5);
  pose.orientation.w = std::cos(yaw * 0.5);
  return pose;
}

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std::vector<double> makeUniformSamples(
  double min_value, double max_value, double desired_spacing)
{
  if (desired_spacing <= 0.0) {
    throw std::invalid_argument("desired_spacing must be > 0");
  }
  if (max_value < min_value) {
    throw std::invalid_argument("max_value must be >= min_value");
  }

  const double span = max_value - min_value;
  if (span <= 1e-9) {
    return {min_value};
  }

  const int sample_count =
    std::max(2, static_cast<int>(std::floor(span / desired_spacing)) + 1);
  const double actual_spacing = span / static_cast<double>(sample_count - 1);

  std::vector<double> samples;
  samples.reserve(static_cast<size_t>(sample_count));
  for (int i = 0; i < sample_count; ++i) {
    samples.push_back(min_value + actual_spacing * static_cast<double>(i));
  }
  return samples;
}

std::string toLowerCopy(const std::string & value)
{
  std::string lowered = value;
  std::transform(
    lowered.begin(), lowered.end(), lowered.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return lowered;
}

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

const char * routeStateToString(uint16_t state)
{
  switch (state) {
    case RouteState::UNKNOWN:
      return "UNKNOWN";
    case RouteState::UNSET:
      return "UNSET";
    case RouteState::SET:
      return "SET";
    case RouteState::ARRIVED:
      return "ARRIVED";
    case RouteState::CHANGING:
      return "CHANGING";
    default:
      return "UNRECOGNIZED";
  }
}

class RectBoundaryPlanner
{
public:
  RectBoundaryPlanner(
    const Vec2 & origin, const Vec2 & forward_unit, const Vec2 & lateral_unit, double forward_length,
    double lateral_length, double margin, double desired_spacing)
  : origin_(origin),
    forward_unit_(normalize(forward_unit)),
    lateral_unit_(normalize(lateral_unit)),
    forward_length_(forward_length),
    lateral_length_(lateral_length),
    margin_(margin),
    desired_spacing_(desired_spacing)
  {
    if (forward_length_ <= 0.0 || lateral_length_ <= 0.0) {
      throw std::invalid_argument("rectangle lengths must be > 0");
    }
    if (margin_ < 0.0) {
      throw std::invalid_argument("margin must be >= 0");
    }
    if (margin_ * 2.0 >= forward_length_ || margin_ * 2.0 >= lateral_length_) {
      throw std::invalid_argument("margin is too large for the rectangle");
    }
    if (desired_spacing_ <= 0.0) {
      throw std::invalid_argument("desired_spacing must be > 0");
    }
  }

  std::vector<Pose2D> generateBoundaryPath() const
  {
    const auto forward_yaw = std::atan2(forward_unit_.y, forward_unit_.x);
    const double near_distance = margin_;
    const double far_distance = forward_length_ - margin_;

    auto row_offsets = makeUniformSamples(margin_, lateral_length_ - margin_, desired_spacing_);

    std::vector<Pose2D> path;
    path.reserve(row_offsets.size() * 3 + 1);

    if (row_offsets.empty()) {
      return path;
    }

    Vec2 current_near_point =
      origin_ + forward_unit_ * near_distance + lateral_unit_ * row_offsets.front();
    path.push_back({current_near_point.x, current_near_point.y, forward_yaw});

    for (size_t i = 0; i < row_offsets.size(); ++i) {
      const double lateral_offset = row_offsets[i];
      const Vec2 far_point =
        origin_ + forward_unit_ * far_distance + lateral_unit_ * lateral_offset;

      path.push_back({far_point.x, far_point.y, forward_yaw});
      path.push_back({current_near_point.x, current_near_point.y, forward_yaw});

      if (i + 1 < row_offsets.size()) {
        current_near_point =
          origin_ + forward_unit_ * near_distance + lateral_unit_ * row_offsets[i + 1];
        path.push_back({current_near_point.x, current_near_point.y, forward_yaw});
      }
    }
    return path;
  }

private:
  Vec2 origin_;
  Vec2 forward_unit_;
  Vec2 lateral_unit_;
  double forward_length_{0.0};
  double lateral_length_{0.0};
  double margin_{0.0};
  double desired_spacing_{0.0};
};

class CoveragePlannerNode : public rclcpp::Node
{
public:
  CoveragePlannerNode()
  : Node("rect_coverage_planner")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    origin_x_ = declare_parameter<double>("origin_x", 0.0);
    origin_y_ = declare_parameter<double>("origin_y", 0.0);
    rect_width_ = declare_parameter<double>("rect_width", 6.0);
    rect_length_ = declare_parameter<double>("rect_length", 20.0);
    margin_ = declare_parameter<double>("margin", 0.3);
    desired_spacing_ = declare_parameter<double>("desired_spacing", 1.0);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1.0);
    marker_scale_ = declare_parameter<double>("pose_marker_scale", 0.18);
    allow_goal_modification_ = declare_parameter<bool>("allow_goal_modification", false);
    use_clicked_points_ = declare_parameter<bool>("use_clicked_points", true);
    inspection_pose_topic_ =
      declare_parameter<std::string>("inspection_pose_topic", "/inspection/goal_pose");
    bulldoze_clicked_point_topic_ =
      declare_parameter<std::string>("bulldoze_clicked_point_topic", "/bulldoze/clicked_point");
    task_command_topic_ =
      declare_parameter<std::string>("task_command_topic", "/planning/task_command");
    odom_topic_ =
      declare_parameter<std::string>("odom_topic", "/localization/kinematic_state");
    loop_inspection_goals_ = declare_parameter<bool>("loop_goals", true);
    inspection_speed_kmh_ = declare_parameter<double>("inspection_speed_kmh", 4.0);
    bulldoze_speed_kmh_ = declare_parameter<double>("bulldoze_speed_kmh", 2.0);
    freespace_planner_node_name_ = declare_parameter<std::string>(
      "freespace_planner_node_name", "/planning/scenario_planning/parking/freespace_planner");
    route_state_topic_ = declare_parameter<std::string>("route_state_topic", "/api/routing/state");
    set_route_service_ =
      declare_parameter<std::string>("set_route_service", "/api/routing/set_route_points");
    clear_route_service_ =
      declare_parameter<std::string>("clear_route_service", "/api/routing/clear_route");

    pose_array_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>("coverage_pose_array", 10);
    marker_array_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("coverage_markers", 10);

    const auto visualization_period =
      std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    visualization_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(visualization_period),
      std::bind(&CoveragePlannerNode::publishVisualization, this));

    const auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    route_state_sub_ = create_subscription<RouteState>(
      route_state_topic_, durable_qos,
      std::bind(&CoveragePlannerNode::onRouteState, this, std::placeholders::_1));
    odom_sub_ = create_subscription<Odometry>(
      odom_topic_, 10, std::bind(&CoveragePlannerNode::onOdometry, this, std::placeholders::_1));
    task_command_sub_ = create_subscription<std_msgs::msg::String>(
      task_command_topic_, 10,
      std::bind(&CoveragePlannerNode::onTaskCommand, this, std::placeholders::_1));

    if (use_clicked_points_) {
      inspection_pose_sub_ = create_subscription<PoseStamped>(
        inspection_pose_topic_, 10,
        std::bind(&CoveragePlannerNode::onInspectionPose, this, std::placeholders::_1));
      bulldoze_clicked_point_sub_ = create_subscription<PointStamped>(
        bulldoze_clicked_point_topic_, 10,
        std::bind(&CoveragePlannerNode::onBulldozeClickedPoint, this, std::placeholders::_1));
    } else {
      configureFromParameters();
      buildCoveragePlan();
    }

    set_route_client_ = create_client<SetRoutePoints>(set_route_service_);
    clear_route_client_ = create_client<ClearRoute>(clear_route_service_);
    planner_param_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(this, freespace_planner_node_name_);
    routing_timer_ = create_wall_timer(500ms, std::bind(&CoveragePlannerNode::processRouting, this));

    RCLCPP_INFO(get_logger(), "Coverage planner started.");
    RCLCPP_INFO(get_logger(), "frame_id=%s", frame_id_.c_str());
    RCLCPP_INFO(
      get_logger(), "Routing uses %s and %s. /api/routing/route is the route readback topic.",
      set_route_service_.c_str(), route_state_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "task_command_topic=%s odom_topic=%s",
      task_command_topic_.c_str(), odom_topic_.c_str());

    if (use_clicked_points_) {
      RCLCPP_INFO(
        get_logger(), "Inspection poses from %s, bulldoze rectangle clicks from %s.",
        inspection_pose_topic_.c_str(), bulldoze_clicked_point_topic_.c_str());
      RCLCPP_INFO(
        get_logger(),
        "Click 1: start corner. Click 2: forward direction and length. Click 3: width and side.");
    } else {
      RCLCPP_INFO(
        get_logger(), "Using parameter rectangle: origin=(%.2f, %.2f) width=%.2f length=%.2f",
        origin_x_, origin_y_, rect_width_, rect_length_);
    }

    // Default mode is inspection, apply matching planning speed.
    applyPlannerSpeedByMode();
  }

private:
  void configureFromParameters()
  {
    rectangle_origin_ = makeVec2(origin_x_, origin_y_);
    forward_unit_ = makeVec2(1.0, 0.0);
    lateral_unit_ = makeVec2(0.0, 1.0);
    forward_length_ = rect_width_;
    lateral_length_ = rect_length_;
  }

  void onOdometry(const Odometry::ConstSharedPtr msg)
  {
    latest_odom_ = msg;
    has_odom_ = true;
  }

  void onInspectionPose(const PoseStamped::ConstSharedPtr msg)
  {
    if (!msg->header.frame_id.empty()) {
      frame_id_ = msg->header.frame_id;
    }

    inspection_poses_.push_back(Pose2D{msg->pose.position.x, msg->pose.position.y, 0.0});
    RCLCPP_INFO(
      get_logger(), "Added inspection point %zu at x=%.2f y=%.2f.",
      inspection_poses_.size(), msg->pose.position.x, msg->pose.position.y);

    if (inspection_poses_.size() == 1 && current_mode_ == TaskMode::Inspection) {
      inspection_goal_index_ = 0;
      resetGoalTracking();
      replan_requested_ = true;
    }
  }

  void onTaskCommand(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    const std::string command = toLowerCopy(msg->data);
    if (
      command == "bulldoze" || command == "start_bulldoze" || command == "push" ||
      command == "doze" || command == "推土")
    {
      startBulldozeTask();
      return;
    }
    if (command == "inspection" || command == "resume_inspection" || command == "巡检") {
      returnToNearestInspection("Inspection command received");
      return;
    }
    if (command == "clear_inspection") {
      clearInspectionGoals();
      return;
    }
    if (command == "clear_bulldoze") {
      clearBulldozeGoals();
      return;
    }
    if (command == "clear_all") {
      clearInspectionGoals();
      clearBulldozeGoals();
      return;
    }
    RCLCPP_WARN(get_logger(), "Unknown task command: %s", msg->data.c_str());
  }

  void onBulldozeClickedPoint(const PointStamped::ConstSharedPtr msg)
  {
    if (!msg->header.frame_id.empty()) {
      frame_id_ = msg->header.frame_id;
    }

    clicked_points_.push_back(makeVec2(msg->point.x, msg->point.y));

    if (clicked_points_.size() == 1) {
      RCLCPP_INFO(
        get_logger(), "Stored point 1 at x=%.2f y=%.2f. Click point 2 to set the forward edge.",
        clicked_points_[0].x, clicked_points_[0].y);
      return;
    }

    if (clicked_points_.size() == 2) {
      RCLCPP_INFO(
        get_logger(),
        "Stored point 2 at x=%.2f y=%.2f. Click point 3 to set the width and the side.",
        clicked_points_[1].x, clicked_points_[1].y);
      return;
    }

    try {
      configureFromThreePoints(clicked_points_[0], clicked_points_[1], clicked_points_[2]);
      buildCoveragePlan();
      if (current_mode_ == TaskMode::Bulldoze) {
        scheduleReplan();
      }
    } catch (const std::exception & e) {
      has_active_rectangle_ = false;
      poses_.clear();
      planner_.reset();
      RCLCPP_ERROR(get_logger(), "Invalid rectangle from clicked points: %s", e.what());
    }

    clicked_points_.clear();
  }

  void configureFromThreePoints(const Vec2 & p0, const Vec2 & p1, const Vec2 & p2)
  {
    const Vec2 forward_vector = p1 - p0;
    const double forward_length = norm(forward_vector);
    if (forward_length <= 1e-6) {
      throw std::invalid_argument("point 1 and point 2 are too close");
    }

    const Vec2 forward_unit = normalize(forward_vector);
    const Vec2 candidate_lateral = leftNormal(forward_unit);
    const double signed_lateral = dot(p2 - p0, candidate_lateral);
    if (std::abs(signed_lateral) <= 1e-6) {
      throw std::invalid_argument("point 3 must not lie on the forward edge");
    }

    rectangle_origin_ = p0;
    forward_unit_ = forward_unit;
    lateral_unit_ = candidate_lateral * (signed_lateral >= 0.0 ? 1.0 : -1.0);
    forward_length_ = forward_length;
    lateral_length_ = std::abs(signed_lateral);

    origin_x_ = rectangle_origin_.x;
    origin_y_ = rectangle_origin_.y;
    rect_width_ = forward_length_;
    rect_length_ = lateral_length_;
  }

  void buildCoveragePlan()
  {
    planner_ = std::make_unique<RectBoundaryPlanner>(
      rectangle_origin_, forward_unit_, lateral_unit_, forward_length_, lateral_length_, margin_,
      desired_spacing_);
    poses_ = planner_->generateBoundaryPath();
    has_active_rectangle_ = true;
    current_goal_index_ = 0;
    coverage_completed_ = false;

    const double heading_deg = std::atan2(forward_unit_.y, forward_unit_.x) * 180.0 / kPi;
    const double lateral_deg = std::atan2(lateral_unit_.y, lateral_unit_.x) * 180.0 / kPi;
    RCLCPP_INFO(
      get_logger(),
      "Rectangle updated: origin=(%.2f, %.2f) forward=%.2f side=%.2f heading=%.1f deg side_dir=%.1f deg",
      rectangle_origin_.x, rectangle_origin_.y, forward_length_, lateral_length_, heading_deg,
      lateral_deg);
    RCLCPP_INFO(get_logger(), "Generated %zu ordered route goals.", poses_.size());
  }

  void scheduleReplan()
  {
    replan_requested_ = true;
    coverage_completed_ = false;
    active_goal_sent_ = false;
    advance_goal_after_clear_ = false;

    RCLCPP_INFO(
      get_logger(),
      "New rectangle accepted. The node will clear the current route and start from the first goal.");
  }

  Vec2 currentPositionOrDefault() const
  {
    if (has_odom_ && latest_odom_) {
      return makeVec2(latest_odom_->pose.pose.position.x, latest_odom_->pose.pose.position.y);
    }
    return makeVec2(0.0, 0.0);
  }

  size_t findNearestInspectionIndex(const Vec2 & current_position) const
  {
    size_t best_index = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < inspection_poses_.size(); ++i) {
      const auto & p = inspection_poses_.at(i);
      const double d = std::hypot(current_position.x - p.x, current_position.y - p.y);
      if (d < best_dist) {
        best_dist = d;
        best_index = i;
      }
    }
    return best_index;
  }

  void startBulldozeTask()
  {
    if (!has_active_rectangle_ || poses_.empty()) {
      RCLCPP_WARN(get_logger(), "Received bulldoze command, but no bulldoze region is available.");
      return;
    }
    if (current_mode_ == TaskMode::Inspection && !inspection_poses_.empty()) {
      paused_inspection_goal_index_ = std::min(inspection_goal_index_, inspection_poses_.size() - 1);
      has_paused_inspection_goal_ = true;
    }
    current_mode_ = TaskMode::Bulldoze;
    current_goal_index_ = 0;
    resetGoalTracking();
    scheduleReplan();
    applyPlannerSpeedByMode();
    RCLCPP_INFO(get_logger(), "Switched to bulldoze mode with %zu goals.", poses_.size());
  }

  void returnToPausedInspection(const std::string & reason)
  {
    current_mode_ = TaskMode::Inspection;
    if (inspection_poses_.empty()) {
      resetGoalTracking();
      coverage_completed_ = true;
      RCLCPP_WARN(get_logger(), "%s, but no inspection point exists.", reason.c_str());
      return;
    }

    if (has_paused_inspection_goal_) {
      inspection_goal_index_ = std::min(paused_inspection_goal_index_, inspection_poses_.size() - 1);
    } else {
      inspection_goal_index_ = std::min(inspection_goal_index_, inspection_poses_.size() - 1);
    }
    has_paused_inspection_goal_ = false;
    resetGoalTracking();
    scheduleReplan();
    applyPlannerSpeedByMode();
    RCLCPP_INFO(
      get_logger(), "%s. Resume inspection from paused goal %zu/%zu.",
      reason.c_str(), inspection_goal_index_ + 1, inspection_poses_.size());
  }

  void returnToNearestInspection(const std::string & reason)
  {
    current_mode_ = TaskMode::Inspection;
    if (inspection_poses_.empty()) {
      resetGoalTracking();
      coverage_completed_ = true;
      RCLCPP_WARN(get_logger(), "%s, but no inspection point exists.", reason.c_str());
      return;
    }
    const size_t nearest = findNearestInspectionIndex(currentPositionOrDefault());
    inspection_goal_index_ = nearest;
    resetGoalTracking();
    scheduleReplan();
    applyPlannerSpeedByMode();
    RCLCPP_INFO(
      get_logger(), "%s. Resume inspection from goal %zu/%zu.",
      reason.c_str(), inspection_goal_index_ + 1, inspection_poses_.size());
  }

  void clearInspectionGoals()
  {
    inspection_poses_.clear();
    inspection_goal_index_ = 0;
    has_paused_inspection_goal_ = false;
    if (current_mode_ == TaskMode::Inspection) {
      resetGoalTracking();
      coverage_completed_ = true;
    }
    RCLCPP_INFO(get_logger(), "Cleared all inspection points.");
  }

  void clearBulldozeGoals()
  {
    has_active_rectangle_ = false;
    poses_.clear();
    clicked_points_.clear();
    planner_.reset();
    current_goal_index_ = 0;
    if (current_mode_ == TaskMode::Bulldoze) {
      returnToNearestInspection("Bulldoze goals cleared");
    }
    RCLCPP_INFO(get_logger(), "Cleared bulldoze rectangle and generated goals.");
  }

  const std::vector<Pose2D> * activePoses() const
  {
    if (current_mode_ == TaskMode::Bulldoze) {
      if (!has_active_rectangle_ || poses_.empty()) {
        return nullptr;
      }
      return &poses_;
    }
    if (inspection_poses_.empty()) {
      return nullptr;
    }
    return &inspection_poses_;
  }

  size_t activeGoalIndex() const
  {
    return current_mode_ == TaskMode::Bulldoze ? current_goal_index_ : inspection_goal_index_;
  }

  void setActiveGoalIndex(size_t index)
  {
    if (current_mode_ == TaskMode::Bulldoze) {
      current_goal_index_ = index;
    } else {
      inspection_goal_index_ = index;
    }
  }

  void incrementActiveGoalIndex()
  {
    if (current_mode_ == TaskMode::Bulldoze) {
      ++current_goal_index_;
    } else {
      ++inspection_goal_index_;
    }
  }

  double inspectionGoalYaw(size_t goal_index) const
  {
    if (inspection_poses_.empty()) return 0.0;
    if (inspection_poses_.size() == 1U) {
      if (has_odom_ && latest_odom_) {
        return yawFromQuat(latest_odom_->pose.pose.orientation);
      }
      return 0.0;
    }

    const size_t current_index = std::min(goal_index, inspection_poses_.size() - 1);
    size_t reference_index = current_index;
    if (current_index + 1U < inspection_poses_.size()) {
      reference_index = current_index + 1U;
    } else if (loop_inspection_goals_) {
      reference_index = 0U;
    } else if (current_index > 0U) {
      reference_index = current_index - 1U;
    }

    const auto & current_pose = inspection_poses_[current_index];
    const auto & reference_pose = inspection_poses_[reference_index];
    const double dx = reference_pose.x - current_pose.x;
    const double dy = reference_pose.y - current_pose.y;
    if (std::hypot(dx, dy) <= 1e-6) {
      return 0.0;
    }
    return std::atan2(dy, dx);
  }

  Pose2D activeGoalPose() const
  {
    const auto * poses_ptr = activePoses();
    if (!poses_ptr || poses_ptr->empty()) return Pose2D{};
    const size_t idx = std::min(activeGoalIndex(), poses_ptr->size() - 1);
    auto goal = poses_ptr->at(idx);
    if (current_mode_ == TaskMode::Inspection) {
      goal.yaw = inspectionGoalYaw(idx);
    }
    return goal;
  }

  void resetGoalTracking()
  {
    active_goal_sent_ = false;
    awaiting_route_clear_ = false;
    advance_goal_after_clear_ = false;
  }

  void applyPlannerSpeedByMode()
  {
    if (!planner_param_client_) {
      return;
    }
    if (!planner_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Planner parameter service is not ready: %s", freespace_planner_node_name_.c_str());
      return;
    }

    const double speed_kmh =
      current_mode_ == TaskMode::Bulldoze ? bulldoze_speed_kmh_ : inspection_speed_kmh_;
    pending_speed_set_future_ = planner_param_client_->set_parameters(
      {rclcpp::Parameter("waypoints_velocity", speed_kmh)});
    pending_speed_set_mode_ = current_mode_ == TaskMode::Bulldoze ? "Bulldoze" : "Inspection";
    pending_speed_set_kmh_ = speed_kmh;
  }

  void processPendingPlannerSpeedUpdate()
  {
    if (!pending_speed_set_future_.valid()) {
      return;
    }
    const auto status = pending_speed_set_future_.wait_for(std::chrono::seconds(0));
    if (status != std::future_status::ready) {
      return;
    }
    const auto results = pending_speed_set_future_.get();
    if (results.empty() || !results.front().successful) {
      const std::string reason = results.empty() ? "empty response" : results.front().reason;
      RCLCPP_WARN(
        get_logger(),
        "Failed to set freespace planner speed for %s mode (%.2f km/h): %s",
        pending_speed_set_mode_.c_str(), pending_speed_set_kmh_, reason.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Set freespace planner waypoints_velocity=%.2f km/h for %s mode.",
      pending_speed_set_kmh_, pending_speed_set_mode_.c_str());
  }

  void onRouteState(const RouteState::ConstSharedPtr msg)
  {
    has_route_state_ = true;
    if (route_state_ != msg->state) {
      RCLCPP_INFO(
        get_logger(), "Route state changed: %s -> %s", routeStateToString(route_state_),
        routeStateToString(msg->state));
    }
    route_state_ = msg->state;
  }

  void processRouting()
  {
    processPendingPlannerSpeedUpdate();

    const auto * active_poses = activePoses();
    if (active_poses == nullptr || active_poses->empty()) {
      return;
    }

    if (!set_route_client_->service_is_ready() || !clear_route_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for routing services to become available.");
      return;
    }

    if (!has_route_state_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for %s.", route_state_topic_.c_str());
      return;
    }

    if (request_in_flight_) {
      return;
    }

    if (replan_requested_) {
      if (route_state_ != RouteState::UNSET) {
        if (!awaiting_route_clear_) {
          requestClearRoute("New rectangle received. Clearing the existing route.", false);
        }
        return;
      }

      awaiting_route_clear_ = false;
      replan_requested_ = false;
      active_goal_sent_ = false;
    }

    if (coverage_completed_ && current_mode_ == TaskMode::Inspection && !loop_inspection_goals_) {
      return;
    }

    if (route_state_ == RouteState::ARRIVED && active_goal_sent_ && !awaiting_route_clear_) {
      const size_t goal_index = std::min(activeGoalIndex(), active_poses->size() - 1);
      const bool has_next_goal = goal_index + 1 < active_poses->size();

      if (!has_next_goal) {
        if (current_mode_ == TaskMode::Bulldoze) {
          RCLCPP_INFO(get_logger(), "Bulldoze goals completed. Returning to paused inspection goal.");
          returnToPausedInspection("Bulldoze completed");
        } else if (loop_inspection_goals_) {
          inspection_goal_index_ = 0;
          resetGoalTracking();
          scheduleReplan();
          RCLCPP_INFO(get_logger(), "Inspection loop completed. Restarting from first goal.");
        } else {
          coverage_completed_ = true;
          active_goal_sent_ = false;
          advance_goal_after_clear_ = false;
          awaiting_route_clear_ = false;
          RCLCPP_INFO(get_logger(), "Final inspection goal reached. Holding position.");
        }
        return;
      }
      requestClearRoute("Reached current goal. Clearing route before sending the next goal.", true);
      return;
    }

    if (route_state_ == RouteState::UNSET) {
      if (awaiting_route_clear_) {
        awaiting_route_clear_ = false;
        active_goal_sent_ = false;
        if (advance_goal_after_clear_) {
          advanceGoalIndex();
        }
        advance_goal_after_clear_ = false;
      }

      if (!active_goal_sent_) {
        requestNextGoal();
      }
    }
  }

  void requestNextGoal()
  {
    const auto * active_poses = activePoses();
    if (!active_poses || active_poses->empty()) {
      return;
    }
    const size_t goal_index = std::min(activeGoalIndex(), active_poses->size() - 1);
    const auto goal = activeGoalPose();
    auto request = std::make_shared<SetRoutePoints::Request>();
    request->header.frame_id = frame_id_;
    request->header.stamp = now();
    request->goal = makePose(goal.x, goal.y, goal.yaw);
    request->option.allow_goal_modification = allow_goal_modification_;

    request_in_flight_ = true;

    RCLCPP_INFO(
      get_logger(), "Sending goal [%s] %zu/%zu: x=%.2f y=%.2f yaw=%.1f deg",
      current_mode_ == TaskMode::Bulldoze ? "Bulldoze" : "Inspection", goal_index + 1,
      active_poses->size(), goal.x, goal.y, goal.yaw * 180.0 / kPi);

    set_route_client_->async_send_request(
      request,
      [this, goal_index](rclcpp::Client<SetRoutePoints>::SharedFuture future) {
        request_in_flight_ = false;

        const auto response = future.get();
        if (!response->status.success) {
          RCLCPP_ERROR(
            get_logger(), "Failed to set goal %zu: code=%u message=%s", goal_index + 1,
            response->status.code, response->status.message.c_str());
          return;
        }

        active_goal_sent_ = true;
        RCLCPP_INFO(get_logger(), "Goal %zu accepted by routing service.", goal_index + 1);
      });
  }

  void requestClearRoute(const std::string & reason, bool advance_goal_after_clear)
  {
    auto request = std::make_shared<ClearRoute::Request>();
    request_in_flight_ = true;
    awaiting_route_clear_ = true;
    advance_goal_after_clear_ = advance_goal_after_clear;

    RCLCPP_INFO(get_logger(), "%s", reason.c_str());

    clear_route_client_->async_send_request(
      request, [this](rclcpp::Client<ClearRoute>::SharedFuture future) {
        request_in_flight_ = false;

        const auto response = future.get();
        if (!response->status.success) {
          awaiting_route_clear_ = false;
          advance_goal_after_clear_ = false;
          RCLCPP_ERROR(
            get_logger(), "Failed to clear route: code=%u message=%s", response->status.code,
            response->status.message.c_str());
          return;
        }

        RCLCPP_INFO(get_logger(), "Route clear accepted. Waiting for UNSET state.");
      });
  }

  void advanceGoalIndex()
  {
    incrementActiveGoalIndex();
    const auto * active_poses = activePoses();
    const size_t goal_count = active_poses ? active_poses->size() : 0;
    RCLCPP_INFO(
      get_logger(), "Advancing to next goal. Next index is %zu/%zu.", activeGoalIndex() + 1, goal_count);
  }

  void publishVisualization()
  {
    const auto stamp = now();

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = frame_id_;
    pose_array.header.stamp = stamp;
    const auto * active_poses = activePoses();
    if (active_poses) {
      pose_array.poses.reserve(active_poses->size());
      for (const auto & pose : *active_poses) {
        pose_array.poses.push_back(makePose(pose.x, pose.y, pose.yaw));
      }
    }
    pose_array_pub_->publish(pose_array);

    visualization_msgs::msg::MarkerArray marker_array;
    if (has_active_rectangle_) {
      marker_array.markers.push_back(makeRectangleMarker(stamp));
      marker_array.markers.push_back(makePathMarker(stamp));
      marker_array.markers.push_back(makePosePointMarker(stamp));
    } else {
      // Explicitly delete bulldoze markers to avoid RViz stale display after clear_bulldoze.
      marker_array.markers.push_back(makeDeleteMarker(stamp, 0));
      marker_array.markers.push_back(makeDeleteMarker(stamp, 1));
      marker_array.markers.push_back(makeDeleteMarker(stamp, 2));
    }
    marker_array.markers.push_back(makeInspectionPathMarker(stamp));
    marker_array.markers.push_back(makeInspectionPointMarker(stamp));
    marker_array.markers.push_back(makeCurrentGoalArrowMarker(stamp));
    marker_array_pub_->publish(marker_array);
  }

  visualization_msgs::msg::Marker makeDeleteMarker(const rclcpp::Time & stamp, int id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "coverage";
    marker.id = id;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    return marker;
  }

  std::vector<Vec2> getRectangleCorners() const
  {
    const Vec2 p0 = rectangle_origin_;
    const Vec2 p1 = rectangle_origin_ + forward_unit_ * forward_length_;
    const Vec2 p2 = p1 + lateral_unit_ * lateral_length_;
    const Vec2 p3 = rectangle_origin_ + lateral_unit_ * lateral_length_;
    return {p0, p1, p2, p3, p0};
  }

  visualization_msgs::msg::Marker makeRectangleMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "coverage";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;
    marker.color = makeColor(0.10f, 0.10f, 0.10f, 1.0f);

    for (const auto & corner : getRectangleCorners()) {
      marker.points.push_back(makePoint(corner));
    }
    return marker;
  }

  visualization_msgs::msg::Marker makePathMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "coverage";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color = makeColor(0.12f, 0.56f, 1.0f, 0.95f);

    marker.points.reserve(poses_.size());
    for (const auto & pose : poses_) {
      marker.points.push_back(makePoint(pose.x, pose.y, 0.03));
    }
    return marker;
  }

  visualization_msgs::msg::Marker makePosePointMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "coverage";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;
    marker.color = makeColor(0.88f, 0.22f, 0.22f, 0.95f);

    marker.points.reserve(poses_.size());
    for (const auto & pose : poses_) {
      marker.points.push_back(makePoint(pose.x, pose.y, 0.06));
    }
    return marker;
  }

  visualization_msgs::msg::Marker makeCurrentGoalArrowMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "coverage";
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.35;
    marker.scale.y = 0.08;
    marker.scale.z = 0.10;
    marker.color = makeColor(0.00f, 0.67f, 0.36f, 0.95f);

    const auto * active_poses = activePoses();
    if (active_poses && !active_poses->empty()) {
      const auto pose = activeGoalPose();
      marker.pose = makePose(pose.x, pose.y, pose.yaw);
    } else {
      marker.pose.orientation.w = 1.0;
    }
    return marker;
  }

  visualization_msgs::msg::Marker makeInspectionPathMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "coverage";
    marker.id = 4;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = inspection_poses_.empty() ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color = makeColor(0.95f, 0.65f, 0.10f, 0.95f);
    if (!inspection_poses_.empty()) {
      marker.points.reserve(inspection_poses_.size() + (loop_inspection_goals_ ? 1 : 0));
      for (const auto & p : inspection_poses_) {
        marker.points.push_back(makePoint(p.x, p.y, 0.03));
      }
      if (loop_inspection_goals_ && inspection_poses_.size() > 1) {
        marker.points.push_back(makePoint(inspection_poses_.front().x, inspection_poses_.front().y, 0.03));
      }
    }
    return marker;
  }

  visualization_msgs::msg::Marker makeInspectionPointMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "coverage";
    marker.id = 5;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = inspection_poses_.empty() ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;
    marker.color = makeColor(0.95f, 0.65f, 0.10f, 0.95f);
    if (!inspection_poses_.empty()) {
      marker.points.reserve(inspection_poses_.size());
      for (const auto & p : inspection_poses_) {
        marker.points.push_back(makePoint(p.x, p.y, 0.06));
      }
    }
    return marker;
  }

  std::string frame_id_;
  std::string inspection_pose_topic_;
  std::string bulldoze_clicked_point_topic_;
  std::string task_command_topic_;
  std::string odom_topic_;
  std::string freespace_planner_node_name_;
  std::string route_state_topic_;
  std::string set_route_service_;
  std::string clear_route_service_;
  double origin_x_{0.0};
  double origin_y_{0.0};
  double rect_width_{0.0};
  double rect_length_{0.0};
  double margin_{0.0};
  double desired_spacing_{0.0};
  double publish_rate_hz_{1.0};
  double marker_scale_{0.18};
  bool allow_goal_modification_{false};
  bool use_clicked_points_{true};
  bool loop_inspection_goals_{true};
  double inspection_speed_kmh_{4.0};
  double bulldoze_speed_kmh_{2.0};

  bool has_route_state_{false};
  bool has_odom_{false};
  bool has_active_rectangle_{false};
  bool request_in_flight_{false};
  bool active_goal_sent_{false};
  bool awaiting_route_clear_{false};
  bool advance_goal_after_clear_{false};
  bool replan_requested_{false};
  bool coverage_completed_{false};
  uint16_t route_state_{RouteState::UNKNOWN};
  TaskMode current_mode_{TaskMode::Inspection};
  size_t current_goal_index_{0};
  size_t inspection_goal_index_{0};
  size_t paused_inspection_goal_index_{0};
  bool has_paused_inspection_goal_{false};

  Vec2 rectangle_origin_{0.0, 0.0};
  Vec2 forward_unit_{1.0, 0.0};
  Vec2 lateral_unit_{0.0, 1.0};
  double forward_length_{0.0};
  double lateral_length_{0.0};
  std::vector<Vec2> clicked_points_;
  std::vector<Pose2D> inspection_poses_;
  Odometry::ConstSharedPtr latest_odom_;

  std::unique_ptr<RectBoundaryPlanner> planner_;
  std::vector<Pose2D> poses_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr inspection_pose_sub_;
  rclcpp::Subscription<PointStamped>::SharedPtr bulldoze_clicked_point_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_command_sub_;
  rclcpp::Subscription<RouteState>::SharedPtr route_state_sub_;
  rclcpp::Client<SetRoutePoints>::SharedPtr set_route_client_;
  rclcpp::Client<ClearRoute>::SharedPtr clear_route_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> planner_param_client_;
  std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> pending_speed_set_future_;
  std::string pending_speed_set_mode_{};
  double pending_speed_set_kmh_{0.0};
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::TimerBase::SharedPtr routing_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<CoveragePlannerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Failed to start node: " << e.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
