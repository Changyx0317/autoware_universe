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

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

using geometry_msgs::msg::PointStamped;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;
constexpr size_t kNoRegionSelected = std::numeric_limits<size_t>::max();

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

struct BulldozeRegion
{
  size_t id{0};
  Vec2 origin{0.0, 0.0};
  Vec2 forward_unit{1.0, 0.0};
  Vec2 lateral_unit{0.0, 1.0};
  double forward_length{0.0};
  double lateral_length{0.0};
  std::vector<Pose2D> poses;
};

enum class TaskMode
{
  Inspection,
  Bulldoze
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

double squaredDistance(const Vec2 & a, const Vec2 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
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

double normalizeAngle(const double angle)
{
  double a = angle;
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
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

    use_clicked_points_ = declare_parameter<bool>("use_clicked_points", true);
    inspection_pose_topic_ =
      declare_parameter<std::string>("inspection_pose_topic", "/inspection/goal_pose");
    bulldoze_clicked_point_topic_ =
      declare_parameter<std::string>("bulldoze_clicked_point_topic", "/bulldoze/clicked_point");
    task_command_topic_ =
      declare_parameter<std::string>("task_command_topic", "/planning/task_command");

    goal_topic_ = declare_parameter<std::string>("goal_topic", "/planning/mission_planning/goal");
    odom_topic_ =
      declare_parameter<std::string>("odom_topic", "/localization/kinematic_state");
    goal_reach_tolerance_m_ = declare_parameter<double>("goal_reach_tolerance_m", 0.6);
    goal_yaw_tolerance_rad_ = declare_parameter<double>("goal_yaw_tolerance_rad", 0.12);
    stop_speed_tolerance_mps_ = declare_parameter<double>("stop_speed_tolerance_mps", 0.05);
    require_goal_yaw_alignment_ = declare_parameter<bool>("require_goal_yaw_alignment", false);
    require_stop_at_goal_ = declare_parameter<bool>("require_stop_at_goal", false);
    goal_settle_time_s_ = declare_parameter<double>("goal_settle_time_s", 1.0);
    republish_goal_interval_s_ = declare_parameter<double>("republish_goal_interval_s", 1.0);
    loop_inspection_goals_ = declare_parameter<bool>("loop_goals", true);

    pose_array_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>("coverage_pose_array", 10);
    marker_array_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("coverage_markers", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);

    const auto visualization_period =
      std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_hz_));
    visualization_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(visualization_period),
      std::bind(&CoveragePlannerNode::publishVisualization, this));

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
      addBulldozeRegion(rectangle_origin_, forward_unit_, lateral_unit_, forward_length_, lateral_length_);
    }

    goal_timer_ = create_wall_timer(200ms, std::bind(&CoveragePlannerNode::processGoals, this));

    RCLCPP_INFO(get_logger(), "Coverage planner started.");
    RCLCPP_INFO(get_logger(), "frame_id=%s", frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "goal_topic=%s odom_topic=%s", goal_topic_.c_str(), odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "task_command_topic=%s", task_command_topic_.c_str());

    if (use_clicked_points_) {
      RCLCPP_INFO(
        get_logger(),
        "Inspection poses from %s, bulldoze rectangle clicks from %s.",
        inspection_pose_topic_.c_str(), bulldoze_clicked_point_topic_.c_str());
      RCLCPP_INFO(
        get_logger(),
        "Bulldoze rectangle input uses three clicks: start corner, forward edge, width side.");
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Loaded one bulldoze rectangle from parameters: origin=(%.2f, %.2f) width=%.2f length=%.2f",
        origin_x_, origin_y_, rect_width_, rect_length_);
    }
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
    updateFrameId(msg->header.frame_id);

    const Pose2D inspection_pose{
      msg->pose.position.x,
      msg->pose.position.y,
      0.0
    };
    inspection_poses_.push_back(inspection_pose);

    RCLCPP_INFO(
      get_logger(), "Added inspection point %zu at x=%.2f y=%.2f from PoseStamped.",
      inspection_poses_.size(), inspection_pose.x, inspection_pose.y);

    if (inspection_poses_.size() == 1 && current_mode_ == TaskMode::Inspection) {
      scheduleInspectionFromIndex(0, "First inspection point added");
    }
  }

  void onBulldozeClickedPoint(const PointStamped::ConstSharedPtr msg)
  {
    updateFrameId(msg->header.frame_id);

    bulldoze_clicked_points_.push_back(makeVec2(msg->point.x, msg->point.y));

    if (bulldoze_clicked_points_.size() == 1) {
      RCLCPP_INFO(
        get_logger(),
        "Stored bulldoze point 1 at x=%.2f y=%.2f. Click point 2 to set the forward edge.",
        bulldoze_clicked_points_[0].x, bulldoze_clicked_points_[0].y);
      return;
    }

    if (bulldoze_clicked_points_.size() == 2) {
      RCLCPP_INFO(
        get_logger(),
        "Stored bulldoze point 2 at x=%.2f y=%.2f. Click point 3 to set the width side.",
        bulldoze_clicked_points_[1].x, bulldoze_clicked_points_[1].y);
      return;
    }

    try {
      configureFromThreePoints(
        bulldoze_clicked_points_[0], bulldoze_clicked_points_[1], bulldoze_clicked_points_[2]);
      addBulldozeRegion(rectangle_origin_, forward_unit_, lateral_unit_, forward_length_, lateral_length_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Invalid bulldoze rectangle from clicked points: %s", e.what());
    }

    bulldoze_clicked_points_.clear();
  }

  void onTaskCommand(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    const std::string command = toLowerCopy(msg->data);

    if (
      command == "bulldoze" || command == "start_bulldoze" || command == "push" ||
      command == "doze" || command == "推土")
    {
      startNearestBulldozeTask();
      return;
    }

    if (
      command == "inspection" || command == "resume_inspection" || command == "巡检")
    {
      returnToNearestInspection("Inspection command received");
      return;
    }

    if (command == "clear_inspection") {
      clearInspectionGoals();
      return;
    }

    if (command == "clear_bulldoze") {
      clearBulldozeRegions();
      return;
    }

    if (command == "clear_all") {
      clearInspectionGoals();
      clearBulldozeRegions();
      RCLCPP_INFO(get_logger(), "Cleared all inspection points and bulldoze regions.");
      return;
    }

    RCLCPP_WARN(get_logger(), "Unknown task command: %s", msg->data.c_str());
  }

  void updateFrameId(const std::string & incoming_frame_id)
  {
    if (!incoming_frame_id.empty()) {
      frame_id_ = incoming_frame_id;
    }
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

  void addBulldozeRegion(
    const Vec2 & origin, const Vec2 & forward_unit, const Vec2 & lateral_unit, double forward_length,
    double lateral_length)
  {
    RectBoundaryPlanner planner(
      origin, forward_unit, lateral_unit, forward_length, lateral_length, margin_, desired_spacing_);

    BulldozeRegion region;
    region.id = next_bulldoze_region_id_++;
    region.origin = origin;
    region.forward_unit = forward_unit;
    region.lateral_unit = lateral_unit;
    region.forward_length = forward_length;
    region.lateral_length = lateral_length;
    region.poses = planner.generateBoundaryPath();

    bulldoze_regions_.push_back(region);

    const double heading_deg = std::atan2(forward_unit.y, forward_unit.x) * 180.0 / kPi;
    RCLCPP_INFO(
      get_logger(),
      "Added bulldoze region %zu: origin=(%.2f, %.2f) forward=%.2f side=%.2f heading=%.1f deg, goals=%zu",
      region.id, origin.x, origin.y, forward_length, lateral_length, heading_deg, region.poses.size());
  }

  void clearInspectionGoals()
  {
    inspection_poses_.clear();
    inspection_goal_index_ = 0;
    if (current_mode_ == TaskMode::Inspection) {
      resetGoalTracking();
    }
    RCLCPP_INFO(get_logger(), "Cleared all inspection points.");
  }

  void clearBulldozeRegions()
  {
    bulldoze_regions_.clear();
    bulldoze_clicked_points_.clear();
    active_bulldoze_region_index_ = kNoRegionSelected;
    bulldoze_goal_index_ = 0;
    if (current_mode_ == TaskMode::Bulldoze) {
      current_mode_ = TaskMode::Inspection;
      resetGoalTracking();
    }
    RCLCPP_INFO(get_logger(), "Cleared all bulldoze regions.");
  }

  void startNearestBulldozeTask()
  {
    if (bulldoze_regions_.empty()) {
      RCLCPP_WARN(get_logger(), "Received bulldoze command, but no bulldoze region is available.");
      return;
    }

    const Vec2 current_position = currentPositionOrDefault();
    active_bulldoze_region_index_ = findNearestBulldozeRegionIndex(current_position);
    bulldoze_goal_index_ = 0;
    current_mode_ = TaskMode::Bulldoze;
    resetGoalTracking();

    const auto & region = bulldoze_regions_.at(active_bulldoze_region_index_);
    RCLCPP_INFO(
      get_logger(),
      "Starting bulldoze task on nearest region %zu with %zu goals.",
      region.id, region.poses.size());
  }

  void returnToNearestInspection(const std::string & reason)
  {
    if (inspection_poses_.empty()) {
      current_mode_ = TaskMode::Inspection;
      resetGoalTracking();
      RCLCPP_WARN(get_logger(), "%s, but no inspection point exists.", reason.c_str());
      return;
    }

    const Vec2 current_position = currentPositionOrDefault();
    const size_t nearest_index = findNearestPoseIndex(inspection_poses_, current_position);
    scheduleInspectionFromIndex(nearest_index, reason);
  }

  void scheduleInspectionFromIndex(size_t index, const std::string & reason)
  {
    if (inspection_poses_.empty()) {
      return;
    }

    inspection_goal_index_ = std::min(index, inspection_poses_.size() - 1);
    current_mode_ = TaskMode::Inspection;
    active_bulldoze_region_index_ = kNoRegionSelected;
    bulldoze_goal_index_ = 0;
    resetGoalTracking();

    RCLCPP_INFO(
      get_logger(), "%s. Resume inspection from goal %zu/%zu.",
      reason.c_str(), inspection_goal_index_ + 1, inspection_poses_.size());
  }

  Vec2 currentPositionOrDefault() const
  {
    if (has_odom_ && latest_odom_) {
      return makeVec2(latest_odom_->pose.pose.position.x, latest_odom_->pose.pose.position.y);
    }
    return makeVec2(0.0, 0.0);
  }

  size_t findNearestBulldozeRegionIndex(const Vec2 & current_position) const
  {
    double best_dist = std::numeric_limits<double>::max();
    size_t best_index = 0;

    for (size_t i = 0; i < bulldoze_regions_.size(); ++i) {
      const auto & region = bulldoze_regions_[i];
      const double dist = region.poses.empty() ?
        squaredDistance(current_position, region.origin) :
        minDistanceToPathSquared(region.poses, current_position);
      if (dist < best_dist) {
        best_dist = dist;
        best_index = i;
      }
    }
    return best_index;
  }

  size_t findNearestPoseIndex(const std::vector<Pose2D> & poses, const Vec2 & current_position) const
  {
    size_t best_index = 0;
    double best_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < poses.size(); ++i) {
      const double dist = squaredDistance(current_position, makeVec2(poses[i].x, poses[i].y));
      if (dist < best_dist) {
        best_dist = dist;
        best_index = i;
      }
    }
    return best_index;
  }

  double minDistanceToPathSquared(const std::vector<Pose2D> & poses, const Vec2 & current_position) const
  {
    double best_dist = std::numeric_limits<double>::max();
    for (const auto & pose : poses) {
      best_dist = std::min(best_dist, squaredDistance(current_position, makeVec2(pose.x, pose.y)));
    }
    return best_dist;
  }

  const std::vector<Pose2D> * activePoses() const
  {
    if (current_mode_ == TaskMode::Bulldoze) {
      if (
        active_bulldoze_region_index_ == kNoRegionSelected ||
        active_bulldoze_region_index_ >= bulldoze_regions_.size())
      {
        return nullptr;
      }
      return &bulldoze_regions_[active_bulldoze_region_index_].poses;
    }

    if (inspection_poses_.empty()) {
      return nullptr;
    }
    return &inspection_poses_;
  }

  Pose2D activeGoalPose() const
  {
    const auto * poses = activePoses();
    if (poses == nullptr || poses->empty()) {
      return Pose2D{};
    }

    const size_t goal_index = std::min(activeGoalIndex(), poses->size() - 1);
    Pose2D goal = poses->at(goal_index);

    if (current_mode_ == TaskMode::Inspection) {
      goal.yaw = inspectionGoalYaw(goal_index);
    }
    return goal;
  }

  double inspectionGoalYaw(size_t goal_index) const
  {
    if (inspection_poses_.empty()) {
      return 0.0;
    }

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
      if (current_index > 0U) {
        const auto & previous_pose = inspection_poses_[current_index - 1U];
        return std::atan2(current_pose.y - previous_pose.y, current_pose.x - previous_pose.x);
      }
      if (has_odom_ && latest_odom_) {
        return yawFromQuat(latest_odom_->pose.pose.orientation);
      }
      return 0.0;
    }

    return std::atan2(dy, dx);
  }

  size_t activeGoalIndex() const
  {
    return current_mode_ == TaskMode::Bulldoze ? bulldoze_goal_index_ : inspection_goal_index_;
  }

  void incrementActiveGoalIndex()
  {
    if (current_mode_ == TaskMode::Bulldoze) {
      ++bulldoze_goal_index_;
      return;
    }
    ++inspection_goal_index_;
  }

  void resetGoalTracking()
  {
    active_goal_sent_ = false;
    waiting_goal_settle_ = false;
  }

  void processGoals()
  {
    const auto * poses = activePoses();
    if (poses == nullptr || poses->empty()) {
      return;
    }

    if (!has_odom_ || !latest_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for odometry on %s.", odom_topic_.c_str());
      return;
    }

    const auto now_time = now();
    const size_t goal_index = std::min(activeGoalIndex(), poses->size() - 1);

    if (!active_goal_sent_) {
      publishCurrentGoal("Initial goal published");
      return;
    }

    const auto goal = activeGoalPose();
    const double px = latest_odom_->pose.pose.position.x;
    const double py = latest_odom_->pose.pose.position.y;
    const double current_yaw = yawFromQuat(latest_odom_->pose.pose.orientation);
    const double current_speed = std::abs(latest_odom_->twist.twist.linear.x);
    const double dist = std::hypot(goal.x - px, goal.y - py);
    const double yaw_err = std::abs(normalizeAngle(goal.yaw - current_yaw));
    const double reach_tolerance = std::max(0.01, goal_reach_tolerance_m_);
    const double yaw_tolerance = std::max(0.01, goal_yaw_tolerance_rad_);
    const double stop_speed_tolerance = std::max(0.0, stop_speed_tolerance_mps_);
    const bool position_reached = dist <= reach_tolerance;
    const bool yaw_aligned = !require_goal_yaw_alignment_ || yaw_err <= yaw_tolerance;
    const bool stopped = !require_stop_at_goal_ || current_speed <= stop_speed_tolerance;

    const bool goal_reached = position_reached && yaw_aligned && stopped;

    if (goal_reached) {
      const double settle_time = std::max(0.0, goal_settle_time_s_);
      if (settle_time <= 0.0) {
        handleReachedGoal(*poses);
        return;
      }

      if (!waiting_goal_settle_) {
        waiting_goal_settle_ = true;
        goal_settle_start_time_ = now_time;
        RCLCPP_INFO(
          get_logger(),
          "Goal %zu/%zu reached in %s mode. Waiting %.2f s before next goal.",
          goal_index + 1, poses->size(), modeName().c_str(), settle_time);
        return;
      }

      const double settle_elapsed = (now_time - goal_settle_start_time_).seconds();
      if (settle_elapsed < settle_time) {
        return;
      }

      waiting_goal_settle_ = false;
      handleReachedGoal(*poses);
      return;
    }
    waiting_goal_settle_ = false;
  }

  void handleReachedGoal(const std::vector<Pose2D> & poses)
  {
    const size_t goal_index = std::min(activeGoalIndex(), poses.size() - 1);
    const bool has_next_goal = goal_index + 1 < poses.size();

    if (has_next_goal) {
      incrementActiveGoalIndex();
      publishCurrentGoal("Goal reached, publishing next");
      return;
    }

    if (current_mode_ == TaskMode::Bulldoze) {
      const size_t finished_region_id =
        bulldoze_regions_.at(active_bulldoze_region_index_).id;
      RCLCPP_INFO(
        get_logger(), "Bulldoze region %zu completed. Returning to nearest inspection goal.",
        finished_region_id);
      returnToNearestInspection("Bulldoze completed");
      return;
    }

    if (loop_inspection_goals_) {
      inspection_goal_index_ = 0;
      publishCurrentGoal("Inspection loop completed, restarting from first goal");
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Final inspection goal reached and loop_goals=false. Holding the last goal.");
    }
  }

  void publishCurrentGoal(const std::string & reason)
  {
    const auto * poses = activePoses();
    if (poses == nullptr || poses->empty()) {
      return;
    }

    const size_t goal_index = std::min(activeGoalIndex(), poses->size() - 1);
    const auto goal = activeGoalPose();

    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = frame_id_;
    msg.header.stamp = now();
    msg.pose = makePose(goal.x, goal.y, goal.yaw);
    goal_pub_->publish(msg);

    active_goal_sent_ = true;

    RCLCPP_INFO(
      get_logger(), "%s: [%s] goal %zu/%zu x=%.2f y=%.2f yaw=%.1f deg",
      reason.c_str(), modeName().c_str(), goal_index + 1, poses->size(),
      goal.x, goal.y, goal.yaw * 180.0 / kPi);
  }

  void publishVisualization()
  {
    const auto stamp = now();

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = frame_id_;
    pose_array.header.stamp = stamp;

    const auto * current_poses = activePoses();
    if (current_poses != nullptr) {
      pose_array.poses.reserve(current_poses->size());
      for (const auto & pose : *current_poses) {
        pose_array.poses.push_back(makePose(pose.x, pose.y, pose.yaw));
      }
    }
    pose_array_pub_->publish(pose_array);

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(makeDeleteAllMarker(stamp));
    marker_array.markers.push_back(makeInspectionPathMarker(stamp));
    marker_array.markers.push_back(makeInspectionPointMarker(stamp));

    int marker_id = 100;
    for (size_t i = 0; i < bulldoze_regions_.size(); ++i) {
      marker_array.markers.push_back(makeBulldozeRectangleMarker(stamp, bulldoze_regions_[i], marker_id++));
      marker_array.markers.push_back(makeBulldozePathMarker(stamp, bulldoze_regions_[i], marker_id++));
      marker_array.markers.push_back(makeBulldozePointMarker(stamp, bulldoze_regions_[i], marker_id++));
    }

    marker_array.markers.push_back(makeCurrentGoalArrowMarker(stamp, marker_id++));
    marker_array_pub_->publish(marker_array);
  }

  visualization_msgs::msg::Marker makeDeleteAllMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
  }

  visualization_msgs::msg::Marker makeInspectionPathMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "inspection";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.color = makeColor(0.95f, 0.60f, 0.10f, 0.95f);

    for (const auto & pose : inspection_poses_) {
      marker.points.push_back(makePoint(pose.x, pose.y, 0.03));
    }
    return marker;
  }

  visualization_msgs::msg::Marker makeInspectionPointMarker(const rclcpp::Time & stamp) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "inspection";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale_ * 1.2;
    marker.scale.y = marker_scale_ * 1.2;
    marker.scale.z = marker_scale_ * 1.2;
    marker.color = makeColor(1.0f, 0.48f, 0.12f, 0.95f);

    for (const auto & pose : inspection_poses_) {
      marker.points.push_back(makePoint(pose.x, pose.y, 0.08));
    }
    return marker;
  }

  std::vector<Vec2> getRectangleCorners(const BulldozeRegion & region) const
  {
    const Vec2 p0 = region.origin;
    const Vec2 p1 = region.origin + region.forward_unit * region.forward_length;
    const Vec2 p2 = p1 + region.lateral_unit * region.lateral_length;
    const Vec2 p3 = region.origin + region.lateral_unit * region.lateral_length;
    return {p0, p1, p2, p3, p0};
  }

  visualization_msgs::msg::Marker makeBulldozeRectangleMarker(
    const rclcpp::Time & stamp, const BulldozeRegion & region, int marker_id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "bulldoze_rectangle";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.06;

    const bool is_active =
      current_mode_ == TaskMode::Bulldoze &&
      active_bulldoze_region_index_ != kNoRegionSelected &&
      active_bulldoze_region_index_ < bulldoze_regions_.size() &&
      bulldoze_regions_[active_bulldoze_region_index_].id == region.id;
    marker.color = is_active ?
      makeColor(0.00f, 0.75f, 0.30f, 1.0f) :
      makeColor(0.10f, 0.20f, 0.10f, 0.75f);

    for (const auto & corner : getRectangleCorners(region)) {
      marker.points.push_back(makePoint(corner));
    }
    return marker;
  }

  visualization_msgs::msg::Marker makeBulldozePathMarker(
    const rclcpp::Time & stamp, const BulldozeRegion & region, int marker_id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "bulldoze_path";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.045;

    const bool is_active =
      current_mode_ == TaskMode::Bulldoze &&
      active_bulldoze_region_index_ != kNoRegionSelected &&
      active_bulldoze_region_index_ < bulldoze_regions_.size() &&
      bulldoze_regions_[active_bulldoze_region_index_].id == region.id;
    marker.color = is_active ?
      makeColor(0.12f, 0.65f, 1.0f, 0.95f) :
      makeColor(0.33f, 0.63f, 0.92f, 0.45f);

    for (const auto & pose : region.poses) {
      marker.points.push_back(makePoint(pose.x, pose.y, 0.03));
    }
    return marker;
  }

  visualization_msgs::msg::Marker makeBulldozePointMarker(
    const rclcpp::Time & stamp, const BulldozeRegion & region, int marker_id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "bulldoze_point";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_scale_;
    marker.scale.y = marker_scale_;
    marker.scale.z = marker_scale_;

    const bool is_active =
      current_mode_ == TaskMode::Bulldoze &&
      active_bulldoze_region_index_ != kNoRegionSelected &&
      active_bulldoze_region_index_ < bulldoze_regions_.size() &&
      bulldoze_regions_[active_bulldoze_region_index_].id == region.id;
    marker.color = is_active ?
      makeColor(0.88f, 0.22f, 0.22f, 0.95f) :
      makeColor(0.88f, 0.22f, 0.22f, 0.35f);

    for (const auto & pose : region.poses) {
      marker.points.push_back(makePoint(pose.x, pose.y, 0.06));
    }
    return marker;
  }

  visualization_msgs::msg::Marker makeCurrentGoalArrowMarker(
    const rclcpp::Time & stamp, int marker_id) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = stamp;
    marker.ns = "active_goal";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.35;
    marker.scale.y = 0.08;
    marker.scale.z = 0.10;
    marker.color = current_mode_ == TaskMode::Bulldoze ?
      makeColor(0.00f, 0.67f, 0.36f, 0.95f) :
      makeColor(0.95f, 0.50f, 0.00f, 0.95f);

    const auto * poses = activePoses();
    if (poses != nullptr && !poses->empty()) {
      const auto pose = activeGoalPose();
      marker.pose = makePose(pose.x, pose.y, pose.yaw);
    } else {
      marker.pose.orientation.w = 1.0;
    }
    return marker;
  }

  std::string modeName() const
  {
    return current_mode_ == TaskMode::Bulldoze ? "bulldoze" : "inspection";
  }

  std::string frame_id_;
  std::string inspection_pose_topic_;
  std::string bulldoze_clicked_point_topic_;
  std::string task_command_topic_;
  std::string goal_topic_;
  std::string odom_topic_;
  double origin_x_{0.0};
  double origin_y_{0.0};
  double rect_width_{0.0};
  double rect_length_{0.0};
  double margin_{0.0};
  double desired_spacing_{0.0};
  double publish_rate_hz_{1.0};
  double marker_scale_{0.18};
  bool use_clicked_points_{true};
  double goal_reach_tolerance_m_{0.6};
  double goal_yaw_tolerance_rad_{0.12};
  double stop_speed_tolerance_mps_{0.05};
  bool require_goal_yaw_alignment_{false};
  bool require_stop_at_goal_{false};
  double goal_settle_time_s_{1.0};
  double republish_goal_interval_s_{1.0};
  bool loop_inspection_goals_{true};

  bool active_goal_sent_{false};
  bool has_odom_{false};
  bool waiting_goal_settle_{false};

  TaskMode current_mode_{TaskMode::Inspection};
  size_t inspection_goal_index_{0};
  size_t bulldoze_goal_index_{0};
  size_t active_bulldoze_region_index_{kNoRegionSelected};
  size_t next_bulldoze_region_id_{1};

  Vec2 rectangle_origin_{0.0, 0.0};
  Vec2 forward_unit_{1.0, 0.0};
  Vec2 lateral_unit_{0.0, 1.0};
  double forward_length_{0.0};
  double lateral_length_{0.0};
  std::vector<Vec2> bulldoze_clicked_points_;

  std::vector<Pose2D> inspection_poses_;
  std::vector<BulldozeRegion> bulldoze_regions_;

  rclcpp::Time goal_settle_start_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr inspection_pose_sub_;
  rclcpp::Subscription<PointStamped>::SharedPtr bulldoze_clicked_point_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_command_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  Odometry::ConstSharedPtr latest_odom_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
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
