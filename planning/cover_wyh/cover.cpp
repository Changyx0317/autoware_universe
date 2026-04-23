#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "autoware_adapi_v1_msgs/msg/route_state.hpp"
#include "autoware_adapi_v1_msgs/srv/clear_route.hpp"
#include "autoware_adapi_v1_msgs/srv/set_route_points.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{

using autoware_adapi_v1_msgs::msg::RouteState;
using autoware_adapi_v1_msgs::srv::ClearRoute;
using autoware_adapi_v1_msgs::srv::SetRoutePoints;
using geometry_msgs::msg::PointStamped;
using namespace std::chrono_literals;

constexpr double kPi = 3.14159265358979323846;

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
    clicked_point_topic_ =
      declare_parameter<std::string>("clicked_point_topic", "/clicked_point");
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

    if (use_clicked_points_) {
      clicked_point_sub_ = create_subscription<PointStamped>(
        clicked_point_topic_, 10,
        std::bind(&CoveragePlannerNode::onClickedPoint, this, std::placeholders::_1));
    } else {
      configureFromParameters();
      buildCoveragePlan();
    }

    set_route_client_ = create_client<SetRoutePoints>(set_route_service_);
    clear_route_client_ = create_client<ClearRoute>(clear_route_service_);
    routing_timer_ = create_wall_timer(500ms, std::bind(&CoveragePlannerNode::processRouting, this));

    RCLCPP_INFO(get_logger(), "Coverage planner started.");
    RCLCPP_INFO(get_logger(), "frame_id=%s", frame_id_.c_str());
    RCLCPP_INFO(
      get_logger(), "Routing uses %s and %s. /api/routing/route is the route readback topic.",
      set_route_service_.c_str(), route_state_topic_.c_str());

    if (use_clicked_points_) {
      RCLCPP_INFO(
        get_logger(),
        "Waiting for three clicks on %s to define a rotated rectangle on the map.",
        clicked_point_topic_.c_str());
      RCLCPP_INFO(
        get_logger(),
        "Click 1: start corner. Click 2: forward direction and length. Click 3: width and side.");
    } else {
      RCLCPP_INFO(
        get_logger(), "Using parameter rectangle: origin=(%.2f, %.2f) width=%.2f length=%.2f",
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

  void onClickedPoint(const PointStamped::ConstSharedPtr msg)
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
      scheduleReplan();
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
    if (!has_active_rectangle_ || poses_.empty()) {
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

    if (coverage_completed_) {
      return;
    }

    if (route_state_ == RouteState::ARRIVED && active_goal_sent_ && !awaiting_route_clear_) {
      if (current_goal_index_ + 1 >= poses_.size()) {
        coverage_completed_ = true;
        active_goal_sent_ = false;
        advance_goal_after_clear_ = false;
        awaiting_route_clear_ = false;
        RCLCPP_INFO(
          get_logger(),
          "Reached the final goal (%zu/%zu). Coverage completed, holding position.",
          current_goal_index_ + 1, poses_.size());
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
    const auto & goal = poses_.at(current_goal_index_);
    auto request = std::make_shared<SetRoutePoints::Request>();
    request->header.frame_id = frame_id_;
    request->header.stamp = now();
    request->goal = makePose(goal.x, goal.y, goal.yaw);
    request->option.allow_goal_modification = allow_goal_modification_;

    request_in_flight_ = true;
    const size_t goal_index = current_goal_index_;

    RCLCPP_INFO(
      get_logger(), "Sending goal %zu/%zu: x=%.2f y=%.2f yaw=%.1f deg", goal_index + 1,
      poses_.size(), goal.x, goal.y, goal.yaw * 180.0 / kPi);

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
    current_goal_index_ = (current_goal_index_ + 1) % poses_.size();
    RCLCPP_INFO(
      get_logger(), "Advancing to next goal. Next index is %zu/%zu.", current_goal_index_ + 1,
      poses_.size());
  }

  void publishVisualization()
  {
    const auto stamp = now();

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = frame_id_;
    pose_array.header.stamp = stamp;
    if (has_active_rectangle_) {
      pose_array.poses.reserve(poses_.size());
      for (const auto & pose : poses_) {
        pose_array.poses.push_back(makePose(pose.x, pose.y, pose.yaw));
      }
    }
    pose_array_pub_->publish(pose_array);

    visualization_msgs::msg::MarkerArray marker_array;
    if (has_active_rectangle_) {
      marker_array.markers.push_back(makeRectangleMarker(stamp));
      marker_array.markers.push_back(makePathMarker(stamp));
      marker_array.markers.push_back(makePosePointMarker(stamp));
      marker_array.markers.push_back(makeCurrentGoalArrowMarker(stamp));
    } else {
      marker_array.markers.push_back(makeDeleteMarker(stamp, 0));
      marker_array.markers.push_back(makeDeleteMarker(stamp, 1));
      marker_array.markers.push_back(makeDeleteMarker(stamp, 2));
      marker_array.markers.push_back(makeDeleteMarker(stamp, 3));
    }
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

    if (!poses_.empty()) {
      const auto & pose = poses_.at(current_goal_index_);
      marker.pose = makePose(pose.x, pose.y, pose.yaw);
    } else {
      marker.pose.orientation.w = 1.0;
    }
    return marker;
  }

  std::string frame_id_;
  std::string clicked_point_topic_;
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

  bool has_route_state_{false};
  bool has_active_rectangle_{false};
  bool request_in_flight_{false};
  bool active_goal_sent_{false};
  bool awaiting_route_clear_{false};
  bool advance_goal_after_clear_{false};
  bool replan_requested_{false};
  bool coverage_completed_{false};
  uint16_t route_state_{RouteState::UNKNOWN};
  size_t current_goal_index_{0};

  Vec2 rectangle_origin_{0.0, 0.0};
  Vec2 forward_unit_{1.0, 0.0};
  Vec2 lateral_unit_{0.0, 1.0};
  double forward_length_{0.0};
  double lateral_length_{0.0};
  std::vector<Vec2> clicked_points_;

  std::unique_ptr<RectBoundaryPlanner> planner_;
  std::vector<Pose2D> poses_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Subscription<PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<RouteState>::SharedPtr route_state_sub_;
  rclcpp::Client<SetRoutePoints>::SharedPtr set_route_client_;
  rclcpp::Client<ClearRoute>::SharedPtr clear_route_client_;
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
