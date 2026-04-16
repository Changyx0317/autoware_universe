#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace autoware::diff_drive_control_adapter
{

class DiffDriveControlAdapterNode : public rclcpp::Node
{
public:
  DiffDriveControlAdapterNode() : Node("diff_drive_control_adapter")
  {
    input_control_cmd_topic_ =
      declare_parameter<std::string>("input_control_cmd_topic", "/control/command/control_cmd");
    input_gear_cmd_topic_ =
      declare_parameter<std::string>("input_gear_cmd_topic", "/control/command/gear_cmd");
    output_twist_stamped_topic_ = declare_parameter<std::string>(
      "output_twist_stamped_topic", "/control/command/diff_drive_twist_cmd");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "base_link");
    output_twist_topic_ =
      declare_parameter<std::string>("output_twist_topic", "/control/command/diff_drive_twist");
    output_left_wheel_speed_topic_ = declare_parameter<std::string>(
      "output_left_wheel_speed_topic", "/control/command/left_wheel_speed_radps");
    output_right_wheel_speed_topic_ = declare_parameter<std::string>(
      "output_right_wheel_speed_topic", "/control/command/right_wheel_speed_radps");

    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.1);
    wheel_tread_m_ = declare_parameter<double>("wheel_tread_m", 0.5);
    max_angular_velocity_rps_ = declare_parameter<double>("max_angular_velocity_rps", 1.2);
    use_gear_sign_ = declare_parameter<bool>("use_gear_sign", false);
    publish_unstamped_twist_ = declare_parameter<bool>("publish_unstamped_twist", true);
    publish_wheel_speeds_ = declare_parameter<bool>("publish_wheel_speeds", true);

    sub_control_cmd_ = create_subscription<autoware_control_msgs::msg::Control>(
      input_control_cmd_topic_, rclcpp::QoS{10},
      std::bind(&DiffDriveControlAdapterNode::onControlCmd, this, std::placeholders::_1));

    if (use_gear_sign_) {
      sub_gear_cmd_ = create_subscription<autoware_vehicle_msgs::msg::GearCommand>(
        input_gear_cmd_topic_, rclcpp::QoS{10},
        std::bind(&DiffDriveControlAdapterNode::onGearCmd, this, std::placeholders::_1));
    }

    pub_twist_stamped_ =
      create_publisher<geometry_msgs::msg::TwistStamped>(output_twist_stamped_topic_, 10);
    if (publish_unstamped_twist_) {
      pub_twist_ = create_publisher<geometry_msgs::msg::Twist>(output_twist_topic_, 10);
    }
    if (publish_wheel_speeds_) {
      pub_left_wheel_speed_ =
        create_publisher<std_msgs::msg::Float64>(output_left_wheel_speed_topic_, 10);
      pub_right_wheel_speed_ =
        create_publisher<std_msgs::msg::Float64>(output_right_wheel_speed_topic_, 10);
    }

    RCLCPP_INFO(
      get_logger(),
      "diff_drive_control_adapter started. input=%s output_twist=%s",
      input_control_cmd_topic_.c_str(), output_twist_stamped_topic_.c_str());
  }

private:
  void onGearCmd(const autoware_vehicle_msgs::msg::GearCommand::SharedPtr msg)
  {
    last_gear_cmd_ = msg->command;
    has_gear_cmd_ = true;
  }

  bool isReverseGear(const uint8_t gear) const
  {
    return (
      gear == autoware_vehicle_msgs::msg::GearCommand::REVERSE ||
      gear == autoware_vehicle_msgs::msg::GearCommand::REVERSE_2);
  }

  bool isDriveGear(const uint8_t gear) const
  {
    return (
      gear >= autoware_vehicle_msgs::msg::GearCommand::DRIVE &&
      gear <= autoware_vehicle_msgs::msg::GearCommand::DRIVE_18);
  }

  double applyGearSign(const double v_cmd) const
  {
    if (!use_gear_sign_ || !has_gear_cmd_) {
      return v_cmd;
    }
    if (isReverseGear(last_gear_cmd_)) {
      return -std::abs(v_cmd);
    }
    if (isDriveGear(last_gear_cmd_)) {
      return std::abs(v_cmd);
    }
    return 0.0;
  }

  void onControlCmd(const autoware_control_msgs::msg::Control::SharedPtr msg)
  {
    const double v_raw = static_cast<double>(msg->longitudinal.velocity);
    const double w_raw = static_cast<double>(msg->lateral.steering_tire_angle);
    const double v = applyGearSign(v_raw);
    const double w = std::clamp(w_raw, -max_angular_velocity_rps_, max_angular_velocity_rps_);

    geometry_msgs::msg::TwistStamped twist_stamped;
    twist_stamped.header.stamp = msg->stamp;
    twist_stamped.header.frame_id = output_frame_id_;
    twist_stamped.twist.linear.x = v;
    twist_stamped.twist.angular.z = w;
    pub_twist_stamped_->publish(twist_stamped);

    if (publish_unstamped_twist_ && pub_twist_) {
      geometry_msgs::msg::Twist twist;
      twist.linear.x = v;
      twist.angular.z = w;
      pub_twist_->publish(twist);
    }

    if (publish_wheel_speeds_ && pub_left_wheel_speed_ && pub_right_wheel_speed_) {
      const double half_tread = 0.5 * std::max(1e-3, wheel_tread_m_);
      const double radius = std::max(1e-3, wheel_radius_m_);
      const double v_left = v - half_tread * w;
      const double v_right = v + half_tread * w;

      std_msgs::msg::Float64 wl;
      std_msgs::msg::Float64 wr;
      wl.data = v_left / radius;
      wr.data = v_right / radius;
      pub_left_wheel_speed_->publish(wl);
      pub_right_wheel_speed_->publish(wr);
    }
  }

private:
  std::string input_control_cmd_topic_;
  std::string input_gear_cmd_topic_;
  std::string output_twist_stamped_topic_;
  std::string output_frame_id_;
  std::string output_twist_topic_;
  std::string output_left_wheel_speed_topic_;
  std::string output_right_wheel_speed_topic_;

  double wheel_radius_m_{0.1};
  double wheel_tread_m_{0.5};
  double max_angular_velocity_rps_{1.2};
  bool use_gear_sign_{false};
  bool publish_unstamped_twist_{true};
  bool publish_wheel_speeds_{true};

  uint8_t last_gear_cmd_{autoware_vehicle_msgs::msg::GearCommand::NONE};
  bool has_gear_cmd_{false};

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr sub_control_cmd_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearCommand>::SharedPtr sub_gear_cmd_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_stamped_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_twist_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_wheel_speed_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_wheel_speed_;
};

}  // namespace autoware::diff_drive_control_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autoware::diff_drive_control_adapter::DiffDriveControlAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
