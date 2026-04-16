#include <memory>

#include "autoware/dwa_trajectory_follower/dwa_trajectory_follower_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // 初始化 ROS2 通信中间件。
  rclcpp::init(argc, argv);

  // 创建 DWA 控制节点实例。
  auto node = std::make_shared<autoware::dwa_trajectory_follower::DwaTrajectoryFollowerNode>();

  // 进入事件循环：处理订阅回调 + 定时器回调。
  rclcpp::spin(node);

  // 正常退出前关闭 ROS2 上下文。
  rclcpp::shutdown();
  return 0;
}
