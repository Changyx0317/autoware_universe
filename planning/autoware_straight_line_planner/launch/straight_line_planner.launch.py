from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('autoware_straight_line_planner')
    param_file = os.path.join(pkg_dir, 'config', 'straight_line_planner.param.yaml')

    return LaunchDescription([
        Node(
            package='autoware_straight_line_planner',
            executable='straight_line_planner_node',
            name='straight_line_planner_node',
            output='screen',
            parameters=[param_file],
        )
    ])
