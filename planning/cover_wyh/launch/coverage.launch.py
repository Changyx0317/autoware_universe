from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("cover_wyh"), "config", "coverage_params.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="cover_wyh",
                executable="rect_coverage_planner",
                name="rect_coverage_planner",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
