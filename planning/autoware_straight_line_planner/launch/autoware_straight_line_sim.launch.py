from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_path_arg = DeclareLaunchArgument("map_path")
    vehicle_model_arg = DeclareLaunchArgument("vehicle_model", default_value="sample_vehicle")
    sensor_model_arg = DeclareLaunchArgument("sensor_model", default_value="sample_sensor_kit")
    target_speed_arg = DeclareLaunchArgument("target_speed_mps", default_value="1.0")

    planning_simulator = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("autoware_launch"), "launch", "planning_simulator.launch.xml"]
            )
        ),
        launch_arguments={
            "map_path": LaunchConfiguration("map_path"),
            "vehicle_model": LaunchConfiguration("vehicle_model"),
            "sensor_model": LaunchConfiguration("sensor_model"),
            "planning_module_preset": "straight_line_external",
            "control_module_preset": "pp_pid_direct_passthrough",
        }.items(),
    )

    straight_line_planner = Node(
        package="autoware_straight_line_planner",
        executable="straight_line_planner_node",
        name="straight_line_planner_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("autoware_straight_line_planner"),
                    "config",
                    "straight_line_planner.param.yaml",
                ]
            ),
            {
                "target_speed_mps": LaunchConfiguration("target_speed_mps"),
                "output_trajectory_topic": "/planning/scenario_planning/trajectory",
            },
        ],
    )

    return LaunchDescription(
        [map_path_arg, vehicle_model_arg, sensor_model_arg, target_speed_arg, planning_simulator, straight_line_planner]
    )

