from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("lidar_angle_filter"),
        "config",
        "angle_filter.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="lidar_angle_filter",
            executable="lidar_angle_filter_node",
            name="lidar_angle_filter",
            output="screen",
            parameters=[LaunchConfiguration("config")],
        ),
    ])
