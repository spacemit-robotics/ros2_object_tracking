"""Launch object_tracking node with config."""
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("object_tracking"), "config", "object_tracking.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="object_tracking",
                executable="object_tracking_node",
                name="object_tracking_node",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
