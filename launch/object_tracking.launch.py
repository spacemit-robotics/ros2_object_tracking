"""Launch object_tracking node with config."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("object_tracking"), "config", "object_tracking.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_camera", default_value="true"),
            DeclareLaunchArgument("tracker_type", default_value="bytetrack"),
            DeclareLaunchArgument("score_threshold", default_value="0.25"),
            Node(
                package="object_tracking",
                executable="object_tracking_node",
                name="object_tracking_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_camera": LaunchConfiguration("use_camera"),
                        "tracker_type": LaunchConfiguration("tracker_type"),
                        "score_threshold": LaunchConfiguration("score_threshold"),
                    },
                ],
            )
        ]
    )
