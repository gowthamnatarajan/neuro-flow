import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/usb_cam/image_raw",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="25",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=os.path.expanduser("~/projects/neuro-flow/datasets"),
        ),
        DeclareLaunchArgument(
            "task",
            default_value="cube_pick",
        ),

        # Collector runs in background — its logs go to this terminal
        Node(
            package="data_collection",
            executable="collector",
            name="collector_node",
            output="screen",
            parameters=[{
                "camera_topic": LaunchConfiguration("camera_topic"),
                "fps": LaunchConfiguration("fps"),
                "output_dir": LaunchConfiguration("output_dir"),
                "task": LaunchConfiguration("task"),
            }],
        ),
    ])
