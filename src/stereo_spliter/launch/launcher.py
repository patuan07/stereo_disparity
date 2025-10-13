from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="stereo_spliter",
            executable="right_stereo_viewer",
            name="right_stereo_publisher",
            output="screen"
        ),

        Node(
            package="stereo_spliter",
            executable="left_stereo_viewer",
            name="left_stereo_publisher",
            output="screen"
        ),

        Node(
            package="stereo_spliter",
            executable="left_pub",
            name="left_camera",
            output="screen"
        ),

        Node(
            package="stereo_spliter",
            executable="right_pub",
            name="right_camera",
            output="screen"
        ),
    ])
