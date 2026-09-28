from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_cpp_practice",
                executable="number_scaler",
                name="number_scaler",
                parameters=[{"scale": 2.0}],
            )
        ]
    )