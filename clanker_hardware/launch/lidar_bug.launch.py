from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="clanker_hardware",
            executable="lidar_bug.py",
            name="lidar_bug",
            parameters=[{}],
        ),
    ])