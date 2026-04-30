from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="clanker_hardware",
            executable="lidar_bug.py",
            name="reverse_lidar_bug",
            parameters=[{
                "reverse_driving": True,
                "neutral_steer": 1400.0,
            }],
        ),
    ])