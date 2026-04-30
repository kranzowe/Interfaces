from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="clanker_hardware",
            executable="lidar_bug_v1.py",
            name="lidar_bug_v1",
            parameters=[{
                "neutral_speed": 1500.0,
                "ol_speed": 0.0,
                "neutral_steer": 1470.0,
                "steer_p": 5.0,
                "steer_d": 0.3,
                "steer_d_filter": 0.2,
                "steer_phi": 45.0,
                "steer_max": 400.0,
                "right_search_limit": 120.0,
                "left_search_limit": 10.0,
                "wall_ahead_distance": 1.5,
                "range_threshold": 0.6,
            }],
        ),
    ])
