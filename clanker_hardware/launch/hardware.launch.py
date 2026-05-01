from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    lidar_serial_port = LaunchConfiguration('serial_port')

    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a1_launch.py',
    )
    rplidar_launch_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch),
        launch_arguments={
            "serial_port": lidar_serial_port,
        }.items()
    )

    return LaunchDescription([
        Node(
            package="telemetry",
            executable="telemetry",
            name="telemetry",
            parameters=[{}],
        ),
        Node(
            package="robo_rover",
            executable="rover_node",
            name="rover_node",
            parameters=[{}],
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='USB port for rplidar'
        ),
        rplidar_launch_action,
    ])