from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('params'),
        'config',
        'veh_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='params',
            executable='params_node',
            name='params',
            parameters=[param_file_path]
        ),
    ])

