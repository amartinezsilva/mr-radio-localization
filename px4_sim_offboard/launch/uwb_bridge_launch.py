from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    uwb_bridge_config = os.path.join(
        get_package_share_directory('px4_sim_offboard'),
        'config',
        'uwb_bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_parameter_bridge',
            output='screen',
            arguments=['--ros-args', '-p', f'config_file:={uwb_bridge_config}']
        )
    ])
