import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()


    config = os.path.join(
        get_package_share_directory('uwb_simulator'),
        'config',
        'params.yaml'
        ),
           
    node1 = Node(
                package='uwb_simulator',
                executable='measurement_simulator',
                name='measurement_simulator',
                parameters=[config]
                )
                
    node2 = Node(
        package='uwb_simulator',
        executable='trajectory_simulator',
        name='trajectory_simulator',
        parameters=[config]
        )
    
    node3 = Node(
        package='uwb_simulator',
        executable='clock_publisher',
        name='clock_publisher',
        parameters=[config]
        )

    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)

    
    return ld