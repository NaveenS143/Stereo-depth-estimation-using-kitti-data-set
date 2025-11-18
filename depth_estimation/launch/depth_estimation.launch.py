import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for starting the depth estimation node.
    """
    
    pkg_share_path = get_package_share_directory('depth_estimation')
    params_file_path = os.path.join(pkg_share_path, 'config', 'config.yaml')
    
    depth_estimation_node = Node(
        package='depth_estimation',
        executable='depth_estimation_node',           
        output='screen',
        emulate_tty=True,
        parameters=[params_file_path]
    )

    ld = LaunchDescription()
    ld.add_action(depth_estimation_node)
    
    return ld