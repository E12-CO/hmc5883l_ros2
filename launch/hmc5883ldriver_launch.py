from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    ld = LaunchDescription()
    
    hmc5883ldriver_node = Node(
        package='hmc5883ldriver',
        executable='hmc5883ldriver',
        name='hmc5883ldriver_node',
        output="screen",
        emulate_tty=True,        
    )

    ld.add_action(hmc5883ldriver_node)
    return ld
