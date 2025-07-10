from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'motor_control_pkg'
    pkg_share_path = get_package_share_directory(package_name)
    params_file_path = os.path.join(pkg_share_path, 'config', 'motors.yaml')

    return LaunchDescription([

        # Converts Twist to speed topics
        Node(
            package=package_name,
            executable='motor_controller_node',
            name='motor_controller',
            output='screen',
        ),

        # Only test left motor driver
        Node(
            package=package_name,
            executable='sided_motor_driver_node',
            name='left_primary_driver',
            output='screen',
            parameters=[params_file_path]
        ),
    ])

