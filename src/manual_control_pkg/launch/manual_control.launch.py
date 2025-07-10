#!/usr/bin/env python3
"""
Launch file for JABARI Manual Control Node

This launch file provides various configurations for running the manual control node
with different parameters and in combination with other nodes.

Usage:
    ros2 launch manual_control manual_control.launch.py
    ros2 launch manual_control manual_control.launch.py use_sim_time:=true
    ros2 launch manual_control manual_control.launch.py max_linear_velocity:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with manual control node and parameters"""
    
    # Get package directory
    pkg_dir = get_package_share_directory('manual_control')
    config_dir = os.path.join(pkg_dir, 'config')
    default_config_file = os.path.join(config_dir, 'manual_control_params.yaml')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to configuration file'
    )
    
    declare_max_linear_velocity = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='1.0',
        description='Maximum linear velocity in m/s'
    )
    
    declare_max_angular_velocity = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='2.0',
        description='Maximum angular velocity in rad/s'
    )
    
    declare_command_timeout = DeclareLaunchArgument(
        'command_timeout',
        default_value='2.0',
        description='Command timeout in seconds'
    )
    
    declare_enable_debug = DeclareLaunchArgument(
        'enable_debug',
        default_value='false',
        description='Enable debug logging'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Manual Control Node
    manual_control_node = Node(
        package='manual_control',
        executable='manual_control_node',
        name='manual_control_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'command_timeout': LaunchConfiguration('command_timeout'),
                'enable_debug_logging': LaunchConfiguration('enable_debug'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Remap topics if needed
            # ('/manual_cmd_vel', '/robot/cmd_vel'),
        ]
    )
    
    # Optional: RQT GUI for manual control (if available)
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_debug')),
        remappings=[
            ('/cmd_vel', '/external_manual_cmd'),
        ]
    )
    
    # Log launch information
    launch_info = LogInfo(
        msg=[
            'Starting JABARI Manual Control Node with parameters:\n',
            '  Max Linear Velocity: ', LaunchConfiguration('max_linear_velocity'), ' m/s\n',
            '  Max Angular Velocity: ', LaunchConfiguration('max_angular_velocity'), ' rad/s\n',
            '  Command Timeout: ', LaunchConfiguration('command_timeout'), ' seconds\n',
            '  Debug Logging: ', LaunchConfiguration('enable_debug'), '\n',
            '  Configuration File: ', LaunchConfiguration('config_file')
        ]
    )
    
    return LaunchDescription([
        # Declare all launch arguments
        declare_use_sim_time,
        declare_config_file,
        declare_max_linear_velocity,
        declare_max_angular_velocity,
        declare_command_timeout,
        declare_enable_debug,
        declare_log_level,
        
        # Log launch information
        launch_info,
        
        # Launch nodes
        manual_control_node,
        rqt_robot_steering,
    ])


# Alternative launch configurations
def generate_simulation_launch_description():
    """Generate launch description for simulation environment"""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('max_linear_velocity', default_value='0.5'),
        DeclareLaunchArgument('max_angular_velocity', default_value='1.0'),
        DeclareLaunchArgument('enable_debug', default_value='true'),
        
        Node(
            package='manual_control',
            executable='manual_control_node',
            name='manual_control_node_sim',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'enable_debug_logging': LaunchConfiguration('enable_debug'),
                'command_timeout': 5.0,  # Longer timeout for simulation
                'deadband_threshold': 0.005,
            }]
        )
    ])