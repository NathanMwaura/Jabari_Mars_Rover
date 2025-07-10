#!/usr/bin/env python3
"""
Launch file for Web Interface Package.
Launches the web bridge node with configurable parameters for Dell Latitude E7440.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    """Generate launch description for web interface package."""
    
    # Declare launch arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index (usually 0 for built-in camera)'
    )
    
    websocket_host_arg = DeclareLaunchArgument(
        'websocket_host',
        default_value='localhost',
        description='WebSocket server host address'
    )
    
    websocket_port_arg = DeclareLaunchArgument(
        'websocket_port',
        default_value='8765',
        description='WebSocket server port'
    )
    
    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value='640',
        description='Camera frame width'
    )
    
    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value='480',
        description='Camera frame height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30',
        description='Target frames per second'
    )
    
    publish_compressed_arg = DeclareLaunchArgument(
        'publish_compressed',
        default_value='true',
        description='Whether to publish compressed images'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='80',
        description='JPEG compression quality (1-100)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='web_bridge_node',
        description='Name of the web bridge node'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Node namespace'
    )
    
    # Configuration file path
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to configuration YAML file'
    )
    
    # Create the web bridge node
    web_bridge_node = Node(
        package='web_interface_pkg',
        executable='web_bridge_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            {
                'camera_index': LaunchConfiguration('camera_index'),
                'websocket_host': LaunchConfiguration('websocket_host'),
                'websocket_port': LaunchConfiguration('websocket_port'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps': LaunchConfiguration('fps'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )
    
    # Add config file if specified
    web_bridge_node_with_config = Node(
        package='web_interface_pkg',
        executable='web_bridge_node',
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'camera_index': LaunchConfiguration('camera_index'),
                'websocket_host': LaunchConfiguration('websocket_host'),
                'websocket_port': LaunchConfiguration('websocket_port'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
                'fps': LaunchConfiguration('fps'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0,
        condition=UnlessCondition(
            LaunchConfiguration('config_file').equals('')
        )
    )
    
    # Launch information
    launch_info = LogInfo(
        msg=[
            'Starting Web Interface Package for Dell Latitude E7440\n',
            'Camera Index: ', LaunchConfiguration('camera_index'), '\n',
            'WebSocket Server: ws://', LaunchConfiguration('websocket_host'), 
            ':', LaunchConfiguration('websocket_port'), '\n',
            'Resolution: ', LaunchConfiguration('frame_width'), 'x', 
            LaunchConfiguration('frame_height'), '\n',
            'Target FPS: ', LaunchConfiguration('fps'), '\n',
            'Log Level: ', LaunchConfiguration('log_level')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        camera_index_arg,
        websocket_host_arg,
        websocket_port_arg,
        frame_width_arg,
        frame_height_arg,
        fps_arg,
        publish_compressed_arg,
        jpeg_quality_arg,
        log_level_arg,
        node_name_arg,
        namespace_arg,
        config_file_arg,
        
        # Launch info
        launch_info,
        
        # Nodes
        web_bridge_node,
        web_bridge_node_with_config,
    ])