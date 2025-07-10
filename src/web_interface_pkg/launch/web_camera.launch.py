from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_stream_pkg',
            executable='camera_publisher',
            name='camera_publisher'
        ),
        Node(
            package='web_interface_pkg',
            executable='web_bridge_node',
            name='web_bridge_node'
        ),
    ])