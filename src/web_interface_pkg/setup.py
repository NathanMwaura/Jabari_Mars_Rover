from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'web_interface_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'opencv-python',
        'websockets',
        'asyncio',
        'numpy',
    ],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='user@example.com',
    description='ROS2 package for streaming camera feed to web interface via WebSocket',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_bridge_node = web_interface_pkg.web_bridge_node:main',
        ],
    },
)