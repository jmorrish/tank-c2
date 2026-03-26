"""
nav2_planner_launch.py — Launch Nav2 planner server.

The planner_server creates its own global_costmap internally — no separate
costmap node needed. Only the planner_server is launched and managed.

Usage:
  ros2 launch /path/to/nav2_planner_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

DETECT = '/home/james/YOLOv8-TensorRT/csrc/jetson/detect'
PARAMS = f'{DETECT}/nav2_params.yaml'


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[PARAMS],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['planner_server'],
            }],
        ),
    ])
