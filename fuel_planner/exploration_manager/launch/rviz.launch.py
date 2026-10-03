#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    plan_manage_share = get_package_share_directory('plan_manage')
    rviz_config = os.path.join(plan_manage_share, 'config', 'traj.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rvizvisualisation',
            output='log',
            arguments=['-d', rviz_config],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_53',
            arguments=['--frame-id', 'world', '--child-frame-id', 'navigation'],
        ),
    ])
