#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    expl_share = get_package_share_directory('exploration_manager')
    lkh_share = get_package_share_directory('lkh_tsp_solver')

    odom_topic = LaunchConfiguration('odom_topic').perform(context)
    depth_topic = LaunchConfiguration('depth_topic').perform(context)
    cloud_topic = LaunchConfiguration('cloud_topic').perform(context)

    map_size_x = float(LaunchConfiguration('map_size_x').perform(context))
    map_size_y = float(LaunchConfiguration('map_size_y').perform(context))
    map_size_z = float(LaunchConfiguration('map_size_z').perform(context))
    init_x = float(LaunchConfiguration('init_x').perform(context))
    init_y = float(LaunchConfiguration('init_y').perform(context))
    init_z = float(LaunchConfiguration('init_z').perform(context))
    max_vel = float(LaunchConfiguration('max_vel').perform(context))
    max_acc = float(LaunchConfiguration('max_acc').perform(context))
    box_min_x = float(LaunchConfiguration('box_min_x').perform(context))
    box_min_y = float(LaunchConfiguration('box_min_y').perform(context))
    box_min_z = float(LaunchConfiguration('box_min_z').perform(context))
    box_max_x = float(LaunchConfiguration('box_max_x').perform(context))
    box_max_y = float(LaunchConfiguration('box_max_y').perform(context))
    box_max_z = float(LaunchConfiguration('box_max_z').perform(context))
    cx = float(LaunchConfiguration('cx').perform(context))
    cy = float(LaunchConfiguration('cy').perform(context))
    fx = float(LaunchConfiguration('fx').perform(context))
    fy = float(LaunchConfiguration('fy').perform(context))
    return_home = LaunchConfiguration('return_home').perform(context).lower() in ('true', '1', 'yes')
    return_home_thresh = float(LaunchConfiguration('return_home_thresh').perform(context))

    exploration_params = [
        os.path.join(expl_share, 'config', 'exploration.yaml'),
        {
            'sdf_map/map_size_x': map_size_x,
            'sdf_map/map_size_y': map_size_y,
            'sdf_map/map_size_z': map_size_z,
            'sdf_map/box_min_x': box_min_x,
            'sdf_map/box_min_y': box_min_y,
            'sdf_map/box_min_z': box_min_z,
            'sdf_map/box_max_x': box_max_x,
            'sdf_map/box_max_y': box_max_y,
            'sdf_map/box_max_z': box_max_z,
            'map_ros/cx': cx,
            'map_ros/cy': cy,
            'map_ros/fx': fx,
            'map_ros/fy': fy,
            'exploration/vm': 1.0 * max_vel,
            'exploration/am': 1.0 * max_acc,
            'exploration/tsp_dir': os.path.join(lkh_share, 'resource'),
            'manager/max_vel': max_vel,
            'manager/max_acc': max_acc,
            'search/max_vel': max_vel,
            'search/max_acc': max_acc,
            'optimization/max_vel': max_vel,
            'optimization/max_acc': max_acc,
            'bspline/limit_vel': max_vel,
            'bspline/limit_acc': max_acc,
            'fsm/return_home': return_home,
            'fsm/return_home_thresh': return_home_thresh,
        },
    ]

    traj_params = [
        os.path.join(expl_share, 'config', 'traj_server.yaml'),
        {
            'traj_server/init_x': init_x,
            'traj_server/init_y': init_y,
            'traj_server/init_z': init_z,
        },
    ]

    exploration_node = Node(
        package='exploration_manager',
        executable='exploration_node',
        name='exploration_node',
        output='screen',
        parameters=exploration_params,
        remappings=[
            ('/odom_world', odom_topic),
            ('/map_ros/depth', depth_topic),
            ('/map_ros/cloud', cloud_topic),
        ],
    )

    traj_server = Node(
        package='plan_manage',
        executable='traj_server',
        name='traj_server',
        output='screen',
        parameters=traj_params,
        remappings=[
            ('/position_cmd', 'planning/pos_cmd'),
            ('/odom_world', odom_topic),
        ],
    )

    return [exploration_node, traj_server]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_size_x', default_value='100.0'),
        DeclareLaunchArgument('map_size_y', default_value='100.0'),
        DeclareLaunchArgument('map_size_z', default_value='10.0'),
        DeclareLaunchArgument('init_x', default_value='0.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='1.0'),
        DeclareLaunchArgument('odom_topic', default_value='/drone260/odom'),
        DeclareLaunchArgument('depth_topic', default_value='/pcl_render_node/depth'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/points_world'),
        DeclareLaunchArgument('cx', default_value='321.04638671875'),
        DeclareLaunchArgument('cy', default_value='243.44969177246094'),
        DeclareLaunchArgument('fx', default_value='387.229248046875'),
        DeclareLaunchArgument('fy', default_value='387.229248046875'),
        DeclareLaunchArgument('max_vel', default_value='1.2'),
        DeclareLaunchArgument('max_acc', default_value='2.0'),
        DeclareLaunchArgument('box_min_x', default_value='-40.0'),
        DeclareLaunchArgument('box_min_y', default_value='-40.0'),
        DeclareLaunchArgument('box_min_z', default_value='0.0'),
        DeclareLaunchArgument('box_max_x', default_value='40.0'),
        DeclareLaunchArgument('box_max_y', default_value='40.0'),
        DeclareLaunchArgument('box_max_z', default_value='2.0'),
        DeclareLaunchArgument('return_home', default_value='false',
                              description='Return to takeoff pose after exploration finishes'),
        DeclareLaunchArgument('return_home_thresh', default_value='0.5',
                              description='Arrival distance threshold for return-home (m)'),
        OpaqueFunction(function=launch_setup),
    ])
