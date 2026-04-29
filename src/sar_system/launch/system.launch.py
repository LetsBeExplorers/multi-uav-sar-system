import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('sar_system'),
        'config', 'params.yaml'
    )
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    nodes = [
        DeclareLaunchArgument(
            'world_name',
            default_value='quadcopter_world',
            description='Gazebo world name (must match <world name=...> in the SDF)'
        )
    ]

    world_name = LaunchConfiguration('world_name')
    uavs = ['x1', 'x2', 'x3']

    for uav in uavs:
        # FSM
        nodes.append(Node(
            package='swarm_coordination',
            executable='uav_state_manager',
            name='uav_state_manager_' + uav,
            parameters=[{
                'uav_id': uav,
                'num_uavs': cfg['num_uavs'],
                'threshold': cfg['threshold'],
            }]
        ))

        # Swarm coordinator
        nodes.append(Node(
            package='swarm_coordination',
            executable='swarm_coordinator',
            name='swarm_coordinator_' + uav,
            parameters=[{
                'uav_id': uav,
                'num_uavs': cfg['num_uavs'],
                'area_bounds': cfg['area_bounds'],
                'rows': cfg['rows'],
                'threshold': cfg['threshold'],
            }]
        ))

        # Platform interface
        nodes.append(Node(
            package='uav_platform',
            executable='platform_interface',
            name='platform_interface_' + uav,
            parameters=[{'uav_name': uav}]
        ))

        # Gazebo driver
        nodes.append(Node(
            package='uav_platform',
            executable='gazebo_driver',
            name='gazebo_driver_' + uav,
            parameters=[{'uav_name': uav}]
        ))

        # World model
        nodes.append(Node(
            package='navigation',
            executable='world_model',
            name='world_model_' + uav,
            parameters=[{
                'uav_id': uav,
                'num_uavs': cfg['num_uavs'],
                'grid_width': cfg['grid_width'],
                'grid_height': cfg['grid_height'],
                'resolution': cfg['resolution'],
                'origin_x': cfg['origin_x'],
                'origin_y': cfg['origin_y'],
                'collision_threshold': cfg['collision_threshold'],
            }]
        ))

        # Lidar bridge
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='scan_bridge_' + uav,
            arguments=[
                [
                    '/world/', world_name,
                    '/model/', uav,
                    '/link/base_link/sensor/lidar/scan'
                    '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                ]
            ],
            remappings=[
                (
                    ['/world/', world_name,
                     '/model/', uav,
                     '/link/base_link/sensor/lidar/scan'],
                    f'/{uav}/scan'
                )
            ]
        ))

        # A* planner
        nodes.append(Node(
            package='navigation',
            executable='astar_navigation_node',
            name='astar_navigation_' + uav,
            parameters=[{
                'uav_id': uav,
                'replan_check_rate': cfg['replan_check_rate'],
            }]
        ))

        # Path executor
        nodes.append(Node(
            package='navigation',
            executable='path_executor',
            name='path_executor_' + uav,
            parameters=[{
                'uav_id': uav,
                'speed': cfg.get(f'speed_{uav}', cfg['speed']),
                'waypoint_threshold': cfg['waypoint_threshold'],
                'lookahead': cfg['lookahead'],
            }]
        ))

    return LaunchDescription(nodes)