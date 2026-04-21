from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    nodes = []

    nodes.append(
        DeclareLaunchArgument(
            'world_name',
            default_value='quadcopter_world',
            description='Gazebo world name (must match <world name=...> in the SDF)'
        )
    )

    world_name = LaunchConfiguration('world_name')

    # List of UAVs in the system
    uavs = ['x1', 'x2', 'x3']

    for uav in uavs:

        # Platform interface (handles control commands)
        nodes.append(
            Node(
                package='uav_platform',
                executable='platform_interface',
                name='platform_interface_' + uav,
                parameters=[{'uav_name': uav}]
            )
        )

        # Gazebo driver (simulation interface)
        nodes.append(
            Node(
                package='uav_platform',
                executable='gazebo_driver',
                name='gazebo_driver_' + uav,
                parameters=[{'uav_name': uav}]
            )
        )

        # World model (occupancy grid + sensor mapping)
        nodes.append(
            Node(
                package='navigation',
                executable='world_model',
                name='world_model_' + uav,
                parameters=[{
                    'uav_id': uav,
                    'num_uavs': 3,
                    'grid_width': 21,
                    'grid_height': 21,
                    'resolution': 1.0,
                    'origin_x': -10.0,
                    'origin_y': -10.0,
                    'inflation_radius': 1,
                    'collision_threshold': 0.5,
                }]
            )
        )

        # Bridge: lidar scan from Gazebo → ROS 2
        nodes.append(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='scan_bridge_' + uav,
                arguments=[
                    [
                        '/world/', world_name,
                        '/model/', uav,
                        '/link/X3/base_link/sensor/lidar/scan'
                        '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                    ]
                ],
                remappings=[
                    (
                        ['/world/', world_name,
                         '/model/', uav,
                         '/link/X3/base_link/sensor/lidar/scan'],
                        f'/{uav}/scan'
                    )
                ]
            )
        )

        # Path executor (follows waypoints)
        nodes.append(
            Node(
                package='navigation',
                executable='path_executor',
                name='path_executor_' + uav,
                parameters=[{'uav_name': uav}]
            )
        )

        # Swarm coordinator (generates waypoints)
        nodes.append(
            Node(
                package='swarm_coordination',
                executable='swarm_coordinator',
                name='swarm_coordinator_' + uav,
                parameters=[
                    {'uav_id': uav},
                    {'num_uavs': 3},
                    {'area_bounds': [-10, 10, -10, 10]},
                    {'rows': 7}
                ]
            )
        )

        # A* navigation (obstacle-aware planning)
        nodes.append(
            Node(
                package='navigation',
                executable='astar_navigation_node',
                name='astar_navigation_' + uav,
                parameters=[
                    {'uav_name': uav}
                ]
            )
        )

    return LaunchDescription(nodes)
