from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    nodes = []

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