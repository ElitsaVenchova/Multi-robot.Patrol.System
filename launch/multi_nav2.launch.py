from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    map_file = '/home/evenchova/turtlebot3_ws/src/patrol_sim/maps/map.yaml'

    # --- Shared global map server ---
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'topic_name': 'map',
            'frame_id': 'map',
            'use_sim_time': True
        }]
    )

    # Nav2 lifecycle manager
    amcl_nodes = [f'TB3_{i+1}/amcl' for i in range(4)]
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'autostart': True,
            'use_sim_time': True,
            'bond_timeout': 0.0,
            'node_names': ['map_server'] + amcl_nodes
        }]
    )

    robots = [f'TB3_{i+1}' for i in range(4)]
    nodes = []

    for ns in robots:
        nav2_yaml = f'/home/evenchova/turtlebot3_ws/src/patrol_sim/config/{ns}_amcl_config.yaml'

        # Initial pose publisher
        init_pose = Node(
            package='patrol_sim',
            executable='set_initial_pose',
            name='set_initial_pose',
            namespace=ns,
            output='screen',
            parameters=[{'robot_ns': ns}]
        )

        # Namespaced AMCL with map remapping
        nav2_amcl = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace=ns,
            output='screen',
            parameters=[nav2_yaml],
            remappings=[
                ('/map', 'map'),
                ('/map_updates', 'map_updates')
            ]
        )

        nodes += [init_pose, nav2_amcl]

    return LaunchDescription([map_server] + nodes + [nav2_lifecycle_manager])
