from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_file = '/home/evenchova/turtlebot3_ws/src/patrol_sim/maps/map.yaml'

    # shared map_server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
    )

    # list of namespaces (your TB3_1..TB3_4)
    robots = [f'TB3_{i+1}' for i in range(4)]
    nodes = [map_server]

    for ns in robots:
        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'use_namespace': 'True',
                'map': map_file,
                'use_sim_time': 'True',
                'autostart': 'True'
            }.items()
        )

        # set initial pose (your node)
        init_pose = Node(
            package='patrol_sim',
            executable='set_initial_pose',
            name='set_initial_pose',
            namespace=ns,
            output='screen',
            parameters=[{'robot_ns': ns}]
        )

        nodes += [bringup, init_pose]

    return LaunchDescription(nodes)
