from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_file = os.path.expanduser('~/turtlebot3_ws/src/patrol_sim/maps/map.yaml')

    robots = [f'TB3_{i+1}' for i in range(4)]
    launches = []

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
            }.items()
        )
        launches.append(bringup)

    return LaunchDescription(launches)
