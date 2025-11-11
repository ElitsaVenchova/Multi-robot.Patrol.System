from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns', default='tb3_0')
    params_file = LaunchConfiguration('params_file', default='params_tb3_0.yaml')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': robot_ns,
            'use_namespace': 'true',
            'params_file': params_file,
            'map': '/path/to/your/map.yaml',
            # other args as needed
        }.items()
    )

    return LaunchDescription([bringup_launch])
