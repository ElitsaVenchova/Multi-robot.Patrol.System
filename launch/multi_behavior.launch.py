from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robots = [f'TB3_{i+1}' for i in range(4)]  # 4 robots from TurtleBot3 example
    nodes = []

    for ns in robots:
        nodes.append(
            Node(
                package='patrol_sim',
                executable='patrol_bot',
                name='patrol_bot',
                namespace=ns,
                output='screen',
                parameters=[{'robot_ns': ns}]
            )
        )

    return LaunchDescription(nodes)
