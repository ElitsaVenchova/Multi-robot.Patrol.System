from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # define waypoint lists per robot (strings "x,y,yaw")
    wp_TB3_1 = ["-1.8,-1.8,0","0.0,-1.8,0","1.8,-1.8,0"]
    wp_TB3_2 = ["1.8,-1.8,90","1.8,0.0,90","1.8,1.8,90"]
    wp_TB3_3 = ["1.8,1.8,180","0.0,1.8,180","-1.8,1.8,180"]
    wp_TB3_4 = ["-1.8,1.8,-90","-1.8,0.0,-90","-1.8,-1.8,-90"]

    nodes = []
    nodes.append(Node(package='patrol_sim', executable='multi_patrol_nav2', name='patrol', namespace='TB3_1', output='screen',
                      parameters=[{'robot_ns':'TB3_1','waypoints': wp_TB3_1}]))
    nodes.append(Node(package='patrol_sim', executable='multi_patrol_nav2', name='patrol', namespace='TB3_2', output='screen',
                      parameters=[{'robot_ns':'TB3_2','waypoints': wp_TB3_2}]))
    nodes.append(Node(package='patrol_sim', executable='multi_patrol_nav2', name='patrol', namespace='TB3_3', output='screen',
                      parameters=[{'robot_ns':'TB3_3','waypoints': wp_TB3_3}]))
    nodes.append(Node(package='patrol_sim', executable='multi_patrol_nav2', name='patrol', namespace='TB3_4', output='screen',
                      parameters=[{'robot_ns':'TB3_4','waypoints': wp_TB3_4}]))

    return LaunchDescription(nodes)
