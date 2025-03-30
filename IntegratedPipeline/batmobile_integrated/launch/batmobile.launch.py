from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='batmobile_integrated',
            executable='coordinates_receiver',
            name='batmobile_node', 
            output='screen',
        ),
        Node(
            package='batmobile_integrated',
            executable='publisher_node.py', 
            name='python_publisher_node', 
            output='screen'
        )
    ])
