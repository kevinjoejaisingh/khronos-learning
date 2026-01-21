from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_first_node',
            executable='simple_publisher',
            name='publisher'
        ),
        Node(
            package='my_first_node',
            executable='simple_subscriber',
            name='subscriber'
        )
    ])