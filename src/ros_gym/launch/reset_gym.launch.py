from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        Node(
        package='ros_gym', 
        executable='reset_spawn_node', 
        output= 'screen',
        name='reset_spawn_node'
        )
    )

generate_launch_description()