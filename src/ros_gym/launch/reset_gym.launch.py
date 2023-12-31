from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_description = LaunchDescription([
        DeclareLaunchArgument(
            name='request',
            default_value='true'
        ), 
        Node(
        package='ros_gym',
        executable='reset_client',
        name='reset_client',
        output='screen',
        parameters=[
            {
                'request': LaunchConfiguration('request')
            },
     ] ),
     Node(
        package='ros_gym',
        executable='reset_service',
        name='reset_service',
        output='screen',
    #     parameters=[
    #         # {
    #         #      'request': LaunchConfiguration('request')
    #         # },
    #  ] 
     )
])

    return launch_description

if __name__ == '__main__':
    generate_launch_description()