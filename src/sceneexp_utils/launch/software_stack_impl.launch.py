from typing import Any, List, Optional

from sceneexp_utils.launch_file_utilities import (Package,
                                                             collection_pkgs)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# import id_common

MISSING = 'MISSING'


def ld_entity_for_package(package: Package, context: Any) -> Optional[LaunchDescriptionEntity]:
    exec_name = LaunchConfiguration(package.name).perform(context)

    if exec_name == MISSING:
        return None

    elif exec_name.endswith('.launch.py'):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=PathJoinSubstitution([
                    FindPackageShare(package.name),
                    exec_name,
                ])
            )
        )
    else:
        return Node(
            package=package.name,
            executable=exec_name,
            name=package.name,
            parameters=[{'use_sim_time': True}],
        )


def generate_launch_description_using_config(context: Any) -> List[LaunchDescriptionEntity]:
    return [ld_entity for package in collection_pkgs if (ld_entity := ld_entity_for_package(package, context)) is not None]


def generate_launch_description():

    # path_from_odom_history_publisher = Node(
    #     package='imperial_driverless_utils',
    #     executable='path_from_odom_history_publisher',
    #     name='path_from_odom_history_publisher',
    #     parameters=[{'use_sim_time': True},
    #                 {'history_length_seconds': 20}]
    # )

    # rviz = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         launch_file_path=PathJoinSubstitution([
    #             FindPackageShare('imperial_driverless_utils'),
    #             'launch',
    #             'rviz.launch.py'
    #         ])
    #     )
    # )

    return LaunchDescription([
        *[DeclareLaunchArgument(pkg.name, default_value=MISSING) for pkg in collection_pkgs],
        OpaqueFunction(function=generate_launch_description_using_config),
    ])
