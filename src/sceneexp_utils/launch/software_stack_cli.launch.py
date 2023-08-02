

import click
from sceneexp_utils.launch_file_utilities import (
    collection_pkgs, get_launch_option_for_package, yellow)
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    click.clear()
    try:
        package_options = {pkg.name: launch_option for pkg in collection_pkgs if (
            launch_option := get_launch_option_for_package(pkg)) is not None}

        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    launch_file_path=PathJoinSubstitution([
                        FindPackageShare('imperial_driverless_utils'),
                        'launch',
                        'software_stack_impl.launch.py'
                    ])
                ),
                launch_arguments=[
                    *package_options.items(),
                ]
            )
        ])

    except click.Abort:
        click.echo()
        click.echo(yellow('Launch aborted by the user'))
        exit()
