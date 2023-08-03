import os
import tempfile
from pathlib import Path
from typing import Any, Tuple, Dict

import click
import yaml
from sceneexp_utils.launch_file_utilities import (
    collection_pkgs, cyan, get_launch_option_for_package, select_cone_map,
    select_simulator)
from launch import LaunchDescription
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

IDLaunchConfiguration = Tuple[str, str, Dict[str, str]]

save_dir = Path(tempfile.gettempdir(), "sceneexp")
save_dir.mkdir(exist_ok=True, parents=True)
launch_options_path = save_dir / "launch_options.yaml"


def generate_launch_description():
    simulator, package_options =  ask_user_for_options()

    # spawn_point_ego ='spawn_point_hero0'
    spawn_point_ego = 'VehicleSpawnPoint154'
    town = LaunchConfiguration('town', default='Town05')
    
    simulation_environment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(str(simulator)),
                'carla_ros_bridge.launch.py',
            ])
        ),
        launch_arguments=[
            ('town', town),
            ('passive', 'true')

        ]
    )

    # spawn_vehicle = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare('carla_spawn_objects'),
    #             'carla_spawn_objects.launch.py',
    #         ]),
    #     ),
    #     launch_arguments=[
    #        ('spawn_point_ego_vehicle_x',spawn_point_ego),
    #     ]
    # )
    vehicle_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carla_manual_control'),
                'carla_manual_control.launch.py',
            ]),
        ),
        launch_arguments=[
            ('role_name', 'ego_vehicle')
        ]
    )

    software_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sceneexp_utils'),
                'launch',
                'software_stack_impl.launch.py',
            ]),
        ),
        launch_arguments=[
            *package_options.items(),
        ]
    )

    return LaunchDescription([
        simulation_environment,
        # spawn_vehicle, 
        software_stack
    ])


def ask_user_to_load_previous() -> bool:
    if not os.path.exists(launch_options_path):
        return False

    with open(launch_options_path, 'r') as f:
        doc = yaml.safe_load(f)

    print('Previous launch configuration: \n___________________________\n')
    print(f'Simulator: {cyan(doc["utils"]["simulator"])}')
    print(f'Cone map: {cyan(doc["utils"]["cone_map_file"].split("/")[-1])}')

    for pkg_type, pkg_name in doc['packages'].items():
        print(f'{pkg_type}: {cyan(pkg_name)}')

    print('Load previous configuration?')

    selection = click.prompt(
        'Please select:',
        type=click.Choice(['yes', 'no']),
        default='yes',
        show_choices=False,
    )

    return selection == 'yes'


def get_previously_selected_options() -> IDLaunchConfiguration:
    with open(launch_options_path, 'r') as f:
        doc = yaml.safe_load(f)

    return (doc['utils']['simulator'], doc['utils']['cone_map_file'], doc['packages'],)


def ask_user_for_options() -> IDLaunchConfiguration:
    simulator = select_simulator()
    # cone_map = str(select_cone_map())
    packages = {pkg.name: launch_option for pkg in collection_pkgs if (
        launch_option := get_launch_option_for_package(pkg)) is not None}

    # save_options(simulator, cone_map, packages)

    return (simulator, packages)


def save_options(simulator: str, cone_map_file: str, package_options: Dict[str, Any]):
    options_to_yaml: dict[str, dict[str, Any]] = {
        'utils': {
            'simulator': simulator,
            'cone_map_file': cone_map_file,
        }, 'packages': package_options}

    with open(launch_options_path, 'w') as file:
        yaml.dump(options_to_yaml, file)
