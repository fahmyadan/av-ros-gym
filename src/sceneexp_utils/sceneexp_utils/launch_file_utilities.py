from pathlib import Path
from typing import Iterable, List, Optional

import click # click module
from ament_index_python.packages import (PackageNotFoundError,
                                         get_package_share_path)
from dataclasses import dataclass

from ros2pkg.api import PackageNotFound
import os

from ros2pkg.api import get_executable_paths

# click colours for terminal output
def red(x: str, /): return click.style(x, fg='red')
def blue(x: str, /): return click.style(x, fg='blue')
def green(x: str, /): return click.style(x, fg='green')
def cyan(x: str, /): return click.style(x, fg='cyan')
def yellow(x: str, /): return click.style(x, fg='yellow')

# only simulators we support
supported_simulators: List[str] = [
    'carla_ros_bridge',
]

def select_cone_map() -> Path:
    cone_map_folder = get_package_share_path('imperial_driverless_utils') / 'cone_maps'

    # filter out non-cone map files (currently only .yaml and .csv files)
    cone_map_paths = [x for x in cone_map_folder.iterdir() if x.suffix in ('.yaml', '.csv')]
    cone_map_paths.sort()
    
    click.echo(f'Select a cone map:')
    for i, cone_map in enumerate(cone_map_paths):
        click.echo(f'{i}. {green(cone_map.name)}')

    selection = click.prompt(
        'Please select:',
        type=click.Choice([str(i) for i, _ in enumerate(cone_map_paths)]),
        default='0',
        show_choices=False,
    )

    assert isinstance(selection, str)

    selected_cone_map_path = cone_map_paths[int(selection)]
    click.echo(f'Using cone map {selected_cone_map_path.name}.\n')
    
    return selected_cone_map_path

# this function is the first to be called when launching
def select_simulator() -> str:
    available_simulators: List[str] = []

    for sim in supported_simulators:
        try:
            get_package_share_path(sim)
            available_simulators.append(sim)
        except PackageNotFoundError:
            # full gazebo simulator will not be found
            click.echo(yellow(f'{sim} not found'))

    if not any(available_simulators):
        click.echo(red(f'No supported simulators found. Supported simulators: {supported_simulators}'))
        raise click.Abort()

    
    click.echo(f'Select a simulator:')
    available_simulators.sort()
    for i, sim in enumerate(available_simulators):
        click.echo(f'{i}. {blue(sim)}')

    selection = click.prompt(
        'Please select:',
        type=click.Choice([str(i) for i, _ in enumerate(available_simulators)]),
        default='0',
        show_choices=False,
    )

    assert isinstance(selection, str)
    selected_sim = available_simulators[int(selection)]

    click.echo(f'Launching {blue(selected_sim)}.\n')

    return selected_sim


@dataclass
class Package:
    '''
    This class represents a ROS2 package.
    '''
    name: str

    @property
    def executable_names(self) -> List[str]:
        return [os.path.basename(path) for path in get_executable_paths(package_name=self.name)]  # type: ignore

    @property
    def launch_files(self) -> Iterable[Path]:
        pkg_path = get_package_share_path(self.name)
        return (pkg_path / 'launch').glob('*.launch*')


collection_pkgs = [Package(name) for name in (
    'carla_manual_control',
    'carla_ackermann_control',
    'carla_waypoint_publisher',
    'carla_ad_agent',
    'ros_gym'
)]

# function gets options from packages. Packages are objects stores in 'collection_pkgs'
def get_launch_option_for_package(pkg: Package) -> Optional[str]:
    try:
        nothing = 'Do not launch anything'

        all_options = (
            [(nothing, nothing)]
            + [(green(x.name), x.name) for x in pkg.launch_files]
            + [(blue(x), x) for x in pkg.executable_names]
        )

        click.echo(f'Select what to launch from {blue(pkg.name)}')

        for i, (text, _) in enumerate(all_options):
            click.echo(f'{i}. {text}')

        selection = click.prompt(
            'Please select:',
            type=click.Choice([str(i) for i, _ in enumerate(all_options)]),
            default='0',
            show_choices=False,
        )

        assert isinstance(selection, str)
        _, selected_option = all_options[int(selection)]

        if selected_option == nothing:
            return None

        click.echo(f'Launching {selected_option} from {blue(pkg.name)}.\n')

        return selected_option

    except PackageNotFound:
        click.echo(blue(pkg.name) + red(' not found.'))
        return None
