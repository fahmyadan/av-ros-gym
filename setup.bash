#!/usr/bin/env bash

source /opt/ros/galactic/setup.bash

rosdep init 
rosdep update --include-eol-distros 
git submodule update --init
python3 -m pip install setuptools gym pygame 
rosdep install --from-paths src --ignore-src -r -y --rosdistro galactic 