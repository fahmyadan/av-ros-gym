#!/usr/bin/env bash

source /opt/ros/galactic/setup.bash
rosdep init 
rosdep update --rosdistro=$ROS_DISTRO
git submodule update --init
/usr/bin/python3  -m pip install setuptools gym pygame 
cd src/ros_gym/ros_gym/gym_carla/
/usr/bin/python3 -m pip install -r requirements.txt
/usr/bin/python3  -m pip install -e . 
rosdep install --from-paths src --ignore-src -r -y --rosdistro galactic 