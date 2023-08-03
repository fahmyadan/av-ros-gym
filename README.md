# av-ros-gym
A CARLA environment for agent and scenario based Robotics and Machine Learning evaluation 

# Installation Requirements
OS: Ubuntu 20.04 LTS \
ROS2: Galactic \
CUDA 11.8 \
CuDNN 8.6.0 \
Carla: 0.9.14 (from source)


# Installation Instructions 
After installing the system requirements above: \
``` bash
git clone --recursive git@github.com:fahmyadan/av-ros-gym.git && cd av-ros-gym

sudo chmod +x setup.sh && ./setup.bash
```

The code comprises of a bunch of submodules; ros_bridge, astuff_msgs and gym-carla. The latter must be installed using setuptools distribution:

``` bash 
cd src/ros_gym/ros_gym/gym_carla

pip install -r requirements.txt

pip install -e .

```
# Build Packages
From PROJECT ROOT:

``` bash 
cd /path/to/av-ros-gym
colcon build && source install/setup.bash 
```

# Under Development 
The ros_bridge integration fully works allowing for autopilot, manual contorl and customized extensions for robotics based algorithms. 

The ros-gym interface aims to serve as a second (primary) client that will tick the world and act as the openai gym wrapper containing reset and step functions as well as sharing obs and rewards. 


