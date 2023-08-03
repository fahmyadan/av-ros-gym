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

sudo chmod +x setup.bash && ./setup.bash
```

The code comprises of a bunch of submodules; ros_bridge, astuff_msgs and gym-carla. As a result, there could be some issues when building and running the ros nodes for the first time. See the issues tab on the common errors and how to fix them. Please also add to this tab as more issues are encountered. 

# Build Packages
From PROJECT ROOT:

``` bash 
cd /path/to/av-ros-gym
colcon build && source install/setup.bash 
```

# Launch 

```bash 
ros2 launch sceneexp_utils custom.launch.py 
```

This node will give you the option to launch all the packages that are currently in development for this project. See the next section. 

# Under Development 
The ros_bridge integration fully works allowing for autopilot, manual contorl and customized extensions for robotics based algorithms. \

Available Options: 

{carla_ros_bridge, carla_manual_control, carla_ackerman_control_node, carla_waypoints_publisher, ad_agent/local_planner}


The **ros-gym** node interface aims to serve as a primary client that will tick the world and act as the openai gym wrapper containing reset and step functions as well as sharing obs and rewards. This is currently under development and **SHOULD NOT** be launched. 


