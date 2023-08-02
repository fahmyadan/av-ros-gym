# imperial_driverless_utils

This is a ROS2 package with the custom launch file and various utilities

## Simple launching procedure

```
ros2 launch imperial_driverless_utils custom.launch.py
```

## Launch files

`custom.launch.py` loads the `ads_dv` vehicle from `vehicle_descriptions` and then includes `setup_simulation_environment.launch.py` and `software_stack.launch.py`. (it used to ask the user to select a vehicle, but we didn't use it. In case it is needed in the future look at commit 3afd6b10e4bf0154a65d9eb7a67d1ecce9414c88). Saves the user launch options and asks whether to use them when launching again. Alternatively, accepts a path to a file that contains the launch options.

`setup_simulation_environment.launch.py` asks for the choice of simulator launch file, and if the simulator supports it, for the choice of cone map, and then includes the selected launch file from the simulator package. Uses the default vehicle config file unless the `vehicle_config_file` argument is provided.

`software_stack.launch.py` has the list of our packages, and asks the user to select an executable or a launch file from each of them. Additionally, includes `rviz.launch.py`, which launches RVIZ with all relevant visualizations set up, and adds an event listener which shuts down all nodes if RVIZ window is closed.

`real_drive.launch.py` launches the `ros_can` and the `orchestrator` nodes, which are responsible for interfacing with the vehicle and loading the appropriate nodes to execute a selected mission. 

## Python utilities

### TFMessageFilter

This class is a substitute for the tf2 message filter utility that's missing in python.

The idea is as follows: store incoming stamped messages in a queue until a transform from their frame to the target frame becomes available, transform them into the target frame and pass them to the supplied callback.

If the queue size is to small, you might encounter a situation where the transforms never become available, and it will fail silently. Be aware of that.

### get_default_vehicle_config_path

This function returns the path to the default vehicle config file, which doesn't sound like much but without it you would have to specify this path manually every time you launch `setup_simulation_environment.launch.py` or `software_stack.launch.py` or `real_drive.launch.py`.

## ROS nodes

### clock_sync_signal_emitter

This node was created to solve a weird time synchronization issue in RVIZ, where the wheels of the vehicle would lag behind, and then jump forward at a very high frequency. **Only needed for visualization purposes**

Emits an empty `sensor_msgs/msg/LaserScan` message on the `/clock_sync_signal` topic every time it receives a `rosgraph_msgs/msg/Clock` message on the `/clock` topic. Why `LaserScan`? This is the only message type that RVIZ could perform experimental synchronization to at the time of writing.

### odometry_noisifier

Rarely used node adding constant drift to the odometry. Used for testing the resistance to odometry drift of our SLAM algorithms.

### path_from_odom_history_publisher

Listens to messages of type `nav_msgs/msg/Odometry` on `/odom` and publishes the last 20 seconds of vehicle trajectory as `nav_msgs/msg/Path` on `/odom_path`. (number of seconds configurable via the `history_length_seconds` parameter)
