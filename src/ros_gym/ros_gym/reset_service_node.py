import rclpy 
from rclpy.node import Node
from .ros_gym_node import RosGymBase
import yaml
from pathlib import Path
from typing import Callable
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import time 
import sys
import os
import pkg_resources
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty, SetBool

try:
    from gym_carla.envs.carla_env import CarlaEnv

except Exception as e: 
    raise ImportError('Carla Gym cannot be imported')

if True:
    current_path = str(Path(__file__))

class ResetNode(Node):

    def __init__(self, name: str):
        super().__init__(name)
        self.reset_called = False
        
        params_path = str(Path(__file__).parents[6]) +'/src/ros_gym/ros_gym/config/carla_params.yaml'
        with open(params_path, 'r') as f:

            params = yaml.safe_load(f)

        self.carla_env = CarlaEnv(params)
        cb_group = MutuallyExclusiveCallbackGroup()
        qos_profile = QoSProfile(depth=10)
        self.reset_service = self.create_service(SetBool, 'reset_service', self.reset_cb, callback_group=cb_group, qos_profile=qos_profile)
        # initial_obs = self.create_service()
        # initial_obs = self.create_timer(2.0, self.reset_cb, callback_group=cb_group)
    
    def reset_cb(self, req, response) -> None:
        self.reset_called = req.data
        self.get_logger().info(f'Reset Bool {self.reset_called}')
        
        if not self.reset_called:
            self.reset_called = True
            response = self.carla_env.reset() 
        
        self.get_logger().info('Resetting Gym environment (service)')


        return response


def spin(node: Callable[[], Node]) -> None:

    rclpy.init(args=sys.argv)

    try:
        rclpy.spin(node())
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        if not "make_tuple(): unable to convert arguments to Python object" in str(e):
            raise e



def main():

    spin(lambda: ResetNode('reset_spawn'))

    

