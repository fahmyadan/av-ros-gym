import rclpy 
from rclpy.node import Node
from typing import Callable
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import time 
import sys
import os
import pkg_resources
from rclpy.qos import QoSProfile
from std_srvs.srv import Empty, SetBool


class ResetClientNode(Node):

    def __init__(self, name: str):
        super().__init__(name)
        self.reset_called = False
        cb_group = MutuallyExclusiveCallbackGroup()
        qos_profile = QoSProfile(depth=10)
        self.reset_client = self.create_client(SetBool, 'reset_service', callback_group=cb_group, qos_profile=qos_profile)
        self.req = SetBool.Request()
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Reset service not available, waiting again...')

    
    def request_reset(self) -> None:
        self.future = self.reset_client.call(self.req)
        rclpy.spin_until_future_complete(self, self.future)


        return self.future.result()
    

def serv_spin(node: Callable[[], Node]) -> None:

    rclpy.init(args=sys.argv)

    try:
        reset = ResetClientNode('reset_client_node')
        response = reset.request_reset()

        reset.get_logger().info('Resetting Gym environment (service)')
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        if not "make_tuple(): unable to convert arguments to Python object" in str(e):
            raise e
    

def main():
    serv_spin(ResetClientNode)