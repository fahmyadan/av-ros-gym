import rclpy 
from abc import ABC, abstractmethod
import sys
from pathlib import Path 
from rclpy.node import Node
from rclpy.publisher import Publisher




class RosGymBase(ABC,Node):

    def __init__(self, name: str):

        self.name = name 

        
        # Node Publishers
        
        # Node Subscriptions 




    @abstractmethod
    def reset_cb(self) -> None: ... 


    @abstractmethod
    def make_pub1(self) -> None: ...

