import rclpy
from rclpy.node import Node

import can

with can.Bus() as bus:
    for msg in bus:
        print(msg.data)