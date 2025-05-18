import rclpy
from rclpy.node import Node

from nhk2025_custom_message.msg import CanFd

import can

class CanReceive(Node):
    def __init__(self):
        super().__init__('can_receive')
        self.can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)
        self.publisher_can = self.create_publisher(
            CanFd,
            'can_msg',
            10
        )

    def candata_to_ros2(self, canmsg:can.Message):
        canfd = CanFd()
        canfd.id = canmsg.arbitration_id
        canfd.dlc = canmsg.dlc
        canfd.data = canmsg.data
        return canfd

    def __call__(self):
        while True:
            msg = self.can0.recv()
            txdata = self.candata_to_ros2(msg)
            self.publisher_can.publish(txdata)

    def __del__(self):
        self.can0.shutdown()
        self.destroy_node()

def main_can_receive():
    rclpy.init()
    can_receive = CanReceive()
    try:
        can_receive()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()