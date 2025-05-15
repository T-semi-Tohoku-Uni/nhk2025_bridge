import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

import can
from nhk2025_bridge.byte_control import ValueBridge

class CanMonitor(Node):
    def __init__(self):
        self.bridge = ValueBridge()
        super().__init__('can_monitor')
        self.can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)
        self.ros2_setup()

    def ros2_setup(self):
        self.publisher_can_state = self.create_publisher(
            Bool,
            '/can_state',
            10
        )
        self.timer = self.create_timer(
            1,
            self.timer_callback
        )
        self.txdata_can_state = Bool()

    def timer_callback(self):
        txdata_f32 = [0, 0, 0]
        self.can_send(txdata_f32, 0x7ff)

    def can_send(self, txdata_list:list, id:int):
        txdata_byte_list = self.bridge.nhk2025_f32_to_byte(txdata_list)
        txdata_can = can.Message(
            arbitration_id=id,
            is_extended_id=False,
            dlc=12,
            data=txdata_byte_list,
            is_fd=True,
        )
        try:
            self.can_state = True
            self.can0.send(txdata_can)
        except can.CanError:
            self.can_state = False
        self.txdata_can_state.data = self.can_state
        self.publisher_can_state.publish(self.txdata_can_state)


def main_can_monitor():
    rclpy.init()
    can_monitor = CanMonitor()
    try:
        rclpy.spin(can_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        can_monitor.can0.shutdown()
        can_monitor.destroy_node()
        rclpy.shutdown()