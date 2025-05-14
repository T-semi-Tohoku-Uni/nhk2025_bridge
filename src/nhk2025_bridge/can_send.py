import rclpy
from rclpy.node import Node

import can
from nhk2025_bridge.byte_control import ValueBridge

class CanSend(Node):
    def __init__(self):
        self.bridge = ValueBridge()
        super().__init__('can_send')
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )
        self.can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)


    def timer_callback(self):
        txdata_f32 = [23, 43, 99.89]
        txdata_byte = self.bridge.nhk2025_f32_to_byte(txdata_f32)
        txdata = can.Message()
        txdata.arbitration_id = 0x200,
        txdata.data = txdata_byte
        txdata.dlc = 12
        self.can0.send(txdata)

def main_can_send():
    rclpy.init()
    can_send = CanSend()
    try:
        rclpy.spin(can_send)
    except KeyboardInterrupt:
        pass
    finally:
        can_send.destroy_node()
        rclpy.shutdown()