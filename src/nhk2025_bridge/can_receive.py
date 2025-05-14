import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import can
from nhk2025_bridge.byte_control import ValueBridge

class CanBridge(Node):
    def __init__(self):
        self.bridge = ValueBridge()
        super().__init__('can_bridge')
        self.can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)
        self.publisher_soten = self.create_publisher(
            Bool,
            "/soten_flag",
            10
        )

    def __del__(self):
        self.can0.shutdown()
        self.destroy_node()

    def __call__(self, msg:can.Message):
        rxdata_f32 = self.bridge.nhk2025_byte_to_f32(msg.data)

        txdata = Bool()
        txdata.data = bool(rxdata_f32[0])
        self.publisher_soten.publish(txdata)
        if msg.arbitration_id == 0x206:
            pass
            


def main_canbridge():
    rclpy.init()
    can_bridge = CanBridge()
    
    try:
        while True:
            can_bridge(can_bridge.can0.recv())
    except KeyboardInterrupt:
            pass
    finally:
        rclpy.shutdown()