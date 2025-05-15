import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import can
from nhk2025_bridge.byte_control import ValueBridge

class CanBridge(Node):
    def __init__(self):
        self.bridge = ValueBridge()
        self.stm_setup()
        self.canid_setup()
        self.can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)
        super().__init__('can_bridge')
        self.ros2_setup()
        

    def __del__(self):
        self.can0.shutdown()
        self.destroy_node()

    def stm_setup(self):
        self.stmid_dic = {
            "suspension":1,
            "hoto":2,
            "defence":4,
            "dronesc":5
        }
        self.stmcanid_dic = {}
        for stm in self.stmid_dic:
            self.stmcanid_dic[stm] = 0x101 + (self.stmid_dic[stm]*16)

    def canid_setup(self):
        self.canid_dic = {
            "soten":0x206
        }

    def ros2_setup(self):
        self.publisher = {}
        self.publisher['/soten_flag'] = self.create_publisher(
            Bool,
            '/soten_flag',
            10
        )

    def __call__(self):
        msg = self.can0.recv()
        rxdata_f32 = self.bridge.nhk2025_byte_to_f32(msg.data)
        topic_name_candidate = [k for k, v in self.canid_dic.items() if v == msg.arbitration_id]
        if topic_name_candidate:
            topic_name = topic_name_candidate[0]

        txdata = Bool()
        self.publisher['/soten_flag'].publish(txdata)


def main_canbridge():
    rclpy.init()
    can_bridge = CanBridge()
    
    try:
        while True:
            can_bridge()
    except KeyboardInterrupt:
            pass
    finally:
        rclpy.shutdown()