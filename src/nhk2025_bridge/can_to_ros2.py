import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nhk2025_custom_message.msg import Float32List

import can
from nhk2025_bridge.byte_control import ValueBridge

class Can2Ros2(Node):
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
            'swervedrive_angle':0x203,
            'soten_flag':0x206,
            'swervedrive_vel':0x208,
        }

    def ros2_setup(self):
        self.publisher_dic = {}
        self.publisher_dic['soten_flag'] = self.create_publisher(
            Bool,
            'soten_flag',
            10
        )
        self.publisher_dic['swervedrive_angle'] = self.create_publisher(
            Float32List,
            'swervedrive_angle',
            10
        )
        self.publisher_dic['swervedrive_vel'] = self.create_publisher(
            Float32List,
            'swervedrive_vel',
            10
        )

    def can_msg_process(self):
        if len(self.msg.data) != 12:
            return None, None
        rxdata_f32 = self.bridge.nhk2025_byte_to_f32(self.msg.data)
        topic_name_candidate = [k for k, v in self.canid_dic.items() if v == self.msg.arbitration_id]
        if topic_name_candidate:
            topic_name_c = topic_name_candidate[0]
        else:
            topic_name_c = None
        return rxdata_f32, topic_name_c

    def __call__(self):
        while True:
            self.msg = self.can0.recv()
            rxdata, topic_name = self.can_msg_process()

            if topic_name == 'soten_flag':
                txdata = Bool()
                txdata.data = bool(rxdata[0])
                self.publisher_dic[topic_name].publish(txdata)

            if topic_name == 'swervedrive_angle':
                txdata = Float32List()
                txdata.num = []
                txdata.num[0] = float(rxdata[0])
                txdata.num[1] = float(rxdata[1])
                txdata.num[2] = float(rxdata[2])
                self.publisher_dic[topic_name].publish(txdata)

            if topic_name == 'swervedrive_vel':
                txdata = Float32List()
                txdata.num = []
                txdata.num[0] = float(rxdata[0])
                txdata.num[1] = float(rxdata[1])
                txdata.num[2] = float(rxdata[2])
                self.publisher_dic[topic_name].publish(txdata)


def main_can_to_ros2():
    rclpy.init()
    can_bridge = Can2Ros2()
    
    try:
        can_bridge()
    except KeyboardInterrupt:
            pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()