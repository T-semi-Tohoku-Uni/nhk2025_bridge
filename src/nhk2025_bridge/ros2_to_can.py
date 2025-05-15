import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D
from nhk2025_custom_message.msg import Float32List
from std_msgs.msg import Bool
from std_msgs.msg import Float32

import can
from nhk2025_bridge.byte_control import ValueBridge

class Ros2Can(Node):
    def __init__(self):
        self.bridge = ValueBridge()
        super().__init__('ros2_to_can')
        self.can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)
        self.ros2_setup()

        self.canid_dic = {
            "vel":0x300,
            "loading_speed":0x301,
            "turret_angle":0x302,
            "velt_speed":0x303,
            "brake":0x304,
            "soten_flag":0x206,
            "pass_speed":0x207
        }

    def ros2_setup(self):
        self.subscriber_vel = self.create_subscription(
            Pose2D,
            "/robot_vel",
            self.vel_callback,
            10
        )
        self.subscriber_tur = self.create_subscription(
            Float32List,
            "/turret_angle",
            self.tur_callback,
            10
        )
        self.subscriber_velt = self.create_subscription(
            Float32List,
            "/velt_speed",
            self.velt_callback,
            10
        )
        self.subscriber_soten = self.create_subscription(
            Float32,
            '/loading_speed',
            self.soten_callback,
            10
        )
        self.subscriber_pass = self.create_subscription(
            Float32,
            "/pass_speed",
            self.pass_callback,
            10
        )
        self.state_bogai = self.create_subscription(
            Bool,
            "/state_defence",
            self.defence_callback,
            10
        )

        self.subscriber_vel
        self.subscriber_tur
        self.subscriber_velt
        self.subscriber_pass
        self.state_bogai

    def can_send(self, txdata_list:list, msg_name:str):
        txdata_byte_list = self.bridge.nhk2025_f32_to_byte(txdata_list)
        txdata_can = can.Message(
            arbitration_id=self.canid_dic[msg_name],
            is_extended_id=False,
            dlc=12,
            data=txdata_byte_list,
            is_fd=True,
        )
        self.can0.send(txdata_can)

    def vel_callback(self, rxdata):
        vx    = rxdata.x
        vy    = rxdata.y
        omega = rxdata.theta
        txdata_f32 = [vx, vy, omega]
        self.can_send(txdata_f32, "vel")

    def tur_callback(self, rxdata):
        tur_ele = rxdata.num[0]
        tur_azi = rxdata.num[1]
        txdata_f32 = [tur_ele, tur_azi, 0]
        self.can_send(txdata_f32, "turret_angle")

    def velt_callback(self, rxdata):
        velt_speed_up_f32 = rxdata.num[0]
        velt_speed_down_f32 = rxdata.num[1]
        txdata_f32 = [velt_speed_up_f32, velt_speed_down_f32]
        self.can_send(txdata_f32, "velt_speed")
        
    def soten_callback(self, rxdata):
        loading_speed = rxdata.data
        txdata_f32 = [loading_speed, 0, 0]
        self.can_send(txdata_f32, "loading_speed")

    def pass_callback(self, rxdata):
        pass_speed = rxdata.data
        txdata_f32 = [pass_speed, 0, 0]
        self.can_send(txdata_f32, 'pass_speed')

    def defence_callback(self, rxdata):
        if rxdata.data:
            self.status_bougai = "1"


def main_ros2_to_can():
    rclpy.init()
    ros2_to_can = Ros2Can()
    try:
        rclpy.spin(ros2_to_can)
    except KeyboardInterrupt:
        pass
    finally:
        ros2_to_can.can0.shutdown()
        ros2_to_can.destroy_node()
        rclpy.shutdown()