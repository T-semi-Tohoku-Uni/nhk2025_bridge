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
        self.can0 = can.interface.Bus(
            channel='can0',
            bustype='socketcan',
            bitrate=1000000,
            fd=True,
            data_bitrate=2000000
        )
        self.subscriber_setup()
        self.state_control_setup()

        self.canid_dic = {
            'vel':0x300,
            'loading_speed':0x301,
            'turret_angle':0x302,
            'velt_speed':0x303,
            'brake_flag':0x304,
            'defence_flag':0x305,
            'soten_flag':0x206,
            'pass_speed':0x207
        }

    def subscriber_setup(self):
        self.subscriber_vel = self.create_subscription(
            Pose2D,
            'vel',
            self.vel_callback,
            10
        )
        self.subscriber_tur = self.create_subscription(
            Float32List,
            'turret_angle',
            self.tur_callback,
            10
        )
        self.subscriber_velt = self.create_subscription(
            Float32List,
            'velt_speed',
            self.velt_callback,
            10
        )
        self.subscriber_soten = self.create_subscription(
            Float32,
            'loading_speed',
            self.soten_callback,
            10
        )
        self.subscriber_pass = self.create_subscription(
            Float32,
            'pass_speed',
            self.pass_callback,
            10
        )
        self.state_bogai = self.create_subscription(
            Bool,
            'defence_flag',
            self.defence_callback,
            10
        )
        self.subscriber_brake = self.create_subscription(
            Bool,
            'brake_flag',
            self.brake_callback,
            10
        )

        self.subscriber_vel
        self.subscriber_tur
        self.subscriber_velt
        self.subscriber_pass
        self.state_bogai
        self.subscriber_brake

        self.can_state = False

    def state_control_setup(self):
        self.publisher_can_state = self.create_publisher(
            Bool,
            'can_state',
            10
        )
        self.txdata_can_state = Bool()

    def can_send(self, txdata_list:list, msg_name:str):
        txdata_byte_list = self.bridge.nhk2025_f32_to_byte(txdata_list)
        txdata_can = can.Message(
            arbitration_id=self.canid_dic[msg_name],
            is_extended_id=False,
            dlc=12,
            data=txdata_byte_list,
            is_fd=True,
        )
        try:
            self.can_state = True
            self.txdata_can_state.data = self.can_state
            self.publisher_can_state.publish(self.txdata_can_state)
            self.can0.send(txdata_can)
        except can.CanError:
            self.can_state = False
            self.txdata_can_state.data = self.can_state
            self.publisher_can_state.publish(self.txdata_can_state)

    def vel_callback(self, rxdata:Pose2D):
        vx    = rxdata.x
        vy    = rxdata.y
        omega = rxdata.theta
        txdata_f32 = [vx, vy, omega]
        self.can_send(txdata_f32, 'vel')

    def tur_callback(self, rxdata:Float32List):
        tur_ele = rxdata.num[0]
        tur_azi = rxdata.num[1]
        txdata_f32 = [tur_ele, tur_azi, 0]
        self.can_send(txdata_f32, 'turret_angle')

    def velt_callback(self, rxdata:Float32List):
        velt_speed_up_f32 = rxdata.num[0]
        velt_speed_down_f32 = rxdata.num[1]
        txdata_f32 = [velt_speed_up_f32, velt_speed_down_f32]
        self.can_send(txdata_f32, 'velt_speed')
        
    def soten_callback(self, rxdata:Float32):
        loading_speed = rxdata.data
        txdata_f32 = [loading_speed, 0, 0]
        self.can_send(txdata_f32, 'loading_speed')

    def pass_callback(self, rxdata:Float32):
        pass_speed = rxdata.data
        txdata_f32 = [pass_speed, 0, 0]
        self.can_send(txdata_f32, 'pass_speed')

    def defence_callback(self, rxdata:Bool):
        defence_flag = int(rxdata.data)
        txdata_f32 = [defence_flag, 0, 0]
        self.can_send(txdata_f32, 'defence')

    def brake_callback(self, rxdata:Bool):
        brake_flag = int(rxdata.data)
        txdata_f32 = [brake_flag, 0, 0]
        self.can_send(txdata_f32, 'brake_flag')

    def destroy_node(self):
        self.can0.shutdown()
        super().destroy_node()
            


def main_ros2_to_can():
    rclpy.init()
    ros2_to_can = Ros2Can()
    try:
        rclpy.spin(ros2_to_can)
    except KeyboardInterrupt:
        pass
    finally:
        ros2_to_can.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()