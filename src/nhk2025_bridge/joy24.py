import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import roslibpy
import time

class WebSocketClientJoy():
    def __init__(self, namespace:str, host:str, port:int):
        self.client = roslibpy.Ros(host=host, port=port)
        self.client.run()
        print("接続成功")
        self.joy_publisher = roslibpy.Topic(self.client, namespace + '/joy', 'sensor_msgs/msg/Joy')
        self.joy_msg = roslibpy.Message()

    def send(self, axes:list, buttons:list):
        now = time.time()
        sec = int(now)
        nanosec = int((now-sec) * 1e9)
        joy_msg = roslibpy.Message({
            'header': {
                'stamp': {'sec': sec, 'nanosec': nanosec},
                'frame_id': 'joy'
            },
            'axes': axes,  
            'buttons': buttons
        })
        self.joy_publisher.publish(joy_msg)

    def destroy_client(self):
        self.joy_publisher.unadvertise()
        self.client.terminate()

class WebsocketClient(Node):
    def __init__(self):
        super().__init__('websocket_client')
        self.subscriber_ps5 = self.create_subscription(
            Joy,
            'joy',
            self.ps5_callback,
            10
        )
        self.timer_publish = self.create_timer(
            0.05,
            self.timer_callback
        )
        self.client_joy = WebSocketClientJoy(self.get_namespace(), '192.168.11.58', 9090)
        self.subscriber_ps5

        self.axes_list = []
        self.buttons_list = []

    def ps5_callback(self, rxdata:Joy):
        self.axes_list = list(rxdata.axes)
        self.buttons_list = list(rxdata.buttons)

    def timer_callback(self):
        self.client_joy.send(self.axes_list, self.buttons_list)



def main_24():
    rclpy.init()
    websocket_client = WebsocketClient()
    try:
        rclpy.spin(websocket_client)
    except KeyboardInterrupt:
        pass
    finally:
        websocket_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()