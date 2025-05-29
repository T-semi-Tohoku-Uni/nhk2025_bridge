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


    def ps5_callback(self, rxdata:Joy):
        headeer = rxdata.header


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