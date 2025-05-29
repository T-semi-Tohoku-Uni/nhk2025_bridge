import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

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