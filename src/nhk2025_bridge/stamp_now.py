import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

class StampNow(Node):
    def __init__(self):
        super().__init__('stamp_now')
        self.stamp_publisher = self.create_publisher(
            Time,
            'stamp',
            10
        )
        self.create_timer(
            0.1,
            self.timer_callback
        )

    def timer_callback(self):
        txdate = self.get_clock().now().to_msg()
        self.stamp_publisher.publish(txdate)

def main_stamp():
    rclpy.init()
    stamp_now = StampNow()
    try:
        rclpy.spin(stamp_now)
    except KeyboardInterrupt:
        pass
    finally:
        stamp_now.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()