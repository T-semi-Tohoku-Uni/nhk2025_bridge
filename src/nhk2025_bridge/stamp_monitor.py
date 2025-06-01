import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclpyTime, Duration as RclpyDuration

class StampMonitor(Node):
    def __init__(self):
        super().__init__('stamp_monitor')
        self.subscriber_stamp = self.create_subscription(
            Time,
            'stamp',
            self.stamp_callback,
            10
        )
        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )
        self.subscriber_stamp
        self.connect_flag = False

    def stamp_callback(self, rxdata:Time):
        self.connect_flag = True
        self.client_time = RclpyTime.from_msg(rxdata)

    def timer_callback(self):
        now = self.get_clock().now()

        if self.connect_flag is False:
            self.get_logger().info('connecting...')
        else:
            disconnect_time_diff = now - self.client_time
            if disconnect_time_diff > RclpyDuration(seconds=0, nanoseconds=500_000_000):
                self.get_logger().info('disconnect!!')

        
def main_stamp_monitor():
    rclpy.init()
    stamp_monitor = StampMonitor()
    try:
        rclpy.spin(stamp_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        stamp_monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()