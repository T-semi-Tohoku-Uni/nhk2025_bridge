import rclpy
from rclpy.node import Node

import can
from nhk2025_bridge.byte_control import ValueBridge

bridge = ValueBridge()
can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)

try:
    while True:
        msg = can0.recv()
        print(msg.data)
        print(bridge.nhk2025_byte_to_f32(msg.data))
except KeyboardInterrupt:
    pass
finally:
    can0.shutdown()