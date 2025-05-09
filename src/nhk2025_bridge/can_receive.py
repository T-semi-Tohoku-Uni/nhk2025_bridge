#import rclpy
#from rclpy.node import Node

import can

# Connect to the CAN bus using python-can
can0 = can.interface.Bus(channel='can0', interface='socketcan')

try:
    while True:
        msg = can0.recv()
        print(msg)
        print('run')
except KeyboardInterrupt:
    pass
finally:
    can0.shutdown()