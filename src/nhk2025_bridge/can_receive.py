import can
from nhk2025_bridge.byte_control import ValueBridge

bridge = ValueBridge()
can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)

try:
    while True:
        msg = can0.recv()
        print(msg.data)
        msg.data.reverse()
        bytelist = [msg.data[:4], msg.data[4:8], msg.data[8:12]]
        print(bytelist)
        for i in range(3):
            f32 = bridge.byte_to_f32(bytes(bytelist[i]))
            print(f32)
except KeyboardInterrupt:
    pass
finally:
    can0.shutdown()