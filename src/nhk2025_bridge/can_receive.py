import can

can0 = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)

try:
    while True:
        msg = can0.recv()
        print(msg)
except KeyboardInterrupt:
    pass
finally:
    can0.shutdown()