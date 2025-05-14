import numpy as np

class ValueBridge():
    def __init__(self):
        pass
    def f32_to_byte(self, f32:float):
        f32_ndarray = np.array([f32]).astype(np.float32)
        bytes_f32 = f32_ndarray.tobytes()
        return bytes_f32
    def byte_to_f32(self, byte:bytes):
        f32_ndarray = np.frombuffer(byte, dtype=np.float32)
        f32 = float(f32_ndarray[0])
        return f32

if __name__ == '__main__':
    bridge = ValueBridge()
    f32_test = 100
    byte_test = bridge.f32_to_byte(f32_test)
    f32_test_test = bridge.byte_to_f32(byte_test)
    print(f32_test)
    print(byte_test)
    print(f32_test_test)