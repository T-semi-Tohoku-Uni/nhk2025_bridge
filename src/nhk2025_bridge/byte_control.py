import numpy as np

class ValueBridge():
    def __init__(self):
        pass
    def f32_to_byte(self, f32:float):
        f32_ndarray = np.array([f32]).astype(np.float32)
        bytes_f32 = f32_ndarray.tobytes()
        return bytes_f32
    def byte_to_f32(self, byte:bytes):
        #f32_ndarray = byte.
        f32 = None
        return f32

if __name__ == '__main__':
    bridge = ValueBridge()
    f32_test = 23
    byte_test = bridge.f32_to_byte(f32_test)
    print(type(byte_test))