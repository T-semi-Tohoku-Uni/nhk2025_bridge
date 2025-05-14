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