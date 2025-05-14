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
    
    def nhk2025_f32_to_byte(self, f32_list):
        for f32 in f32_list:
            byte_list += self.f32_to_byte(f32)
        return bytearray(byte_list)
    
    def nhk2025_byte_to_f32(self, byte_list):
        f32_list = []
        byte_list.reverse()
        bytelist = [byte_list[:4], byte_list[4:8], byte_list[8:12]]
        for i in range(3):
            f32 = self.byte_to_f32(bytes(bytelist[i]))
            f32_list.append(f32)
        f32_list.reverse()
        return f32_list