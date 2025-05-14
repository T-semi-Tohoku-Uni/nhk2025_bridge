from nhk2025_bridge.byte_control import ValueBridge

def main():
    bridge = ValueBridge()
    f32_list = [32, 20.78, 45.33]
    byte_list = bridge.nhk2025_f32_to_byte(f32_list)
    f32_list_return = bridge.nhk2025_byte_to_f32(byte_list)
    print(f32_list_return)