# test_canalyst_official.py
import can
import warnings
warnings.filterwarnings("ignore")  # 忽略弃用警告

def test_canalyst():
    try:
        # 直接按照官方示例的方式初始化
        print("尝试官方示例的连接方式...")
        try:
            # 使用 can.interface.Bus 而不是 can.Bus
            bus = can.interface.Bus(bustype='canalystii', channel=0, bitrate=1000000)
            print("✓ 成功连接到 CANalyst-II")
            
            # 发送测试消息
            msg = can.Message(
                arbitration_id=0x123,
                data=[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
                is_extended_id=False
            )
            
            bus.send(msg)
            print(f"✓ 消息发送成功: {msg}")
            
            # 接收消息
            received_msg = bus.recv(timeout=1.0)
            if received_msg:
                print(f"✓ 接收到消息: {received_msg}")
            
            bus.shutdown()
            return 'canalystii'
            
        except Exception as e:
            print(f"✗ 连接失败: {e}")
            return None
            
    except Exception as e:
        print(f"测试过程中出现错误: {e}")
        return None

if __name__ == "__main__":
    result = test_canalyst()
    if result:
        print(f"\n连接成功，建议使用: can.interface.Bus(bustype='canalystii', ...)")
    else:
        print("\n连接失败")
        
        