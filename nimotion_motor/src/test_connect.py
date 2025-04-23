import can

# 手动创建 CANalyst-II 总线对象
bus = can.Bus(
    interface='canalystii',
    channel=0,        # 或 1，根据实际硬件通道
    bitrate=1000000,
    # 某些情况下需要指定 dll 路径（若驱动未自动注册）
    # dll=r'C:\Program Files\ControlCAN\ControlCAN.dll'
)

try:
    # 发送一条测试CAN帧（标准帧，ID=0x123，数据=0xAA）
    msg = can.Message(
        arbitration_id=0x123,
        data=[0xAA],
        is_extended_id=False
    )
    bus.send(msg)
    print("CAN帧发送成功")

    # 接收数据（超时1秒）
    recv_msg = bus.recv(timeout=1)
    if recv_msg:
        print(f"接收到数据: {recv_msg.data}")

except Exception as e:
    print(f"错误: {e}")

finally:
    bus.shutdown()