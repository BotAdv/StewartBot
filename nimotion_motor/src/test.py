import canopen
import time

# 1. 初始化CAN网络
network = canopen.Network()

try:
    # 配置CAN接口（根据实际硬件调整）
    network.connect(channel=0, bustype='canalystii', bitrate=1000000)
    print("CAN网络连接成功")

    # 2. 添加节点（假设伺服节点ID为1）
    node_id = 1
    node = network.add_node(node_id, 'E:/Project/python/Bachelor/DegreeProject/motor/NiMotion_STM42A_V1.07.eds')  # 需替换为实际EDS文件路径
    
    # 3. 预操作模式（配置参数）
    node.nmt.state = 'PRE-OPERATIONAL'
    
    # 配置PDO通信（示例：设置TPDO1）
    node.tpdo.read()
    node.tpdo[1].clear()
    node.tpdo[1].add_variable(0x607A,0x00)  # 根据对象字典中的实际变量名替换
    node.tpdo[1].trans_type = 255  # 异步传输
    node.tpdo[1].enabled = True
    node.tpdo.save()
    
    # 4. 切换至操作模式
    node.nmt.state = 'OPERATIONAL'
    
    # 5. 设置控制模式（例如位置模式）
    node.sdo['0x6060'].raw = 1  # 假设1代表位置模式
    
    # 6. 启用电机
    node.sdo['0x6040'].raw = 0x0F  # 具体值需参考设备文档
    
    # 7. 设置目标位置（示例：1000个位置单位）
    target_position = 1000
    node.sdo['0x607A'].raw = target_position
    
    # 等待运动完成（简单示例）
    time.sleep(2)
    
    # 8. 停止电机
    node.sdo['0x6040'].raw = 0x0
    
    # 9. 关闭连接
    node.nmt.state = 'PRE-OPERATIONAL'

except Exception as e:
    print(f"发生错误: {str(e)}")

finally:
    network.disconnect()
    print("CAN网络已断开")