import canopen
import time

# CAN总线参数配置
CAN_BIT_RATE = 1000000                      # 比特率
CAN_BUS_TYPE = 'canalystii'                 # CAN总线类型
CAN_CHANNEL  = 0                            # CAN通道
CAN_EDS_FILE = 'NiMotion_STM42A_V1.07.eds'  # 伺服电机EDS文件
CAN_NODE_CNT = 1                            # 伺服电机节点数量

class StepMotor(object):
    def __init__(self, network, node_id, eds_file):
        """
        :param network: CAN总线网络对象
        :param node_id: 电机节点ID
        :param eds_file: 电机的EDS文件路径
        """
        # print(f"传入的eds_file路径: {eds_file}")  # 调试输出eds文件路径
        self.network = network
        self.node = canopen.RemoteNode(node_id, eds_file)       
        self.network.add_node(self.node)
        # 避免重复初始化
        if not self.network.bus:
            try:
                self.network.connect(bustype=CAN_BUS_TYPE, channel=CAN_CHANNEL, bitrate=CAN_BIT_RATE)
                print("CAN总线连接成功")
            except Exception as e:
                print(f"CAN总线连接失败: {e}")

        self.mcs = 2             # 电机细分
        self.current_acc = 1.0   # 电机加速电流，默认为1A
        self.current_run = 1.0   # 电机工作电流
        self.current_hold = 1.0  # 电机保持电流
        self.acc = 0             # 电机加速度
        self.dec = 0             # 电机减速度

        self.smooth_K = 0        # 平滑系数
        self.cur_position = 0    # 电机当前编码器位置
        self.pulse_cycle = 16384 # 电机每转动一周的编码器脉冲数量

    def download_fixed_params(self):
        """
        设置伺服电机的固定参数。这些参数通常需要电机重新启动后生效，且参数下载后即存储在电机驱动器中，因此该函数仅需要在相关参数修改后调用一次
        :return:
        """
        # 周期同步位置控制参数
        self.node.sdo["Basic control parameters"]["Contrl Mode Select"].raw         = 0     # 设置CiA402模式 2002
        self.node.sdo["Mode Operation"].raw = 8                                             # 设置位置控制模式 P33 607B
        self.node.sdo["Position range limit"]["Min position range limit"].raw  = -1048576   # 最小位置范围限制 607B
        self.node.sdo["Position range limit"]["Max position range limit"].raw  = 1048576    # 最大位置范围限制 607B
        self.node.sdo["Software position limit"]["Min position limit"].raw = -65535         # 最小软件位置限制 607D
        self.node.sdo["Software position limit"]["Max position limit"].raw = 65535          # 最大软件位置限制 607D
        self.node.sdo["Polarity"].raw             = 0                                       # 极性设置 607E
        self.node.sdo["Profile acceleration"].raw = 409600                                  # 加速度设置 6083
        self.node.sdo["Profile deceleration"].raw = 409600                                  # 减速度设置 6084
        self.node.sdo["Max motor speed"].raw      = 3000                                    # 最大电机速度 6080
        self.node.sdo["Max profile velocity"].raw = 500000                                  # 最大轮廓速度 607F
        self.node.sdo["Target Position"].raw = -180000

        # # 原点回归参数
        # self.node.sdo["Homing method"].raw = 18                                             # 设置原点回归方式为18
        # self.node.sdo["Input terminal parameters"]["DI1FunSelec"].raw = 14                  # 实体输入端子设置为原点开关
        # self.node.sdo["Input terminal parameters"]["DI1LogicSelec"].raw = 1                 # 实体输入端子下降沿有效(NPN型)
        # self.node.sdo["Position control parameters"]["StepAmount"].raw = 1000               # 步进量:-32768~32767
        # self.node.sdo["Homing speed"]["Speed during search for zero"].raw = 5000            # 6099寻找原点信号的速度(用户单位/s)

        # 配置RPDO映射
        self.node.nmt.state = 'PRE-OPERATIONAL' # 设置NMT状态为预操作
        self.node.rpdo.read()
        self.node.rpdo[1].clear()
        # self.node.rpdo[1].add_variable(0x607A, 0)                                   # 预设的目标位置（用户单位）607A 
        self.node.rpdo[1].add_variable("Target Position")                                   # 预设的目标位置（用户单位）607A Target Position
        self.node.rpdo[1].enabled = True
        self.node.rpdo.save()
        self.node.nmt.state = 'OPERATIONAL'     # 设置NMT状态为操作
        print("RPDO1映射内容:", self.node.rpdo[1].map)
        # 配置TPDO映射
        pass
    
    def print_fixed_params(self):
        """
        打印显示伺服电机的固定参数，以便于调试使用
        :return:
        """
        print("CtrlModeSelec        = %d" % self.node.sdo["Basic control parameters"]["Contrl Mode Select"].raw) #2002
        print("Mode Operation       = %d" % self.node.sdo["Mode Operation"].raw) #6060
        print("MinPosRangLimt       = %d" % self.node.sdo["Position range limit"]["Min position range limit"].raw) #607B
        print("MaxPosRangLimt       = %d" % self.node.sdo["Position range limit"]["Max position range limit"].raw) #607B
        print("MinPosLimt           = %d" % self.node.sdo["Software position limit"]["Min position limit"].raw) #607D
        print("MaxPosLimt           = %d" % self.node.sdo["Software position limit"]["Max position limit"].raw) #607D
        print("Polarity             = %d" % self.node.sdo["Polarity"].raw) #607E
        print("Profile acceleration = %d" % self.node.sdo["Profile acceleration"].raw) #6083
        print("Profile deceleration = %d" % self.node.sdo["Profile deceleration"].raw) #6084
        print("Statusword           = %d" % self.node.sdo["Statusword"].raw) #6041
        # print("Pre-defined Error Field = %d" % self.node.sdo["Pre-defined Error Field"]["Standard Error Field"].raw)
        print("Homing method        = %d" % self.node.sdo["Homing method"].raw) #6098
        print("Position ctrl para   = %d" % self.node.sdo["Position control parameters"]["StepAmount"].raw) #2005
        print("Encoder resolution   = %d" % self.node.sdo["Position encoder resolution"]["Encoder increments"].raw) #608F
        print("RatedVoltage(V)      = %d" % self.node.sdo["Stepper motor parameters"]["RatedVoltage"].raw) #2000
        print("Max motor speed(rpm) = %d" % self.node.sdo["Max motor speed"].raw) #6080
        print("Controlword          = %d" % self.node.sdo["Controlword"].raw) #6040
        print("Pos Actual Value     = %d" % self.node.sdo["Pos Actual User Value"].raw) #6064
        print("Target Position      = %d" % self.node.sdo["Target Position"].raw) #607A
        print("Pos Demand Value     = %d" % self.node.sdo["Pos Demand Value"].raw) #6062
        
    
    def enable(self):
        """
        使能电机
        """
        # 这里换成索引名
        self.node.sdo["Controlword"].raw = 6   # 电机准备
        self.node.sdo["Controlword"].raw = 7   # 电机失能
        self.node.sdo["Controlword"].raw = 15  # 电机使能
        time.sleep(0.1)  # 等待状态更新
        status = self.node.sdo["Statusword"].raw
        print(f"使能后状态字: {bin(status)}")  # 应为0bxxxxxx1xxx（驱动使能）
        
    def get_status(self):
        status_word = self.node.sdo["Statusword"].raw
        print(f"电机状态字: {status_word}")
    
    def start_position_ctrl(self):
        """
        启动位置控制模式
        """
        # 设置同步位置运行模式
        self.node.sdo["Mode Operation"].raw = 8

        # 读取电机当前编码器值
        self.cur_position = self.node.sdo["Pos Actual User Value"].raw

        # RPDO已经离线配置好了，仅需要读取配置
        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.rpdo.read()
        self.node.rpdo[1].enabled = True
        self.node.nmt.state = 'OPERATIONAL'

        # 重新使能电机
        self.enable()
        
    def send_position_order(self, target_pos):
        """
        发送位置命令
        :param target_pos: 目标位置
        """
        self.node.rpdo[1]['Target Position'].raw = target_pos
        self.node.rpdo[1].transmit()        

# def position_ctrl_test(motor):
#     """
#     位置控制测试
#     :param motors: 电机组
#     """
#     max_rpm = 1000
#     motor.start_position_ctrl()
#     target_pos = motor.cur_position
#     rpm = 0
#     acc = True
#     for t in range(100):
#         if rpm > max_rpm:
#             acc = False
#         elif rpm < -max_rpm:
#             acc = True
#         rpm += 5 if acc else -5    
#         target_pos += rpm * motor.pulse_cycle // (60 * 50)
#         motor.send_position_order(target_pos)
#         actual_pos = motor.node.sdo["Pos Actual User Value"].raw
#         print(f"目标位置: {target_pos}, 实际位置: {actual_pos}")
#         time.sleep(0.005)
#         # time.sleep(0.005)

#     time.sleep(1)
    
#     print("Target position = %d" % target_pos)

def position_ctrl_test(motor):
    max_rpm = 75
    A = -1000000
    B = -700000
    total_travel = B - A  # 计算总行程长度
    direction = 1  # 初始方向为A到B
    motor.start_position_ctrl()
    target_pos = A  # 起始位置设为A
    reversed_flag = False  # 用于标记是否刚刚反转方向

    for t in range(30000):
        # 根据当前方向确定起点和终点
        if direction == 1:
            current_start, current_end = A, B
            d = target_pos - current_start
        else:
            current_start, current_end = B, A
            d = current_start - target_pos

        # 处理方向反转后的初始微小移动
        if reversed_flag:
            d = 1  # 设置微小位移以启动运动
            reversed_flag = False
        else:
            d = max(0, min(d, total_travel))  # 确保位移在合理范围内

        # 计算速度比例系数
        if d < total_travel / 2:
            rpm_ratio = d / (total_travel / 2)
        else:
            rpm_ratio = (total_travel - d) / (total_travel / 2)

        # 计算实际转速（考虑方向）
        actual_rpm = rpm_ratio * max_rpm * direction

        # 计算位置增量
        delta = actual_rpm * motor.pulse_cycle // (60 * 50)
        # 确保至少产生1个脉冲的位移
        if delta == 0:
            delta = 1 if direction == 1 else -1

        # 检查边界并处理方向反转
        new_target = target_pos + delta
        if direction == 1 and new_target > current_end:
            delta = current_end - target_pos
            direction = -1
            reversed_flag = True
        elif direction == -1 and new_target < current_end:
            delta = current_end - target_pos
            direction = 1
            reversed_flag = True

        target_pos += delta

        # 发送位置指令并读取实际位置
        motor.send_position_order(target_pos)
        actual_pos = motor.node.sdo["Pos Actual User Value"].raw
        print(f"目标位置: {target_pos}, 实际位置: {actual_pos}")
        time.sleep(0.005)

if __name__ == "__main__":

    network = canopen.Network()  
    motor = StepMotor(network, 1, CAN_EDS_FILE)
    motor.download_fixed_params()
    motor.print_fixed_params()
    # motor.get_status()

    position_ctrl_test(motor)
    
    # motor.enable()
    # motor.send_position_order(-750000)
    
    print("目标位置      = %d" % motor.node.sdo["Target Position"].raw) #607A
    print("当前位置     = %d" % motor.node.sdo["Pos Actual User Value"].raw) #6064
