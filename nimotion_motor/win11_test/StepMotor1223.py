import canopen
import time

# CAN总线参数配置
CAN_BIT_RATE = 1000000                      # 比特率
CAN_BUS_TYPE = 'canalystii'                 # CAN总线类型
CAN_CHANNEL  = 0                            # CAN通道
CAN_EDS_FILE = 'NiMotion_STM42A_V1.07.eds'  # 伺服电机EDS文件
CAN_NODE_CNT = 4                            # 伺服电机节点数量

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
            self.network.connect(bustype=CAN_BUS_TYPE, channel=CAN_CHANNEL, bitrate=CAN_BIT_RATE)  # 根据硬件配置修改参数
            print("CAN总线连接成功")

        self.mcs = 2             # 电机细分
        self.current_acc = 1.0   # 电机加速电流，默认为1A
        self.current_run = 1.0   # 电机工作电流
        self.current_hold = 1.0  # 电机保持电流
        self.acc = 0             # 电机加速度
        self.dec = 0             # 电机减速度

        self.smooth_K = 0        # 平滑系数
        self.cur_pulse = 0.0     # 当前位置(以脉冲为单位)
        self.pulse_cycle  = 10000
        
        # self.initialize()

    # def initialize(self):

    #     # 设置电机细分

    #     sec_table = {1:0, 2:1, 4:2, 6:3, 8:4, 16:5, 32:6, 64:7, 128:8}
    #     cur_pluse = 200 * pow(2, self.mcs)
    #     self.node.sdo.download(0x2002, 0, b'\x02')  # 设置细分为2

    #     # 电机使能，设置当前位置为原点位置
    #     self.node.sdo.download(0x2010, 0, b'\x01')             # 使能电机
    #     self.node.sdo.download(0x2010, 0, b'\x00\x02')  # 设置当前位置为原点

    #     # 设置电机加速度、减速度
    #     self.node.sdo.download(0x2024, 0, b'\x00\x40\x9c\x47') # 设置加速度
    #     self.node.sdo.download(0x2025, 0, b'\x00\x40\x9c\x47') # 设置减速度

    #     # 设置电机加速电流，工作电流和保持电流
    #     self.node.sdo.download(0x2027, 0, b'\x00\x00\x80\x3f') # 设置加速电流,影响加速过程中的扭矩。
    #     self.node.sdo.download(0x2027, 0, b'\x00\x00\x80\x3f') # 设置工作电流,影响持续运行时的扭矩和发热。
    #     self.node.sdo.download(0x2028, 0, b'\x00\x00\x80\x3f') # 设置保持电流,影响电机在停止状态的稳定性和功耗

    def download_fixed_params(self):
        """
        设置伺服电机的固定参数。这些参数通常需要电机重新启动后生效，且参数下载后即存储在电机驱动器中，因此该函数仅需要在相关参数修改后调用一次
        :return:
        """
        # 周期同步位置控制参数
        self.node.sdo["Basic control parameters"]["Contrl Mode Select"].raw         = 0          # 设置CiA402模式
        self.node.sdo["Mode Operation"].raw = 8                                             # 设置CSP同步位置控制模式
        self.node.sdo["Position range limit"]["Min position range limit"].raw  = -1048576   # 最小位置范围限制
        self.node.sdo["Position range limit"]["Max position range limit"].raw  = 1048576    # 最大位置范围限制
        self.node.sdo["Software position limit"]["Min position limit"].raw = -65535     # 最小软件位置限制
        self.node.sdo["Software position limit"]["Max position limit"].raw = 65535      # 最大软件位置限制
        self.node.sdo["Polarity"].raw             = 0                                       # 极性设置
        self.node.sdo["Profile acceleration"].raw = 409600                                  # 加速度设置
        self.node.sdo["Profile deceleration"].raw = 409600                                  # 减速度设置
        self.node.sdo["Max motor speed"].raw      = 3000                                    # 最大电机速度
        self.node.sdo["Max profile velocity"].raw = 500000                              # 最大轮廓速度

        # 原点回归参数
        self.node.sdo["Homing method"].raw = 18                                             # 设置原点回归方式为18
        self.node.sdo["Input terminal parameters"]["DI1FunSelec"].raw = 14                  # 实体输入端子设置为原点开关
        self.node.sdo["Input terminal parameters"]["DI1LogicSelec"].raw = 1                 # 实体输入端子下降沿有效(NPN型)
        # self.node.sdo["Position control parameters"]["HomingDurationLimit"].raw = 30000   # 设置原点回归超时(单位：(0-65535)ms)
        self.node.sdo["Homing speed"]["Speed during search for zero"].raw = 5000            # 寻找原点信号的速度(用户单位/s)

        # 配置RPDO映射
        self.node.rpdo.read()
        self.node.rpdo[1].clear()
        self.node.rpdo[1].add_variable("Target Position")
        self.node.rpdo[1].enabled = True

        self.node.nmt.state = 'PRE-OPERATIONAL' # 设置NMT状态为预操作
        self.node.rpdo.save()
        self.node.nmt.state = 'OPERATIONAL'     # 设置NMT状态为操作
        # 配置TPDO映射
        pass
    
    def print_fixed_params(self):
        """
        打印显示伺服电机的固定参数，以便于调试使用
        :return:
        """
        print("Contrl Mode Select   = %d" % self.node.sdo["Basic control parameters"]["Contrl Mode Select"].raw)
        print("Mode Operation       = %d" % self.node.sdo["Mode Operation"].raw)
        print("MinPosRangLimt       = %d" % self.node.sdo["Position range limit"]["Min position range limit"].raw)
        print("MaxPosRangLimt       = %d" % self.node.sdo["Position range limit"]["Max position range limit"].raw)
        print("MinPosLimt           = %d" % self.node.sdo["Software position limit"]["Min position limit"].raw)
        print("MaxPosLimt           = %d" % self.node.sdo["Software position limit"]["Max position limit"].raw)
        print("Polarity             = %d" % self.node.sdo["Polarity"].raw)
        print("Profile acceleration = %d" % self.node.sdo["Profile acceleration"].raw)
        print("Profile deceleration = %d" % self.node.sdo["Profile deceleration"].raw)
        print("Statusword           = %d" % self.node.sdo["Statusword"].raw)
        # print("Pre-defined Error Field = %d" % self.node.sdo["Pre-defined Error Field"]["standard error field"].raw)
        print("Homing method       = %d" % self.node.sdo["Homing method"].raw)
        print("Position control parameters       = %d" % self.node.sdo["Position control parameters"]["StepAmount"].raw)
        print("Encoder resolution       = %d" % self.node.sdo["Position encoder resolution"]["Encoder increments"].raw)
        print("RatedVoltage       = %d" % self.node.sdo["Stepper motor parameters"]["RatedVoltage"].raw)
        print("Actual position = %d" % motors.motors[0].node.sdo["Pos Actual User Value"].raw)

    
    def enable(self):
        """
        使能电机
        """
        # 这里换成索引名
        self.node.sdo["Controlword"].raw = 6   # 电机准备
        self.node.sdo["Controlword"].raw = 7   # 电机失能
        self.node.sdo["Controlword"].raw = 15  # 电机使能
        
    def get_status(self):
        status_word = self.node.sdo["Statusword"].raw
        print(f"电机状态字: {status_word}")
        
    
    def position_ctrl(self, ang_order, interval):
        """
        位置控制
        :param pos_order: 目标位置，单位：度
        :param interval: 控制周期(s)
        :return:
        """
        try:
            pulse = self.mcs * (ang_order / 1.8)  # 计算目标脉冲数
            print(f"目标脉冲数: {pulse}")

            # 确保脉冲值是 int32 类型
            pulse_int32 = int(pulse).to_bytes(4, byteorder='little', signed=True)
            
            self.node.rpdo.read()
            self.node.rpdo[2]['Target Position'].raw = pulse_int32  # 设置电机目标位置
            print(f"发送的目标位置: {pulse_int32.hex()}")

            self.node.rpdo[2].transmit()
            print("RPDO 发送成功")
            
            # 读取状态字以确认电机是否运行
            self.get_status()
        except RuntimeError as e:
            print(f"发送消息失败: {e}")
                    
            # 发送RPDO
    
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
        
class MotorGroup(object):
    def __init__(self):
        """
        初始化电机组
        """
        # 连接CAN总线网络
        self.network = canopen.Network()
        self.network.connect(bustype=CAN_BUS_TYPE, channel=CAN_CHANNEL, bitrate=CAN_BIT_RATE)

        # 初始化电机节点
        self.motors = list()
        for i in range(CAN_NODE_CNT):
            motor = StepMotor(network=self.network, node_id=i + 1, eds_file=CAN_EDS_FILE)
            self.motors.append(motor)

    def __del__(self):
        """
        析构函数
        """
        pass

    def count(self):
        """
        获取电机数量
        :return: 电机数量
        """
        return len(self.motors)
    
    def print_motor_params(self, motor_index):
        """
        打印指定电机的固定参数
        :param motor_index: 电机索引（0到count()-1）
        """
        if 1 <= motor_index < len(self.motors) + 1:
            print(f"\n=== 电机 {motor_index} (节点ID: {motor_index}) 参数 ===")
            self.motors[motor_index - 1].print_fixed_params()
        else:
            print(f"错误：电机索引 {motor_index} 超出范围，有效范围：1-{len(self.motors)}")

    def print_all_motors_params(self):
        """
        打印所有电机的固定参数
        """
        for i, motor in enumerate(self.motors):
            print(f"\n=== 电机 {i + 1} (节点ID: {i + 1}) 参数 ===")
            motor.print_fixed_params()

    def start_position_ctrl(self, motor_index=None):
        """
        启动电机的位置控制模式
        :param motor_index: 电机索引，None表示所有电机
        """
        if motor_index is None:
            # 启动所有电机的位置控制模式
            for _, motor in enumerate(self.motors):
                motor.start_position_ctrl()
        else:
            # 启动指定电机的位置控制模式
            if 0 <= motor_index < len(self.motors):
                self.motors[motor_index].start_position_ctrl()
            else:
                print(f"错误：电机索引 {motor_index} 超出范围，有效范围：0-{len(self.motors)-1}")

    def send_position_order(self, target_pos, motor_index=None):
        """
        向电机发送位置命令
        :param target_pos: 目标位置（单个值或列表）
        :param motor_index: 电机索引，None表示所有电机
        """
        if motor_index is None:
            # 向所有电机发送位置命令
            if isinstance(target_pos, list) and len(target_pos) == len(self.motors):
                for i, motor in enumerate(self.motors):
                    motor.send_position_order(target_pos[i])
            else:
                print(f"错误：目标位置必须是长度为{len(self.motors)}的列表")
        else:
            # 向指定电机发送位置命令
            if 0 <= motor_index < len(self.motors):
                if isinstance(target_pos, list):
                    print("警告：当指定电机索引时，目标位置应为单个值")
                    # 使用列表中的第一个值
                    self.motors[motor_index].send_position_order(target_pos[0])
                else:
                    self.motors[motor_index].send_position_order(target_pos)
            else:
                print(f"错误：电机索引 {motor_index} 超出范围，有效范围：0-{len(self.motors)-1}")
    def start_home_ctrl(self, motor_index=None):
        """
        启动电机的原点回归模式
        :param motor_index: 电机索引，None表示所有电机
        """
        if motor_index is None:
            # 启动所有电机的原点回归模式
            for _, motor in enumerate(self.motors):
                motor.start_home_ctrl()
        else:
            # 启动指定电机的原点回归模式
            if 0 <= motor_index < len(self.motors):
                self.motors[motor_index].start_home_ctrl()
            else:
                print(f"错误：电机索引 {motor_index} 超出范围，有效范围：0-{len(self.motors)-1}")

    def homing(self, motor_index=None):
        """
        检查电机是否完成原点回归
        :param motor_index: 电机索引，None表示所有电机
        :return: 是否完成原点回归
        """
        if motor_index is None:
            # 检查所有电机
            for i, motor in enumerate(self.motors):
                if motor.homing():
                    return True
            return False
        else:
            # 检查指定电机
            if 0 <= motor_index < len(self.motors):
                return self.motors[motor_index].homing()
            else:
                print(f"错误：电机索引 {motor_index} 超出范围，有效范围：0-{len(self.motors)-1}")
                return False

    def get_motor_object(self, motor_index):
        """
        获取指定电机的对象，以便直接操作
        :param motor_index: 电机索引
        :return: StepMotor对象
        """
        if 1 <= motor_index < len(self.motors)+1:
            return self.motors[motor_index]
        else:
            print(f"错误：电机索引 {motor_index} 超出范围，有效范围：1-{len(self.motors)}")
            return None
        
# def position_ctrl_test(motors):
#     """
#     位置控制测试
#     :param motors: 电机组
#     """
#     max_rpm = 1000
#     motors.start_position_ctrl()

#     target_pos = list()
#     for _, motor in enumerate(motors.motors):
#         target_pos.append(motor.cur_position)

#     rpm = 0
#     acc = True
#     for t in range(10000):
#         if rpm > max_rpm:
#             acc = False
#         elif rpm < -max_rpm:
#             acc = True
#         rpm += 5 if acc else -5

#         for i, motor in enumerate(motors.motors):
#             target_pos[i] += rpm * motor.pulse_cycle // (60 * 50)
#         motors.send_position_order(target_pos)
#         time.sleep(0.005)

#     time.sleep(1)
#     for i in range(motors.motor_count()):
#         print("Motor %d" % (i + 1))
#         print("Target Position = %d" % target_pos[i])
#         print("Target Position in motor = %d" % motors.motors[0].node.sdo["Target Position"].raw)
#         print("Actual position = %d" % motors.motors[0].node.sdo["Pos Actual User Value"].raw)

def position_ctrl_test(motors):
    """
    位置控制测试 - 变速往复运动
    :param motors: 电机组
    """
    max_rpm = 300
    min_rpm = -300
    start_rpm = -50
    rpm_step = 5  # 转速变化步长
    
    motors.start_position_ctrl()

    # 初始化目标位置
    target_pos = list()
    for _, motor in enumerate(motors.motors):
        target_pos.append(motor.cur_position)

    rpm = start_rpm
    direction = -1  # 初始方向：减速（从-50到-300）
    
    for t in range(10000):
        # 更新转速
        rpm += rpm_step * direction
        
        # 检查边界，改变方向
        if rpm <= min_rpm:  # 达到-300rpm，转为加速
            rpm = min_rpm
            direction = 1  # 改为加速方向
        elif rpm >= max_rpm:  # 达到300rpm，转为减速
            rpm = max_rpm
            direction = -1  # 改为减速方向
        
        # 更新每个电机的目标位置
        for i, motor in enumerate(motors.motors):
            target_pos[i] += rpm * motor.pulse_cycle // (60 * 50)
        
        # 发送位置指令
        motors.send_position_order(target_pos)
        time.sleep(0.005)

    # 等待运动完成并打印最终状态
    time.sleep(1)
    for i in range(motors.motor_count()):
        print("Motor %d" % (i + 1))
        print("Target Position = %d" % target_pos[i])
        print("Target Position in motor = %d" % motors.motors[0].node.sdo["Target Position"].raw)
        print("Actual position = %d" % motors.motors[0].node.sdo["Pos Actual User Value"].raw)


if __name__ == "__main__":

    # network = canopen.Network()  
    # motor = StepMotor(network, 1, CAN_EDS_FILE)
    # motor.download_fixed_params()
    # motor.print_fixed_params()
    
    # 电机组运动
    motors = MotorGroup()
    # motors.print_all_motors_params()
    # motors.start_position_ctrl(1)
    # motors.send_position_order(-5000,1)
    motors.print_motor_params(1)
    motors.get_motor_object(5)
    # position_ctrl_test(motors)
    
    # motor.position_ctrl(360.0, 0.2)
    # motor.start_position_ctrl()
    # motor.send_position_order(-5000)