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
        self.network = network
        self.node = canopen.RemoteNode(node_id, eds_file)       
        self.network.add_node(self.node)
        # 避免重复初始化
        if not self.network.bus:
            self.network.connect(bustype=CAN_BUS_TYPE, channel=CAN_CHANNEL, bitrate=CAN_BIT_RATE)
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
        self.cur_position = 0    # 当前位置

    def download_fixed_params(self):
        """
        设置伺服电机的固定参数。
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
        self.node.sdo["Homing speed"]["Speed during search for zero"].raw = 5000            # 寻找原点信号的速度(用户单位/s)

        # 配置RPDO映射
        self.configure_rpdo_mapping()
    
    def configure_rpdo_mapping(self):
        """
        配置RPDO映射
        """
        # 设置NMT状态为预操作以配置PDO
        self.node.nmt.state = 'PRE-OPERATIONAL'
        
        # 配置RPDO1
        self.node.rpdo.read()
        self.node.rpdo[1].clear()
        
        # 使用对象字典索引和子索引来添加变量
        # 0x607A是目标位置的对象字典索引，0x00是子索引
        try:
            # 尝试使用索引方式添加变量
            self.node.rpdo[1].add_variable(0x607A, 0x00)  # Target Position
        except Exception as e:
            print(f"添加RPDO变量失败: {e}")
            # 尝试使用名称方式
            try:
                self.node.rpdo[1].add_variable("Target Position")
            except Exception as e2:
                print(f"使用名称添加RPDO变量也失败: {e2}")
                # 使用EDS文件中可能存在的其他名称
                try:
                    self.node.rpdo[1].add_variable("Target position")
                except:
                    print("无法添加Target Position变量，请检查EDS文件")
        
        # 使能RPDO1
        self.node.rpdo[1].enabled = True
        
        # 保存RPDO配置
        try:
            self.node.rpdo.save()
            print("RPDO配置已保存")
        except Exception as e:
            print(f"保存RPDO配置失败: {e}")
        
        # 设置NMT状态为操作
        self.node.nmt.state = 'OPERATIONAL'
        
        # 等待配置生效
        time.sleep(0.1)
    
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
        print("Homing method       = %d" % self.node.sdo["Homing method"].raw)
        print("Position control parameters       = %d" % self.node.sdo["Position control parameters"]["StepAmount"].raw)
        print("Encoder resolution       = %d" % self.node.sdo["Position encoder resolution"]["Encoder increments"].raw)
        print("RatedVoltage       = %d" % self.node.sdo["Stepper motor parameters"]["RatedVoltage"].raw)

    def enable(self):
        """
        使能电机
        """
        # 先清除可能的错误状态
        self.node.sdo["Controlword"].raw = 0x80  # 故障复位
        time.sleep(0.1)
        
        # 标准使能序列
        self.node.sdo["Controlword"].raw = 0x06   # 切换为准备状态
        time.sleep(0.1)
        self.node.sdo["Controlword"].raw = 0x07   # 切换为启动状态
        time.sleep(0.1)
        self.node.sdo["Controlword"].raw = 0x0F   # 使能操作
        
    def disable(self):
        """
        失能电机
        """
        self.node.sdo["Controlword"].raw = 0x00   # 失能
        
    def get_status(self):
        """
        获取电机状态字
        """
        status_word = self.node.sdo["Statusword"].raw
        print(f"电机状态字: 0x{status_word:04X}")
        return status_word
    
    def start_position_ctrl(self):
        """
        启动位置控制模式
        """
        # 设置同步位置运行模式
        self.node.sdo["Mode Operation"].raw = 8

        # 读取电机当前编码器值
        self.cur_position = self.node.sdo["Pos Actual User Value"].raw
        print(f"当前位置: {self.cur_position}")

        # 确保RPDO映射已配置
        self.configure_rpdo_mapping()

        # 使能电机
        self.enable()
        
        # 等待电机使能
        time.sleep(0.5)
        
        # 检查电机状态
        status = self.get_status()
        if status & 0x4000:  # 检查是否处于运行状态
            print("电机已使能并准备运行")
        else:
            print("警告：电机可能未正确使能")
        
    def send_position_order(self, target_pos):
        """
        发送位置命令
        :param target_pos: 目标位置
        """
        try:
            # 尝试使用索引方式访问RPDO变量
            self.node.rpdo[1][0x607A, 0x00].raw = target_pos
            self.node.rpdo[1].transmit()
            print(f"发送目标位置: {target_pos}")
            
            # 检查发送状态
            status = self.get_status()
            if status & 0x1000:  # 目标到达位
                print("目标位置已到达")
                
        except Exception as e:
            print(f"发送位置命令失败: {e}")
            # 尝试使用名称方式
            try:
                self.node.rpdo[1]['Target Position'].raw = target_pos
                self.node.rpdo[1].transmit()
                print(f"发送目标位置(使用名称): {target_pos}")
            except Exception as e2:
                print(f"使用名称发送也失败: {e2}")
                # 尝试使用SDO直接设置目标位置
                try:
                    self.node.sdo["Target Position"].raw = target_pos
                    print(f"使用SDO设置目标位置: {target_pos}")
                except Exception as e3:
                    print(f"SDO设置也失败: {e3}")
        
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
    
    def _validate_motor_index(self, motor_index):
        """
        验证电机索引，并转换为内部索引（从0开始）
        :param motor_index: 外部索引（从1开始）
        :return: 内部索引（从0开始）或None
        """
        if motor_index is None:
            return None
        if 1 <= motor_index <= len(self.motors):
            return motor_index - 1
        else:
            print(f"错误：电机索引 {motor_index} 超出范围，有效范围：1-{len(self.motors)}")
            return None
    
    def print_motor_params(self, motor_index):
        """
        打印指定电机的固定参数
        :param motor_index: 电机索引（1到count()）
        """
        internal_index = self._validate_motor_index(motor_index)
        if internal_index is not None:
            print(f"\n=== 电机 {motor_index} (节点ID: {motor_index}) 参数 ===")
            self.motors[internal_index].print_fixed_params()
            try:
                actual_pos = self.motors[internal_index].node.sdo["Pos Actual User Value"].raw
                print(f"Actual position = {actual_pos}")
            except:
                pass
            try:
                target_pos = self.motors[internal_index].node.sdo["Target Position"].raw
                print(f"Target Position in motor = {target_pos}")
            except:
                pass

    def print_all_motors_params(self):
        """
        打印所有电机的固定参数
        """
        for i, motor in enumerate(self.motors):
            print(f"\n=== 电机 {i + 1} (节点ID: {i + 1}) 参数 ===")
            motor.print_fixed_params()
            try:
                actual_pos = self.motors[i].node.sdo["Pos Actual User Value"].raw
                print(f"Actual position = {actual_pos}")
            except:
                pass
            try:
                target_pos = self.motors[i].node.sdo["Target Position"].raw
                print(f"Target Position in motor = {target_pos}")
            except:
                pass

    def start_position_ctrl(self, motor_index=None):
        """
        启动电机的位置控制模式
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 启动所有电机的位置控制模式
            for i, motor in enumerate(self.motors):
                print(f"\n启动电机 {i+1} 的位置控制模式")
                motor.start_position_ctrl()
        else:
            # 启动指定电机的位置控制模式
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                print(f"\n启动电机 {motor_index} 的位置控制模式")
                self.motors[internal_index].start_position_ctrl()
    
    def enable_motor(self, motor_index=None):
        """
        使能电机
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 使能所有电机
            for _, motor in enumerate(self.motors):
                motor.enable()
        else:
            # 使能指定电机
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                self.motors[internal_index].enable()

    def disable_motor(self, motor_index=None):
        """
        失能电机
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 失能所有电机
            for _, motor in enumerate(self.motors):
                motor.disable()
        else:
            # 失能指定电机
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                self.motors[internal_index].disable()

    def get_motor_status(self, motor_index=None):
        """
        获取电机状态
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 获取所有电机状态
            for i, motor in enumerate(self.motors):
                print(f"\n电机 {i + 1} (节点ID: {i + 1}) 状态:")
                motor.get_status()
        else:
            # 获取指定电机状态
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                print(f"\n电机 {motor_index} (节点ID: {motor_index}) 状态:")
                self.motors[internal_index].get_status()

    def send_position_order(self, target_pos, motor_index=None):
        """
        向电机发送位置命令
        :param target_pos: 目标位置（单个值或列表）
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
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
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                if isinstance(target_pos, list):
                    print("警告：当指定电机索引时，目标位置应为单个值，将使用列表的第一个值")
                    self.motors[internal_index].send_position_order(target_pos[0])
                else:
                    self.motors[internal_index].send_position_order(target_pos)
                    
    def set_motor_fixed_params(self, motor_index=None):
        """
        设置电机的固定参数
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 设置所有电机的固定参数
            for i, motor in enumerate(self.motors):
                print(f"\n设置电机 {i+1} 的固定参数")
                motor.download_fixed_params()
        else:
            # 设置指定电机的固定参数
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                print(f"\n设置电机 {motor_index} 的固定参数")
                self.motors[internal_index].download_fixed_params()
        
    def clear_motor_errors(self, motor_index=None):
        """
        清除电机错误
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 清除所有电机错误
            for i, motor in enumerate(self.motors):
                print(f"清除电机 {i+1} 错误")
                motor.enable()  # enable方法中已包含故障复位
        else:
            # 清除指定电机错误
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                print(f"清除电机 {motor_index} 错误")
                self.motors[internal_index].enable()


if __name__ == "__main__":
    # 创建电机组
    motors = MotorGroup()
    
    # 清除可能的错误状态
    motors.clear_motor_errors(1)
    
    # 设置固定参数（第一次运行时需要）
    motors.set_motor_fixed_params(1)
    
    # 启动位置控制模式
    motors.start_position_ctrl(1)
    
    # 等待电机准备就绪
    time.sleep(1)
    
    # 发送位置命令
    motors.send_position_order(-100000, 1)
    
    # 等待运动完成
    time.sleep(2)
    
    # 打印参数
    motors.print_motor_params(1)
    
    # 发送另一个位置命令
    motors.send_position_order(-50000, 1)
    
    # 等待运动完成
    time.sleep(2)
    
    # 打印参数
    motors.print_motor_params(1)
    
    # 失能电机
    motors.disable_motor(1)