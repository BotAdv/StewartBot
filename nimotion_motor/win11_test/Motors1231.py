import canopen
import time
import keyboard
import threading
from queue import Queue
import sys

# CAN总线参数配置
CAN_BIT_RATE = 1000000                      # 比特率
CAN_BUS_TYPE = 'canalystii'                 # CAN总线类型
CAN_CHANNEL  = 0                            # CAN通道
CAN_EDS_FILE = 'NiMotion_PMM80B_V1.14.eds'  # 伺服电机EDS文件
CAN_NODE_CNT = 10                           # 伺服电机节点数量

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
        self.enabled = False     # 电机使能状态
        self.paused = False      # 电机暂停状态
        self.running = False     # 电机运行状态
        self.target_position = 0 # 目标位置

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
        try:
            # 先清除可能的错误状态
            self.node.sdo["Controlword"].raw = 0x80  # 故障复位
            time.sleep(0.1)
            
            # 标准使能序列
            self.node.sdo["Controlword"].raw = 0x06   # 切换为准备状态
            time.sleep(0.1)
            self.node.sdo["Controlword"].raw = 0x07   # 切换为启动状态
            time.sleep(0.1)
            self.node.sdo["Controlword"].raw = 0x0F   # 使能操作
            self.enabled = True
            self.running = True
            self.paused = False
            print("电机已使能")
        except Exception as e:
            print(f"使能电机失败: {e}")
            self.enabled = False

    def disable(self):
        """
        失能电机
        """
        try:
            self.node.sdo["Controlword"].raw = 0x00   # 失能
            self.enabled = False
            self.running = False
            self.paused = False
            print("电机已失能")
        except Exception as e:
            print(f"失能电机失败: {e}")

    def pause(self):
        """
        暂停电机运动
        """
        if self.enabled and not self.paused:
            try:
                # 设置控制字为暂停（根据CiA402标准）
                self.node.sdo["Controlword"].raw = 0x0F  # 确保在运行状态
                time.sleep(0.01)
                self.node.sdo["Controlword"].raw = 0x0   # 暂停（或者使用其他暂停命令，取决于驱动器）
                self.paused = True
                self.running = False
                print("电机已暂停")
            except Exception as e:
                print(f"暂停电机失败: {e}")

    def resume(self):
        """
        恢复电机运动
        """
        if self.enabled and self.paused:
            try:
                # 恢复到运行状态
                self.node.sdo["Controlword"].raw = 0x0F  # 使能操作
                self.paused = False
                self.running = True
                print("电机已恢复运行")
            except Exception as e:
                print(f"恢复电机失败: {e}")

    def toggle_pause(self):
        """
        切换暂停状态
        """
        if not self.enabled:
            print("电机未使能，无法暂停")
            return
            
        if self.paused:
            self.resume()
        else:
            self.pause()

    def get_status(self):
        """
        获取电机状态字
        """
        try:
            status_word = self.node.sdo["Statusword"].raw
            return status_word
        except:
            return 0

    def get_status_str(self):
        """
        获取电机状态字符串
        """
        status = self.get_status()
        enabled_str = "已使能" if self.enabled else "未使能"
        paused_str = "已暂停" if self.paused else "运行中"
        running_str = "运行" if self.running else "停止"
        return f"状态: {enabled_str}, {paused_str}, {running_str}, 状态字: 0x{status:04X}"

    def start_position_ctrl(self):
        """
        启动位置控制模式
        """
        try:
            # 设置同步位置运行模式
            self.node.sdo["Mode Operation"].raw = 8

            # 读取电机当前编码器值
            self.cur_position = self.node.sdo["Pos Actual User Value"].raw
            self.target_position = self.cur_position
            print(f"当前位置: {self.cur_position}")

            # 确保RPDO映射已配置
            self.configure_rpdo_mapping()

            print("位置控制模式已启动")
        except Exception as e:
            print(f"启动位置控制模式失败: {e}")
        
    def send_position_order(self, target_pos):
        """
        发送位置命令
        :param target_pos: 目标位置
        """
        if not self.enabled or self.paused:
            print("电机未使能或已暂停，不发送位置命令")
            return
            
        try:
            self.target_position = target_pos
            
            # 尝试使用索引方式访问RPDO变量
            self.node.rpdo[1][0x607A, 0x00].raw = target_pos
            self.node.rpdo[1].transmit()
            
            # 更新当前位置
            self.cur_position = self.node.sdo["Pos Actual User Value"].raw
            
        except Exception as e:
            print(f"发送位置命令失败: {e}")
            # 尝试使用名称方式
            try:
                self.node.rpdo[1]['Target Position'].raw = target_pos
                self.node.rpdo[1].transmit()
                self.cur_position = self.node.sdo["Pos Actual User Value"].raw
            except Exception as e2:
                print(f"使用名称发送也失败: {e2}")
                # 尝试使用SDO直接设置目标位置
                try:
                    self.node.sdo["Target Position"].raw = target_pos
                    self.cur_position = self.node.sdo["Pos Actual User Value"].raw
                except Exception as e3:
                    print(f"SDO设置也失败: {e3}")
        
    def get_current_position(self):
        """
        获取当前位置
        """
        try:
            self.cur_position = self.node.sdo["Pos Actual User Value"].raw
            return self.cur_position
        except:
            return self.cur_position

    def is_target_reached(self):
        """
        检查是否到达目标位置
        """
        try:
            status = self.get_status()
            # 检查状态字的第10位（目标到达位）
            return (status & 0x1000) != 0
        except:
            return False
        
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

        # 命令队列
        self.command_queue = Queue()
        self.exit_flag = False
        self.control_thread = None

    def __del__(self):
        """
        析构函数
        """
        self.exit_flag = True
        if self.control_thread:
            self.control_thread.join(timeout=1)
        for motor in self.motors:
            motor.disable()

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
                actual_pos = self.motors[internal_index].get_current_position()
                print(f"Actual position = {actual_pos}")
            except:
                pass
            try:
                target_pos = self.motors[internal_index].target_position
                print(f"Target Position = {target_pos}")
            except:
                pass
            print(self.motors[internal_index].get_status_str())

    def print_all_motors_params(self):
        """
        打印所有电机的固定参数
        """
        for i, motor in enumerate(self.motors):
            print(f"\n=== 电机 {i + 1} (节点ID: {i + 1}) 参数 ===")
            motor.print_fixed_params()
            try:
                actual_pos = motor.get_current_position()
                print(f"Actual position = {actual_pos}")
            except:
                pass
            try:
                target_pos = motor.target_position
                print(f"Target Position = {target_pos}")
            except:
                pass
            print(motor.get_status_str())

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
            for i, motor in enumerate(self.motors):
                print(f"\n使能电机 {i+1}")
                motor.enable()
        else:
            # 使能指定电机
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                print(f"\n使能电机 {motor_index}")
                self.motors[internal_index].enable()

    def disable_motor(self, motor_index=None):
        """
        失能电机
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 失能所有电机
            for i, motor in enumerate(self.motors):
                print(f"\n失能电机 {i+1}")
                motor.disable()
        else:
            # 失能指定电机
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                print(f"\n失能电机 {motor_index}")
                self.motors[internal_index].disable()

    def toggle_motor_enable(self, motor_index):
        """
        切换电机使能状态
        :param motor_index: 电机索引
        """
        internal_index = self._validate_motor_index(motor_index)
        if internal_index is not None:
            motor = self.motors[internal_index]
            if motor.enabled:
                self.disable_motor(motor_index)
            else:
                self.enable_motor(motor_index)

    def pause_motor(self, motor_index=None):
        """
        暂停电机运动
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 暂停所有电机
            for i, motor in enumerate(self.motors):
                print(f"\n暂停电机 {i+1}")
                motor.pause()
        else:
            # 暂停指定电机
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                print(f"\n暂停电机 {motor_index}")
                self.motors[internal_index].pause()

    def resume_motor(self, motor_index=None):
        """
        恢复电机运动
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 恢复所有电机
            for i, motor in enumerate(self.motors):
                print(f"\n恢复电机 {i+1}")
                motor.resume()
        else:
            # 恢复指定电机
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                print(f"\n恢复电机 {motor_index}")
                self.motors[internal_index].resume()

    def toggle_motor_pause(self, motor_index):
        """
        切换电机暂停状态
        :param motor_index: 电机索引
        """
        internal_index = self._validate_motor_index(motor_index)
        if internal_index is not None:
            motor = self.motors[internal_index]
            if motor.enabled:
                motor.toggle_pause()
            else:
                print("电机未使能，无法切换暂停状态")

    def get_motor_status(self, motor_index=None):
        """
        获取电机状态
        :param motor_index: 电机索引，None表示所有电机，1到count()表示指定电机
        """
        if motor_index is None:
            # 获取所有电机状态
            for i, motor in enumerate(self.motors):
                print(f"\n电机 {i + 1} (节点ID: {i + 1}) 状态:")
                print(motor.get_status_str())
        else:
            # 获取指定电机状态
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                print(f"\n电机 {motor_index} (节点ID: {motor_index}) 状态:")
                print(self.motors[internal_index].get_status_str())

    def send_position_order(self, target_pos):
        """
        发送位置命令
        :param target_pos: 目标位置
        """
        if not self.enabled or self.paused:
            print("电机未使能或已暂停，不发送位置命令")
            return
            
        try:
            # 确保目标位置是整数类型
            if not isinstance(target_pos, int):
                target_pos = int(target_pos)
                
            self.target_position = target_pos
            
            # 尝试使用索引方式访问RPDO变量
            try:
                self.node.rpdo[1][0x607A, 0x00].raw = target_pos
                self.node.rpdo[1].transmit()
            except Exception as e:
                print(f"RPDO发送失败: {e}")
                # 尝试使用SDO直接设置
                try:
                    # 确保值是整数
                    self.node.sdo[0x607A].raw = int(target_pos)
                except Exception as e2:
                    print(f"SDO设置失败: {e2}")
            
            # 更新当前位置
            try:
                pos_value = self.node.sdo["Pos Actual User Value"].raw
                if isinstance(pos_value, (int, float)):
                    self.cur_position = int(pos_value)
                else:
                    # 尝试转换
                    self.cur_position = int(str(pos_value))
            except Exception as e:
                print(f"更新当前位置失败: {e}")
                
        except Exception as e:
            print(f"发送位置命令失败: {e}")
                    
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

    def start_interactive_control(self, control_motor):
        """
        启动交互式控制
        :param control_motor: 要控制的电机索引（默认1）
        """
        print("\n" + "="*50)
        print("交互式电机控制程序")
        print("="*50)
        print("控制命令：")
        print("  O 键 - 启动/停止电机1")
        print("  P 键 - 暂停/恢复电机1")
        print("  ↑ 键 - 增加目标位置 (+100)")
        print("  ↓ 键 - 减少目标位置 (-100)")
        print("  ← 键 - 小步减少目标位置 (-10)")
        print("  → 键 - 小步增加目标位置 (+10)")
        print("  1 键 - 设置目标位置为 -2000")
        print("  2 键 - 设置目标位置为 0")
        print("  3 键 - 设置目标位置为 2000")
        print("  S 键 - 显示电机状态")
        print("  R 键 - 重新读取当前位置")
        print("  Q 键 - 退出程序")
        print("="*50)
        
        # 启动键盘监听线程
        keyboard_thread = threading.Thread(target=self._keyboard_listener, args=(control_motor,), daemon=True)
        keyboard_thread.start()
        
        # 控制循环
        try:
            while not self.exit_flag:
                # 处理命令队列中的命令
                while not self.command_queue.empty():
                    command = self.command_queue.get()
                    self._process_command(command, control_motor)
                
                # 显示状态
                internal_index = self._validate_motor_index(control_motor)
                if internal_index is not None:
                    motor = self.motors[internal_index]
                    if motor.enabled and not motor.paused:
                        # 检查是否到达目标位置
                        if motor.is_target_reached():
                            print(f"电机{control_motor}已到达目标位置: {motor.target_position}")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        finally:
            self.exit_flag = True
            print("正在退出程序...")
            self.disable_motor(control_motor)
            print("程序退出")
            
    def _keyboard_listener(self, control_motor):
        """
        键盘监听函数
        """
        print("键盘监听已启动...")
        
        while not self.exit_flag:
            try:
                # 检查按键
                if keyboard.is_pressed('o'):
                    self.command_queue.put(('toggle_enable', control_motor))
                    time.sleep(0.3)  # 防抖
                elif keyboard.is_pressed('p'):
                    self.command_queue.put(('toggle_pause', control_motor))
                    time.sleep(0.3)
                elif keyboard.is_pressed('up'):
                    self.command_queue.put(('position', control_motor, 'increase', 500))
                    time.sleep(0.1)
                elif keyboard.is_pressed('down'):
                    self.command_queue.put(('position', control_motor, 'decrease', 500))
                    time.sleep(0.1)
                elif keyboard.is_pressed('left'):
                    self.command_queue.put(('position', control_motor, 'decrease', 100))
                    time.sleep(0.1)
                elif keyboard.is_pressed('right'):
                    self.command_queue.put(('position', control_motor, 'increase', 100))
                    time.sleep(0.1)
                elif keyboard.is_pressed('1'):
                    self.command_queue.put(('position', control_motor, 'set', -2000))
                    time.sleep(0.3)
                elif keyboard.is_pressed('2'):
                    self.command_queue.put(('position', control_motor, 'set', 0))
                    time.sleep(0.3)
                elif keyboard.is_pressed('3'):
                    self.command_queue.put(('position', control_motor, 'set', 2000))
                    time.sleep(0.3)
                elif keyboard.is_pressed('s'):
                    self.command_queue.put(('status', control_motor))
                    time.sleep(0.3)
                elif keyboard.is_pressed('r'):
                    self.command_queue.put(('read_position', control_motor))
                    time.sleep(0.3)
                elif keyboard.is_pressed('c'):  # 添加清除故障命令
                    self.command_queue.put(('clear_fault', control_motor))
                    time.sleep(0.3)
                elif keyboard.is_pressed('q'):
                    self.command_queue.put(('exit',))
                    time.sleep(0.3)
                    
                time.sleep(0.01)  # 降低CPU使用率
                
            except Exception as e:
                print(f"键盘监听错误: {e}")

    def _process_command(self, command, control_motor):
        """
        处理命令
        """
        cmd_type = command[0]
        
        if cmd_type == 'toggle_enable':
            motor_index = command[1]
            print(f"\n[命令] 切换电机{motor_index}使能状态")
            self.toggle_motor_enable(motor_index)
            
        elif cmd_type == 'toggle_pause':
            motor_index = command[1]
            print(f"\n[命令] 切换电机{motor_index}暂停状态")
            self.toggle_motor_pause(motor_index)
            
        elif cmd_type == 'position':
            motor_index = command[1]
            action = command[2]
            
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                motor = self.motors[internal_index]
                
                # 检查电机状态，如果报警先清除
                if motor.enabled:
                    status = motor.get_status()
                    # 检查是否在错误状态 (状态字第3位)
                    if status & 0x0008:
                        print(f"电机{motor_index}处于错误状态，尝试清除...")
                        motor.disable()
                        time.sleep(0.1)
                        motor.enable()
                        time.sleep(0.1)
                
                if action == 'increase':
                    amount = command[3]
                    # 获取当前位置作为基准
                    try:
                        current_pos = motor.get_current_position()
                        new_position = current_pos + amount
                        print(f"\n[命令] 增加电机{motor_index}目标位置: {current_pos} -> {new_position}")
                        # 这里改为调用 motor.send_position_order，而不是 self.send_position_order
                        motor.send_position_order(new_position)
                    except Exception as e:
                        print(f"获取当前位置失败: {e}")
                        
                elif action == 'decrease':
                    amount = command[3]
                    try:
                        current_pos = motor.get_current_position()
                        new_position = current_pos - amount
                        print(f"\n[命令] 减少电机{motor_index}目标位置: {current_pos} -> {new_position}")
                        # 这里改为调用 motor.send_position_order，而不是 self.send_position_order
                        motor.send_position_order(new_position)
                    except Exception as e:
                        print(f"获取当前位置失败: {e}")
                        
                elif action == 'set':
                    position = command[3]
                    # 检查位置是否在合理范围内
                    if abs(position) > 1000000:  # 根据实际需要调整
                        print(f"警告: 目标位置{position}超出建议范围")
                        
                    print(f"\n[命令] 设置电机{motor_index}目标位置为: {position}")
                    # 先检查电机状态
                    if not motor.enabled:
                        print("电机未使能，先使能电机...")
                        motor.enable()
                        time.sleep(0.5)
                    
                    # 这里改为调用 motor.send_position_order，而不是 self.send_position_order
                    motor.send_position_order(position)
                    
        elif cmd_type == 'status':
            motor_index = command[1]
            print(f"\n[命令] 显示电机{motor_index}状态")
            self.get_motor_status(motor_index)
            # 增加错误代码读取
            if 1 <= motor_index <= len(self.motors):
                try:
                    error_code = self.motors[motor_index-1].node.sdo[0x603F].raw
                    if error_code != 0:
                        print(f"错误代码: 0x{error_code:08X}")
                    else:
                        print("无错误代码")
                except:
                    print("无法读取错误代码")
                
        elif cmd_type == 'read_position':
            motor_index = command[1]
            internal_index = self._validate_motor_index(motor_index)
            if internal_index is not None:
                try:
                    position = self.motors[internal_index].get_current_position()
                    print(f"\n[命令] 电机{motor_index}当前位置: {position}")
                except Exception as e:
                    print(f"读取位置失败: {e}")
                        
        elif cmd_type == 'exit':
            print("\n[命令] 退出程序")
            self.exit_flag = True

    def clear_fault(self):
        """
        清除电机故障
        """
        try:
            # 读取错误代码
            error_code = self.node.sdo[0x603F].raw
            if error_code != 0:
                print(f"当前错误代码: 0x{error_code:08X}")
            
            # 标准故障清除序列
            self.node.sdo["Controlword"].raw = 0x80  # 故障复位
            time.sleep(0.1)
            self.node.sdo["Controlword"].raw = 0x06  # 切换到准备状态
            time.sleep(0.1)
            self.node.sdo["Controlword"].raw = 0x07  # 切换到启动状态
            time.sleep(0.1)
            self.node.sdo["Controlword"].raw = 0x0F  # 切换到操作状态
            
            print("故障已清除")
            return True
        except Exception as e:
            print(f"清除故障失败: {e}")
            return False

if __name__ == "__main__":
    # 创建电机组
    motors = MotorGroup()
    ctrl_motor = 2
    
    # 设置固定参数前先检查状态
    print("检查电机状态...")
    motors.get_motor_status(ctrl_motor)
    
    # 如果有错误，先清除
    motors.clear_motor_errors(ctrl_motor)
    
    # 设置固定参数（第一次运行时需要）
    print("正在设置电机参数...")
    motors.set_motor_fixed_params(ctrl_motor)
    
    # 启动位置控制模式
    print("正在启动位置控制模式...")
    motors.start_position_ctrl(ctrl_motor)
    
    # 等待电机准备就绪
    time.sleep(1)
    
    # 启动交互式控制（默认控制电机1）
    motors.start_interactive_control(control_motor=ctrl_motor)
    