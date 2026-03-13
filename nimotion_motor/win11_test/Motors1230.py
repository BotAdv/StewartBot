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
CAN_EDS_FILE = 'NiMotion_STM42A_V1.07.eds'  # 伺服电机EDS文件
CAN_NODE_CNT = 4                            # 伺服电机节点数量

class StepMotor(object):
    def __init__(self, network, node_id, eds_file):
        """
        :param network: CAN总线网络对象
        :param node_id: 电机节点ID
        :param eds_file: 电机的EDS文件路径
        """
        self.node_id = node_id
        self.name = f"电机{node_id}"
        self.network = network
        
        print(f"{self.name}: 正在创建节点...")
        try:
            self.node = canopen.RemoteNode(node_id, eds_file)
            self.network.add_node(self.node)
        except Exception as e:
            print(f"{self.name}: 创建节点失败: {e}")
            raise
        
        # 连接CAN总线（只连接一次）
        if not self.network.bus:
            try:
                self.network.connect(bustype=CAN_BUS_TYPE, channel=CAN_CHANNEL, bitrate=CAN_BIT_RATE)
                print("CAN总线连接成功")
            except Exception as e:
                print(f"CAN总线连接失败: {e}")
                raise

        self.mcs = 2                          # 电机细分
        self.current_acc = 1.0                # 电机加速电流，默认为1A
        self.current_run = 1.0                # 电机工作电流
        self.current_hold = 1.0               # 电机保持电流
        self.acc = 0                          # 电机加速度
        self.dec = 0                          # 电机减速度

        self.smooth_K = 0                     # 平滑系数
        self.cur_pulse = 0.0                  # 当前位置(以脉冲为单位)
        self.pulse_cycle = 10000
        self.cur_position = 0                 # 当前位置
        self.enabled = False                  # 电机使能状态
        self.paused = False                   # 电机暂停状态
        self.running = False                  # 电机运行状态
        self.target_position = 0              # 目标位置
        self.rpdo_configured = False          # PDO是否已配置
        self.operation_mode = None            # 当前操作模式
        
        print(f"{self.name}: 初始化完成")

    def reset_communication(self):
        """
        重置通信，解决SDO通信错误
        """
        try:
            # 发送NMT复位通信
            self.node.nmt.state = 'RESET COMMUNICATION'
            time.sleep(0.1)
            self.node.nmt.state = 'OPERATIONAL'
            time.sleep(0.1)
            print(f"{self.name}: 通信已重置")
            return True
        except Exception as e:
            print(f"{self.name}: 重置通信失败: {e}")
            return False

    def clear_errors(self):
        """
        清除电机错误 - 简化版
        """
        try:
            # 首先重置通信
            self.reset_communication()
            time.sleep(0.2)
            
            # 尝试读取错误代码
            try:
                error_code = self.node.sdo[0x603F].raw
                if error_code != 0:
                    print(f"{self.name}: 当前错误代码: 0x{error_code:04X}")
                    
                    # 方法1：标准故障复位
                    print(f"{self.name}: 尝试标准故障复位...")
                    self.node.sdo["Controlword"].raw = 0x80
                    time.sleep(0.3)
                    
                    # 检查错误是否清除
                    error_code = self.node.sdo[0x603F].raw
                    if error_code == 0:
                        print(f"{self.name}: 错误已清除")
                        return True
                    
                    # 方法2：如果错误仍在，尝试驱动器特定方法
                    print(f"{self.name}: 尝试驱动器特定错误清除...")
                    # 对于STM42A驱动器，可能需要设置特定参数来清除错误
                    try:
                        # 尝试设置错误寄存器为0
                        self.node.sdo[0x603F].raw = 0
                        time.sleep(0.1)
                    except:
                        pass
                    
                    # 再次检查
                    error_code = self.node.sdo[0x603F].raw
                    if error_code == 0:
                        print(f"{self.name}: 错误已清除（方法2）")
                        return True
                    
                    # 方法3：尝试重新初始化
                    print(f"{self.name}: 尝试重新初始化...")
                    self.initialize_drive()
                    time.sleep(0.5)
                    
                    error_code = self.node.sdo[0x603F].raw
                    if error_code == 0:
                        print(f"{self.name}: 错误已清除（方法3）")
                        return True
                    
                    print(f"{self.name}: 清除错误失败，错误代码: 0x{error_code:04X}")
                    return False
                else:
                    print(f"{self.name}: 无错误")
                    return True
                    
            except Exception as e:
                print(f"{self.name}: 读取错误代码失败: {e}")
                # 尝试通过控制字复位
                try:
                    self.node.sdo["Controlword"].raw = 0x80
                    time.sleep(0.2)
                    print(f"{self.name}: 已发送复位命令")
                    return True
                except:
                    print(f"{self.name}: 发送复位命令失败")
                    return False
                    
        except Exception as e:
            print(f"{self.name}: 清除错误时发生异常: {e}")
            return False

    def initialize_drive(self):
        """
        初始化驱动器
        """
        print(f"{self.name}: 正在初始化驱动器...")
        try:
            # 发送NMT复位节点
            self.node.nmt.state = 'RESET NODE'
            time.sleep(0.5)
            
            # 等待节点复位
            self.node.nmt.state = 'OPERATIONAL'
            time.sleep(0.5)
            
            # 设置基本参数
            self.node.sdo["Basic control parameters"]["Contrl Mode Select"].raw = 0  # CiA402模式
            time.sleep(0.1)
            
            # 设置CSP模式
            self.node.sdo["Mode Operation"].raw = 8
            time.sleep(0.1)
            
            print(f"{self.name}: 驱动器初始化完成")
            return True
        except Exception as e:
            print(f"{self.name}: 驱动器初始化失败: {e}")
            return False

    def configure_rpdo_mapping(self):
        """
        配置RPDO映射 - 简化版本
        """
        if self.rpdo_configured:
            print(f"{self.name}: RPDO已配置，跳过")
            return True
            
        print(f"{self.name}: 开始配置RPDO映射...")
        
        try:
            # 进入预操作状态
            self.node.nmt.state = 'PRE-OPERATIONAL'
            time.sleep(0.1)
            
            # 读取现有RPDO配置
            self.node.rpdo.read()
            
            # 使用RPDO1作为默认（许多驱动器预配置了RPDO1）
            rpdo_index = 1
            
            if rpdo_index not in self.node.rpdo:
                print(f"{self.name}: 创建RPDO{rpdo_index}")
                self.node.rpdo[rpdo_index] = canopen.pdo.Rpdo(self.node, rpdo_index)
            
            # 清除现有映射
            self.node.rpdo[rpdo_index].clear()
            
            # 添加目标位置变量
            try:
                self.node.rpdo[rpdo_index].add_variable(0x607A, 0x00)  # Target Position
                print(f"{self.name}: 添加目标位置变量成功")
            except Exception as e:
                print(f"{self.name}: 添加目标位置变量失败: {e}")
                # 尝试使用名称
                try:
                    self.node.rpdo[rpdo_index].add_variable("Target Position")
                    print(f"{self.name}: 使用名称添加目标位置变量成功")
                except:
                    print(f"{self.name}: 添加变量失败，使用SDO进行位置控制")
                    # 如果RPDO配置失败，我们将使用SDO直接控制
                    self.node.rpdo[rpdo_index].enabled = False
                    return False
            
            # 使能RPDO
            self.node.rpdo[rpdo_index].enabled = True
            
            # 保存配置
            try:
                self.node.rpdo.save()
                print(f"{self.name}: RPDO配置已保存")
            except:
                print(f"{self.name}: 无法保存RPDO配置，使用当前配置")
            
            # 返回操作状态
            self.node.nmt.state = 'OPERATIONAL'
            time.sleep(0.1)
            
            self.rpdo_configured = True
            print(f"{self.name}: RPDO映射配置完成")
            return True
            
        except Exception as e:
            print(f"{self.name}: 配置RPDO映射时发生错误: {e}")
            # 尝试返回操作状态
            try:
                self.node.nmt.state = 'OPERATIONAL'
            except:
                pass
            return False

    def start_position_ctrl(self):
        """
        启动位置控制模式
        """
        try:
            # 清除错误
            self.clear_errors()
            
            # 设置操作模式为CSP
            print(f"{self.name}: 设置CSP模式...")
            self.node.sdo["Mode Operation"].raw = 8
            time.sleep(0.1)
            
            # 验证模式设置
            mode = self.node.sdo["Mode Operation"].raw
            if mode != 8:
                print(f"{self.name}: 模式设置失败，当前模式: {mode}")
                return False
                
            self.operation_mode = 8
            
            # 读取当前位置
            self.cur_position = self.node.sdo["Pos Actual User Value"].raw
            self.target_position = self.cur_position
            
            # 配置RPDO映射
            self.configure_rpdo_mapping()
            
            print(f"{self.name}: 位置控制模式已启动，当前位置: {self.cur_position}")
            return True
            
        except Exception as e:
            print(f"{self.name}: 启动位置控制模式失败: {e}")
            return False

    def enable(self):
        """
        使能电机 - 改进版
        """
        try:
            # 清除错误
            if not self.clear_errors():
                print(f"{self.name}: 无法清除错误")
                return False
                
            print(f"{self.name}: 开始使能序列...")
            
            # 第一步：检查当前状态
            status = self.get_status()
            print(f"{self.name}: 当前状态: 0x{status:04X}")
            
            # 如果已经使能，直接返回
            if self.enabled:
                print(f"{self.name}: 已使能")
                return True
            
            # 标准CiA402使能序列
            # 1. 切换禁用 (如果需要)
            if (status & 0x0040) == 0:  # 如果不在准备运行状态
                print(f"{self.name}: 发送准备运行命令...")
                self.node.sdo["Controlword"].raw = 0x06  # 准备运行
                time.sleep(0.2)
                
                # 等待进入准备运行状态
                for _ in range(10):
                    status = self.get_status()
                    if (status & 0x0040):  # 准备运行位
                        break
                    time.sleep(0.1)
            
            # 检查是否在准备运行状态
            status = self.get_status()
            if not (status & 0x0040):
                print(f"{self.name}: 无法进入准备运行状态，状态字: 0x{status:04X}")
                return False
            
            # 2. 切换到启动状态
            print(f"{self.name}: 发送启动命令...")
            self.node.sdo["Controlword"].raw = 0x07  # 启动
            time.sleep(0.2)
            
            # 3. 使能操作
            print(f"{self.name}: 发送使能命令...")
            self.node.sdo["Controlword"].raw = 0x0F  # 使能操作
            time.sleep(0.3)
            
            # 检查是否使能成功
            status = self.get_status()
            if (status & 0x2000):  # 操作使能位
                self.enabled = True
                self.running = True
                print(f"{self.name}: 使能成功，状态字: 0x{status:04X}")
                return True
            else:
                print(f"{self.name}: 使能失败，状态字: 0x{status:04X}")
                return False
                
        except Exception as e:
            print(f"{self.name}: 使能过程中发生错误: {e}")
            return False

    def disable(self):
        """
        失能电机
        """
        try:
            print(f"{self.name}: 正在失能...")
            
            # 首先停止运动
            self.node.sdo["Controlword"].raw = 0x00  # 快速停止
            time.sleep(0.1)
            
            # 发送失能命令
            self.node.sdo["Controlword"].raw = 0x06  # 准备运行
            time.sleep(0.1)
            
            self.enabled = False
            self.running = False
            self.paused = False
            print(f"{self.name}: 已失能")
            return True
            
        except Exception as e:
            print(f"{self.name}: 失能失败: {e}")
            return False

    def get_status(self):
        """
        获取电机状态字
        """
        try:
            status_word = self.node.sdo["Statusword"].raw
            return status_word
        except Exception as e:
            print(f"{self.name}: 读取状态字失败: {e}")
            return 0

    def get_status_str(self):
        """
        获取电机状态字符串
        """
        status = self.get_status()
        enabled_str = "已使能" if self.enabled else "未使能"
        paused_str = "已暂停" if self.paused else "运行中"
        return f"{self.name}: {enabled_str}, {paused_str}, 状态: 0x{status:04X}"

    def send_position_order(self, target_pos):
        """
        发送位置命令
        :param target_pos: 目标位置
        """
        if not self.enabled:
            print(f"{self.name}: 未使能，不发送位置命令")
            return False
            
        try:
            self.target_position = target_pos
            
            # 尝试使用RPDO发送
            if self.rpdo_configured:
                try:
                    self.node.rpdo[1][0x607A, 0x00].raw = target_pos
                    self.node.rpdo[1].transmit()
                except:
                    # 如果RPDO失败，使用SDO
                    self.node.sdo["Target Position"].raw = target_pos
            else:
                # 直接使用SDO
                self.node.sdo["Target Position"].raw = target_pos
            
            # 更新当前位置
            self.cur_position = self.node.sdo["Pos Actual User Value"].raw
            print(f"{self.name}: 目标位置 -> {target_pos}, 当前位置: {self.cur_position}")
            return True
            
        except Exception as e:
            print(f"{self.name}: 发送位置命令失败: {e}")
            return False

    def get_current_position(self):
        """
        获取当前位置
        """
        try:
            self.cur_position = self.node.sdo["Pos Actual User Value"].raw
            return self.cur_position
        except:
            return self.cur_position

    def pause(self):
        """
        暂停电机运动
        """
        if self.enabled and not self.paused:
            try:
                # 暂停命令
                self.node.sdo["Controlword"].raw = 0x0F & ~0x0010  # 清除第4位
                self.paused = True
                self.running = False
                print(f"{self.name}: 已暂停")
                return True
            except Exception as e:
                print(f"{self.name}: 暂停失败: {e}")
                return False
        return False

    def resume(self):
        """
        恢复电机运动
        """
        if self.enabled and self.paused:
            try:
                # 恢复命令
                self.node.sdo["Controlword"].raw = 0x0F  # 使能操作
                self.paused = False
                self.running = True
                print(f"{self.name}: 已恢复")
                return True
            except Exception as e:
                print(f"{self.name}: 恢复失败: {e}")
                return False
        return False

    def toggle_pause(self):
        """
        切换暂停状态
        """
        if not self.enabled:
            print(f"{self.name}: 未使能，无法暂停")
            return False
            
        if self.paused:
            return self.resume()
        else:
            return self.pause()

    def is_target_reached(self):
        """
        检查是否到达目标位置
        """
        try:
            status = self.get_status()
            return (status & 0x1000) != 0  # 检查目标到达位
        except:
            return False

class MotorGroup(object):
    def __init__(self):
        """
        初始化电机组
        """
        print("正在初始化CAN网络...")
        
        # 创建网络但不立即连接
        self.network = canopen.Network()
        
        # 初始化电机节点列表
        self.motors = []
        
        print(f"正在初始化{CAN_NODE_CNT}个电机节点...")
        for i in range(CAN_NODE_CNT):
            node_id = i + 1
            print(f"初始化电机{node_id}...")
            try:
                motor = StepMotor(network=self.network, node_id=node_id, eds_file=CAN_EDS_FILE)
                self.motors.append(motor)
                time.sleep(0.1)
            except Exception as e:
                print(f"初始化电机{node_id}失败: {e}")
        
        print(f"成功初始化{len(self.motors)}个电机")

        # 命令队列
        self.command_queue = Queue()
        self.exit_flag = False
        self.current_control_motor = 1  # 当前控制的电机索引
        self.control_history = []       # 控制历史记录
        self.all_enabled = False        # 所有电机使能状态

    def __del__(self):
        """
        析构函数
        """
        self.exit_flag = True
        print("正在关闭电机组...")

    def count(self):
        """
        获取电机数量
        """
        return len(self.motors)
    
    def get_motor_by_index(self, motor_index):
        """
        获取指定索引的电机对象
        :param motor_index: 电机索引（1到count()）
        :return: StepMotor对象或None
        """
        if 1 <= motor_index <= len(self.motors):
            return self.motors[motor_index - 1]
        return None
    
    def switch_control_motor(self, new_motor_index):
        """
        切换当前控制的电机
        :param new_motor_index: 新的电机索引
        """
        if 1 <= new_motor_index <= len(self.motors):
            old_motor = self.get_motor_by_index(self.current_control_motor)
            new_motor = self.get_motor_by_index(new_motor_index)
            
            if new_motor:
                self.current_control_motor = new_motor_index
                self.control_history.append(new_motor_index)
                if len(self.control_history) > 5:
                    self.control_history.pop(0)
                
                print(f"\n切换控制: {new_motor.name}")
                print(f"当前位置: {new_motor.get_current_position()}")
                print(f"目标位置: {new_motor.target_position}")
                print(f"状态: {new_motor.get_status_str()}")
                return True
        print(f"切换失败: 电机{new_motor_index}不存在")
        return False
    
    def get_current_motor(self):
        """
        获取当前控制的电机
        """
        return self.get_motor_by_index(self.current_control_motor)
    
    def print_all_motors_status(self):
        """
        打印所有电机状态
        """
        print("\n" + "="*60)
        print("所有电机状态:")
        print("="*60)
        for motor in self.motors:
            print(motor.get_status_str())
            print(f"  当前位置: {motor.get_current_position()}, 目标位置: {motor.target_position}")
        print("="*60)
    
    def start_position_ctrl_all(self):
        """
        启动所有电机的位置控制模式
        """
        print("\n启动所有电机的位置控制模式...")
        success_count = 0
        for motor in self.motors:
            print(f"\n启动{motor.name}的位置控制模式...")
            if motor.start_position_ctrl():
                success_count += 1
            time.sleep(0.2)  # 给每个电机留出时间
        
        print(f"成功启动{success_count}/{len(self.motors)}个电机的位置控制模式")
        return success_count == len(self.motors)
    
    def enable_all_motors(self):
        """
        使能所有电机 - 逐个使能
        """
        print("\n使能所有电机...")
        success_count = 0
        
        # 逐个使能电机，避免同时使能导致的通信问题
        for i, motor in enumerate(self.motors):
            print(f"\n使能{motor.name}...")
            if motor.enable():
                success_count += 1
            time.sleep(0.3)  # 给每个电机使能留出时间
        
        self.all_enabled = (success_count == len(self.motors))
        print(f"成功使能{success_count}/{len(self.motors)}个电机")
        return self.all_enabled
    
    def disable_all_motors(self):
        """
        失能所有电机
        """
        print("\n失能所有电机...")
        success_count = 0
        for motor in self.motors:
            if motor.disable():
                success_count += 1
            time.sleep(0.1)
        
        self.all_enabled = False
        print(f"成功失能{success_count}/{len(self.motors)}个电机")
        return success_count == len(self.motors)
    
    def toggle_all_motors(self):
        """
        切换所有电机使能状态
        """
        if self.all_enabled:
            return self.disable_all_motors()
        else:
            return self.enable_all_motors()
    
    def send_position_order_to_current(self, target_pos):
        """
        向当前控制的电机发送位置命令
        :param target_pos: 目标位置
        """
        motor = self.get_current_motor()
        if motor:
            return motor.send_position_order(target_pos)
        return False
    
    def set_all_motors_fixed_params(self):
        """
        设置所有电机的固定参数 - 简化版
        """
        print("\n设置所有电机的固定参数...")
        success_count = 0
        for motor in self.motors:
            print(f"\n设置{motor.name}的参数...")
            try:
                # 只设置必要的参数
                motor.node.sdo["Basic control parameters"]["Contrl Mode Select"].raw = 0
                time.sleep(0.1)
                motor.node.sdo["Mode Operation"].raw = 8
                time.sleep(0.1)
                success_count += 1
                print(f"{motor.name}: 基本参数设置完成")
            except Exception as e:
                print(f"{motor.name}: 设置参数失败: {e}")
        
        print(f"成功设置{success_count}/{len(self.motors)}个电机的参数")
        return success_count > 0
    
    def clear_all_errors(self):
        """
        清除所有电机错误
        """
        print("\n清除所有电机错误...")
        success_count = 0
        for motor in self.motors:
            print(f"\n清除{motor.name}错误...")
            if motor.clear_errors():
                success_count += 1
            time.sleep(0.2)
        
        print(f"成功清除{success_count}/{len(self.motors)}个电机的错误")
        return success_count == len(self.motors)
    
    def start_interactive_control(self):
        """
        启动多电机交互式控制
        """
        print("\n" + "="*60)
        print("多电机交互式控制程序")
        print("="*60)
        print("当前控制: 电机1 (按F1-F4切换)")
        print("\n通用控制命令:")
        print("  O 键 - 启动/停止所有电机")
        print("  E 键 - 清除所有电机错误")
        print("  S 键 - 显示所有电机状态")
        print("  A 键 - 设置所有电机参数")
        print("  Q 键 - 退出程序")
        print("\n当前电机控制命令:")
        print("  P 键 - 暂停/恢复当前电机")
        print("  ↑ 键 - 增加目标位置 (+100)")
        print("  ↓ 键 - 减少目标位置 (-100)")
        print("  ← 键 - 小步减少 (-10)")
        print("  → 键 - 小步增加 (+10)")
        print("  1 键 - 设置目标位置为 -2000")
        print("  2 键 - 设置目标位置为 0")
        print("  3 键 - 设置目标位置为 2000")
        print("  R 键 - 重新读取当前位置")
        print("\n电机选择命令:")
        for i in range(1, len(self.motors) + 1):
            print(f"  F{i} 键 - 切换到电机{i}")
        print("  TAB 键 - 切换到上一个电机")
        print("="*60)
        
        # 先清除所有错误
        self.clear_all_errors()
        
        # 启动键盘监听线程
        keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
        keyboard_thread.start()
        
        # 显示初始状态
        current_motor = self.get_current_motor()
        if current_motor:
            print(f"\n当前控制: {current_motor.name}")
            print(f"当前位置: {current_motor.get_current_position()}")
            print(f"目标位置: {current_motor.target_position}")
            print(f"状态: {current_motor.get_status_str()}")
        
        # 主控制循环
        try:
            last_status_time = time.time()
            
            while not self.exit_flag:
                current_time = time.time()
                
                # 处理命令队列中的命令
                while not self.command_queue.empty():
                    command = self.command_queue.get()
                    self._process_command(command)
                
                # 定期显示状态
                if current_time - last_status_time > 5.0:  # 每5秒更新一次状态
                    self._show_control_status()
                    last_status_time = current_time
                
                time.sleep(0.05)  # 降低CPU使用率
                
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        finally:
            self.exit_flag = True
            print("\n正在退出程序...")
            self.disable_all_motors()
            print("程序退出")
    
    def _keyboard_listener(self):
        """
        键盘监听函数 - 简化版
        """
        print("键盘监听已启动...")
        
        while not self.exit_flag:
            try:
                # 检查功能键（F1-F4）
                for i in range(1, 5):
                    if keyboard.is_pressed(f'f{i}'):
                        if i <= len(self.motors):
                            self.command_queue.put(('switch_motor', i))
                        time.sleep(0.3)
                
                # 检查其他按键
                if keyboard.is_pressed('o'):
                    self.command_queue.put(('toggle_all',))
                    time.sleep(0.3)
                elif keyboard.is_pressed('p'):
                    self.command_queue.put(('toggle_pause_current',))
                    time.sleep(0.3)
                elif keyboard.is_pressed('s'):
                    self.command_queue.put(('show_all_status',))
                    time.sleep(0.3)
                elif keyboard.is_pressed('a'):
                    self.command_queue.put(('set_all_params',))
                    time.sleep(0.3)
                elif keyboard.is_pressed('e'):
                    self.command_queue.put(('clear_all_errors',))
                    time.sleep(0.3)
                elif keyboard.is_pressed('q'):
                    self.command_queue.put(('exit',))
                    time.sleep(0.3)
                elif keyboard.is_pressed('r'):
                    self.command_queue.put(('read_position',))
                    time.sleep(0.3)
                elif keyboard.is_pressed('up'):
                    self.command_queue.put(('position', 'increase', 100))
                    time.sleep(0.15)
                elif keyboard.is_pressed('down'):
                    self.command_queue.put(('position', 'decrease', 100))
                    time.sleep(0.15)
                elif keyboard.is_pressed('left'):
                    self.command_queue.put(('position', 'decrease', 10))
                    time.sleep(0.15)
                elif keyboard.is_pressed('right'):
                    self.command_queue.put(('position', 'increase', 10))
                    time.sleep(0.15)
                elif keyboard.is_pressed('1'):
                    self.command_queue.put(('position', 'set', -2000))
                    time.sleep(0.3)
                elif keyboard.is_pressed('2'):
                    self.command_queue.put(('position', 'set', 0))
                    time.sleep(0.3)
                elif keyboard.is_pressed('3'):
                    self.command_queue.put(('position', 'set', 2000))
                    time.sleep(0.3)
                elif keyboard.is_pressed('tab'):
                    self.command_queue.put(('switch_prev',))
                    time.sleep(0.3)
                
                time.sleep(0.01)
                
            except Exception as e:
                print(f"键盘监听错误: {e}")
                break
    
    def _process_command(self, command):
        """
        处理命令
        """
        cmd_type = command[0]
        
        if cmd_type == 'toggle_all':
            print("\n[命令] 切换所有电机使能状态")
            self.toggle_all_motors()
            
        elif cmd_type == 'toggle_pause_current':
            current_motor = self.get_current_motor()
            if current_motor:
                print(f"\n[命令] 切换{current_motor.name}暂停状态")
                current_motor.toggle_pause()
            
        elif cmd_type == 'show_all_status':
            print("\n[命令] 显示所有电机状态")
            self.print_all_motors_status()
            
        elif cmd_type == 'set_all_params':
            print("\n[命令] 设置所有电机参数")
            self.set_all_motors_fixed_params()
            
        elif cmd_type == 'clear_all_errors':
            print("\n[命令] 清除所有电机错误")
            self.clear_all_errors()
            
        elif cmd_type == 'position':
            current_motor = self.get_current_motor()
            if not current_motor:
                return
                
            action = command[1]
            
            if action == 'increase':
                amount = command[2]
                new_position = current_motor.target_position + amount
                print(f"\n[命令] 增加{current_motor.name}目标位置: {current_motor.target_position} -> {new_position}")
                current_motor.send_position_order(new_position)
                
            elif action == 'decrease':
                amount = command[2]
                new_position = current_motor.target_position - amount
                print(f"\n[命令] 减少{current_motor.name}目标位置: {current_motor.target_position} -> {new_position}")
                current_motor.send_position_order(new_position)
                
            elif action == 'set':
                position = command[2]
                print(f"\n[命令] 设置{current_motor.name}目标位置为: {position}")
                current_motor.send_position_order(position)
            
        elif cmd_type == 'read_position':
            current_motor = self.get_current_motor()
            if current_motor:
                position = current_motor.get_current_position()
                print(f"\n[命令] {current_motor.name}当前位置: {position}")
            
        elif cmd_type == 'switch_motor':
            motor_index = command[1]
            print(f"\n[命令] 切换到电机{motor_index}")
            self.switch_control_motor(motor_index)
            
        elif cmd_type == 'switch_prev':
            if len(self.control_history) > 1:
                prev_motor = self.control_history[-2] if len(self.control_history) >= 2 else 1
                print(f"\n[命令] 切换到上一个电机")
                self.switch_control_motor(prev_motor)
            
        elif cmd_type == 'exit':
            print("\n[命令] 退出程序")
            self.exit_flag = True
    
    def _show_control_status(self):
        """
        显示控制状态
        """
        current_motor = self.get_current_motor()
        if not current_motor:
            return
            
        print("\n" + "-"*60)
        print(f"当前控制: {current_motor.name}")
        print(f"当前位置: {current_motor.get_current_position()}")
        print(f"目标位置: {current_motor.target_position}")
        print(f"状态: {current_motor.get_status_str()}")
        
        # 显示其他电机的简要状态
        other_motors = [m for m in self.motors if m.node_id != current_motor.node_id]
        if other_motors:
            print("\n其他电机状态:")
            for motor in other_motors:
                status = motor.get_status()
                enabled_str = "✓" if motor.enabled else "✗"
                paused_str = "⏸" if motor.paused else "▶"
                print(f"  {motor.name}: {enabled_str}{paused_str} Pos:{motor.get_current_position():8d}")
        print("-"*60)


if __name__ == "__main__":
    print("多电机交互控制系统 v3.0")
    print("="*60)
    
    try:
        # 创建电机组
        motors = MotorGroup()
        
        # 初始化所有电机
        print("\n1. 启动所有电机的位置控制模式...")
        if not motors.start_position_ctrl_all():
            print("注意: 部分电机位置控制模式启动失败，尝试继续...")
        
        # 等待初始化完成
        time.sleep(2)
        
        # 启动交互式控制
        motors.start_interactive_control()
        
    except Exception as e:
        print(f"程序运行出错: {e}")
        import traceback
        traceback.print_exc()