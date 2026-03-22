import canopen
import time
import math
import matplotlib.pyplot as plt


class PMM60Motor(object):
    def __init__(self, network, node_id, eds_file):
        self.network = network
        self.node = canopen.RemoteNode(node_id, eds_file)
        self.network.add_node(self.node)
        self.pulse_cycle = 10000  # 电机每转动一周的编码器脉冲数量
        self.init_position = 0    # 电机启动运行时的初始位置
        self.cur_position  = 0    # 电机当前给定位置
        

    def download_fixed_params(self):
        """
        设置伺服电机的固定参数。这些参数通常需要电机重新启动后生效，且参数下载后即存储在电机驱动器中，因此该函数仅需要在相关参数修改后调用一次
        :return:
        """
        # 周期同步位置控制参数
        self.node.sdo["Basic control parameters"]["CtrlModeSelec"].raw         = 0      # 设置CiA402模式

        self.node.sdo["Position range limit"]["Min position range limit"].raw  = -2147483628
        self.node.sdo["Position range limit"]["Max position range limit"].raw  = 2147483628
        self.node.sdo["Software position limit"]["Minimal position limit"].raw = -2147483628
        self.node.sdo["Software position limit"]["Maximal position limit"].raw = 2147483628
        self.node.sdo["Polarity"].raw             = 0
        self.node.sdo["Profile acceleration"].raw = 409600
        self.node.sdo["Profile deceleration"].raw = 409600
        self.node.sdo["Max motor speed"].raw      = 5000
        self.node.sdo["Maximal profile velocity"].raw = 500000

        # 原点回归参数
        self.node.sdo["Homing method"].raw = 18  # 设置原点回归方式为18
        self.node.sdo["Input terminal parameters"]["DI1FunSelec"].raw = 14                # 实体输入端子设置为原点开关
        self.node.sdo["Input terminal parameters"]["DI1LogicSelec"].raw = 1               # 实体输入端子下降沿有效(NPN型)
        self.node.sdo["Position control parameters"]["HomingDurationLimit"].raw = 65535   # 设置原点回归超时(单位：(0-65535)ms)
        self.node.sdo["Homing speeds"]["Speed for zero search"].raw = 5000                 # 寻找原点信号的速度(用户单位/s)

    def print_fixed_params(self):
        """
        打印显示伺服电机的固定参数，以便于调试使用
        :return:
        """
        print("CtrlModeSelec        = %d" % self.node.sdo["Basic control parameters"]["CtrlModeSelec"].raw)
        print("Modes of operation   = %d" % self.node.sdo["Modes of operation"].raw)
        print("MinPosRangLimt       = %d" % self.node.sdo["Position range limit"]["Min position range limit"].raw)
        print("MaxPosRangLimt       = %d" % self.node.sdo["Position range limit"]["Max position range limit"].raw)
        print("MinPosLimt           = %d" % self.node.sdo["Software position limit"]["Minimal position limit"].raw)
        print("MaxPosLimt           = %d" % self.node.sdo["Software position limit"]["Maximal position limit"].raw)
        print("Polarity             = %d" % self.node.sdo["Polarity"].raw)
        print("Profile acceleration = %d" % self.node.sdo["Profile acceleration"].raw)
        print("Profile deceleration = %d" % self.node.sdo["Profile deceleration"].raw)
        print("Profile deceleration = %d" % ["Homing method"].raw)
    def start_position_ctrl(self):
        # 设置同步位置运行模式
        self.node.sdo["Modes of operation"].raw = 8
        # 读取电机当前编码器值
        self.init_position = self.cur_position = self.node.sdo["Position actual value"].raw

        # 配置RPDO
        self.node.rpdo.read()
        self.node.rpdo[1].clear()
        self.node.rpdo[1].add_variable("Target position")
        self.node.rpdo[1].enabled = True

        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.rpdo.save()
        self.node.nmt.state = 'OPERATIONAL'

        # 使能电机
        self.enable()

    def start_home_ctrl(self):
        self.download_fixed_params()
        # 设置原点回归模式
        self.node.sdo["Modes of operation"].raw = 6
        self.enable()
        # 设置控制字第4位为1，启动归零
        ctrl_word = self.node.sdo["Controlword"].raw
        ctrl_word |= 0x10
        self.node.sdo["Controlword"].raw = ctrl_word



    def init_tpdo(self):
        pass

    def enable(self):
        # 这里换成索引名
        self.node.sdo["Controlword"].raw = 6   # 电机准备
        self.node.sdo["Controlword"].raw = 7   # 电机失能
        self.node.sdo["Controlword"].raw = 15  # 电机使能

    def disable(self):
        self.node.sdo["Controlword"].raw = 7

    def send_position_order(self, target_pos):
        self.node.rpdo[1]['Target position'].raw = target_pos
        self.node.rpdo[1].transmit()

    def wait_for_home(self):
        status_word = self.node.sdo["Statusword"].raw
        return status_word & 0x1000
    
    def init_tpdo(self):
        """配置TPDO，用于接收实际位置"""
        self.node.tpdo.read()
        self.node.tpdo[1].clear()
        self.node.tpdo[1].add_variable("Position actual value")
        self.node.tpdo[1].enabled = True
        self.node.tpdo[1].event_timer = 5  # 每5ms触发一次（可选）
        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.tpdo.save()
        self.node.nmt.state = 'OPERATIONAL'

    def get_actual_position(self):
        """获取当前实际位置（从TPDO读取）"""
        # 如果有最新接收的TPDO数据则直接返回，否则等待
        return self.node.tpdo[1]['Position actual value'].raw


class MotorGroup(object):
    def __init__(self, bustype, channel, bitrate, motor_cnt, ctrl_interval=0.01):
        # 连接CAN总线网络
        self.network = canopen.Network()  # 创建总线网络
        self.network.connect(bustype=bustype, channel=channel, bitrate=bitrate)   # 启动通信

        # 初始化电机节点
        self.motors = list()
        for i in range(motor_cnt):
            motor = PMM60Motor(network=self.network, node_id=i + 1, eds_file='NiMotion_PMM80B_V1.14.eds')
            self.motors.append(motor)

        self.ctrl_interval = ctrl_interval
        self.ctrl_callback = None
        self.ctrl_start_time = None
        
        # 新增：数据记录列表
        self.time_list = []
        self.target_pos_list = []
        self.actual_pos_list = []

    def start_position_ctrl(self):
        for _, motor in enumerate(self.motors):
            motor.start_position_ctrl()
            motor.init_tpdo()  # 配置TPDO

    def send_position_order(self, target_pos):
        # 发送目标位置（多电机）
        for i, motor in enumerate(self.motors):
            motor.send_position_order(target_pos[i])
            # print(target_pos)

    def start_home_ctrl(self):
        for _, motor in enumerate(self.motors):
            motor.start_home_ctrl()

    def wait_for_home(self):
        for _, motor in enumerate(self.motors):
            if not motor.wait_for_home():
                return False
        return True

    def run(self):
        while True:
            # 调用回调函数，在回调函数中计算产生下一周期目标位置
            # 如果回调函数返回None，则退出控制循环
            target_pos = self.ctrl_callback(self.motors)
            if target_pos is None:
                break

            # 输出新的目标位置
            self.send_position_order(target_pos)
            time.sleep(self.ctrl_interval)
            
    def run_with_velocity_plan(self, max_rpm, acc_rpm_per_sec, target_rpm):
        """
        梯形速度规划控制
        :param max_rpm:       最大速度(rpm)
        :param acc_rpm_per_sec: 加速度(rpm/s)
        :param target_rpm:    目标速度(rpm)，正负表示方向
        """
        current_rpm = 0.0
        acc_step = acc_rpm_per_sec * self.ctrl_interval  # 每周期速度变化量

        # 初始化位置（使用实际位置）
        init_pos = self.motors[0].get_actual_position()
        pos = init_pos  # 当前累积位置（用户单位）

        t = 0
        while True:
            # 速度规划：根据当前速度与目标速度的差值，以加速度步长调整
            if current_rpm < target_rpm:
                current_rpm = min(current_rpm + acc_step, target_rpm)
            elif current_rpm > target_rpm:
                current_rpm = max(current_rpm - acc_step, target_rpm)

            # 限制最大速度
            current_rpm = max(-max_rpm, min(current_rpm, max_rpm))

            # 计算位置增量（使用浮点数）
            delta_pos = current_rpm * self.motors[0].pulse_cycle * self.ctrl_interval / 60.0
            pos += delta_pos
            target_pos = int(round(pos))  # 取整

            # 发送目标位置（支持多电机，此处简化为单电机）
            self.send_position_order([target_pos])

            # 记录数据
            actual_pos = self.motors[0].get_actual_position()
            self.time_list.append(t)
            self.target_pos_list.append(target_pos)
            self.actual_pos_list.append(actual_pos)

            # 可选：打印调试信息
            # print(f"t={t:.2f}, rpm={current_rpm:.1f}, target={target_pos}, actual={actual_pos}")

            t += self.ctrl_interval
            time.sleep(self.ctrl_interval)

            # 退出条件：例如达到目标圈数或按需停止
            # 这里示例为运行5秒后停止
            if t > 5.0:
                break

        # 绘制曲线
        self.plot_data()
        
    def plot_data(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_list, self.target_pos_list, label='Target Position')
        plt.plot(self.time_list, self.actual_pos_list, label='Actual Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (pulses)')
        plt.legend()
        plt.title('Position Tracking Performance')
        plt.grid(True)
        plt.show()

def position_ctrl_test():
    group = MotorGroup(bustype='canalystii', channel=0, bitrate=1000000, motor_cnt=1)
    group.motors[0].download_fixed_params()
    
    group.start_position_ctrl()

    init_pos = group.motors[0].node.sdo['Target position'].raw
    print("Init position = %d" % init_pos)

    pos = list()
    for i in range(len(group.motors)):
        pos.append(0)

    ctrl_interval = 0.0001

    rpm = 100
    acc = True
    for t in range(100000):
        if rpm > 40000:
            acc = False
        elif rpm < -40000:
            acc = True
        rpm += 80 if acc else -80

        target_pos = list()
        for i, motor in enumerate(group.motors):
            pos[i] += rpm * motor.pulse_cycle * ctrl_interval // 60
            target_pos.append(pos[i] + motor.init_position)
        try:
            group.send_position_order(target_pos)
        except Exception as argument:
            print(argument)

        time.sleep(ctrl_interval)

    time.sleep(2)
    end_pos = group.motors[0].node.sdo["Position actual value"].raw
    real = end_pos - init_pos
    print("pos = %d, real = %d, init = %d, end = %d" % (pos[0], real, init_pos, end_pos))
    print("target position = %d" % group.motors[0].node.sdo["Position actual value"].raw)

def home_ctrl_test():
    group = MotorGroup(bustype='canalystii', channel=0, bitrate=1000000, motor_cnt=1)
    group.motors[0].download_fixed_params()
    group.start_home_ctrl()

    for t in range(100):
        time.sleep(0.2)
        if group.wait_for_home():
            print("All the motors are homed")
            break
        else:
            print("Motors are homing")
            
def position_ctrl_with_smooth_velocity():
    group = MotorGroup(bustype='canalystii', channel=0, bitrate=1000000, motor_cnt=1,ctrl_interval=0.01)
    group.motors[0].download_fixed_params()
    group.start_position_ctrl()

    # 设定运动参数
    max_rpm = 500          # 最大速度 500 rpm
    acc = 2000             # 加速度 2000 rpm/s
    target_rpm = 400       # 目标速度 400 rpm，运行 5 秒后停止

    # 运行速度规划控制
    group.run_with_velocity_plan(max_rpm, acc, target_rpm)


if __name__ == "__main__":
    # position_ctrl_test()
    # home_ctrl_test()
    position_ctrl_with_smooth_velocity()
