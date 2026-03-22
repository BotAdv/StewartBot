import canopen
import time


CAN_BIT_RATE = 1000000
CAN_BUS_TYPE = 'canalystii'
CAN_CHANNEL  = 0
CAN_EDS_FILE = 'NiMotion_PMM80B_V1.14.eds'
CAN_NODE_CNT = 1


class ServoMotor(object):
    def __init__(self, network, node_id, eds_file):
        """
        定义一台位置控制伺服电机，电机采用CANOpen总线，基于CiA402模式
        :param network: canopen总线网络对象
        :param node_id: 伺服电机的节点编号
        :param eds_file: 伺服电机的eds文件
        """
        self.network = network
        self.node = canopen.RemoteNode(node_id, eds_file)
        self.network.add_node(self.node)
        self.download_fixed_params()

        self.pulse_cycle  = 10000            # 电机每转动一周的编码器脉冲数量
        self.cur_position = 0                # 电机当前编码器位置

    def download_fixed_params(self):
        """
        设置伺服电机的固定参数。这些参数通常需要电机重新启动后生效，且参数下载后即存储在电机驱动器中，因此该函数仅需要在相关参数修改后调用一次
        :return:
        """
        # 周期同步位置控制参数
        self.node.sdo["Basic control parameters"]["CtrlModeSelec"].raw         = 0      # 设置CiA402模式
        self.node.sdo["Modes of operation"].raw = 8
        self.node.sdo["Position range limit"]["Min position range limit"].raw  = -1048576
        self.node.sdo["Position range limit"]["Max position range limit"].raw  = 1048576
        self.node.sdo["Software position limit"]["Minimal position limit"].raw = -65535
        self.node.sdo["Software position limit"]["Maximal position limit"].raw = 65535
        self.node.sdo["Polarity"].raw             = 0
        self.node.sdo["Profile acceleration"].raw = 409600
        self.node.sdo["Profile deceleration"].raw = 409600
        self.node.sdo["Max motor speed"].raw      = 3000
        self.node.sdo["Maximal profile velocity"].raw = 500000

        # 原点回归参数
        self.node.sdo["Homing method"].raw = 18  # 设置原点回归方式为18
        self.node.sdo["Input terminal parameters"]["DI1FunSelec"].raw = 14                # 实体输入端子设置为原点开关
        self.node.sdo["Input terminal parameters"]["DI1LogicSelec"].raw = 1               # 实体输入端子下降沿有效(NPN型)
        self.node.sdo["Position control parameters"]["HomingDurationLimit"].raw = 30000   # 设置原点回归超时(单位：(0-65535)ms)
        self.node.sdo["Homing speeds"]["Speed for zero search"].raw = 5000                 # 寻找原点信号的速度(用户单位/s)

        # 配置RPDO映射
        self.node.rpdo.read()
        self.node.rpdo[1].clear()
        self.node.rpdo[1].add_variable("Target position")
        self.node.rpdo[1].enabled = True

        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.rpdo.save()
        self.node.nmt.state = 'OPERATIONAL'

        # 配置TPDO映射
        pass

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
        print("Statusword           =  %d" % self.node.sdo["Statusword"].raw)
        print("Pre-defined Error Field = %d" % self.node.sdo["Pre-defined Error Field"]["Standard Error Field"].raw)
        print("Homing method       = %d" % self.node.sdo["Homing method"].raw)
        print("Position control parameters       = %d" % self.node.sdo["Position control parameters"]["StepAmount"].raw)
        print("Encoder resolution       = %d" % self.node.sdo["Position encoder resolution"]["Encoder increments"].raw)
        print("RatedVoltage       = %d" % self.node.sdo["Servo motor parameters"]["RatedVoltage"].raw)

    def enable(self):
        # 这里换成索引名
        self.node.sdo["Controlword"].raw = 6   # 电机准备
        self.node.sdo["Controlword"].raw = 7   # 电机失能
        self.node.sdo["Controlword"].raw = 15  # 电机使能

    def disable(self):
        self.node.sdo["Controlword"].raw = 7

    def start_position_ctrl(self):
        # 设置同步位置运行模式
        self.node.sdo["Modes of operation"].raw = 8

        # 读取电机当前编码器值
        self.cur_position = self.node.sdo["Position actual value"].raw

        # RPDO已经离线配置好了，仅需要读取配置
        self.node.nmt.state = 'PRE-OPERATIONAL'
        self.node.rpdo.read()
        self.node.rpdo[1].enabled = True
        self.node.nmt.state = 'OPERATIONAL'

        # 重新使能电机
        self.enable()

    def start_home_ctrl(self):
        # 设置原点回归模式
        self.node.sdo["Modes of operation"].raw = 6

        # 设置控制字第4位为1，启动归零
        ctrl_word = self.node.sdo["Controlword"].raw
        ctrl_word |= 0x10
        self.node.sdo["Controlword"].raw = ctrl_word

        # 重新使能电机
        self.enable()

    def send_position_order(self, target_pos):
        self.node.rpdo[1]['Target position'].raw = target_pos
        self.node.rpdo[1].transmit()

    def homing(self):
        status_word = self.node.sdo["Statusword"].raw
        return status_word & 0x1000


class MotorGroup(object):
    def __init__(self):
        # 连接CAN总线网络
        self.network = canopen.Network()
        self.network.connect(bustype=CAN_BUS_TYPE, channel=CAN_CHANNEL, bitrate=CAN_BIT_RATE)

        # 初始化电机节点
        self.motors = list()
        for i in range(CAN_NODE_CNT):
            motor = ServoMotor(network=self.network, node_id=i + 1, eds_file=CAN_EDS_FILE)
            self.motors.append(motor)

    def __del__(self):
        pass

    def count(self):
        return len(self.motors)

    def start_position_ctrl(self):
        for _, motor in enumerate(self.motors):
            motor.start_position_ctrl()

    def send_position_order(self, target_pos):
        for i, motor in enumerate(self.motors):
            motor.send_position_order(target_pos[i])

    def start_home_ctrl(self):
        for _, motor in enumerate(self.motors):
            motor.start_home_ctrl()

    def homing(self):
        for _, motor in enumerate(self.motors):
            if motor.homing():
                return True
        return False


def position_ctrl_test(motors):
    max_rpm = 1000
    motors.start_position_ctrl()

    target_pos = list()
    for _, motor in enumerate(motors.motors):
        target_pos.append(motor.cur_position)

    rpm = 0
    acc = True
    for t in range(10000):
        if rpm > max_rpm:
            acc = False
        elif rpm < -max_rpm:
            acc = True
        rpm += 5 if acc else -5

        for i, motor in enumerate(motors.motors):
            target_pos[i] += rpm * motor.pulse_cycle // (60 * 50)
        motors.send_position_order(target_pos)
        time.sleep(0.005)

    time.sleep(1)
    for i in range(motors.motor_count()):
        print("Motor %d" % (i + 1))
        print("Target position = %d" % target_pos[i])
        print("Target position in motor = %d" % motors.motors[0].node.sdo["Target position"].raw)
        print("Actual position = %d" % motors.motors[0].node.sdo["Position actual value"].raw)


def home_ctrl_test(motors):
    motors.start_home_ctrl()
    begin_time = time.perf_counter()

    print("Start homing")
    while motors.homing():
        time.sleep(0.05)
        if time.perf_counter() - begin_time > 20.0:
            print("Homing stopped due to timeout")
            return
    print("All the motors are homed")


if __name__ == "__main__":
    motors = MotorGroup()
    home_ctrl_test(motors)
    #position_ctrl_test(motors)


