import canopen
import struct
import math
import logging


class StepMotor(object):
    def __init__(self, network, node_id, eds_file):
        """
        初始化步进电机对象
        :param network: CAN总线网络对象
        :param node_id: 电机节点ID
        :param eds_file: 电机的EDS文件路径
        """
        self.network = network
        self.node = canopen.RemoteNode(node_id, eds_file)
        self.network.add_node(self.node)

        self.mcs = 2             # 电机细分
        self.current_acc = 1.0   # 电机加速电流，默认为1A
        self.current_run = 1.0   # 电机工作电流
        self.current_hold = 1.0  # 电机保持电流
        self.acc = 0             # 电机加速度
        self.dec = 0             # 电机减速度

        self.smooth_K = 0        # 平滑系数
        self.cur_pulse = 0.0     # 当前位置(以脉冲为单位)

    def __to_u32(self, float_value):
        """
        将浮点数转换为32位无符号整数
        :param float_value: 浮点数
        :return: 32位无符号整数的十六进制表示
        """
        return hex(struct.unpack('>I', struct.pack('<f', float_value))[0])

    def initialize(self):
        """
        初始化电机参数
        """
        # 设置电机细分
        sec_table = {1:0, 2:1, 4:2, 6:3, 8:4, 16:5, 32:6, 64:7, 128:8}
        #cur_pluse = 200 * pow(2, self.mcs)
        self.node.sdo.download(0x2022, 0, b'\x02')  # 设置细分为2

        # 电机使能，设置当前位置为原点位置
        self.node.sdo.download(0x2010, 0, )             # 使能电机
        self.node.sdo.download(0x2010, 0, b'\x00\x02')  # 设置当前位置为原点

        # 设置电机加速度、减速度
        self.node.sdo.download(0x2024, 0, b'\x00\x40\x9c\x47') # 设置加速度
        self.node.sdo.download(0x2025, 0, b'\x00\x40\x9c\x47') # 设置减速度

        # 设置电机加速电流，工作电流和保持电流
        self.node.sdo.download(0x2027, 0, b'\x00\x00\x80\x3f') # 设置加速电流
        self.node.sdo.download(0x2027, 0, b'\x00\x00\x80\x3f') # 设置工作电流
        self.node.sdo.download(0x2028, 0, b'\x00\x00\x80\x3f') # 设置保持电流

    def go_zero(self):
        """
        执行归零操作
        """
        self.node.sdo.download(0x2040, 0, b'\x01')              # 归零模式设置
        self.node.sdo.download(0x2033, 0, b'\x00')              # 端口工作模式为输入
        self.node.sdo.download(0x2043, 0, b'\x00')              # 设置传感器1为原点传感器
        self.node.sdo.download(0x2030, 0, b'\x04\x00\x00\x00')  # 传感器1检测到上升沿后立刻停止
        self.node.sdo.download(0x2035, 0, b'\x00')              # 设置原点传感器1内部为下拉
        self.node.sdo.download(0x2042, 0, b'\x00')              # 原点传感器开放电平(低)
        self.node.sdo.download(0x2044, 0, b'\x00\x00\xfa\x45')  # 归零速度(zsd)
        # self.node.sdo.download(0x2045, 0, b'\x00\x00\xc8\x42')# 归零安全位置(zsp)
        self.node.sdo.download(0x2010, 0, b'\x01\x09')          # 启动归零控制

    def position_ctrl(self, ang_order, interval):
        """
        位置控制
        :param pos_order: 目标位置，单位：度
        :param interval: 控制周期(s)
        :return:
        """
        pulse = self.mcs * (ang_order/ 1.8)                                     # 计算目标脉冲数
        pulse = self.smooth_K * self.cur_pulse + (1.0 - self.smooth_K) * pulse  # 平滑处理
        speed = (pulse - self.cur_pulse) / interval                             # 计算目标速度
        self.cur_pulse = pulse                                                  # 更新当前位置

        self.node.rpdo.read()
        self.node.rpdo[2]['Target Speed (spd)'].raw = b'\x00\x40\x1c\x46'   # 设置电机目标速度
        self.node.rpdo[2]['Target Position (moveto)'].raw = pulse           # 设置电机目标位置
        self.node.rpdo[2].transmit()                                        # 发送RPDO


class StepMotorGroup(object):
    def __init__(self, bustype, channel, bitrate, motor_cnt):
        """
        初始化步进电机组
        :param bustype: CAN总线类型
        :param channel: CAN通道
        :param bitrate: CAN比特率
        :param motor_cnt: 电机数量
        """
        # 初始化Motor类
        self.network = canopen.Network()  # 创建总线网络
        self.network.connect(bustype=bustype, channel=channel, bitrate=bitrate)

        self.motors = list()
        for i in range(motor_cnt):
            motor = StepMotor(self.network, 0, "C:\Users\Administrator\Desktop\opencan")
            self.motors.append(motor)

        # 示例：控制第一个电机移动360度，控制周期为0.2秒
        motor.position_ctrl(360.0, 0.2)
    def NMT_manage(self):
        """
        NMT管理，广播各个节点状态为操作模式
        """
        self.network.nmt.send_command(0x1)                  # 广播各个节点状态为操作模式
        print("网络状态为 %s" % self.network.nmt.state)

    def send_sync(self):
        """
        发送同步信号
        """
        self.network.sync.start(0.05)                       # 每0.05秒发送一次同步信号

class GimbalCamera(object):
    pass # 云台相机类，暂未实现

if __name__ == "__main__":
    group = StepMotorGroup(bustype='canalystii', channel=0, bitrate=1000000, motor_cnt=1)
    group[1]  # 这行代码似乎有误，应该是 group.motors[0] 或其他有效操作