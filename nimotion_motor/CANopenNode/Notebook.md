# 如何将CANopenNode协议集成到硬件设备

## 适配CANopenNode

### main_blank.c主要功能

1.​初始化阶段​​：分配内存、配置存储、初始化 CAN 控制器和协议栈模块。
​2.​通信复位​​：根据 LSS 配置动态设置节点 ID 和波特率，初始化 PDO 和 SDO。
​3.​主运行循环​​：周期性处理协议栈任务，更新设备状态（如 LED），支持参数持久化。
4.​​中断处理​​：定时器中断处理 SYNC 和 PDO，CAN 中断处理报文收发（需平台适配）。

### 待修改的文件

#### CO_driver.h文件(声明新的驱动接口)

#### CO_driver_target.h(CO_driver_target)

#### CO_driver_controlcan(ControlCAN具体实现)

1.O_ReturnError_t CO_CANinit(CO_t *co, void *CANptr, uint16_t bitRate)

2.int16_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *txMsg)

3.void* CO_ControlCAN_ReceiveThread(void *arg) 
