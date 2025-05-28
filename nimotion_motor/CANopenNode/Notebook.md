# CANopenNode学习笔记

## 1. 学习CANopenNode

### 1.1 代码结构

原目录：https://github.com/CANopenNode/CANopenNode  
<pre>
.
├── 301                         // CANopen 核心协议实现  
│   ├── CO_config.h
│   ├── CO_driver.h
│   ├── CO_Emergency.c
│   ├── CO_Emergency.h
│   ├── CO_fifo.c
│   ├── CO_fifo.h
│   ├── CO_HBconsumer.c
│   ├── CO_HBconsumer.h
│   ├── CO_NMT_Heartbeat.c
│   ├── CO_NMT_Heartbeat.h
│   ├── CO_Node_Guarding.c
│   ├── CO_Node_Guarding.h
│   ├── CO_ODinterface.c
│   ├── CO_ODinterface.h
│   ├── CO_PDO.c
│   ├── CO_PDO.h
│   ├── CO_SDOclient.c
│   ├── CO_SDOclient.h
│   ├── CO_SDOserver.c
│   ├── CO_SDOserver.h
│   ├── CO_SYNC.c
│   ├── CO_SYNC.h
│   ├── CO_TIME.c
│   ├── CO_TIME.h
│   ├── crc16-ccitt.c
│   └── crc16-ccitt.h
├── 303
│   ├── CO_LEDs.c
│   └── CO_LEDs.h
├── 304
│   ├── CO_GFC.c
│   ├── CO_GFC.h
│   ├── CO_SRDO.c
│   └── CO_SRDO.h
├── 305
│   ├── CO_LSS.h
│   ├── CO_LSSmaster.c
│   ├── CO_LSSmaster.h
│   ├── CO_LSSslave.c
│   └── CO_LSSslave.h
├── 309
│   ├── CO_gateway_ascii.c
│   └── CO_gateway_ascii.h
├── build
├── CANopen.c
├── CANopen.h
├── doc
│   ├── CANopenNode.png
│   ├── CHANGELOG.md
│   ├── deviceSupport.md
│   ├── objectDictionary.md
│   └── traceUsage.md
├── Doxyfile
├── example
│   ├── CANopen.h
│   ├── CO_driver_target.h
│   ├── CO_storageBlank.c
│   ├── CO_storageBlank.h
│   ├── DS301_profile.eds
│   ├── DS301_profile.md
│   ├── DS301_profile.xpd
│   ├── main_blank.c
│   ├── Makefile
│   ├── OD.c
│   └── OD.h
├── extra
│   ├── CO_trace.c
│   └── CO_trace.h
├── LICENSE
├── MISRA.md
├── README.md
└── storage
    ├── CO_eeprom.h
    ├── CO_storage.c
    ├── CO_storageEeprom.c
    ├── CO_storageEeprom.h
    └── CO_storage.h
</pre>
本项目：https://github.com/BotAdv/StewartBot/tree/feature/Nimotion  
<pre>
├── include
│   ├── CO_config.h
│   ├── CO_driver.h
│   ├── CO_driver_target.h
│   ├── CO_Emergency.h
│   ├── CO_fifo.h
│   ├── CO_gateway_ascii.h
│   ├── CO_GFC.h
│   ├── CO_HBconsumer.h
│   ├── CO_LEDs.h
│   ├── CO_LSS.h
│   ├── CO_LSSmaster.h
│   ├── CO_LSSslave.h
│   ├── CO_NMT_Heartbeat.h
│   ├── CO_Node_Guarding.h
│   ├── controlcan.h
│   ├── CO_ODinterface.h
│   ├── CO_PDO.h
│   ├── CO_SDOclient.h
│   ├── CO_SDOserver.h
│   ├── CO_SRDO.h
│   ├── CO_storageBlank.h
│   ├── CO_storage.h
│   ├── CO_SYNC.h
│   ├── CO_TIME.h
│   ├── CO_trace.h
│   └── crc16-ccitt.h
├── src
│   ├── CO_Emergency.c
│   ├── CO_fifo.c
│   ├── CO_gateway_ascii.c
│   ├── CO_GFC.c
│   ├── CO_HBconsumer.c
│   ├── CO_LEDs.c
│   ├── CO_LSSmaster.c
│   ├── CO_LSSslave.c
│   ├── CO_NMT_Heartbeat.c
│   ├── CO_Node_Guarding.c
│   ├── CO_ODinterface.c
│   ├── CO_PDO.c
│   ├── CO_SDOclient.c
│   ├── CO_SDOserver.c
│   ├── CO_SRDO.c
│   ├── CO_storageBlank.c
│   ├── CO_storage.c
│   ├── CO_SYNC.c
│   ├── CO_TIME.c
│   ├── CO_trace.c
│   ├── crc16-ccitt.c
│   ├── Makefile                    // 源代码编译文件
│   ├── socket_test.cpp             // 驱动层测试文件
│   └── test_socket                 // P-CAN device socketCAN test
├── temp
│   ├── CO_driver_controlcan.cpp    // 尝试使用VCI_Transmit等厂商API，封装CO_CANsend、CO_ControlCAN_ReceiveThread等函数，以对接集成CANopenNode
│   └── CO_driver_target.h.bak      // 集成CANalyst的接口函数声明
├── libcontrolcan.so                // CANalyst-linux版的链接库
├── main_blank.c                    // 入口函数
├── main.cpp                        // 测试CANalyst链接库的文件（回环测试）
├── main_socketcan.cpp              // 测试P-CAN的socketCAN库的文件（NMT报文与数据监听）
├── Makefile                        // 编译main.cpp 为 controlcan，编译main_socketcan.cpp 为 test（均为测试，后续删除）
├── NiMotion_STM42A_V1.07.eds       // 电子数据表，后续需要通过CANopenEditor 生成 OD.h 和 OD.c 文件，定义设备的行为。
├── CANopen.c                       // 原CANopenNode文件
├── CANopen.h                       // 原CANopenNode文件
├── CO_driver_socketcan.cpp         // 重点！！ 关于集成socketCAN的驱动层程序
├── CO_driver_target.h              // 原CANopenNode文件
├── OD.h                            // 由eds生成的文件
├── Notebook.md                 <---------
├── controlcan
└── test

</pre>

### 1.2 核心功能

---

- __对象字典（Object Dictionary, OD）__

> `所有网络可访问的变量（通信参数、设备配置、应用数据）均存储在对象字典中，支持直接访问(硬编码)或通过读写函数（CANopen 协议支持）操作。`

---

- __CANopen 协议支持__

>     NMT（网络管理）：控制节点状态（启动、停止、复位），支持主从模式。

>     PDO（过程数据对象）：高效传输实时数据，支持动态映射对象字典变量。

>     SDO（服务数据对象）：提供对对象字典的读写访问，支持客户端（主）和服务器（从）模式。

>     Heartbeat/Sync/Time/Emergency：心跳检测、同步、时间戳、紧急事件处理等。

>     LSS（层设置服务）：动态配置节点 ID 和波特率。

>     Safety（安全协议）：支持 EN 50325-5 安全相关通信。

---

- __多线程架构__

>     CAN 接收线程：快速处理 CAN 帧，解析协议并更新对象字典。 

>     定时器线程（通常 1ms）：实时任务，如 PDO 映射、硬件输入输出同步。

>     主线程：处理耗时任务（SDO 服务、心跳、NMT 状态机）。

## 2.适配CANopenNode步骤

### 2.1 入口函数main_blank.c主要功能

1.​初始化阶段​​：分配内存、配置存储、初始化 CAN 控制器和协议栈模块。  
​2.​通信复位​​：根据 LSS 配置动态设置节点 ID 和波特率，初始化 PDO 和 SDO。  
​3.​主运行循环​​：周期性处理协议栈任务，更新设备状态（如 LED），支持参数持久化。  
4.​​中断处理​​：定时器中断处理 SYNC 和 PDO，CAN 中断处理报文收发（需平台适配）。  

### 2.2 待修改的文件

#### CO_driver.h文件(声明新的驱动接口)

#### CO_driver_target.h(CO_driver_target)

#### CO_driver_controlcan(ControlCAN具体实现)

1.O_ReturnError_t CO_CANinit(CO_t *co, void *CANptr, uint16_t bitRate)

2.CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *txMsg)

3.void* CO_ControlCAN_ReceiveThread(void *arg) 
