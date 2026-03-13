# 步进电机控制系统

## 项目概述

基于CANopen协议的6-DOF步进电机控制系统，支持NiMotion STM42A系列步进电机。

## 功能特性

- ✅ 多电机同步控制
- ✅ CANopen协议通信
- ✅ 周期同步位置控制（CSP模式）
- ✅ 交互式键盘控制
- ✅ 故障检测与恢复
- ✅ 日志记录系统
- 🔄 上位机控制模式（开发中）

## 系统要求

### 硬件要求

- CANalyst-II CAN总线分析仪
- NiMotion STM42A系列步进电机驱动器
- 24V直流电源
- CAN总线终端电阻（120Ω）

### 软件要求

- Python 3.7+
- 依赖包：canopen, keyboard, pyserial
- 驱动： ControlCAN.dll

## 安装配置

### 1. 拉取仓库

```shell
git clone -b feature/Nimotion git@github.com:BotAdv/StewartBot.git
```

### 2. 安装依赖

```bash
pip install -r requirements.txt
```

## TODO  

### 1.最小ROS节点

- 实现CAN报文收发  
