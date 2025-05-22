#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "lib/controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"

#include "lib/CO_driver.h"
#include "lib/controlcan.h"  // 厂商提供的CAN驱动头文件
#include "CO_driver_target.h"

// 实现CAN模块初始化
CO_ReturnError_t CO_CANinit(CO_t *co, void *CANptr, uint16_t bitRate) {
    // 打开设备
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK)
        return CO_ERROR_HARDWARE;
    
    // 配置波特率（需将bitRate转换为Timing0/Timing1）
    VCI_INIT_CONFIG config = {
        .Timing0 = 0x03, // 125 Kbps
        .Timing1 = 0x1C,
        .Mode = 0 // 正常模式
    };
    if (VCI_InitCAN(VCI_USBCAN2, 0, 0, &config) != STATUS_OK)
        return CO_ERROR_HARDWARE;
    
    // 启动CAN通道
    VCI_StartCAN(VCI_USBCAN2, 0, 0);
    return CO_ERROR_NO;
}

// 实现CAN发送函数
int16_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *txMsg) {
    VCI_CAN_OBJ frame = {
        .ID = txMsg->ident,
        .ExternFlag = (txMsg->ident & CO_CAN_ID_EXTENDED) ? 1 : 0,
        .RemoteFlag = txMsg->rtr ? 1 : 0,
        .DataLen = txMsg->DLC,
        .Data = {0}
    };
    memcpy(frame.Data, txMsg->data, txMsg->DLC);
    
    ULONG ret = VCI_Transmit(VCI_USBCAN2, 0, CANmodule->CANidx, &frame, 1);
    return (ret == 1) ? 0 : -1;
}


// 实现接收回调绑定
void* CO_ControlCAN_ReceiveThread(void *arg) {
    CO_CANmodule_t *CANmodule = (CO_CANmodule_t*)arg;
    VCI_CAN_OBJ recFrames[100];
    
    while (1) {
        // 接收通道0的数据
        ULONG num = VCI_Receive(VCI_USBCAN2, 0, 0, recFrames, 100, 10);
        for (ULONG i = 0; i < num; i++) {
            CO_CANrxMsg_t rxMsg = {
                .ident = recFrames[i].ID,
                .DLC = recFrames[i].DataLen,
                .rtr = recFrames[i].RemoteFlag,
                .data = recFrames[i].Data
            };
            CO_CANrxMsg(CANmodule, &rxMsg);
        }
        // 处理其他通道...
    }
    return NULL;
}