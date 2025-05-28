// CO_driver_socketcan.cpp
#include "./include/CO_driver.h"        // 主头文件
#include "CO_driver_target.h"           // 包含修改后的驱动声明
#include <linux/can.h>                  // SocketCAN类型定义
#include <linux/can/raw.h> 
#include <sys/socket.h>                 // socket API
#include <unistd.h>                     // 包含 read() 函数声明
#include <cstring>                      // memcpy
#include <errno.h>
#include <fcntl.h>
#include <cstdio>
#include <cstdlib>
#include "CANopen.h"


extern int can_sock;  // 主程序中定义的SocketCAN描述符

// 定义CANopen需要的消息类型（与SocketCAN结构体兼容）
typedef struct can_frame CO_CANrxMsg_t;

// 实现接收缓冲区初始化函数,调整自定义函数名称或使用作用域解析
CO_ReturnError_t CO_CANrxBufferInit_Impl(
    CO_CANmodule_t *CANmodule,
    uint16_t index,
    uint16_t ident,
    uint16_t mask,
    bool_t rtr,
    void *object,
    void (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
    struct can_filter filter;
    filter.can_id = ident | CAN_EFF_FLAG;
    filter.can_mask = (mask ? mask : 0x1FFFFFU) | CAN_EFF_FLAG;

    // 添加过滤器到socket（带错误检查）
    if (setsockopt(can_sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) == -1) {
        perror("[ERROR] CO_CANrxBufferInit: setsockopt failed");
        return CO_ERROR_SYSCALL; // 返回系统调用错误码
    }

    CANmodule->CANptr = (void*)(intptr_t)can_sock;

    // 调用库函数（注意避免名称冲突，假设实际调用的是 CANopenNode 的 API）
    return CO_CANrxBufferInit(
        CANmodule, // 假设实际函数名不同
        index,
        ident,
        mask,
        rtr,
        object,
        (void (*)(void*, void*))pFunct
    );
}

// 实现CAN接收处理函数（需在主循环中调用）
void CO_CANreceive(CO_CANmodule_t *CANmodule) {
    struct can_frame frame;
    int sock = (int)(intptr_t)CANmodule->CANptr;

    while (read(sock, &frame, sizeof(frame)) > 0) {
        for (uint16_t i = 0; i < CANmodule->rxSize; ++i) {
            CO_CANrx_t *rx = &CANmodule->rxArray[i];
            
            // 检查 CAN ID 是否匹配过滤器
            if ((frame.can_id & rx->mask) == (rx->ident & rx->mask)) {
                // 调用注册的回调函数
                if (rx->CANrx_callback != NULL) {
                    rx->CANrx_callback(rx->object, (void*)&frame);
                }
            }
        }
    }
}