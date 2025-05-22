/*
 * CANopen main program file.
 *
 * This file is a template for other microcontrollers.
 *
 * @file        main_generic.c
 * @author      Janez Paternoster
 * @copyright   2021 Janez Paternoster
 *
 * This file is part of <https://github.com/CANopenNode/CANopenNode>, a CANopen Stack.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this
 * file except in compliance with the License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is
 * distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 */

#include <stdio.h>
// 包含 CANopen 协议栈核心功能（NMT、SDO、PDO 等）
#include "CANopen.h"
// 定义对象字典（Object Dictionary）结构，包含设备参数和通信配置。
#include "OD.h"
// 提供存储模块接口，用于持久化参数。
#include "lib/CO_storageBlank.h"
// 日志输出宏，用于调试信息打印。
#define log_printf(macropar_message, ...) printf(macropar_message, ##__VA_ARGS__)

/* 定义网络管理（NMT）控制标志。*/
#define NMT_CONTROL                                                                                                    \
    CO_NMT_STARTUP_TO_OPERATIONAL                                                                                      \
    | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME        500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK        false
#define OD_STATUS_BITS       NULL

/* 全局变量和对象 */
CO_t* CO = NULL; /* CANopen 主对象，管理协议栈所有模块。 */
// LED 状态变量，用于指示 CANopen 状态。
uint8_t LED_red, LED_green;

/* main ***********************************************************************/
int
main(void) {
    // 定义和初始化变量
    CO_ReturnError_t err;           // 错误码
    CO_NMT_reset_cmd_t reset = CO_RESET_NOT;        // 复位命令标志
    uint32_t heapMemoryUsed;        // 堆内存使用统计
    void* CANptr = NULL;            // CAN 控制器地址
    uint8_t pendingNodeId = 10;     /* read from dip switches or nonvolatile memory, configurable by LSS slave */ // 待定节点 ID（通过 LSS 配置）
    uint8_t activeNodeId = 10;      /* Copied from CO_pendingNodeId in the communication reset section */
    uint16_t pendingBitRate = 125;  /* read from dip switches or nonvolatile memory, configurable by LSS slave */ // 待定波特率（125 kbps）

// 存储模块初始化（条件编译）
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_storage_t storage;           // 存储对象
    CO_storage_entry_t storageEntries[] = {{.addr = &OD_PERSIST_COMM,                   // 持久化存储对象字典条目
                                            .len = sizeof(OD_PERSIST_COMM),             // 数据长度
                                            .subIndexOD = 2,                            // 子索引（可能对应 OD 中的特定参数）
                                            .attr = CO_storage_cmd | CO_storage_restore,// 属性：存储命令和恢复默认值
                                            .addrNV = NULL}};                           // 非易失性存储地址（需平台实现）
    uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);   // 存储条目数量
    uint32_t storageInitError = 0;      
#endif

    /* Configure microcontroller. */

    /* Allocate memory */
    CO_config_t* config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    /* example usage of CO_MULTIPLE_OD (but still single OD here) */
    CO_config_t co_config = {0};
    OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
    co_config.CNT_LEDS = 1;
    co_config.CNT_LSS_SLV = 1;
    config_ptr = &co_config;
#endif /* CO_MULTIPLE_OD */
// 分配CANopen对象内存,创建 CANopen 对象​​

    CO = CO_new(config_ptr, &heapMemoryUsed);
    if (CO == NULL) {
        log_printf("Error: Can't allocate memory\n");
        return 0;
    } else {
        log_printf("Allocated %u bytes for CANopen objects\n", heapMemoryUsed);
    }

// 初始化存储系统
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    // 配置存储相关参数 CO_storageBlank
    err = CO_storageBlank_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT) {
        log_printf("Error: Storage %d\n", storageInitError);
        return 0;
    }
#endif
    // 主循环（通信复位与初始化）​​
    while (reset != CO_RESET_APP) {
        /* CANopen communication reset - initialize CANopen objects *******************/
        log_printf("CANopenNode - Reset communication...\n");

        /* Wait rt_thread. */
        CO->CANmodule->CANnormal = false;

        // 进入 CAN 配置模式
        CO_CANsetConfigurationMode((void*)&CANptr);
        CO_CANmodule_disable(CO->CANmodule);

        /* 配置 CAN 控制器波特率 */
        err = CO_CANinit(CO, CANptr, pendingBitRate);
        if (err != CO_ERROR_NO) {
            log_printf("Error: CAN initialization failed: %d\n", err);
            return 0;
        }

        CO_LSS_address_t lssAddress = {.identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                                    .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                                    .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                                    .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};
        // 设置 LSS 从机参数（节点 ID、波特率）
        err = CO_LSSinit(CO, &lssAddress, &pendingNodeId, &pendingBitRate);
        if (err != CO_ERROR_NO) {
            log_printf("Error: LSS slave initialization failed: %d\n", err);
            return 0;
        }

        activeNodeId = pendingNodeId;
        uint32_t errInfo = 0;
        // 初始化 NMT、SDO、心跳、SYNC 等模块,核心协议栈初始化
        err = CO_CANopenInit(CO,                   /* CANopen object */
                             NULL,                 /* alternate NMT */
                             NULL,                 /* alternate em */
                             OD,                   /* Object dictionary */
                             OD_STATUS_BITS,       /* Optional OD_statusBits */
                             NMT_CONTROL,          /* CO_NMT_control_t */
                             FIRST_HB_TIME,        /* firstHBTime_ms */
                             SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                             SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                             SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                             activeNodeId, &errInfo);
        if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
            } else {
                log_printf("Error: CANopen initialization failed: %d\n", err);
            }
            return 0;
        }
        // 配置 PDO 通信参数和映射。
        err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeId, &errInfo);
        if (err != CO_ERROR_NO) {
            if (err == CO_ERROR_OD_PARAMETERS) {
                log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
            } else {
                log_printf("Error: PDO initialization failed: %d\n", err);
            }
            return 0;
        }

        /* Configure Timer interrupt function for execution every 1 millisecond */

        /* Configure CAN transmit and receive interrupt */

        /* Configure CANopen callbacks, etc */
        if (!CO->nodeIdUnconfigured) {

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
            if (storageInitError != 0) {
                CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
            }
#endif
        } else {
            log_printf("CANopenNode - Node-id not initialized\n");
        }

        /* start CAN */
        CO_CANsetNormalMode(CO->CANmodule);

        reset = CO_RESET_NOT;

        log_printf("CANopenNode - Running...\n");
        fflush(stdout);

        while (reset == CO_RESET_NOT) {
            /* loop for normal program execution ******************************************/
            /* get time difference since last function call */
            uint32_t timeDifference_us = 500;

            /* 处理周期性任务（如心跳、SDO 请求、PDO 传输等），返回是否需要复位。 */
            reset = CO_process(CO, false, timeDifference_us, NULL);
            LED_red = CO_LED_RED(CO->LEDs, CO_LED_CANopen);
            LED_green = CO_LED_GREEN(CO->LEDs, CO_LED_CANopen);

            /* Nonblocking application code may go here. */

            /* Process automatic storage */

            /* optional sleep for short time */
        }
    }

    /* program exit ***************************************************************/
    /* stop threads */

    /* delete objects from memory */
    CO_CANsetConfigurationMode((void*)&CANptr);
    CO_delete(CO);

    log_printf("CANopenNode finished\n");

    /* reset */
    return 0;
}

/* timer thread executes in constant intervals ********************************/
void
tmrTask_thread(void) {

    for (;;) {
        CO_LOCK_OD(CO->CANmodule);
        if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal) {
            bool_t syncWas = false;
            /* get time difference since last function call */
            uint32_t timeDifference_us = 1000;

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif

            /* Further I/O or nonblocking application code may go here. */
        }
        CO_UNLOCK_OD(CO->CANmodule);
    }
}

/* CAN interrupt function executes on received CAN message ********************/
void /* interrupt */
CO_CAN1InterruptHandler(void) {

    /* clear interrupt flag */
}
