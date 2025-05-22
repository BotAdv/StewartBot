#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "lib/controlcan.h"

// 全局变量定义
int count = 0;
volatile int m_run0 = 1;

// CAN接口名称（假设CAN1=can0，CAN2=can1）
#define CAN1 "can0"
#define CAN2 "can1"

// 初始化CAN接口
int init_can_socket(const char* ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Socket creation failed");
        return -1;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("Ioctl failed");
        close(s);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr))) {
        perror("Bind failed");
        close(s);
        return -1;
    }

    return s;
}

// 接收线程函数
void* receive_func(void* param) {
    int* sockets = (int*)param;
    int s1 = sockets[0], s2 = sockets[1];
    struct can_frame frame;
    int nbytes;

    while (m_run0) {
        // 读取CAN1数据
        nbytes = read(s1, &frame, sizeof(frame));
        if (nbytes > 0) {
            printf("CAN1 RX ID:0x%03X DLC:%d Data:", frame.can_id, frame.can_dlc);
            for (int i = 0; i < frame.can_dlc; i++) {
                printf(" %02X", frame.data[i]);
            }
            printf("\n");
        }

        // 读取CAN2数据
        nbytes = read(s2, &frame, sizeof(frame));
        if (nbytes > 0) {
            printf("CAN2 RX ID:0x%03X DLC:%d Data:", frame.can_id, frame.can_dlc);
            for (int i = 0; i < frame.can_dlc; i++) {
                printf(" %02X", frame.data[i]);
            }
            printf("\n");
        }

        usleep(1000); // 防止CPU占用过高
    }

    pthread_exit(NULL);
}

int main() {
    // 配置CAN接口（需提前执行或在代码中调用system）
    // system("sudo modprobe vcan");
    // system("sudo ip link add dev can0 type vcan");
    // system("sudo ip link add dev can1 type vcan");
    // system("sudo ip link set can0 up");
    // system("sudo ip link set can1 up");

    system("sudo ip link set can0 type can bitrate 125000");
    system("sudo ip link set can1 type can bitrate 125000");
    system("sudo ip link set can0 up");
    system("sudo ip link set can1 up");

    // 初始化CAN接口socket
    int s_can1 = init_can_socket(CAN1);
    int s_can2 = init_can_socket(CAN2);
    if (s_can1 < 0 || s_can2 < 0) {
        fprintf(stderr, "CAN socket初始化失败\n");
        exit(1);
    }

    // 创建接收线程（传入两个socket）
    pthread_t threadid;
    int sockets[2] = {s_can1, s_can2};
    if (pthread_create(&threadid, NULL, receive_func, sockets) != 0) {
        perror("线程创建失败");
        exit(1);
    }

    // 发送测试帧到CAN1和CAN2
    struct can_frame send_frame;
    send_frame.can_id = 0x123;
    send_frame.can_dlc = 8;
    for (int i = 0; i < send_frame.can_dlc; i++) {
        send_frame.data[i] = i;
    }

    int times = 5;
    while (times--) {
        // 发送到CAN1
        if (write(s_can1, &send_frame, sizeof(send_frame)) != sizeof(send_frame)) {
            perror("CAN1发送失败");
        } else {
            printf("CAN1发送成功: ID=0x%03X\n", send_frame.can_id);
        }

        // 发送到CAN2
        if (write(s_can2, &send_frame, sizeof(send_frame)) != sizeof(send_frame)) {
            perror("CAN2发送失败");
        } else {
            printf("CAN2发送成功: ID=0x%03X\n", send_frame.can_id);
        }

        send_frame.can_id++;
        sleep(1); // 间隔1秒
    }

    // 关闭线程和socket
    m_run0 = 0;
    pthread_join(threadid, NULL);
    close(s_can1);
    close(s_can2);

    // 关闭CAN接口（可选）
    system("sudo ip link set can0 down");
    system("sudo ip link set can1 down");

    return 0;
}
