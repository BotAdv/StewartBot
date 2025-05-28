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

// 全局控制变量
volatile int m_run0 = 1;

// CAN接口配置
#define CAN_IFACE "can0"
#define BITRATE 1000000
#define COB_ID 0x000
#define FRAME_DATA {0x01, 0x01}

// CAN初始化函数
int init_can_socket(const char* ifname) {
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Socket创建失败");
        return -1;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("接口索引获取失败");
        close(s);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("绑定失败");
        close(s);
        return -1;
    }

    return s;
}

// 接收线程函数
void* receive_func(void* param) {
    int sock = *(int*)param;
    struct can_frame frame;
    
    while (m_run0) {
        int nbytes = read(sock, &frame, sizeof(frame));
        if (nbytes > 0) {
            printf("接收报文 - ID:0x%03X DLC:%d 数据:", 
                  frame.can_id, frame.can_dlc);
            for (int i = 0; i < frame.can_dlc; i++) {
                printf(" %02X", frame.data[i]);
            }
            printf("\n");
        }
        usleep(1000);  // 降低CPU占用
    }
    
    pthread_exit(NULL);
}

int main() {
    // 配置CAN接口
    char cmd_buf[128];
    snprintf(cmd_buf, sizeof(cmd_buf), 
            "sudo ip link set %s type can bitrate %d", CAN_IFACE, BITRATE);
    system(cmd_buf);
    snprintf(cmd_buf, sizeof(cmd_buf), "sudo ip link set %s up", CAN_IFACE);
    system(cmd_buf);

    // 初始化CAN socket
    int can_sock = init_can_socket(CAN_IFACE);
    if (can_sock < 0) {
        fprintf(stderr, "CAN接口初始化失败\n");
        return 1;
    }

    // 创建接收线程
    pthread_t recv_thread;
    if (pthread_create(&recv_thread, NULL, receive_func, &can_sock) != 0) {
        perror("接收线程创建失败");
        close(can_sock);
        return 1;
    }

    // 构造发送帧
    struct can_frame send_frame = {
        .can_id = COB_ID,
        .can_dlc = 2,
        .data = FRAME_DATA
    };

    // 发送固定帧
    printf("正在发送报文...\n");
    if (write(can_sock, &send_frame, sizeof(send_frame)) != sizeof(send_frame)) {
        perror("发送失败");
    } else {
        printf("成功发送报文 - ID:0x%03X 数据:", COB_ID);
        for (int i = 0; i < send_frame.can_dlc; i++) {
            printf(" %02X", send_frame.data[i]);
        }
        printf("\n");
    }

    // 保持接收状态
    printf("进入持续接收模式...\n");
    while (1) {
        sleep(1);  // 保持主线程运行
    }

    // 清理资源（正常情况下不会执行到这里）
    m_run0 = 0;
    pthread_join(recv_thread, NULL);
    close(can_sock);
    system("sudo ip link set can0 down");
    
    return 0;
}