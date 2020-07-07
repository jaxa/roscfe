/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "udp.h"

#define DATA_SIZE 256

int8_t udp_server_init(const char* ip_addr, unsigned short port) {

    int udp_sock;
    int status;
    int8_t dst_sock;
    struct sockaddr_in dst_addr;
    int dst_addr_size = sizeof(dst_addr);

    dst_addr.sin_port = htons(port);
    dst_addr.sin_family = AF_INET;
    dst_addr.sin_addr.s_addr = inet_addr(ip_addr);

    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        return -1;
    }

    status = bind(udp_sock, (struct sockaddr *)&dst_addr, sizeof(dst_addr));
    if (status < 0) {
        return -1;
    }

    return udp_sock;
}

int8_t udp_client_init(const char* ip_addr, unsigned short port) {

    int udp_sock;
    int status;
    int8_t dst_sock;
    struct sockaddr_in dst_addr;

    dst_addr.sin_port = htons(port);
    dst_addr.sin_family = AF_INET;
    dst_addr.sin_addr.s_addr = inet_addr(ip_addr);

    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        return -1;
    }

    return udp_sock;
}

int8_t udp_read(int8_t dst_sock, unsigned int buf_size, unsigned char *read_data) {
    int read_size;

    uint32_t sum_size = 0;
    while (sum_size < buf_size) {
        read_size = recv(dst_sock, read_data + sum_size, DATA_SIZE, 0);
        if (read_size < 0) {
            return -1;
        }
        sum_size += read_size;
    }
    read_data[buf_size] = 0;

    return 0;
}

uint32_t udp_write(int8_t dst_sock, uint8_t* write_data, uint32_t data_size, const char* ipaddr, uint16_t port, char *ret) {
    int ret_value;
    struct sockaddr_in dst_addr;
    dst_addr.sin_port = htons(port);
    dst_addr.sin_family = AF_INET;
    dst_addr.sin_addr.s_addr = inet_addr(ipaddr);

    *ret = 0;
    uint32_t sum_size = 0;
    while (sum_size < data_size) {
        ret_value = sendto(dst_sock, write_data + sum_size, DATA_SIZE, 0, (struct sockaddr *)&dst_addr, sizeof(dst_addr));
        if (ret_value < 0) {
            *ret = -1;
            data_size = 0;
            break;
        }
        sum_size += ret_value;
    }

    return data_size;
}

int8_t udp_close(int8_t dst_sock) {
    int8_t status;
    status = close(dst_sock);

    return status;
}
