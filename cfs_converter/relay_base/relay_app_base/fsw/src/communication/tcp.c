/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include "tcp.h"

#define DATA_SIZE 256

int8_t tcp_server_init(const char* ip_addr, unsigned short port) {

    int tcp_sock;
    int status;
    int8_t dst_sock;
    struct sockaddr_in dst_addr;
    int dst_addr_size = sizeof(dst_addr);

    struct sockaddr_in srv_addr;
    srv_addr.sin_port = htons(port);
    srv_addr.sin_family = AF_INET;
    srv_addr.sin_addr.s_addr = inet_addr(ip_addr);

    tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_sock < 0) {
        return -1;
    }

    status = bind(tcp_sock, (struct sockaddr *)&srv_addr, sizeof(srv_addr));
    if (status < 0) {
        return -1;
    }
    
    status = listen(tcp_sock, 5);
    if (status < 0) {
        return -1;
    }

    dst_sock = accept(tcp_sock, (struct sockaddr *)&dst_addr, &dst_addr_size);
    if (status < 0) {
        return -1;
    }

    return dst_sock;
}

int8_t tcp_client_init(const char* ip_addr, unsigned short port) {
    int dst_sock;
    struct sockaddr_in dst_addr;
    int status;

    dst_addr.sin_family = AF_INET;
    dst_addr.sin_port = htons(port);
    dst_addr.sin_addr.s_addr = inet_addr(ip_addr);

    dst_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (dst_sock < 0) {
        return -1;
    }

    status = connect(dst_sock, (struct sockaddr *)&dst_addr, sizeof(dst_addr));
    if (status < 0) {
        return -2;
    }

    return dst_sock;
}

int8_t tcp_read(int8_t dst_sock, unsigned int buf_size, unsigned char *read_data) {
    int read_size;

    uint32_t sum_size = 0;
    while (sum_size < buf_size) {
        read_size = read(dst_sock, read_data + sum_size, DATA_SIZE);
        if (read_size < 0) {
            return -1;
        }
        sum_size += read_size;
    }
    read_data[buf_size] = 0;

    return 0;
}

uint32_t tcp_write(int8_t dst_sock, uint8_t* write_data, uint32_t data_size, char *ret) {
    int ret_value;
    *ret = 0;
    uint32_t sum_size = 0;
    while (sum_size < data_size) {
        ret_value = write(dst_sock, write_data + sum_size, DATA_SIZE);
        if (ret_value < 0) {
            *ret = -1;
            data_size = 0;
        }
        sum_size += ret_value;
    }

    return data_size;
}

int8_t tcp_close(int8_t dst_sock) {
    int8_t status;
    status = close(dst_sock);

    return status;
}
