/*
**
**  Copyright (c) 2020 Japan Aerospace Exploration Agency.
**  All Rights Reserved.
**
**  This file is covered by the LICENSE.txt in the root of this project.
**
*/
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#define MAX_RETRY_NUM 3600
#define TIMEOUT_SEC 1
#define TIMEOUT_NSEC 0
#define DATA_SIZE 256

/**
 * Init TCP server.
 * @param ip_addr   IP address
 * @param port      Port number
 * @return File descriptor of generated socket
 */
int8_t tcp_server_init(const char* ip_addr, uint16_t port) {

    int tcp_sock;
    int status;
    int8_t dst_sock;
    struct sockaddr_in dst_addr;
    int dst_addr_size = sizeof(dst_addr);
    fd_set fds, readfds;
    int n;
    int maxfd;
    struct timeval tv;
    int count = 0;

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
        return -2;
    }
    
    status = listen(tcp_sock, 5);
    if (status < 0) {
        return -3;
    }

    // Init fd_set
    FD_ZERO(&readfds);
    // Register tcp_sock as reading-socket waiting for select
    FD_SET(tcp_sock, &readfds);

    maxfd = tcp_sock;
    while (1) {
        memcpy(&fds, &readfds, sizeof(fd_set));
        maxfd = maxfd + 1;
        // Timeout is set to 1 sec
        tv.tv_sec = TIMEOUT_SEC;
        tv.tv_usec = TIMEOUT_NSEC;
        // Wait for fds socket to be readable
        n = select(maxfd, &fds, NULL, NULL, &tv);
        if (n == 0) {
            count++;
            if (count >= MAX_RETRY_NUM) {
                // Not connected for 1 hour
                return -4;
            }
        } else if (n < 0) {
            // Terminated
            return -5;
        } else {
            // Connected from cfe_relay_app
            break;
        }
    }

    dst_sock = accept(tcp_sock, (struct sockaddr *)&dst_addr, &dst_addr_size);
    if (status < 0) {
        return -6;
    }

    return dst_sock;
}

/**
 * Init TCP client.
 * @param ip_addr   IP address
 * @param port      Port number
 * @return File descriptor of generated socket
 */
int8_t tcp_client_init(const char* ip_addr, uint16_t port) {
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
        return -1;
    }

    return dst_sock;
}

/**
 * Receive data with TCP/IP.
 * @param dst_sock  File descriptor of socket
 * @param buf_size  Received data size
 * @param read_data Received data
 * @return Result
 */
int8_t tcp_read(int8_t dst_sock, uint32_t buf_size, uint8_t *read_data) {
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

/**
 * Send data with TCP/IP.
 * @param dst_sock   File descriptor of socket
 * @param write_data Sent data
 * @param data_size  Sent data size
 * @param ret        Result(0: success, -1: error)
 * @return Sent data size. If error occurs, return 0.
 */
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

/**
 * Close socket.
 * @param dst_sock File descriptor of socket
 * @return Result
 */
int8_t tcp_close(int8_t dst_sock) {
    int8_t status;
    status = close(dst_sock);

    return status;
}
