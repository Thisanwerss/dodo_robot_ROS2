#ifndef CAN_PORT_H
#define CAN_PORT_H

#include <iostream>
#include <cstring>
#include <memory>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

class CanPort {
public:
    using SharedPtr = std::shared_ptr<CanPort>;

    explicit CanPort(const std::string& ifname = "can0") {
        struct ifreq ifr{};
        struct sockaddr_can addr{};

        // 打开 CAN socket
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            perror("Error while opening CAN socket");
            exit(1);
        }

        // 获取接口索引
        std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            perror("Error getting interface index");
            close(socket_fd_);
            exit(1);
        }

        // 绑定到指定 CAN 接口
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("Error in CAN socket bind");
            close(socket_fd_);
            exit(1);
        }

        std::cout << "✅ Connected to CAN interface: " << ifname << std::endl;
    }

    ~CanPort() {
        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }
    }

    // 发送 CAN 帧
    ssize_t send(uint32_t can_id, const uint8_t* data, uint8_t len) {
        struct can_frame frame{};
        frame.can_id = can_id;
        frame.can_dlc = len;
        std::memcpy(frame.data, data, len);
        return write(socket_fd_, &frame, sizeof(frame));
    }

    // 接收 CAN 帧
    ssize_t recv(uint32_t& can_id, uint8_t* data, uint8_t& len) {
        struct can_frame frame{};
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
        if (nbytes > 0) {
            can_id = frame.can_id;
            len = frame.can_dlc;
            std::memcpy(data, frame.data, len);
        }
        return nbytes;
    }

private:
    int socket_fd_;
};

#endif // CAN_PORT_H
