#include "amr_core/can.hpp"


CAN_::CAN_()
{
    Init();
}

CAN_::~CAN_()
{
    running_ = false;
    if (can_receive_thread_.joinable()) {
        can_receive_thread_.join();
    }
    close(can_socket_);
}

void CAN_::Init()
{
    // ----- CAN Socket Setting -----
    if ((can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        return;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, "can0");
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
        close(can_socket_);
        return;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        close(can_socket_);
        return;
    }

    // Start CAN Receive Thread
    can_receive_thread_ = boost::thread(&CAN_::receiveCanMessages, this);
}

bool CAN_::sendCan(int id, uint8_t *data)
{
    struct can_frame frame;
    int32_t header = id | (int32_t)TMID << 8 | (int32_t)RMID << 16;
    
    frame.can_id = header | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    memcpy(frame.data, data, frame.can_dlc);

    int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        std::cerr << "[CAN] Failed to send CAN message, attempting recovery" << std::endl;
        recover();
        return false;
    }
    // std::cout << "[CAN] Sent CAN message: id=" << std::hex << frame.can_id << std::dec << std::endl;
    return true;
}

bool CAN_::sendRawCan(uint32_t id, uint8_t *data)
{
    struct can_frame frame;
    int32_t header = id;
    
    frame.can_id = header | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    memcpy(frame.data, data, frame.can_dlc);

    int nbytes = write(can_socket_, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
        std::cerr << "[CAN] Failed to send CAN message, attempting recovery" << std::endl;
        recover();
        return false;
    }
    // std::cout << "[CAN] Sent CAN message: id=" << std::hex << frame.can_id << std::dec << std::endl;
    return true;
}



void CAN_::recover() {
    std::cerr << "[CAN] Recovering CAN interface..." << std::endl;
    close(can_socket_);
    Init();
}

void CAN_::setReceiveCallback(std::function<void(const can_frame&)> cb) {
    receive_callback_ = cb;
}

void CAN_::receiveCanMessages()
{
    while (running_) {
        struct can_frame frame;
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
        if (nbytes > 0 && receive_callback_) {
            receive_callback_(frame);
        }
    }
}



