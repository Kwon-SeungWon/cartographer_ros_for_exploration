#pragma once
#include <linux/can.h>
#include <functional>
#include <boost/thread.hpp>
#include <cstring>
#include <sys/types.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/can/raw.h>
#include <iostream> // Added for std::cerr and std::cout
#include <iomanip>  // Added for std::hex and std::dec

#define RMID 183
#define TMID 184

#define DRIVER_ID 1

#define PID_DEFAULT_SET 3
#define PID_REQ_PID_DATA 4
#define PID_TQ_OFF 5
#define PID_BRAKE 6
#define PID_COMMAND 10
#define PID_POSI_RESET 13
#define PID_HALL_TYPE 21
#define PID_CTRL_STATUS 34
#define PID_HALL2_TYPE 65
#define PID_HALL1_TYPE 68
#define PID_CYCLIC_FAULT 81
#define PID_MAX_RPM1 121
#define PID_MAX_RPM2 122
#define PID_VEL_CMD 130
#define PID_ECAN_BITRATE 137
#define PID_TQ_DATA 139
#define PID_GAIN1 252
#define PID_GAIN2 253
#define PID_PNT_TQ_OFF 174
#define PID_PNT_BRAKE 175
#define PID_PNT_POSI_DATA 188
#define PID_MAIN_DATA 193
#define PID_MONITOR 196 //c4  //D1:제어기 상태정보 D2,3:모터회전수 D4,5,6,7:모터위치정보
#define PID_MAIN_DATA2 200
#define PID_MONITOR2 201 //c9 //D1:제어기 상태정보 D2,3:모터회전수 D4,5,6,7:모터위치정보
#define PID_PNT_VEL_CMD 207 //cf 
#define PID_PNT_TQ_CMD 209
#define PID_MAX_LOAD 211
#define PID_PNT_MONITOR 216
#define PID_ALARM_LOG 229

class CAN_ {
public:
    CAN_();
    ~CAN_();
    void Init();
    bool sendCan(int id, uint8_t *data);
    void setReceiveCallback(std::function<void(const can_frame&)> cb);
    void recover();
private:
    int can_socket_;
    boost::thread can_receive_thread_;
    bool running_ = true;
    std::function<void(const can_frame&)> receive_callback_;
    void receiveCanMessages();
};