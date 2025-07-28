#ifndef __STATE_HPP__
#define __STATE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>


// #include "my_robot_states/docking.hpp"
// #include "my_robot_states/charging.hpp"
// #include "my_robot_states/working.hpp"
// #include "my_robot_states/manual.hpp"

namespace AMR
{

// 상태 정의
enum class RobotState {
    INIT = 0x00, // 0
    IDLE = 0x10, // 16
    AUTO = 0x20, // 32
    MANUAL = 0x30, // 48
    DOCKING = 0x40, // 64
    CHARGING = 0x50, // 80
    MANIPULATION = 0x60, // 96
    EMERGENCY = 0x70, // 112
    STOP = 0x80 // 128
};

class StateMachine : public rclcpp::Node
{
public:
    StateMachine();
    ~StateMachine() = default;

private:
    // 상태머신을 주기적으로 돌리는 타이머
    rclcpp::TimerBase::SharedPtr timer_;

    // 현재 상태
    RobotState current_state_;

    // 상태별 알고리즘 모듈
    std::unique_ptr<Docking>   docking_module_;
    std::unique_ptr<Charging>  charging_module_;
    std::unique_ptr<Working>   working_module_;
    std::unique_ptr<Manual>    manual_module_;

    // 상태 전이를 위한 플래그 (예시)
    bool auto_mode_requested_;
    bool manual_mode_requested_;
    bool docking_requested_;
    bool emergency_triggered_;

    // 타이머 초기화 함수
    void initStateTimer();
    void stateLoopCallback();

    // 상태별 처리 함수
    void handleInitState();
    void handleIdleState();
    void handleAutoState();
    void handleManualState();
    void handleDockingState();
    void handleChargingState();
    void handleWorkingState();
    void handleEmergencyState();
    void handleStopState();

    // 상태 변경 함수
    void setState(RobotState new_state);
    RobotState getState() const;

    // 예시: 외부 이벤트/콜백
    void commandCallback(const std::string & cmd);
    void batteryCallback(float battery_level);
};

}  // namespace AMR

#endif  // STATE_HPP
