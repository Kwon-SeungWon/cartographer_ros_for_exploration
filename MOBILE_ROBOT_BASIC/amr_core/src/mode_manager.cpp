#include "amr_core/mode_manager.hpp"
#include "amr_serial/srv/get_mode.hpp"

ModeManager::ModeManager(rclcpp::Node* node)
    : node_(node), state_(State::INIT) {
    client_ = node_->create_client<amr_serial::srv::GetMode>("get_select_switch_status");
}

void ModeManager::checkMode() {
    if (!node_) {
        RCLCPP_ERROR(rclcpp::get_logger("ModeManager"), "node_ is nullptr!");
        return;
    }
    if (!client_) {
        RCLCPP_ERROR(node_->get_logger(), "client_ is nullptr!");
        return;
    }
    // 서비스가 생성될 때까지 최대 5초 대기
    if (!client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_WARN(node_->get_logger(), "Service /get_select_switch_status not available after waiting, fallback to AUTO");
        state_ = State::AUTO;
        return;
    }
    if (emergency_flag_) {
        RCLCPP_DEBUG(node_->get_logger(), "[ModeManager] emergency_flag_ true, setting state EMERGENCY");
        state_ = State::EMERGENCY;
        return;
    }
    if (manual_flag_) {
        RCLCPP_DEBUG(node_->get_logger(), "[ModeManager] manual_flag_ true, setting state MANUAL");
        state_ = State::MANUAL;
        return;
    }
    // 비동기 서비스 요청
    auto request = std::make_shared<amr_serial::srv::GetMode::Request>();
    request->req = true;
    // RCLCPP_INFO(node_->get_logger(), "[ModeManager] Sending async service request to get_select_switch_status");
    client_->async_send_request(request,
        std::bind(&ModeManager::handle_get_mode_response, this, std::placeholders::_1));
}

void ModeManager::handle_get_mode_response(rclcpp::Client<amr_serial::srv::GetMode>::SharedFuture future) {
    auto response = future.get();
    if (!response) {
        // RCLCPP_ERROR(node_->get_logger(), "[ModeManager] Service response is nullptr!");
        state_ = State::AUTO;
        return;
    }
    // RCLCPP_INFO(node_->get_logger(), "[ModeManager] Service response: joystick=%d, autonomous=%d", response->joystick, response->autonomous);
    if (response->joystick) {
        // RCLCPP_INFO(node_->get_logger(), "[ModeManager] Setting state JOYSTICK");
        state_ = State::JOYSTICK;
        return;
    }
    if (response->autonomous) {
        // RCLCPP_INFO(node_->get_logger(), "[ModeManager] Setting state AUTO");
        state_ = State::AUTO;
        return;
    }
    // RCLCPP_INFO(node_->get_logger(), "[ModeManager] No mode flag set, defaulting to AUTO");
    state_ = State::AUTO;
}

ModeManager::State ModeManager::getState() const {
    return state_;
}

void ModeManager::setManualFlag(bool flag) {
    manual_flag_ = flag;
}

void ModeManager::setEmergencyFlag(bool flag) {
    emergency_flag_ = flag;
} 