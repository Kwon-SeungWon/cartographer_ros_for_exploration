#include "amr_interface/path_status_manager.hpp"

PathStatusManager::PathStatusManager(rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr pub, rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub2, double threshold, double missed_dist)
: path_pub_(pub), path_status_pub_(pub2), threshold_(threshold), missed_dist_(missed_dist) {}


void PathStatusManager::setWaypoints(const geometry_msgs::msg::PoseArray& waypoints) {
    if (waypoints.poses.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("PathStatusManager"), "setWaypoints called with empty poses!");
        return;
    }
    current_path_ = waypoints;
    path_status_.assign(waypoints.poses.size(), 1);
    prev_dist_to_next_.assign(waypoints.poses.size(), 0.0);
    // Add tracking flag to prevent immediate missed detection
    tracking_started_.assign(waypoints.poses.size(), false);
    path_status_publish();
}

void PathStatusManager::setNodePath(const std::vector<uint16_t>& node_path) {
    if (node_path.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("PathStatusManager"), "setNodePath called with empty node_path!");
        return;
    }
    
    // Validate path connectivity before setting
    if (!node_connections_.empty()) {
        // 임시로 node_path를 사용해서 검증
        std::vector<uint16_t> temp_path = node_path;
        for (size_t i = 0; i < temp_path.size() - 1; ++i) {
            if (!areNodesConnected(temp_path[i], temp_path[i + 1])) {
                RCLCPP_ERROR(rclcpp::get_logger("PathStatusManager"), 
                    "🚫 INVALID PATH DETECTED: Node %d is not connected to Node %d. Rejecting path.", 
                    temp_path[i], temp_path[i + 1]);
                
                // 연결되지 않은 노드들을 찾아서 로그 출력
                RCLCPP_ERROR(rclcpp::get_logger("PathStatusManager"), 
                    "Node %d connections: ", temp_path[i]);
                auto it = node_connections_.find(temp_path[i]);
                if (it != node_connections_.end()) {
                    for (uint16_t conn : it->second) {
                        RCLCPP_ERROR(rclcpp::get_logger("PathStatusManager"), "  - %d", conn);
                    }
                }
                
                return; // ❌ 경로 거부하고 함수 종료
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("PathStatusManager"), 
            "✅ Path connectivity validation passed for %zu nodes", node_path.size());
    }
    
    current_node_path_ = node_path;
    path_publish();
}

void PathStatusManager::updateStatus(const geometry_msgs::msg::Pose& robot_pose) {
    for (size_t i = 0; i < current_path_.poses.size(); ++i) {
        if (path_status_[i] != 1) continue; // only waiting is processed
        // Last Goal Node
        if (i + 1 >= current_path_.poses.size()) {
            
            double dx = robot_pose.position.x - current_path_.poses[i].position.x;
            double dy = robot_pose.position.y - current_path_.poses[i].position.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            if (dist < threshold_) {
                path_status_[i] = 2;
                prev_dist_to_next_[i] = 0.0;
            }
            continue;
        }

        double dx_next = robot_pose.position.x - current_path_.poses[i+1].position.x;
        double dy_next = robot_pose.position.y - current_path_.poses[i+1].position.y;
        double dist_next = std::sqrt(dx_next*dx_next + dy_next*dy_next);

        // reached
        double dx = robot_pose.position.x - current_path_.poses[i].position.x;
        double dy = robot_pose.position.y - current_path_.poses[i].position.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < threshold_) {
            path_status_[i] = 2;
            prev_dist_to_next_[i] = 0.0;
            continue;
        }

        // missed - only check if we've started tracking and have a valid previous distance
        // Also ensure we've had at least one update cycle to establish a baseline
        // if (tracking_started_[i] && prev_dist_to_next_[i] > 0.0 && dist_next - prev_dist_to_next_[i] > missed_dist_) {
        //     RCLCPP_WARN(rclcpp::get_logger("PathStatusManager"), 
        //         "Waypoint %zu marked as missed! dist_next: %.2f, prev_dist: %.2f, diff: %.2f, missed_dist: %.2f", 
        //         i, dist_next, prev_dist_to_next_[i], dist_next - prev_dist_to_next_[i], missed_dist_);
        //     path_status_[i] = 3;
        //     prev_dist_to_next_[i] = 0.0;
        //     tracking_started_[i] = false;
        //     continue;
        // }
        
        // prev_dist_to_next_ 갱신 - only update if we're getting closer or it's the first time
        if (prev_dist_to_next_[i] == 0.0 || dist_next < prev_dist_to_next_[i]) {
            if (prev_dist_to_next_[i] == 0.0) {
                tracking_started_[i] = true;
                RCLCPP_INFO(rclcpp::get_logger("PathStatusManager"), 
                    "Started tracking waypoint %zu, initial dist_next: %.2f", i, dist_next);
            }
            prev_dist_to_next_[i] = dist_next;
        }
    }
    path_status_publish();
}

void PathStatusManager::path_publish() {
    if (current_node_path_.empty()) return;
    std_msgs::msg::UInt16MultiArray msg;
    msg.data = current_node_path_;
    path_pub_->publish(msg);
}

void PathStatusManager::path_status_publish() {
    if (path_status_.empty()) return;
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = path_status_;
    path_status_pub_->publish(msg);
}

void PathStatusManager::resetForRepeat() {
    if (path_status_.empty()) return;
    
    // Reset all waypoint status to waiting (1)
    for (size_t i = 0; i < path_status_.size(); ++i) {
        path_status_[i] = 1;
        prev_dist_to_next_[i] = 0.0;
        tracking_started_[i] = false;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("PathStatusManager"), "Reset waypoints for REPEAT mission");
    path_status_publish();
}

bool PathStatusManager::isAllWaypointsCompleted() const {
    if (path_status_.empty()) return false;
    
    for (size_t i = 0; i < path_status_.size(); ++i) {
        if (path_status_[i] != 2) { // 2 = completed
            return false;
        }
    }
    return true;
}

void PathStatusManager::resetWaypointStatus() {
    if (path_status_.empty()) return;
    
    for (size_t i = 0; i < path_status_.size(); ++i) {
        path_status_[i] = 1; // Reset to waiting
        prev_dist_to_next_[i] = 0.0;
        tracking_started_[i] = false;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("PathStatusManager"), "Reset waypoint status");
    path_status_publish();
}

// Connectivity validation methods
void PathStatusManager::setNodeConnections(const std::map<uint16_t, std::vector<uint16_t>>& connections) {
    node_connections_ = connections;
    RCLCPP_INFO(rclcpp::get_logger("PathStatusManager"), 
        "Set node connections with %zu nodes", connections.size());
}

bool PathStatusManager::validatePathConnectivity() const {
    if (current_node_path_.size() < 2) return true; // Single node path is always valid
    
    for (size_t i = 0; i < current_node_path_.size() - 1; ++i) {
        uint16_t current_node = current_node_path_[i];
        uint16_t next_node = current_node_path_[i + 1];
        
        if (!areNodesConnected(current_node, next_node)) {
            RCLCPP_ERROR(rclcpp::get_logger("PathStatusManager"), 
                "Path connectivity validation failed: Node %d is not connected to Node %d", 
                current_node, next_node);
            return false;
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("PathStatusManager"), 
        "Path connectivity validation passed for %zu nodes", current_node_path_.size());
    return true;
}

bool PathStatusManager::areNodesConnected(uint16_t node1, uint16_t node2) const {
    // Check if node1 has node2 in its connections
    auto it = node_connections_.find(node1);
    if (it != node_connections_.end()) {
        for (uint16_t connected_node : it->second) {
            if (connected_node == node2) {
                return true;
            }
        }
    }
    
    // Check if node2 has node1 in its connections (bidirectional)
    it = node_connections_.find(node2);
    if (it != node_connections_.end()) {
        for (uint16_t connected_node : it->second) {
            if (connected_node == node1) {
                return true;
            }
        }
    }
    
    return false;
} 