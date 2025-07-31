#include "amr_interface/enhanced_execution.hpp"

namespace AMR {

/**
 * @brief Validates if a state transition is allowed
 * @description Checks if the transition from one state to another is valid according to the defined state machine rules
 * @param from The current state of the robot
 * @param to The target state to transition to
 * @return true if the transition is valid, false otherwise
 */
bool StateTransitionManager::validateTransition(RobotState from, RobotState to) {
    static const std::map<RobotState, std::vector<RobotState>> validTransitions = {
        {RobotState::INIT, {RobotState::IDLE}},
        {RobotState::IDLE, {RobotState::AUTO, RobotState::SLAM, RobotState::UNDOCKING, RobotState::EMERGENCY}},
        {RobotState::AUTO, {RobotState::DOCKING, RobotState::EMERGENCY}},
        {RobotState::DOCKING, {RobotState::MANIPULATION, RobotState::CHARGING, RobotState::EMERGENCY}},
        {RobotState::UNDOCKING, {RobotState::IDLE, RobotState::DOCKING, RobotState::EMERGENCY}},
        {RobotState::CHARGING, {RobotState::UNDOCKING, RobotState::EMERGENCY}},
        {RobotState::MANIPULATION, {RobotState::UNDOCKING, RobotState::EMERGENCY}},
        {RobotState::EMERGENCY, {RobotState::STOP}},
        {RobotState::STOP, {RobotState::INIT, RobotState::IDLE, RobotState::EMERGENCY}}
    };

    auto it = validTransitions.find(from);
    if (it == validTransitions.end()) return false;
    
    return std::find(it->second.begin(), it->second.end(), to) != it->second.end();
}

/**
 * @brief Logs a state transition
 * @description Records a state transition in the history with thread safety
 * @param log The transition log containing state change information
 */
void StateTransitionManager::logTransition(const StateTransitionLog& log) {
    std::lock_guard<std::mutex> lock(historyMutex_);
    if (transitionHistory_.size() >= MAX_HISTORY_SIZE) {
        transitionHistory_.erase(transitionHistory_.begin());
    }
    transitionHistory_.push_back(log);
}

/**
 * @brief Retrieves the transition history
 * @description Returns a copy of the transition history with thread safety
 * @return Vector containing the history of state transitions
 */
std::vector<StateTransitionLog> StateTransitionManager::getTransitionHistory() const {
    std::lock_guard<std::mutex> lock(historyMutex_);
    return transitionHistory_;
}

/**
 * @brief Clears the transition history
 * @description Removes all recorded state transitions with thread safety
 */
void StateTransitionManager::clearHistory() {
    std::lock_guard<std::mutex> lock(historyMutex_);
    transitionHistory_.clear();
}

/**
 * @brief Constructs a LogManager
 * @description Initializes the logger with ROS2 node and opens the log file
 * @param node ROS2 node for logging functionality
 */
LogManager::LogManager(rclcpp::Node::SharedPtr node) 
    : logger_(node ? node->get_logger() : rclcpp::get_logger("default_logger")) {
    logFile_.open("robot_log.txt", std::ios::app);
}

/**
 * @brief Destructor
 * @description Ensures proper cleanup by closing the log file
 */
LogManager::~LogManager() {
    if (logFile_.is_open()) {
        logFile_.close();
    }
}

/**
 * @brief Logs a message with specified level and context
 * @description Records a log message with thread safety and writes to both file and ROS
 * @param level Severity level of the log message
 * @param component Component generating the log
 * @param message Content of the log message
 * @param context Additional context information for the log
 */
void LogManager::log(LogLevel level, const std::string& component, 
                    const std::string& message, 
                    const std::map<std::string, std::string>& context) {
    LogMessage logMsg{
        level,
        component,
        message,
        rclcpp::Clock().now(),
        context
    };

    logToFile(logMsg);
    logToROS(logMsg);
    
    std::lock_guard<std::mutex> lock(logMutex_);
    if (logHistory_.size() >= MAX_LOG_SIZE) {
        logHistory_.erase(logHistory_.begin());
    }
    logHistory_.push_back(logMsg);
}

/**
 * @brief Writes a log message to file
 * @description Formats and writes the log message to the log file
 * @param msg The log message to write
 */
void LogManager::logToFile(const LogMessage& msg) {
    if (logFile_.is_open()) {
        logFile_ << formatLogMessage(msg) << std::endl;
    }
}

/**
 * @brief Logs a message to ROS
 * @description Sends the log message to ROS with appropriate severity level
 * @param msg The log message to send
 */
void LogManager::logToROS(const LogMessage& msg) {
    switch(msg.level) {
        case LogLevel::DEBUG:
            RCLCPP_DEBUG(logger_, "%s", formatLogMessage(msg).c_str());
            break;
        case LogLevel::INFO:
            RCLCPP_INFO(logger_, "%s", formatLogMessage(msg).c_str());
            break;
        case LogLevel::WARNING:
            RCLCPP_WARN(logger_, "%s", formatLogMessage(msg).c_str());
            break;
        case LogLevel::ERROR:
            RCLCPP_ERROR(logger_, "%s", formatLogMessage(msg).c_str());
            break;
        case LogLevel::FATAL:
            RCLCPP_FATAL(logger_, "%s", formatLogMessage(msg).c_str());
            break;
    }
}

/**
 * @brief Formats a log message for output
 * @description Creates a formatted string representation of the log message
 * @param msg The log message to format
 * @return Formatted string containing timestamp, component, level, message and context
 */
std::string LogManager::formatLogMessage(const LogMessage& msg) {
    std::stringstream ss;
    ss << "[" << msg.timestamp.seconds() << "] "
       << "[" << msg.component << "] "
       << "[" << toString(msg.level) << "] "
       << msg.message;
    
    if (!msg.context.empty()) {
        ss << " Context: ";
        for (const auto& [key, value] : msg.context) {
            ss << key << "=" << value << " ";
        }
    }
    return ss.str();
}

/**
 * @brief Converts a log level to string
 * @description Returns the string representation of a log level
 * @param level The log level to convert
 * @return String representation of the log level
 */
std::string LogManager::toString(LogLevel level) {
    switch(level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARNING: return "WARNING";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::FATAL: return "FATAL";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Retrieves the log history
 * @description Returns a copy of the log history with thread safety
 * @return Vector containing the history of log messages
 */
std::vector<LogMessage> LogManager::getLogHistory() const {
    std::lock_guard<std::mutex> lock(logMutex_);
    return logHistory_;
}

/**
 * @brief Clears the log history
 * @description Removes all recorded log messages with thread safety
 */
void LogManager::clearHistory() {
    std::lock_guard<std::mutex> lock(logMutex_);
    logHistory_.clear();
}

} // end namespace AMR 