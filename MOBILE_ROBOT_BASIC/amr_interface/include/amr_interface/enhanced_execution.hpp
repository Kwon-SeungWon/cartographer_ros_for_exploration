#pragma once

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <chrono>
#include <memory>
#include <algorithm>
#include <string>
#include <stdexcept>

namespace AMR {

/**
 * @brief Enumeration representing different states of the robot
 * @description Defines all possible states that the robot can be in during operation
 */
enum class RobotState {
    INIT,           ///< Initial state when robot starts
    IDLE,           ///< Robot is idle and waiting for commands
    AUTO,           ///< Robot is in autonomous navigation mode
    SLAM,           ///< Robot is performing SLAM operations
    DOCKING,        ///< Robot is in the process of docking
    UNDOCKING,      ///< Robot is in the process of undocking
    CHARGING,       ///< Robot is currently charging
    MANIPULATION,   ///< Robot is performing manipulation tasks
    EMERGENCY,      ///< Robot is in emergency state
    STOP            ///< Robot is stopped
};

/**
 * @brief Structure for logging state transitions
 * @description Contains information about state transitions including timestamps and context
 */
struct StateTransitionLog {
    RobotState fromState;    ///< Previous state
    RobotState toState;      ///< New state
    rclcpp::Time timestamp;  ///< Time of transition
    std::string reason;      ///< Reason for transition
    std::map<std::string, std::string> context;  ///< Additional context information
};

/**
 * @brief Enumeration for log levels
 * @description Defines different severity levels for logging
 */
enum class LogLevel {
    DEBUG,      ///< Debug level messages
    INFO,       ///< Information level messages
    WARNING,    ///< Warning level messages
    ERROR,      ///< Error level messages
    FATAL       ///< Fatal level messages
};

/**
 * @brief Structure for log messages
 * @description Contains all information needed for a log entry
 */
struct LogMessage {
    LogLevel level;                          ///< Severity level of the log
    std::string component;                   ///< Component generating the log
    std::string message;                     ///< Log message content
    rclcpp::Time timestamp;                  ///< Time of the log entry
    std::map<std::string, std::string> context;  ///< Additional context information
};

/**
 * @brief Manages state transitions and their history
 * @description Handles validation of state transitions and maintains a history of transitions
 */
class StateTransitionManager {
public:
    /**
     * @brief Validates if a state transition is allowed
     * @param from The current state
     * @param to The target state
     * @return true if transition is valid, false otherwise
     */
    bool validateTransition(RobotState from, RobotState to);

    /**
     * @brief Logs a state transition
     * @param log The transition log to record
     */
    void logTransition(const StateTransitionLog& log);

    /**
     * @brief Retrieves the transition history
     * @return Vector of transition logs
     */
    std::vector<StateTransitionLog> getTransitionHistory() const;

    /**
     * @brief Clears the transition history
     */
    void clearHistory();

private:
    static constexpr size_t MAX_HISTORY_SIZE = 1000;  ///< Maximum number of transitions to store
    std::vector<StateTransitionLog> transitionHistory_;  ///< History of state transitions
    mutable std::mutex historyMutex_;  ///< Mutex for thread safety
};

/**
 * @brief Manages logging functionality
 * @description Handles logging to both file and ROS, with different severity levels
 */
class LogManager {
public:
    /**
     * @brief Constructs a LogManager
     * @param node ROS2 node for logging
     */
    explicit LogManager(rclcpp::Node::SharedPtr node);

    /**
     * @brief Destructor
     * @description Ensures log file is properly closed
     */
    ~LogManager();

    /**
     * @brief Logs a message with specified level and context
     * @param level Severity level of the log
     * @param component Component generating the log
     * @param message Log message content
     * @param context Additional context information
     */
    void log(LogLevel level, const std::string& component, 
             const std::string& message, 
             const std::map<std::string, std::string>& context = {});

    /**
     * @brief Retrieves the log history
     * @return Vector of log messages
     */
    std::vector<LogMessage> getLogHistory() const;

    /**
     * @brief Clears the log history
     */
    void clearHistory();

private:
    static constexpr size_t MAX_LOG_SIZE = 1000;  ///< Maximum number of logs to store
    rclcpp::Logger logger_;  ///< ROS2 logger
    std::ofstream logFile_;  ///< Log file stream
    std::vector<LogMessage> logHistory_;  ///< History of log messages
    mutable std::mutex logMutex_;  ///< Mutex for thread safety

    /**
     * @brief Writes a log message to file
     * @param msg The log message to write
     */
    void logToFile(const LogMessage& msg);

    /**
     * @brief Logs a message to ROS
     * @param msg The log message to write
     */
    void logToROS(const LogMessage& msg);

    /**
     * @brief Formats a log message for output
     * @param msg The log message to format
     * @return Formatted string representation of the log message
     */
    std::string formatLogMessage(const LogMessage& msg);

    /**
     * @brief Converts a log level to string
     * @param level The log level to convert
     * @return String representation of the log level
     */
    static std::string toString(LogLevel level);
};

} // end namespace AMR 