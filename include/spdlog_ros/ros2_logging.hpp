//
// Created by Fabian Hirmann on 19.09.2025.
//

#pragma once

#include <rclcpp/rclcpp.hpp>

// Note that the target version is ROS 2 Humble which does not yet have integrated services and messages for logging
// Beginning with ROS 2 Iron and upward, the existing service definitions of rcl_interfaces could be used:
// https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/README.md#services
#include <spdlog_ros/srv/get_loggers.hpp>
#include <spdlog_ros/srv/set_logger_level.hpp>

namespace spdlog_ros
{

class ROSLoggingManager
{
public:
  ROSLoggingManager(const ROSLoggingManager& other) = delete;
  ROSLoggingManager& operator=(const ROSLoggingManager& other) = delete;
  ROSLoggingManager(rclcpp::Node::SharedPtr node);
  ~ROSLoggingManager();

private:
  bool getLoggersCallback(
    const std::shared_ptr<spdlog_ros::srv::GetLoggers::Request> request,
    const std::shared_ptr<spdlog_ros::srv::GetLoggers::Response> response);

  bool setLoggerLevelCallback(
    const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Request> request,
    const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Response> response);

  void setUpROSLogging(rclcpp::Node::SharedPtr node);

  rclcpp::Service<spdlog_ros::srv::GetLoggers>::SharedPtr get_loggers_srv_;
  rclcpp::Service<spdlog_ros::srv::SetLoggerLevel>::SharedPtr set_logger_level_srv_;

  static size_t reference_count_;
};

}  // namespace spdlog_ros