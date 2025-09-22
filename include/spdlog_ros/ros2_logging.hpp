//
// Created by Fabian Hirmann on 19.09.2025.
//

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <spdlog_ros/srv/get_loggers.hpp>
#include <spdlog_ros/srv/set_logger_level.hpp>

namespace spdlog_ros
{

bool GetLoggersCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<spdlog_ros::srv::GetLoggers::Request> request,
  const std::shared_ptr<spdlog_ros::srv::GetLoggers::Response> response);

bool SetLoggerLevelCallback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Request> request,
  const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Response> response);

void SetUpROSLogging(rclcpp::Node::SharedPtr node);

}  // namespace spdlog_ros