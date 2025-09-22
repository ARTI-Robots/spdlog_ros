//
// Created by Fabian Hirmann on 19.09.2025.
//

#include "spdlog_ros/ros2_logging.hpp"

#include <spdlog/logger.h>

#include <rclcpp/rclcpp.hpp>

#include "spdlog_ros/logger.hpp"
#include "spdlog_ros/ros2_sink.hpp"
#include "spdlog_ros/ros2_get_time_point.h"
#include "spdlog_ros/ros2_utils.hpp"

#include <spdlog_ros/msg/logger.hpp>

namespace spdlog_ros
{

bool GetLoggersCallback(
  const std::shared_ptr<rmw_request_id_t> /* request_header */,
  const std::shared_ptr<spdlog_ros::srv::GetLoggers::Request> /* request */,
  const std::shared_ptr<spdlog_ros::srv::GetLoggers::Response> response)
{
  response->loggers.clear();

  const std::lock_guard<std::mutex> lock(GetLoggerMapMutex());

  for (const auto& logger : spdlog_ros::GetLoggerMap())
  {
    spdlog_ros::msg::Logger logger_msg;
    logger_msg.name = logger.first;
    logger_msg.level =
      spdlog_ros::ConvertROSLogLevelToString(spdlog_ros::ConvertSeverityToROS(logger.second->level()));
    response->loggers.push_back(logger_msg);
  }
  return true;
}

bool SetLoggerLevelCallback(
  const std::shared_ptr<rmw_request_id_t> /* request_header */,
  const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Request> request,
  const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Response> /* response */)
{
  const std::lock_guard<std::mutex> lock(GetLoggerMapMutex());
  auto it = spdlog_ros::GetLoggerMap().find(request->logger);
  if (it != spdlog_ros::GetLoggerMap().end())
  {
    auto logger = it->second;
    logger->set_level(spdlog_ros::ConvertROSLogLevelToSpdlog(spdlog_ros::ConvertStringToROSLogLevel(request->level)));
  }

  return true;
}

void SetUpROSLogging(rclcpp::Node::SharedPtr node)
{
  // Set the root logger name, all loggers will be prefixed with this name
  // Note that the file name for logging is "~/logfiles/{root_logger_name}_yyyy-mm-ddThh:mm:ssZ.log")
  spdlog_ros::SetRootLoggerName(node->get_name());

  // Set up spdlog_ros to use the ROS time (instead of the default std::chrono time)
  spdlog_ros::UseROSTime(node->get_clock());

  // Create a ROS sink
  auto ros_sink = std::make_shared<spdlog_ros::RosSink>(node);

  // The default sinks are stdout/stderr and file logging
  // When adding here a default sink, all other loggers will have that sink
  spdlog_ros::AddSinkToDefaultSinks(ros_sink);

  // Create the services to get and set the logger levels at runtime
  static auto get_loggers_srv = 
    node->create_service<spdlog_ros::srv::GetLoggers>(
      "~/spdlog_ros/get_loggers", &spdlog_ros::GetLoggersCallback);
  static auto set_logger_level_srv =
    node->create_service<spdlog_ros::srv::SetLoggerLevel>(
      "~/spdlog_ros/set_logger_level", &spdlog_ros::SetLoggerLevelCallback);
}

}  // namespace spdlog_ros