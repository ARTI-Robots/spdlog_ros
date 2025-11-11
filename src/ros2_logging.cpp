//
// Created by Fabian Hirmann on 19.09.2025.
//

#include "spdlog_ros/ros2_logging.hpp"

#include <spdlog/logger.h>
#include <spdlog/cfg/env.h>

#include <rclcpp/rclcpp.hpp>

#include "spdlog_ros/logger_manager.hpp"
#include "spdlog_ros/ros2_sink.hpp"
#include "spdlog_ros/ros2_get_time_point.h"
#include "spdlog_ros/ros2_utils.hpp"

#include <spdlog_ros/msg/logger.hpp>

namespace spdlog_ros
{

size_t ROSLoggingManager::reference_count_ = 0;

ROSLoggingManager::ROSLoggingManager(rclcpp::Node::SharedPtr node)
{
  if (reference_count_ == 0)
  {
    // initialize once at the very first instantiation
    // subsequent instantiations just increase the reference count
    setUpROSLogging(node);

    auto ctx = rclcpp::contexts::get_global_default_context();
    if (ctx)
    {
      ctx->add_pre_shutdown_callback([]() {
        spdlog_ros::LoggerManager::GetLoggerManager()->shutdownLogging();
      });
    }
  }

  ++reference_count_;
}

ROSLoggingManager::~ROSLoggingManager()
{
  --reference_count_;

  if (reference_count_ == 0)
  {
    // shutdown once only at the very last destruction
    spdlog_ros::LoggerManager::GetLoggerManager()->shutdownLogging();
  }
}

bool ROSLoggingManager::getLoggersCallback(
  const std::shared_ptr<spdlog_ros::srv::GetLoggers::Request> /* request */,
  const std::shared_ptr<spdlog_ros::srv::GetLoggers::Response> response)
{
  response->loggers.clear();

  for (const auto& logger : spdlog_ros::LoggerManager::GetLoggerManager()->getLoggerLevels())
  {
    spdlog_ros::msg::Logger logger_msg;
    logger_msg.name = logger.first;
    logger_msg.level =
      spdlog_ros::ConvertROSLogLevelToString(spdlog_ros::ConvertSeverityToROS(logger.second));
    response->loggers.push_back(logger_msg);
  }
  return true;
}

bool ROSLoggingManager::setLoggerLevelCallback(
  const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Request> request,
  const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Response> /* response */)
{
  spdlog_ros::LoggerManager::GetLoggerManager()->setLoggerLevel(
    request->logger, spdlog_ros::ConvertROSLogLevelToSpdlog(spdlog_ros::ConvertStringToROSLogLevel(request->level)));

  return true;
}

void ROSLoggingManager::setUpROSLogging(rclcpp::Node::SharedPtr node)
{
  // Load log levels from the environment variable SPDLOG_LEVEL
  // Note that this works for all existing as well as future loggers BUT the log levels need to match the spdlog levels
  // and NOT the ROS log levels
  // Examples:
  //
  // set global level to debug:
  // export SPDLOG_LEVEL=debug
  //
  // turn off all logging except for node_name.named_logger1:
  // export SPDLOG_LEVEL="off,node_name.named_logger1=debug"
  //
  // turn off all logging except for node_name.named_logger1 (debug) and node_name.named_logger2 (info):
  // export SPDLOG_LEVEL="off,node_name.named_logger1=debug,node_name.named_logger2=info"
  spdlog::cfg::load_env_levels();

  // Set the base logger name, all loggers will be prefixed with this name
  // Note that the file name for logging is "~/logfiles/{cleaned_base_logger_name}_yyyy-mm-ddThh:mm:ssZ.log")
  // where cleaned_base_logger_name is the base logger name with leading slash removed and all other slashes
  // replaced with underscore
  spdlog_ros::LoggerManager::CreateLoggerManager(node->get_fully_qualified_name());

  // Set up spdlog_ros to use the ROS time (instead of the default std::chrono time)
  spdlog_ros::UseROSTime(node->get_clock());

  // Create a ROS sink
  auto ros_sink = std::make_shared<spdlog_ros::RosSink>(node);

  // The default sinks are stdout/stderr and file logging
  // When adding here a default sink, all other loggers will have that sink
  spdlog_ros::LoggerManager::GetLoggerManager()->addSinkToDefaultSinks(ros_sink);

  // Create the services to get and set the logger levels at runtime
  // Note that the target version is ROS 2 Humble which does not yet have integrated services and messages for logging
  // Beginning with ROS 2 Iron and upward, the existing service definitions of rcl_interfaces could be used:
  // https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/README.md#services
  get_loggers_srv_ = node->create_service<spdlog_ros::srv::GetLoggers>(
    "~/spdlog_ros/get_loggers",
    std::bind(&ROSLoggingManager::getLoggersCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_logger_level_srv_ = node->create_service<spdlog_ros::srv::SetLoggerLevel>(
    "~/spdlog_ros/set_logger_level",
    std::bind(&ROSLoggingManager::setLoggerLevelCallback, this, std::placeholders::_1, std::placeholders::_2));
}

}  // namespace spdlog_ros