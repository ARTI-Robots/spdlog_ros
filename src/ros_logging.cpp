//
// Created by Fabian Hirmann on 19.09.2025.
//

#include "spdlog_ros/ros_logging.hpp"

#include <spdlog/logger.h>
#include <spdlog/cfg/env.h>

#include "ros/node_handle.h"
#include "spdlog_ros/logger_manager.hpp"
#include "spdlog_ros/ros_sink.hpp"
#include "spdlog_ros/ros_get_time_point.h"
#include "spdlog_ros/ros_utils.hpp"

#include <ros/ros.h>

#include <roscpp/GetLoggers.h>
#include <roscpp/SetLoggerLevel.h>
#include <rosgraph_msgs/Log.h>

namespace spdlog_ros
{

bool GetLoggersCallback(roscpp::GetLoggers::Request& /* req */, roscpp::GetLoggers::Response& res)
{
  res.loggers.clear();

  for (const auto& logger : spdlog_ros::LoggerManager::GetLoggerManager()->getLoggerLevels())
  {
    roscpp::Logger logger_msg;
    logger_msg.name = logger.first;
    logger_msg.level =
      spdlog_ros::ConvertROSLogLevelToString(spdlog_ros::ConvertSeverityToROS(logger.second));
    res.loggers.push_back(logger_msg);
  }
  return true;
}

bool SetLoggerLevelCallback(roscpp::SetLoggerLevel::Request& req, roscpp::SetLoggerLevel::Response& /* res */)
{
  spdlog_ros::LoggerManager::GetLoggerManager()->setLoggerLevel(
    req.logger, spdlog_ros::ConvertROSLogLevelToSpdlog(spdlog_ros::ConvertStringToROSLogLevel(req.level)));

  return true;
}

void SetUpROSLogging()
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
  spdlog_ros::LoggerManager::CreateLoggerManager(ros::this_node::getName());

  // Set up spdlog_ros to use the ROS time (instead of the default std::chrono time)
  spdlog_ros::UseROSTime();

  // Use the private node handle to create the ROS sink and services in the private namespace
  ros::NodeHandle nh_priv("~");

  // Create a ROS sink
  auto ros_sink = std::make_shared<spdlog_ros::RosSink>(nh_priv);

  // The default sinks are stdout/stderr and file logging
  // When adding here a default sink, all other loggers will have that sink
  spdlog_ros::LoggerManager::GetLoggerManager()->addSinkToDefaultSinks(ros_sink);

  // Create the services to get and set the logger levels at runtime
  static auto get_loggers_srv = 
    nh_priv.advertiseService("spdlog_ros/get_loggers", &spdlog_ros::GetLoggersCallback);
  static auto set_logger_level_srv =
    nh_priv.advertiseService("spdlog_ros/set_logger_level", &spdlog_ros::SetLoggerLevelCallback);
}

}  // namespace spdlog_ros