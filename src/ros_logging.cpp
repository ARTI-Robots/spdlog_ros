//
// Created by Fabian Hirmann on 19.09.2025.
//

#pragma once

#include "spdlog_ros/ros_logging.hpp"

#include <spdlog/logger.h>

#include "ros/node_handle.h"
#include "spdlog_ros/logger.hpp"
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

  const std::lock_guard<std::mutex> lock(GetLoggerMapMutex());

  for (const auto& logger : spdlog_ros::GetLoggerMap())
  {
    roscpp::Logger logger_msg;
    logger_msg.name = logger.first;
    logger_msg.level =
      spdlog_ros::ConvertROSLogLevelToString(spdlog_ros::ConvertSeverityToROS(logger.second->level()));
    res.loggers.push_back(logger_msg);
  }
  return true;
}

bool SetLoggerLevelCallback(roscpp::SetLoggerLevel::Request& req, roscpp::SetLoggerLevel::Response& /* res */)
{
  const std::lock_guard<std::mutex> lock(GetLoggerMapMutex());
  auto it = spdlog_ros::GetLoggerMap().find(req.logger);
  if (it != spdlog_ros::GetLoggerMap().end())
  {
    auto logger = it->second;
    logger->set_level(spdlog_ros::ConvertROSLogLevelToSpdlog(spdlog_ros::ConvertStringToROSLogLevel(req.level)));
  }

  return true;
}

void SetUpROSLogging()
{
  // Use the private node handle to create the ROS sink and services in the private namespace
  ros::NodeHandle nh_priv("~");

  // Set up spdlog_ros to use the ROS time (instead of the default std::chrono time)
  spdlog_ros::UseROSTime();

  // Create a ROS sink
  auto ros_sink = std::make_shared<spdlog_ros::RosSink>(nh_priv);

  // The default sinks are stdout/stderr and file logging
  // When adding here a default sink, all other loggers will have that sink
  spdlog_ros::AddSinkToDefaultSinks(ros_sink);

  // Create the services to get and set the logger levels at runtime
  static auto get_loggers_srv = 
    nh_priv.advertiseService("spdlog_ros/get_loggers", &spdlog_ros::GetLoggersCallback);
  static auto set_logger_level_srv =
    nh_priv.advertiseService("spdlog_ros/set_logger_level", &spdlog_ros::SetLoggerLevelCallback);
}

}  // namespace spdlog_ros