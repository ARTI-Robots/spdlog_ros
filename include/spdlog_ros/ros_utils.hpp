//
// Created by Fabian Hirmann on 19.09.2025.
//

#pragma once

#include <spdlog/logger.h>

#include <rosgraph_msgs/Log.h>

namespace spdlog_ros
{

inline static int8_t ConvertSeverityToROS(spdlog::level::level_enum level)
{
  switch (level)
  {
  case spdlog::level::trace:
    return rosgraph_msgs::Log::DEBUG;
  case spdlog::level::debug:
    return rosgraph_msgs::Log::DEBUG;
  case spdlog::level::info:
    return rosgraph_msgs::Log::INFO;
  case spdlog::level::warn:
    return rosgraph_msgs::Log::WARN;
  case spdlog::level::err:
    return rosgraph_msgs::Log::ERROR;
  case spdlog::level::critical:
    return rosgraph_msgs::Log::FATAL;
  default:
    return 0;
  }
}

inline static std::string ConvertROSLogLevelToString(int8_t level)
{
  switch (level)
  {
  case rosgraph_msgs::Log::DEBUG:
    return "debug";
  case rosgraph_msgs::Log::INFO:
    return "info";
  case rosgraph_msgs::Log::WARN:    
    return "warn";
  case rosgraph_msgs::Log::ERROR:
    return "error";
  case rosgraph_msgs::Log::FATAL:
    return "fatal";
  default:
    return "";
  }
}

inline static spdlog::level::level_enum ConvertROSLogLevelToSpdlog(int8_t level)
{
  switch (level)
  {
  case rosgraph_msgs::Log::DEBUG:
    return spdlog::level::debug;
  case rosgraph_msgs::Log::INFO:
    return spdlog::level::info;
  case rosgraph_msgs::Log::WARN:    
    return spdlog::level::warn;
  case rosgraph_msgs::Log::ERROR:
    return spdlog::level::err;
  case rosgraph_msgs::Log::FATAL:
    return spdlog::level::critical;
  default:
    return spdlog::level::off;
  }
}

inline static int8_t ConvertStringToROSLogLevel(const std::string& level)
{
  if (level == "debug")
  {
    return rosgraph_msgs::Log::DEBUG;
  }
  else if (level == "info")
  {
    return rosgraph_msgs::Log::INFO;
  }
  else if (level == "warn")
  {
    return rosgraph_msgs::Log::WARN;
  }
  else if (level == "error")
  {
    return rosgraph_msgs::Log::ERROR;
  }
  else if (level == "fatal")
  {
    return rosgraph_msgs::Log::FATAL;
  }
  return 0;
}

}  // namespace spdlog_ros