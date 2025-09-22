//
// Created by Fabian Hirmann on 19.09.2025.
//

#pragma once

#include <rcutils/logging.h>
#include <spdlog/logger.h>

namespace spdlog_ros
{

inline RCUTILS_LOG_SEVERITY ConvertSeverityToROS(spdlog::level::level_enum level)
{
  switch (level)
  {
  case spdlog::level::trace:
    return RCUTILS_LOG_SEVERITY_DEBUG;
  case spdlog::level::debug:
    return RCUTILS_LOG_SEVERITY_DEBUG;
  case spdlog::level::info:
    return RCUTILS_LOG_SEVERITY_INFO;
  case spdlog::level::warn:
    return RCUTILS_LOG_SEVERITY_WARN;
  case spdlog::level::err:
    return RCUTILS_LOG_SEVERITY_ERROR;
  case spdlog::level::critical:
    return RCUTILS_LOG_SEVERITY_FATAL;
  case spdlog::level::off:
    return RCUTILS_LOG_SEVERITY_UNSET;
  default:
    return RCUTILS_LOG_SEVERITY_UNSET;
  }
}

inline static std::string ConvertROSLogLevelToString(RCUTILS_LOG_SEVERITY level)
{
  switch (level)
  {
  case RCUTILS_LOG_SEVERITY_DEBUG:
    return "debug";
  case RCUTILS_LOG_SEVERITY_INFO:
    return "info";
  case RCUTILS_LOG_SEVERITY_WARN:    
    return "warn";
  case RCUTILS_LOG_SEVERITY_ERROR:
    return "error";
  case RCUTILS_LOG_SEVERITY_FATAL:
    return "fatal";
  default:
    return "";
  }
}

inline static spdlog::level::level_enum ConvertROSLogLevelToSpdlog(RCUTILS_LOG_SEVERITY level)
{
  switch (level)
  {
  case RCUTILS_LOG_SEVERITY_DEBUG:
    return spdlog::level::debug;
  case RCUTILS_LOG_SEVERITY_INFO:
    return spdlog::level::info;
  case RCUTILS_LOG_SEVERITY_WARN:    
    return spdlog::level::warn;
  case RCUTILS_LOG_SEVERITY_ERROR:
    return spdlog::level::err;
  case RCUTILS_LOG_SEVERITY_FATAL:
    return spdlog::level::critical;
  default:
    return spdlog::level::off;
  }
}

inline static RCUTILS_LOG_SEVERITY ConvertStringToROSLogLevel(const std::string& level)
{
  if (level == "debug")
  {
    return RCUTILS_LOG_SEVERITY_DEBUG;
  }
  else if (level == "info")
  {
    return RCUTILS_LOG_SEVERITY_INFO;
  }
  else if (level == "warn")
  {
    return RCUTILS_LOG_SEVERITY_WARN;
  }
  else if (level == "error")
  {
    return RCUTILS_LOG_SEVERITY_ERROR;
  }
  else if (level == "fatal")
  {
    return RCUTILS_LOG_SEVERITY_FATAL;
  }
  return RCUTILS_LOG_SEVERITY_UNSET;
}

}  // namespace spdlog_ros