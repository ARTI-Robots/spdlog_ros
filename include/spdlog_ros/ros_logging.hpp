//
// Created by Fabian Hirmann on 19.09.2025.
//

#pragma once

#include "ros/service_server.h"
#include <roscpp/GetLoggers.h>
#include <roscpp/SetLoggerLevel.h>

namespace spdlog_ros
{

class ROSLoggingManager
{
public:
  ROSLoggingManager(const ROSLoggingManager& other) = delete;
  ROSLoggingManager& operator=(const ROSLoggingManager& other) = delete;
  ROSLoggingManager();
  ~ROSLoggingManager();

private:
  bool getLoggersCallback(roscpp::GetLoggers::Request& req, roscpp::GetLoggers::Response& res);

  bool setLoggerLevelCallback(roscpp::SetLoggerLevel::Request& req, roscpp::SetLoggerLevel::Response& res);

  void setUpROSLogging();

  ros::ServiceServer get_loggers_srv_;
  ros::ServiceServer set_logger_level_srv_;

  static size_t reference_count_;
};

}  // namespace spdlog_ros