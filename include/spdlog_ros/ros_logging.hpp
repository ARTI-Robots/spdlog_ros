//
// Created by Fabian Hirmann on 19.09.2025.
//

#pragma once

#include <roscpp/GetLoggers.h>
#include <roscpp/SetLoggerLevel.h>

namespace spdlog_ros
{

bool GetLoggersCallback(roscpp::GetLoggers::Request& req, roscpp::GetLoggers::Response& res);

bool SetLoggerLevelCallback(roscpp::SetLoggerLevel::Request& req, roscpp::SetLoggerLevel::Response& res);

void SetUpROSLogging();

}  // namespace spdlog_ros