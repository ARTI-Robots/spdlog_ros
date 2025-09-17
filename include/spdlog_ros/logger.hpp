//
// Created by Fabian Hirmann on 17.09.2025.
//

#pragma once

#include "spdlog/logger.h"

namespace spdlog_ros
{

std::string& GetRootLoggerName();

void SetRootLoggerName(const std::string& name);

std::string GetFullLoggerName(const std::string& name);

std::vector<spdlog::sink_ptr>& GetDefaultSinks();

bool AddSinkToDefaultSinks(spdlog::sink_ptr sink);

std::shared_ptr<spdlog::logger> CreateAsyncLogger(const std::string& name,
                                                  std::vector<spdlog::sink_ptr> sinks = {},
                                                  bool add_default_sinks = true);

std::shared_ptr<spdlog::logger> GetLogger(const std::string& name,
                                          bool create_if_not_existing = true,
                                          bool add_default_sinks = true);

}  // namespace spdlog_ros