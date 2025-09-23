//
// Created by Fabian Hirmann on 17.09.2025.
//

#pragma once

#include <mutex>
#include <unordered_map>
#include <string>
#include <vector>
#include <memory>

#include "spdlog_ros/logging_util_macros.hpp"

#include "spdlog/logger.h"

namespace spdlog_ros
{

class Logger
{
public:
  Logger(const Logger& other) = delete;
  Logger& operator=(const Logger& other) = delete;

  static void CreateRootLogger(const std::string& root_logger_name);

  static std::shared_ptr<Logger> GetInstance();

  bool addSinkToDefaultSinks(spdlog::sink_ptr sink);

  std::shared_ptr<spdlog::logger> createAsyncLogger(const std::string& name,
                                                    std::vector<spdlog::sink_ptr> sinks = {},
                                                    bool add_default_sinks = true);

  std::shared_ptr<spdlog::logger> getLogger(const std::string& name,
                                            bool create_if_not_existing = true,
                                            bool add_default_sinks = true);

  std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)>& getTimePointCallback();

  void setTimePointCallback(
    std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)> clock_callback);
  
  bool setLoggerLevel(const std::string& name, spdlog::level::level_enum level);

  std::unordered_map<std::string, spdlog::level::level_enum> getLoggerLevels();

private:
  Logger(const std::string& root_logger_name);

  std::string getFullLoggerName(const std::string& name);

  void createDefaultSinks();

  static std::shared_ptr<Logger> instance_;

  std::string root_logger_name_ = "";

  std::vector<spdlog::sink_ptr> default_sinks_;

  std::unordered_map<std::string, std::shared_ptr<spdlog::logger>> logger_map_;
  std::mutex logger_map_mutex_;

  std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)> clock_callback_;
};

}  // namespace spdlog_ros