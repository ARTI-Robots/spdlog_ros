//
// Created by Fabian Hirmann on 17.09.2025.
//

#pragma once

#include <mutex>
#include <spdlog/common.h>
#include <unordered_map>
#include <string>
#include <vector>
#include <memory>

#include "spdlog_ros/logging_util_macros.hpp"

#include "spdlog/logger.h"

namespace spdlog_ros
{

class LoggerManager
{
public:

  struct LoggerLocation
  {
    LoggerLocation(bool initialized = false,
                   bool enabled = false,
                   spdlog::level::level_enum level = spdlog::level::debug)
      : initialized(initialized), enabled(enabled), level(level) {}

    bool initialized;
    bool enabled;
    spdlog::level::level_enum level;
  };

  struct LoggerEntry
  {
    std::shared_ptr<spdlog::logger> logger;
    std::vector<LoggerLocation*> logger_locations;
  };

  LoggerManager(const LoggerManager& other) = delete;
  LoggerManager& operator=(const LoggerManager& other) = delete;

  /**
   * @brief Initializes the singleton Logger Manager with a base name
   * 
   * @note The log file name replaces slashes and dots in the base name with underscores
   *       and prepends the current date and time. Multiple calls have no effect after
   *       the first initialization.
   *
   * @param base_logger_name The base logger name
   */
  static void CreateLoggerManager(const std::string& base_logger_name);

  static std::shared_ptr<LoggerManager> GetLoggerManager();

  bool addSinkToDefaultSinks(spdlog::sink_ptr sink);

  std::shared_ptr<spdlog::logger> createAsyncLogger(const std::string& name,
                                                    std::vector<spdlog::sink_ptr> sinks = {},
                                                    bool add_default_sinks = true);

  std::shared_ptr<spdlog::logger> getLogger(const std::string& name,
                                            bool create_if_not_existing = true,
                                            bool add_default_sinks = true);
  
  void shutdownLogging();

  std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)>& getTimePointCallback();

  void setTimePointCallback(
    std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)> clock_callback);
  
  bool setLoggerLevel(const std::string& name, spdlog::level::level_enum level);

  std::unordered_map<std::string, spdlog::level::level_enum> getLoggerLevels();

  void initializeLogLocation(LoggerLocation* logger_status, const std::string& name, spdlog::level::level_enum level);
  void setLogLocationLevel(LoggerLocation* logger_status, spdlog::level::level_enum level);
  void checkLogLocationEnabled(LoggerLocation* logger_status, const std::string& name);

private:
  LoggerManager(const std::string& base_logger_name);

  std::string getFullLoggerName(const std::string& name);

  void createDefaultSinks();

  std::shared_ptr<spdlog::logger> getLoggerNoLock(const std::string& name,
                                                  bool create_if_not_existing = true,
                                                  bool add_default_sinks = true);

  static std::shared_ptr<LoggerManager> instance_;

  std::string base_logger_name_ = "";

  std::vector<spdlog::sink_ptr> default_sinks_;

  std::unordered_map<std::string, LoggerEntry> logger_map_;
  std::mutex logger_map_mutex_;

  std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)> clock_callback_;
};

}  // namespace spdlog_ros