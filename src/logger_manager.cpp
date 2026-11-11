#include "spdlog_ros/logger_manager.hpp"

#include <string>
#include <chrono>
#include <ctime>
#include <unordered_map>
#include <mutex>
#include <stdexcept>

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

namespace spdlog_ros
{

// Definition of the static member
std::shared_ptr<LoggerManager> LoggerManager::instance_ = nullptr;

void LoggerManager::CreateLoggerManager(const std::string& base_logger_name)
{
  // Create LoggerManager only if not yet existing such that multiple calls do not have any side effects
  static std::mutex create_logger_mutex;
  const std::lock_guard<std::mutex> lock(create_logger_mutex);
  if (!instance_)
  {
    instance_ = std::shared_ptr<LoggerManager>(new LoggerManager(base_logger_name));
  }
}

std::shared_ptr<LoggerManager> LoggerManager::GetLoggerManager()
{
  if (!instance_)
  {
    throw std::runtime_error("LoggerManager instance not created yet. Call CreateLoggerManager(...) first.");
  }
  return instance_;
}

LoggerManager::LoggerManager(const std::string& base_logger_name)
  : base_logger_name_(base_logger_name)
{
  spdlog::init_thread_pool(32768, 1); // queue with max 32k items 1 backing thread.

  createDefaultSinks();
}

std::string LoggerManager::getFullLoggerName(const std::string& name)
{
  std::string full_logger_name = "";

  if (!base_logger_name_.empty())
  {
    full_logger_name += base_logger_name_;
  }

  if (!base_logger_name_.empty() && !name.empty())
  {
    full_logger_name += SPDLOG_ROS_LOGGER_DELIMITER;
  }

  if (!name.empty())
  {
    full_logger_name += name;
  }

  return full_logger_name;
}

std::string LoggerManager::getLoggerNameFromFullName(const std::string& full_logger_name)
{
  // remove the base logger name part from the full logger name
  if (full_logger_name.find(base_logger_name_) == 0)
  {
    std::string remaining = full_logger_name.substr(base_logger_name_.size());
    // Remove the leading delimiter if present
    if (!remaining.empty() && remaining.find(SPDLOG_ROS_LOGGER_DELIMITER) == 0)
    {
      remaining.erase(0, std::string(SPDLOG_ROS_LOGGER_DELIMITER).length());
    }
    return remaining;
  }
  return full_logger_name;
}

std::vector<std::string> LoggerManager::splitLoggerName(const std::string& name, std::string delimiter)
{
  std::vector<std::string> parts;
  size_t start = 0;
  size_t end = name.find(delimiter);
  while (end != std::string::npos)
  {
    parts.push_back(name.substr(start, end - start));
    start = end + delimiter.length();
    end = name.find(delimiter, start);
  }
  parts.push_back(name.substr(start));
  return parts;
}

std::string LoggerManager::getParentLoggerName(const std::string& name)
{
  auto parts = splitLoggerName(name);
  if (parts.size() <= 1)
  {
    return "";
  }
  std::string parent = parts[0];
  for (size_t i = 1; i < parts.size() - 1; ++i)
  {
    parent += SPDLOG_ROS_LOGGER_DELIMITER + parts[i];
  }
  return parent;
}

void LoggerManager::createDefaultSinks()
{
  default_sinks_.clear();

  // add console sink
  static auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  default_sinks_.push_back(console_sink);

  // add file sink as YYYY-MM-DD_HH-MM-SS and set that as file path

  // Check for environment variable SPDLOG_ROS_LOGFILES
  // If not set, fallback to $HOME/logfiles
  // If $HOME is not set, fallback to ./logfiles
  const char* spdlog_env = std::getenv("SPDLOG_ROS_LOGFILES");
  std::string log_dir;
  if (spdlog_env != nullptr)
  {
    log_dir = spdlog_env;
  }
  else
  {
    // Fallback to $HOME/logfiles
    const char* home_env = std::getenv("HOME");
    if (home_env != nullptr)
    {
      log_dir = std::string(home_env) + "/logfiles";
    }
    else
    {
      // Fallback if HOME is not defined (relative to where the executable is started)
      log_dir = "logfiles";
    }
  }

  // Get the current time and format it properly
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::tm buf;
#ifdef _WIN32
  gmtime_s(&buf, &in_time_t);
#else
  gmtime_r(&in_time_t, &buf);
#endif
  char time_str[21];
  std::strftime(time_str, sizeof(time_str), "%Y-%m-%d_%H-%M-%SZ", &buf);

  // Replace the possible slashes and dots of the base_logger_name with underscores
  std::string cleaned_base_logger_name = base_logger_name_;
  std::replace(cleaned_base_logger_name.begin(), cleaned_base_logger_name.end(), '.', '_');
  std::replace(cleaned_base_logger_name.begin(), cleaned_base_logger_name.end(), '/', '_');
  // If there is one or multiple underscores at the beginning now, remove it
  cleaned_base_logger_name.erase(0, cleaned_base_logger_name.find_first_not_of('_'));

  // Create the full log file path
  // Example: /home/user/logfiles/my_logger_2023-10-05_14-30-00Z.log
  // Note that if no base logger name is set (yet), the file name is just "_yyyy-mm-ddThh:mm:ssZ.log"
  std::string log_file_path = log_dir + "/" + cleaned_base_logger_name + "_" + std::string(time_str) + ".log";
  static auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file_path, true);
  default_sinks_.push_back(file_sink);
}

bool LoggerManager::addSinkToDefaultSinks(spdlog::sink_ptr sink)
{
  auto it = std::find(default_sinks_.begin(), default_sinks_.end(), sink);
  if (it != default_sinks_.end())
  {
    return false;
  }

  default_sinks_.push_back(sink);
  return true;
}

std::shared_ptr<spdlog::logger> LoggerManager::createAsyncLogger(
  const std::string& name, std::vector<spdlog::sink_ptr> sinks, bool add_default_sinks)
{
  if (add_default_sinks)
  {
    for (const auto& sink : default_sinks_)
    {
      auto it = std::find(sinks.begin(), sinks.end(), sink);
      if (it == sinks.end())
      {
        sinks.push_back(sink);
      }
    }
  }

  return std::make_shared<spdlog::async_logger>(
    getFullLoggerName(name),
    sinks.begin(), sinks.end(),
    spdlog::thread_pool(),
    spdlog::async_overflow_policy::overrun_oldest);
}

void LoggerManager::shutdownLogging()
{
  const std::lock_guard<std::mutex> lock(logger_map_mutex_);

  logger_map_.clear();
  default_sinks_.clear();
  spdlog::drop_all();
  spdlog::shutdown();
}

std::shared_ptr<spdlog::logger> LoggerManager::getLogger(
  const std::string& name, bool create_if_not_existing, bool add_default_sinks)
{
  const std::lock_guard<std::mutex> lock(logger_map_mutex_);

  return getLoggerNoLock(name, create_if_not_existing, add_default_sinks);
}

std::shared_ptr<spdlog::logger> LoggerManager::getLoggerNoLock(
  const std::string& name, bool create_if_not_existing, bool add_default_sinks)
{
  std::shared_ptr<spdlog::logger> logger = nullptr;

  const std::string full_logger_name = getFullLoggerName(name);

  const auto logger_search_it = logger_map_.find(full_logger_name);
  if (logger_search_it != logger_map_.end())
  {
    logger = logger_search_it->second.logger;
  }
  else if (create_if_not_existing)
  {
    // Create the logger including all parent loggers if they do not exist yet
    std::vector<std::string> logger_parts = splitLoggerName(full_logger_name);
    std::string current_logger_name;
    std::shared_ptr<spdlog::logger> parent_logger;
    for (size_t i = 0; i < logger_parts.size(); ++i)
    {
      current_logger_name += (i == 0 ? "" : SPDLOG_ROS_LOGGER_DELIMITER) + logger_parts[i];

      const auto current_logger_it = logger_map_.find(current_logger_name);
      if (current_logger_it == logger_map_.end())
      {
        const auto current_logger = createAsyncLogger(
          getLoggerNameFromFullName(current_logger_name), {}, add_default_sinks);
        spdlog::initialize_logger(current_logger);
        // If a child logger level is unset, it should inherit the level from its parent logger
        // Note that the info level is considered the "unset" level for child loggers since this is the default level
        // for spdlog if not set otherwise:
        // https://github.com/gabime/spdlog/blob/8e5613379f5140fefb0b60412fbf1f5406e7c7f8/include/spdlog/details/registry.h#L114
        // https://github.com/gabime/spdlog/blob/8e5613379f5140fefb0b60412fbf1f5406e7c7f8/include/spdlog/cfg/helpers-inl.h#L81
        if (parent_logger && current_logger->level() == spdlog::level::info)
        {
          current_logger->set_level(parent_logger->level());
        }

        logger_map_[current_logger_name] = {current_logger, {}};
      }

      parent_logger = logger_map_[current_logger_name].logger;
    }

    logger = parent_logger;
  }

  return logger;
}

bool LoggerManager::setLoggerLevel(const std::string& name, spdlog::level::level_enum level)
{
  const std::lock_guard<std::mutex> lock(logger_map_mutex_);

  const auto logger_search_it = logger_map_.find(name);
  if (logger_search_it != logger_map_.end())
  {
    logger_search_it->second.logger->set_level(level);

    // go through all locations and enable/disable them depending on the log level there and the new log level
    // There are three things to note:
    // 1. This can be many many locations but as this happens only when the log level is changed compared to vice
    //    versa at each log checking if it should be logged, this is very much faster (except exactly at that point
    //    when the logger level changes)
    // 2. Technically there is a race condition here that the logger level is above changed but below is not yet
    //    changed and without a mutex in each logger statement after SPDLOG_ROS_LOGGING_ENABLED, it could be that
    //    a few log messages are lost (the other way around that there are too many cannot be the case because even
    //    when here it would still be enabled but actually not, later at logging the log messages are anyway discarded
    //    from spdlog (but only later meaning a higher performance loss)). However this is neglectable because the
    //    performance gain is much more important than a few lost log messages.
    // 3. Technically it can also be that in the meanwhile when the write at location_it->enabled is done that a read
    //    happens (most likely because of out-of-order execution from the CPU) but because of the same reason than
    //    above, this is no issue and std::atomic<bool> is probably not worth the effort
    for (auto&& location_it : logger_search_it->second.logger_locations)
    {
      location_it->enabled = logger_search_it->second.logger->should_log(location_it->level);
    }

    return true;
  }
  else
  {
    // Logger with the given name not found -> create it and set the default value such that later when it is actually
    // used, the logger will have the correct level
    const auto logger = getLoggerNoLock(
      getLoggerNameFromFullName(name), true, true);
    logger->set_level(level);

    return true;
  }
  return false;
}

std::unordered_map<std::string, spdlog::level::level_enum> LoggerManager::getLoggerLevels()
{
  std::unordered_map<std::string, spdlog::level::level_enum> logger_levels;

  const std::lock_guard<std::mutex> lock(logger_map_mutex_);

  for (const auto& logger : logger_map_)
  {
    logger_levels[logger.first] = logger.second.logger->level();
  }

  return logger_levels;
}

std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)>& LoggerManager::getTimePointCallback()
{
  if (!clock_callback_)
  {
    // The default clock callback uses std::chrono to get the current time in nanoseconds since epoch
    // The clock callback can be overridden by the user to use a different clock source (e.g. ROS time)
    clock_callback_ =
      [](spdlog_ros_utils_time_point_value_t * time_point) -> spdlog_ros_utils_ret_t
      {
        try
        {
          auto now = std::chrono::system_clock::now();
          *time_point = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        }
        catch (...)
        {
          return SPDLOG_ROS_UTILS_RET_ERROR;
        }
        return SPDLOG_ROS_UTILS_RET_OK;
      };
  }

  return clock_callback_;
}

void LoggerManager::setTimePointCallback(
    std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)> clock_callback)
{
  clock_callback_ = clock_callback;
}

void LoggerManager::initializeLogLocation(
  LoggerLocation* log_location, const std::string& name, spdlog::level::level_enum level)
{
  const std::lock_guard<std::mutex> lock(logger_map_mutex_);

  log_location->initialized = true;
  log_location->enabled = getLoggerNoLock(name)->should_log(level);
  log_location->level = level;

  const std::string full_logger_name = getFullLoggerName(name);

  logger_map_[full_logger_name].logger_locations.push_back(log_location);
}

void LoggerManager::setLogLocationLevel(LoggerLocation* log_location, spdlog::level::level_enum level)
{
  const std::lock_guard<std::mutex> lock(logger_map_mutex_);

  log_location->level = level;
}

void LoggerManager::checkLogLocationEnabled(LoggerLocation* log_location, const std::string& name)
{
  const std::lock_guard<std::mutex> lock(logger_map_mutex_);

  log_location->enabled = getLoggerNoLock(name)->should_log(log_location->level);
}

}  // namespace spdlog_ros