#include "spdlog_ros/logger.hpp"

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
std::shared_ptr<Logger> Logger::instance_ = nullptr;

void Logger::CreateRootLogger(const std::string& root_logger_name)
{
  instance_ = std::shared_ptr<Logger>(new Logger(root_logger_name));
}

std::shared_ptr<Logger> Logger::GetInstance()
{
  if (!instance_)
  {
    throw std::runtime_error("Logger instance not created yet. Call createRootLogger() first.");
  }
  return instance_;
}

Logger::Logger(const std::string& root_logger_name)
  : root_logger_name_(root_logger_name)
{
  spdlog::init_thread_pool(32768, 1); // queue with max 32k items 1 backing thread.

  createDefaultSinks();
}

std::string Logger::getFullLoggerName(const std::string& name)
{
  std::string full_logger_name = "";

  if (!root_logger_name_.empty())
  {
    full_logger_name += root_logger_name_;
  }

  if (!root_logger_name_.empty() && !name.empty())
  {
    full_logger_name += ".";
  }

  if (!name.empty())
  {
    full_logger_name += name;
  }

  return full_logger_name;
}

void Logger::createDefaultSinks()
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

  // Create the full log file path
  // Example: /home/user/logfiles/my_logger_2023-10-05_14-30-00Z.log
  // Note that if no root logger name is set (yet), the file name is just "_yyyy-mm-ddThh:mm:ssZ.log"
  std::string log_file_path = log_dir + "/" + root_logger_name_ + "_" + std::string(time_str) + ".log";
  static auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file_path, true);
  default_sinks_.push_back(file_sink);
}

bool Logger::addSinkToDefaultSinks(spdlog::sink_ptr sink)
{
  auto it = std::find(default_sinks_.begin(), default_sinks_.end(), sink);
  if (it != default_sinks_.end())
  {
    return false;
  }

  default_sinks_.push_back(sink);
  return true;
}

std::shared_ptr<spdlog::logger> Logger::createAsyncLogger(
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

std::shared_ptr<spdlog::logger> Logger::getLogger(
  const std::string& name, bool create_if_not_existing, bool add_default_sinks)
{
  std::shared_ptr<spdlog::logger> logger = nullptr;

  const std::lock_guard<std::mutex> lock(logger_map_mutex_);

  const std::string full_logger_name = getFullLoggerName(name);
  
  const auto logger_search_it = logger_map_.find(full_logger_name);
  if (logger_search_it != logger_map_.end())
  {
    logger = logger_search_it->second;
  }
  else if (create_if_not_existing)
  {
    logger = createAsyncLogger(name, {}, add_default_sinks);
    spdlog::initialize_logger(logger);
    logger_map_[full_logger_name] = logger;
  }

  return logger;
}

bool Logger::setLoggerLevel(const std::string& name, spdlog::level::level_enum level)
{
  const std::lock_guard<std::mutex> lock(logger_map_mutex_);
  
  const auto logger_search_it = logger_map_.find(name);
  if (logger_search_it != logger_map_.end())
  {
    logger_search_it->second->set_level(level);
    return true;
  }
  return false;
}

std::unordered_map<std::string, spdlog::level::level_enum> Logger::getLoggerLevels()
{
  std::unordered_map<std::string, spdlog::level::level_enum> logger_levels;

  const std::lock_guard<std::mutex> lock(logger_map_mutex_);

  for (const auto& logger : logger_map_)
  {
    logger_levels[logger.first] = logger.second->level();
  }

  return logger_levels;
}

std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)>& Logger::getTimePointCallback()
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

void Logger::setTimePointCallback(
    std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)> clock_callback)
{
  clock_callback_ = clock_callback;
}

}  // namespace spdlog_ros