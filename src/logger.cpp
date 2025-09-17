#include "spdlog_ros/logger.hpp"

#include <chrono>
#include <ctime>

#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/basic_file_sink.h"

namespace spdlog_ros
{

std::string& GetRootLoggerName()
{
  static std::string root_logger_name = "";
  return root_logger_name;
}

void SetRootLoggerName(const std::string& name)
{
  GetRootLoggerName() = name;
}

std::string GetFullLoggerName(const std::string& name)
{
  std::string full_logger_name = "";

  if (!GetRootLoggerName().empty())
  {
    full_logger_name += GetRootLoggerName();
  }

  if (!GetRootLoggerName().empty() && !name.empty())
  {
    full_logger_name += ".";
  }

  if (!name.empty())
  {
    full_logger_name += name;
  }

  return full_logger_name;
}

std::vector<spdlog::sink_ptr>& GetDefaultSinks()
{
  static std::vector<spdlog::sink_ptr> default_sinks;
  if (default_sinks.empty())
  {
    // add console sink
    static auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    default_sinks.push_back(console_sink);

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
    std::string log_file_path = log_dir + "/" + GetRootLoggerName() + "_" + std::string(time_str) + ".log";
    static auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file_path, true);
    default_sinks.push_back(file_sink);
  }
  return default_sinks;
}

bool AddSinkToDefaultSinks(spdlog::sink_ptr sink)
{
  auto& default_sinks = GetDefaultSinks();
  auto it = std::find(default_sinks.begin(), default_sinks.end(), sink);
  if (it != default_sinks.end())
  {
    return false;
  }

  default_sinks.push_back(sink);
  return true;
}

std::shared_ptr<spdlog::logger> CreateAsyncLogger(const std::string& name, std::vector<spdlog::sink_ptr> sinks, bool add_default_sinks)
{
  static bool oninit = true;
  if(oninit)
  {
    spdlog::init_thread_pool(32768, 1); // queue with max 32k items 1 backing thread.
    oninit = false;
  }

  if (add_default_sinks)
  {
    for (const auto& sink : GetDefaultSinks())
    {
      auto it = std::find(sinks.begin(), sinks.end(), sink);
      if (it == sinks.end())
      {
        sinks.push_back(sink);
      }
    }
  }

  return std::make_shared<spdlog::async_logger>(
    GetFullLoggerName(name),
    sinks.begin(), sinks.end(),
    spdlog::thread_pool(),
    spdlog::async_overflow_policy::overrun_oldest);
}

std::shared_ptr<spdlog::logger> GetLogger(const std::string& name, bool create_if_not_existing, bool add_default_sinks)
{
  // Note that according to the spdlog documentation, get() is thread-safe but might slow down execution because
  // of mutex locking:
  // https://github.com/gabime/spdlog/wiki/Creating-loggers#accessing-loggers-using-spdlogget
  std::shared_ptr<spdlog::logger> logger = spdlog::get(GetFullLoggerName(name));
  if (logger == nullptr && create_if_not_existing)
  {
    auto logger = spdlog_ros::CreateAsyncLogger(name, {}, add_default_sinks);
    spdlog::register_logger(logger);
    return logger;
  }
  return logger;
}

}  // namespace spdlog_ros