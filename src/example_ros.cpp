#include <ros/ros.h>
#include <spdlog/spdlog.h>
#include "spdlog_ros/logger.hpp"
#include "spdlog_ros/logging.hpp"
#include "spdlog_ros/ros_logging.hpp"

int mainWithRos(int argc, char** argv)
{
  ros::init(argc, argv, "spdlog_ros_example");

  // Create a ROS2 node
  ros::NodeHandle node;

  // Set up ROS logging
  spdlog_ros::SetUpROSLogging();

  // As an alternative to the above setup function, one could also do the following manually here.
  // This is however missing the service setup for getting/setting logger levels at runtime
  // and is therefore not recommended.

  // Set the root logger name, all loggers will be prefixed with this name (e.g. your ros node name)
  // If none is set, the logger name is directly the full logger name given at creation
  // Note that this should happen before all other calls to spdlog_ros such that the file name for logging
  // is set properly (otherwise the file name is just "~/logfiles/_yyyy-mm-ddThh:mm:ssZ.log")
  // spdlog_ros::SetRootLoggerName("my_logger_root");

  // // Set up spdlog_ros to use the ROS time (instead of the default std::chrono time)
  // spdlog_ros::UseROSTime();

  // // Using ROS is optional
  // auto ros_sink = std::make_shared<spdlog_ros::RosSink>(node);

  // // The default sinks are stdout/stderr and file logging
  // // When adding here a default sink, all other loggers will have that sink
  // spdlog_ros::AddSinkToDefaultSinks(ros_sink);

  // A logger can be created manually although not recommended and required because the SPDLOG_ROS_* macros
  // create loggers on the fly when used with a new logger name

  // // Create an async logger that logs to the console, file and ROS (note that the logger name is prefixed
  // // with the root logger name)
  // auto logger = spdlog_ros::CreateAsyncLogger("my_logger");
  // // if one wants to use this logger everywhere, it needs to be registered to spdlog because
  // // otherwise the macros won't find it
  // spdlog::register_logger(logger);
  // // Optionally, make this the default logger, accessible globally (registering is not required then)
  // spdlog::set_default_logger(logger);

  // // Log some messages
  // logger->info("Hello, world!");
  // logger->warn("This is a warning!");
  // logger->error("This is an error!");
  // spdlog::info("This message is logged using the default logger");

  ROS_INFO("This info is logged using ROS logging");
  ROS_ERROR("This error is logged using ROS logging");

  SPDLOG_ROS_DEBUG("debug message");

  SPDLOG_ROS_DEBUG_ONCE("debug message once");
  SPDLOG_ROS_DEBUG_ONCE("debug message once");

  SPDLOG_ROS_DEBUG_EXPRESSION(true, "debug message expression");
  SPDLOG_ROS_DEBUG_EXPRESSION(false, "debug message expression");

  SPDLOG_ROS_DEBUG_FUNCTION([]() -> bool {return true;},"debug message function");
  SPDLOG_ROS_DEBUG_FUNCTION([]() -> bool {return false;},"debug message function");

  SPDLOG_ROS_DEBUG_SKIPFIRST("debug message skip first 1");
  SPDLOG_ROS_DEBUG_SKIPFIRST("debug message skip first 2");

  SPDLOG_ROS_DEBUG_THROTTLE(100, "debug message throttle");

  SPDLOG_ROS_DEBUG_SKIPFIRST_THROTTLE(100, "debug message throttle 1");
  SPDLOG_ROS_DEBUG_SKIPFIRST_THROTTLE(100, "debug message throttle 2");

  SPDLOG_ROS_DEBUG_STREAM("debug message " << " stream");

  SPDLOG_ROS_DEBUG_STREAM_ONCE("debug message " << " stream once");
  SPDLOG_ROS_DEBUG_STREAM_ONCE("debug message " << " stream once");

  SPDLOG_ROS_DEBUG_STREAM_EXPRESSION(true, "debug message " << " stream expression");
  SPDLOG_ROS_DEBUG_STREAM_EXPRESSION(false, "debug message " << " stream expression");

  SPDLOG_ROS_DEBUG_STREAM_FUNCTION([]() -> bool {return true;},"debug message " << " stream function");
  SPDLOG_ROS_DEBUG_STREAM_FUNCTION([]() -> bool {return false;},"debug message " << " stream function");

  SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST("debug message " << " stream skip first 1");
  SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST("debug message " << " stream skip first 2");

  SPDLOG_ROS_DEBUG_STREAM_THROTTLE(100, "debug message " << " stream throttle");

  SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_THROTTLE(100, "debug message " << " stream throttle first 1");
  SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_THROTTLE(100, "debug message " << " stream throttle first 2");

  SPDLOG_ROS_DEBUG_STREAM_NAMED("fancy_name", "debug message " << " stream");

  SPDLOG_ROS_DEBUG_STREAM_ONCE_NAMED("fancy_name", "debug message " << " stream once");
  SPDLOG_ROS_DEBUG_STREAM_ONCE_NAMED("fancy_name", "debug message " << " stream once");

  SPDLOG_ROS_DEBUG_STREAM_EXPRESSION_NAMED("fancy_name", true, "debug message " << " stream expression");
  SPDLOG_ROS_DEBUG_STREAM_EXPRESSION_NAMED("fancy_name", false, "debug message " << " stream expression");

  SPDLOG_ROS_DEBUG_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return true;},"debug message " << " stream function");
  SPDLOG_ROS_DEBUG_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return false;},"debug message " << " stream function");

  SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_NAMED("fancy_name", "debug message " << " stream skip first 1");
  SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_NAMED("fancy_name", "debug message " << " stream skip first 2");

  SPDLOG_ROS_DEBUG_STREAM_THROTTLE_NAMED("fancy_name", 100, "debug message " << " stream throttle");

  SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "debug message " << " stream throttle first 1");
  SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "debug message " << " stream throttle first 2");

  SPDLOG_ROS_INFO("INFO message");

  SPDLOG_ROS_INFO_ONCE("INFO message once");
  SPDLOG_ROS_INFO_ONCE("INFO message once");

  SPDLOG_ROS_INFO_EXPRESSION(true, "INFO message expression");
  SPDLOG_ROS_INFO_EXPRESSION(false, "INFO message expression");

  SPDLOG_ROS_INFO_FUNCTION([]() -> bool {return true;},"INFO message function");
  SPDLOG_ROS_INFO_FUNCTION([]() -> bool {return false;},"INFO message function");

  SPDLOG_ROS_INFO_SKIPFIRST("INFO message skip first 1");
  SPDLOG_ROS_INFO_SKIPFIRST("INFO message skip first 2");

  SPDLOG_ROS_INFO_THROTTLE(100, "INFO message throttle");

  SPDLOG_ROS_INFO_SKIPFIRST_THROTTLE(100, "INFO message throttle 1");
  SPDLOG_ROS_INFO_SKIPFIRST_THROTTLE(100, "INFO message throttle 2");

  SPDLOG_ROS_INFO_STREAM("INFO message " << " stream");

  SPDLOG_ROS_INFO_STREAM_ONCE("INFO message " << " stream once");
  SPDLOG_ROS_INFO_STREAM_ONCE("INFO message " << " stream once");

  SPDLOG_ROS_INFO_STREAM_EXPRESSION(true, "INFO message " << " stream expression");
  SPDLOG_ROS_INFO_STREAM_EXPRESSION(false, "INFO message " << " stream expression");

  SPDLOG_ROS_INFO_STREAM_FUNCTION([]() -> bool {return true;},"INFO message " << " stream function");
  SPDLOG_ROS_INFO_STREAM_FUNCTION([]() -> bool {return false;},"INFO message " << " stream function");

  SPDLOG_ROS_INFO_STREAM_SKIPFIRST("INFO message " << " stream skip first 1");
  SPDLOG_ROS_INFO_STREAM_SKIPFIRST("INFO message " << " stream skip first 2");

  SPDLOG_ROS_INFO_STREAM_THROTTLE(100, "INFO message " << " stream throttle");

  SPDLOG_ROS_INFO_STREAM_SKIPFIRST_THROTTLE(100, "INFO message " << " stream throttle first 1");
  SPDLOG_ROS_INFO_STREAM_SKIPFIRST_THROTTLE(100, "INFO message " << " stream throttle first 2");

  SPDLOG_ROS_INFO_STREAM_NAMED("fancy_name", "INFO message " << " stream");

  SPDLOG_ROS_INFO_STREAM_ONCE_NAMED("fancy_name", "INFO message " << " stream once");
  SPDLOG_ROS_INFO_STREAM_ONCE_NAMED("fancy_name", "INFO message " << " stream once");

  SPDLOG_ROS_INFO_STREAM_EXPRESSION_NAMED("fancy_name", true, "INFO message " << " stream expression");
  SPDLOG_ROS_INFO_STREAM_EXPRESSION_NAMED("fancy_name", false, "INFO message " << " stream expression");

  SPDLOG_ROS_INFO_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return true;},"INFO message " << " stream function");
  SPDLOG_ROS_INFO_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return false;},"INFO message " << " stream function");

  SPDLOG_ROS_INFO_STREAM_SKIPFIRST_NAMED("fancy_name", "INFO message " << " stream skip first 1");
  SPDLOG_ROS_INFO_STREAM_SKIPFIRST_NAMED("fancy_name", "INFO message " << " stream skip first 2");

  SPDLOG_ROS_INFO_STREAM_THROTTLE_NAMED("fancy_name", 100, "INFO message " << " stream throttle");

  SPDLOG_ROS_INFO_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "INFO message " << " stream throttle first 1");
  SPDLOG_ROS_INFO_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "INFO message " << " stream throttle first 2");

  SPDLOG_ROS_WARN("WARN message");

  SPDLOG_ROS_WARN_ONCE("WARN message once");
  SPDLOG_ROS_WARN_ONCE("WARN message once");

  SPDLOG_ROS_WARN_EXPRESSION(true, "WARN message expression");
  SPDLOG_ROS_WARN_EXPRESSION(false, "WARN message expression");

  SPDLOG_ROS_WARN_FUNCTION([]() -> bool {return true;},"WARN message function");
  SPDLOG_ROS_WARN_FUNCTION([]() -> bool {return false;},"WARN message function");

  SPDLOG_ROS_WARN_SKIPFIRST("WARN message skip first 1");
  SPDLOG_ROS_WARN_SKIPFIRST("WARN message skip first 2");

  SPDLOG_ROS_WARN_THROTTLE(100, "WARN message throttle");

  SPDLOG_ROS_WARN_SKIPFIRST_THROTTLE(100, "WARN message throttle 1");
  SPDLOG_ROS_WARN_SKIPFIRST_THROTTLE(100, "WARN message throttle 2");

  SPDLOG_ROS_WARN_STREAM("WARN message " << " stream");

  SPDLOG_ROS_WARN_STREAM_ONCE("WARN message " << " stream once");
  SPDLOG_ROS_WARN_STREAM_ONCE("WARN message " << " stream once");

  SPDLOG_ROS_WARN_STREAM_EXPRESSION(true, "WARN message " << " stream expression");
  SPDLOG_ROS_WARN_STREAM_EXPRESSION(false, "WARN message " << " stream expression");

  SPDLOG_ROS_WARN_STREAM_FUNCTION([]() -> bool {return true;},"WARN message " << " stream function");
  SPDLOG_ROS_WARN_STREAM_FUNCTION([]() -> bool {return false;},"WARN message " << " stream function");

  SPDLOG_ROS_WARN_STREAM_SKIPFIRST("WARN message " << " stream skip first 1");
  SPDLOG_ROS_WARN_STREAM_SKIPFIRST("WARN message " << " stream skip first 2");

  SPDLOG_ROS_WARN_STREAM_THROTTLE(100, "WARN message " << " stream throttle");

  SPDLOG_ROS_WARN_STREAM_SKIPFIRST_THROTTLE(100, "WARN message " << " stream throttle first 1");
  SPDLOG_ROS_WARN_STREAM_SKIPFIRST_THROTTLE(100, "WARN message " << " stream throttle first 2");

  SPDLOG_ROS_WARN_STREAM_NAMED("fancy_name", "WARN message " << " stream");

  SPDLOG_ROS_WARN_STREAM_ONCE_NAMED("fancy_name", "WARN message " << " stream once");
  SPDLOG_ROS_WARN_STREAM_ONCE_NAMED("fancy_name", "WARN message " << " stream once");

  SPDLOG_ROS_WARN_STREAM_EXPRESSION_NAMED("fancy_name", true, "WARN message " << " stream expression");
  SPDLOG_ROS_WARN_STREAM_EXPRESSION_NAMED("fancy_name", false, "WARN message " << " stream expression");

  SPDLOG_ROS_WARN_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return true;},"WARN message " << " stream function");
  SPDLOG_ROS_WARN_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return false;},"WARN message " << " stream function");

  SPDLOG_ROS_WARN_STREAM_SKIPFIRST_NAMED("fancy_name", "WARN message " << " stream skip first 1");
  SPDLOG_ROS_WARN_STREAM_SKIPFIRST_NAMED("fancy_name", "WARN message " << " stream skip first 2");

  SPDLOG_ROS_WARN_STREAM_THROTTLE_NAMED("fancy_name", 100, "WARN message " << " stream throttle");

  SPDLOG_ROS_WARN_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "WARN message " << " stream throttle first 1");
  SPDLOG_ROS_WARN_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "WARN message " << " stream throttle first 2");

  SPDLOG_ROS_ERROR("ERROR message");

  SPDLOG_ROS_ERROR_ONCE("ERROR message once");
  SPDLOG_ROS_ERROR_ONCE("ERROR message once");

  SPDLOG_ROS_ERROR_EXPRESSION(true, "ERROR message expression");
  SPDLOG_ROS_ERROR_EXPRESSION(false, "ERROR message expression");

  SPDLOG_ROS_ERROR_FUNCTION([]() -> bool {return true;},"ERROR message function");
  SPDLOG_ROS_ERROR_FUNCTION([]() -> bool {return false;},"ERROR message function");

  SPDLOG_ROS_ERROR_SKIPFIRST("ERROR message skip first 1");
  SPDLOG_ROS_ERROR_SKIPFIRST("ERROR message skip first 2");

  SPDLOG_ROS_ERROR_THROTTLE(100, "ERROR message throttle");

  SPDLOG_ROS_ERROR_SKIPFIRST_THROTTLE(100, "ERROR message throttle 1");
  SPDLOG_ROS_ERROR_SKIPFIRST_THROTTLE(100, "ERROR message throttle 2");

  SPDLOG_ROS_ERROR_STREAM("ERROR message " << " stream");

  SPDLOG_ROS_ERROR_STREAM_ONCE("ERROR message " << " stream once");
  SPDLOG_ROS_ERROR_STREAM_ONCE("ERROR message " << " stream once");

  SPDLOG_ROS_ERROR_STREAM_EXPRESSION(true, "ERROR message " << " stream expression");
  SPDLOG_ROS_ERROR_STREAM_EXPRESSION(false, "ERROR message " << " stream expression");

  SPDLOG_ROS_ERROR_STREAM_FUNCTION([]() -> bool {return true;},"ERROR message " << " stream function");
  SPDLOG_ROS_ERROR_STREAM_FUNCTION([]() -> bool {return false;},"ERROR message " << " stream function");

  SPDLOG_ROS_ERROR_STREAM_SKIPFIRST("ERROR message " << " stream skip first 1");
  SPDLOG_ROS_ERROR_STREAM_SKIPFIRST("ERROR message " << " stream skip first 2");

  SPDLOG_ROS_ERROR_STREAM_THROTTLE(100, "ERROR message " << " stream throttle");

  SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_THROTTLE(100, "ERROR message " << " stream throttle first 1");
  SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_THROTTLE(100, "ERROR message " << " stream throttle first 2");

  SPDLOG_ROS_ERROR_STREAM_NAMED("fancy_name", "ERROR message " << " stream");

  SPDLOG_ROS_ERROR_STREAM_ONCE_NAMED("fancy_name", "ERROR message " << " stream once");
  SPDLOG_ROS_ERROR_STREAM_ONCE_NAMED("fancy_name", "ERROR message " << " stream once");

  SPDLOG_ROS_ERROR_STREAM_EXPRESSION_NAMED("fancy_name", true, "ERROR message " << " stream expression");
  SPDLOG_ROS_ERROR_STREAM_EXPRESSION_NAMED("fancy_name", false, "ERROR message " << " stream expression");

  SPDLOG_ROS_ERROR_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return true;},"ERROR message " << " stream function");
  SPDLOG_ROS_ERROR_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return false;},"ERROR message " << " stream function");

  SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_NAMED("fancy_name", "ERROR message " << " stream skip first 1");
  SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_NAMED("fancy_name", "ERROR message " << " stream skip first 2");

  SPDLOG_ROS_ERROR_STREAM_THROTTLE_NAMED("fancy_name", 100, "ERROR message " << " stream throttle");

  SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "ERROR message " << " stream throttle first 1");
  SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "ERROR message " << " stream throttle first 2");

  SPDLOG_ROS_FATAL("FATAL message");

  SPDLOG_ROS_FATAL_ONCE("FATAL message once");
  SPDLOG_ROS_FATAL_ONCE("FATAL message once");

  SPDLOG_ROS_FATAL_EXPRESSION(true, "FATAL message expression");
  SPDLOG_ROS_FATAL_EXPRESSION(false, "FATAL message expression");

  SPDLOG_ROS_FATAL_FUNCTION([]() -> bool {return true;},"FATAL message function");
  SPDLOG_ROS_FATAL_FUNCTION([]() -> bool {return false;},"FATAL message function");

  SPDLOG_ROS_FATAL_SKIPFIRST("FATAL message skip first 1");
  SPDLOG_ROS_FATAL_SKIPFIRST("FATAL message skip first 2");

  SPDLOG_ROS_FATAL_THROTTLE(100, "FATAL message throttle");

  SPDLOG_ROS_FATAL_SKIPFIRST_THROTTLE(100, "FATAL message throttle 1");
  SPDLOG_ROS_FATAL_SKIPFIRST_THROTTLE(100, "FATAL message throttle 2");

  SPDLOG_ROS_FATAL_STREAM("FATAL message " << " stream");

  SPDLOG_ROS_FATAL_STREAM_ONCE("FATAL message " << " stream once");
  SPDLOG_ROS_FATAL_STREAM_ONCE("FATAL message " << " stream once");

  SPDLOG_ROS_FATAL_STREAM_EXPRESSION(true, "FATAL message " << " stream expression");
  SPDLOG_ROS_FATAL_STREAM_EXPRESSION(false, "FATAL message " << " stream expression");

  SPDLOG_ROS_FATAL_STREAM_FUNCTION([]() -> bool {return true;},"FATAL message " << " stream function");
  SPDLOG_ROS_FATAL_STREAM_FUNCTION([]() -> bool {return false;},"FATAL message " << " stream function");

  SPDLOG_ROS_FATAL_STREAM_SKIPFIRST("FATAL message " << " stream skip first 1");
  SPDLOG_ROS_FATAL_STREAM_SKIPFIRST("FATAL message " << " stream skip first 2");

  SPDLOG_ROS_FATAL_STREAM_THROTTLE(100, "FATAL message " << " stream throttle");

  SPDLOG_ROS_FATAL_STREAM_SKIPFIRST_THROTTLE(100, "FATAL message " << " stream throttle first 1");
  SPDLOG_ROS_FATAL_STREAM_SKIPFIRST_THROTTLE(100, "FATAL message " << " stream throttle first 2");

  SPDLOG_ROS_FATAL_STREAM_NAMED("fancy_name", "FATAL message " << " stream");

  SPDLOG_ROS_FATAL_STREAM_ONCE_NAMED("fancy_name", "FATAL message " << " stream once");
  SPDLOG_ROS_FATAL_STREAM_ONCE_NAMED("fancy_name", "FATAL message " << " stream once");

  SPDLOG_ROS_FATAL_STREAM_EXPRESSION_NAMED("fancy_name", true, "FATAL message " << " stream expression");
  SPDLOG_ROS_FATAL_STREAM_EXPRESSION_NAMED("fancy_name", false, "FATAL message " << " stream expression");

  SPDLOG_ROS_FATAL_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return true;},"FATAL message " << " stream function");
  SPDLOG_ROS_FATAL_STREAM_FUNCTION_NAMED("fancy_name", []() -> bool {return false;},"FATAL message " << " stream function");

  SPDLOG_ROS_FATAL_STREAM_SKIPFIRST_NAMED("fancy_name", "FATAL message " << " stream skip first 1");
  SPDLOG_ROS_FATAL_STREAM_SKIPFIRST_NAMED("fancy_name", "FATAL message " << " stream skip first 2");

  SPDLOG_ROS_FATAL_STREAM_THROTTLE_NAMED("fancy_name", 100, "FATAL message " << " stream throttle");

  SPDLOG_ROS_FATAL_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "FATAL message " << " stream throttle first 1");
  SPDLOG_ROS_FATAL_STREAM_SKIPFIRST_THROTTLE_NAMED("fancy_name", 100, "FATAL message " << " stream throttle first 2");

  // make an infinite loop to test throttling
  ros::Rate rate(10);
  while (ros::ok())
  {
    SPDLOG_ROS_DEBUG("looping...");
    SPDLOG_ROS_INFO_THROTTLE(1e3, "1s throttle");
    SPDLOG_ROS_ERROR_THROTTLE(2e3, "2s throttle");
    SPDLOG_ROS_FATAL_THROTTLE(5e3, "5s throttle");
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

int main(int argc, char** argv)
{
  return mainWithRos(argc, argv);
}
