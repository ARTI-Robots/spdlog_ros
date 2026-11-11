#include <cstdlib>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/base_sink.h>

#ifdef USING_ROS2
#include <rclcpp/rclcpp.hpp>
#include "spdlog_ros/ros2_logging.hpp"
#else
#include <ros/ros.h>
#include "spdlog_ros/ros_logging.hpp"
#endif

#include "spdlog_ros/logger_manager.hpp"
#include "spdlog_ros/logging.hpp"

namespace
{

class CollectingSink : public spdlog::sinks::base_sink<std::mutex>
{
public:
  bool waitForMessageCount(size_t expected_count)
  {
    std::unique_lock<std::mutex> lock(messages_mutex_);
    return messages_condition_.wait_for(lock, std::chrono::seconds(2), [this, expected_count]() {
      return messages_.size() >= expected_count;
    });
  }

  std::vector<std::string> messages()
  {
    const std::lock_guard<std::mutex> lock(messages_mutex_);
    return messages_;
  }

protected:
  void sink_it_(const spdlog::details::log_msg& message) override
  {
    {
      const std::lock_guard<std::mutex> lock(messages_mutex_);
      messages_.emplace_back(message.payload.data(), message.payload.size());
    }
    messages_condition_.notify_all();
  }

  void flush_() override {}

private:
  std::mutex messages_mutex_;
  std::condition_variable messages_condition_;
  std::vector<std::string> messages_;
};

class SpdlogEnvironmentLevelsTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("SPDLOG_ROS_LOGFILES", "/tmp", 1);

#ifdef USING_ROS2
    node_ = std::make_shared<rclcpp::Node>("spdlog_ros_env_levels_test");
    base_logger_name_ = node_->get_fully_qualified_name();
#else
    base_logger_name_ = ros::this_node::getName();
#endif

    const std::string level_configuration = "off," + base_logger_name_ + ".configured=debug";
    setenv("SPDLOG_LEVEL", level_configuration.c_str(), 1);

#ifdef USING_ROS2
    ros_logging_manager_ = std::make_unique<spdlog_ros::ROSLoggingManager>(node_);
#else
    ros_logging_manager_ = std::make_unique<spdlog_ros::ROSLoggingManager>();
#endif

    logger_manager_ = spdlog_ros::LoggerManager::GetLoggerManager();
    collecting_sink_ = std::make_shared<CollectingSink>();
    EXPECT_TRUE(logger_manager_->addSinkToDefaultSinks(collecting_sink_));
  }

  static void TearDownTestSuite()
  {
    ros_logging_manager_.reset();
    logger_manager_.reset();
    collecting_sink_.reset();
#ifdef USING_ROS2
    node_.reset();
#endif
  }

  void SetUp() override
  {
    configured_logger_ = logger_manager_->getLogger("configured", true, false);
    defaulted_logger_ = logger_manager_->getLogger("defaulted", true, false);
  }

  static std::string base_logger_name_;
  static std::unique_ptr<spdlog_ros::ROSLoggingManager> ros_logging_manager_;
  static std::shared_ptr<spdlog_ros::LoggerManager> logger_manager_;
  static std::shared_ptr<CollectingSink> collecting_sink_;
#ifdef USING_ROS2
  static rclcpp::Node::SharedPtr node_;
#endif

  std::shared_ptr<spdlog::logger> configured_logger_;
  std::shared_ptr<spdlog::logger> defaulted_logger_;
};

std::string SpdlogEnvironmentLevelsTest::base_logger_name_;
std::unique_ptr<spdlog_ros::ROSLoggingManager> SpdlogEnvironmentLevelsTest::ros_logging_manager_;
std::shared_ptr<spdlog_ros::LoggerManager> SpdlogEnvironmentLevelsTest::logger_manager_;
std::shared_ptr<CollectingSink> SpdlogEnvironmentLevelsTest::collecting_sink_;
#ifdef USING_ROS2
rclcpp::Node::SharedPtr SpdlogEnvironmentLevelsTest::node_;
#endif

TEST_F(SpdlogEnvironmentLevelsTest, AppliesNamedEnvironmentOverrideAndGlobalFallback)
{
  ASSERT_NE(nullptr, configured_logger_);
  ASSERT_NE(nullptr, defaulted_logger_);
  EXPECT_EQ(spdlog::level::debug, configured_logger_->level());
  EXPECT_EQ(spdlog::level::off, defaulted_logger_->level());
}

TEST_F(SpdlogEnvironmentLevelsTest, LoggingMacrosEmitExpectedMessagesAndHonorConditions)
{
  const std::string default_macro_name = SPDLOG_ROS_DEFAULT_NAME;
  const std::string named_macro_name = std::string(SPDLOG_ROS_NAME_PREFIX) + ".macro_named";
  ASSERT_TRUE(logger_manager_->setLoggerLevel(
    logger_manager_->getFullLoggerName(default_macro_name), spdlog::level::debug));
  ASSERT_TRUE(logger_manager_->setLoggerLevel(
    logger_manager_->getFullLoggerName(named_macro_name), spdlog::level::debug));

  SPDLOG_ROS_DEBUG("debug message");
  SPDLOG_ROS_INFO("formatted value {}", 42);
  SPDLOG_ROS_DEBUG_NAMED("macro_named", "named value {}", 7);

  const bool expression_values[] = {false, true};
  const char* expression_messages[] = {"expression suppressed", "expression accepted"};
  for (size_t index = 0; index < 2; ++index)
  {
    SPDLOG_ROS_INFO_EXPRESSION(expression_values[index], "{}", expression_messages[index]);
  }

  bool (*function)() = +[]() { return false; };
  const char* function_messages[] = {"function suppressed", "function accepted"};
  for (size_t index = 0; index < 2; ++index)
  {
    function = index == 0 ? +[]() { return false; } : +[]() { return true; };
    SPDLOG_ROS_INFO_FUNCTION(function, "{}", function_messages[index]);
  }

  for (size_t index = 0; index < 2; ++index)
  {
    SPDLOG_ROS_INFO_ONCE("once only");
  }

  const char* skipfirst_messages[] = {"skipfirst suppressed", "skipfirst accepted"};
  for (size_t index = 0; index < 2; ++index)
  {
    SPDLOG_ROS_INFO_SKIPFIRST("{}", skipfirst_messages[index]);
  }

  SPDLOG_ROS_INFO_STREAM("stream value " << 9);

  spdlog_ros_utils_time_point_value_t now_ns = 100000000;
  logger_manager_->setTimePointCallback([&now_ns](spdlog_ros_utils_time_point_value_t* time_point) {
    *time_point = now_ns;
    return SPDLOG_ROS_UTILS_RET_OK;
  });
  const char* throttle_messages[] = {"throttle first", "throttle suppressed", "throttle after interval"};
  for (size_t index = 0; index < 3; ++index)
  {
    if (index == 2)
    {
      now_ns += SPDLOG_ROS_UTILS_MS_TO_NS(100);
    }
    SPDLOG_ROS_INFO_THROTTLE(100, "{}", throttle_messages[index]);
  }
  logger_manager_->setTimePointCallback({});

  const std::vector<std::string> expected_messages = {
    "debug message",
    "formatted value 42",
    "named value 7",
    "expression accepted",
    "function accepted",
    "once only",
    "skipfirst accepted",
    "stream value 9",
    "throttle first",
    "throttle after interval"
  };
  ASSERT_TRUE(collecting_sink_->waitForMessageCount(expected_messages.size()));
  EXPECT_EQ(expected_messages, collecting_sink_->messages());
}

}  // namespace

int main(int argc, char** argv)
{
#ifdef USING_ROS2
  rclcpp::init(argc, argv);
#else
  ros::init(argc, argv, "spdlog_ros_env_levels_test");
#endif

  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();

#ifdef USING_ROS2
  rclcpp::shutdown();
#else
  ros::shutdown();
#endif

  return result;
}