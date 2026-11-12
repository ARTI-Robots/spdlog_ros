#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "spdlog_ros/logger_manager.hpp"

namespace spdlog_ros
{
namespace
{

class LoggerManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    setenv("SPDLOG_ROS_LOGFILES", "/tmp", 1);
    LoggerManager::CreateLoggerManager("spdlog_ros_test");
    manager_ = LoggerManager::GetLoggerManager();
  }

  static void TearDownTestSuite()
  {
    manager_->shutdownLogging();
  }

  static std::shared_ptr<LoggerManager> manager_;
};

std::shared_ptr<LoggerManager> LoggerManagerTest::manager_;

TEST_F(LoggerManagerTest, getFullLoggerName)
{
  EXPECT_EQ("spdlog_ros_test", manager_->getFullLoggerName(""));
  EXPECT_EQ("spdlog_ros_test.alpha.beta",
            manager_->getFullLoggerName("alpha.beta"));
}

TEST_F(LoggerManagerTest, getLoggerNameFromFullName)
{
  // Get the logger name based on the full logger name (full_logger_name = base_logger_name + "." + logger_name)
  // For the set up logging manager from the test suite, the base logger name is "spdlog_ros_test"

  // The base logger name is contained and just stripped from the full logger name
  EXPECT_EQ("alpha.beta", manager_->getLoggerNameFromFullName("spdlog_ros_test.alpha.beta"));

  // The full logger name is just the base logger name with no additional logger name so an empty string is returned
  EXPECT_EQ("", manager_->getLoggerNameFromFullName("spdlog_ros_test"));

  // The full logger name does not contain the base logger name, so it is returned as is
  EXPECT_EQ("unrelated.logger", manager_->getLoggerNameFromFullName("unrelated.logger"));

  // The full logger name contains a similar prefix but is not the base logger name, so it is returned as is
  EXPECT_EQ("spdlog_ros_testing.logger", manager_->getLoggerNameFromFullName("spdlog_ros_testing.logger"));
}

TEST_F(LoggerManagerTest, splitLoggerName)
{
  EXPECT_EQ((std::vector<std::string>{"alpha", "beta", "gamma"}),
            LoggerManager::splitLoggerName("alpha.beta.gamma"));
  EXPECT_EQ((std::vector<std::string>{"alpha", "beta"}),
            LoggerManager::splitLoggerName("alpha/beta", "/"));
  EXPECT_EQ((std::vector<std::string>{"alpha", "", "beta"}),
            LoggerManager::splitLoggerName("alpha//beta", "/"));
  EXPECT_THROW(LoggerManager::splitLoggerName("alpha", ""), std::invalid_argument);
}

TEST_F(LoggerManagerTest, getParentLoggerName)
{
  EXPECT_EQ("alpha.beta", LoggerManager::getParentLoggerName("alpha.beta.gamma"));
  EXPECT_EQ("", LoggerManager::getParentLoggerName("alpha"));
}

TEST_F(LoggerManagerTest, LookupCanAvoidCreatingMissingLogger)
{
  EXPECT_EQ(nullptr, manager_->getLogger("lookup_missing", false));
}

TEST_F(LoggerManagerTest, CreatesParentLoggersForHierarchicalNames)
{
  const auto child = manager_->getLogger("hierarchy.child", true, false);
  ASSERT_NE(nullptr, child);
  EXPECT_EQ("spdlog_ros_test.hierarchy.child", child->name());

  const auto parent = manager_->getLogger("hierarchy", false, false);
  ASSERT_NE(nullptr, parent);
  EXPECT_EQ("spdlog_ros_test.hierarchy", parent->name());
}

TEST_F(LoggerManagerTest, NewChildInheritsExistingParentLevel)
{
  const auto parent = manager_->getLogger("parent_level", true, false);
  ASSERT_NE(nullptr, parent);
  ASSERT_TRUE(manager_->setLoggerLevel("spdlog_ros_test.parent_level", spdlog::level::warn));

  const auto child = manager_->getLogger("parent_level.child", true, false);
  ASSERT_NE(nullptr, child);
  EXPECT_EQ(spdlog::level::warn, child->level());
}

TEST_F(LoggerManagerTest, SettingLevelBeforeLoggerCreationAppliesToNewLogger)
{
  ASSERT_TRUE(manager_->setLoggerLevel("spdlog_ros_test.preconfigured", spdlog::level::critical));

  const auto logger = manager_->getLogger("preconfigured", false, false);
  ASSERT_NE(nullptr, logger);
  EXPECT_EQ(spdlog::level::critical, logger->level());
}

TEST_F(LoggerManagerTest, SettingLoggerLevelUpdatesRegisteredLocations)
{
  LoggerManager::LoggerLocation debug_location;
  LoggerManager::LoggerLocation info_location;
  manager_->initializeLogLocation(&debug_location, "levels", spdlog::level::debug);
  manager_->initializeLogLocation(&info_location, "levels", spdlog::level::info);
  ASSERT_FALSE(debug_location.enabled);
  ASSERT_TRUE(info_location.enabled);

  EXPECT_TRUE(manager_->setLoggerLevel("spdlog_ros_test.levels", spdlog::level::debug));
  EXPECT_TRUE(debug_location.enabled);
  EXPECT_TRUE(info_location.enabled);

  EXPECT_TRUE(manager_->setLoggerLevel("spdlog_ros_test.levels", spdlog::level::warn));
  EXPECT_FALSE(debug_location.enabled);
  EXPECT_FALSE(info_location.enabled);
}

TEST_F(LoggerManagerTest, ChangingLocationLevelRequiresExplicitRefresh)
{
  LoggerManager::LoggerLocation location;
  manager_->initializeLogLocation(&location, "location_refresh", spdlog::level::info);
  ASSERT_TRUE(location.enabled);

  manager_->setLogLocationLevel(&location, spdlog::level::debug);
  EXPECT_TRUE(location.enabled);

  manager_->checkLogLocationEnabled(&location, "location_refresh");
  EXPECT_FALSE(location.enabled);
}

}  // namespace
}  // namespace spdlog_ros

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}