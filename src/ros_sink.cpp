#include "spdlog_ros/ros_sink.hpp"

#include "rosgraph_msgs/Log.h"

#include "spdlog/spdlog.h"

#include "spdlog_ros/ros_utils.hpp"

namespace spdlog_ros
{

RosSink::~RosSink() {}

//Pimpl idiom
struct RosSink::Pimpl
{
  ros::Publisher log_publisher;
  std::mutex mutex;
};

RosSink::RosSink(ros::NodeHandle& node): pimpl_(std::make_unique<Pimpl>())
{
  pimpl_->log_publisher = node.advertise<rosgraph_msgs::Log>("/rosout", 10);
}


void RosSink::log(const spdlog::details::log_msg& msg)
{
  const std::lock_guard<std::mutex> lock(pimpl_->mutex);

  // Publish the log message
  rosgraph_msgs::Log log_msg;

  log_msg.level = spdlog_ros::ConvertSeverityToROS(msg.level);
  log_msg.name = fmt::format("{}", msg.logger_name);
  log_msg.msg = fmt::format("{}", msg.payload);

  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(msg.time.time_since_epoch()).count();
  auto sec_part = static_cast<uint32_t>(ns / 1000000000);
  auto nsec_part = static_cast<uint32_t>(ns % 1000000000);
  log_msg.header.stamp = ros::Time(sec_part, nsec_part);

  if(msg.source.filename)
  {
    log_msg.file = msg.source.filename;
  }
  if(msg.source.funcname)
  {
    log_msg.function = msg.source.funcname;
  }
  log_msg.line = msg.source.line;
  log_msg.topics = {"/rosout"};

  pimpl_->log_publisher.publish(log_msg);
}

}  // namespace spdlog_ros