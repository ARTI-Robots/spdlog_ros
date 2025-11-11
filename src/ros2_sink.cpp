#include "spdlog_ros/ros2_sink.hpp"

#include "rcl_interfaces/msg/log.hpp"

#include "spdlog/spdlog.h"

#include "spdlog_ros/ros2_utils.hpp"

namespace spdlog_ros
{

RosSink::~RosSink() {}

//Pimpl idiom
struct RosSink::Pimpl
{
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr log_publisher;
  std::mutex mutex;
};

RosSink::RosSink(rclcpp::Node::SharedPtr node): pimpl_(std::make_unique<Pimpl>())
{
  pimpl_->node = node;
  pimpl_->log_publisher = pimpl_->node->create_publisher<rcl_interfaces::msg::Log>("rosout", rclcpp::RosoutQoS());
}


void RosSink::log(const spdlog::details::log_msg& msg)
{
  const std::lock_guard<std::mutex> lock(pimpl_->mutex);

  // Publish the log message
  rcl_interfaces::msg::Log log_msg;

  log_msg.level = spdlog_ros::ConvertSeverityToROS(msg.level);
  log_msg.name = fmt::format("{}", msg.logger_name);
  log_msg.msg = fmt::format("{}", msg.payload);

  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(msg.time.time_since_epoch()).count();
  log_msg.stamp.sec = static_cast<uint32_t>(ns / 1000000000);
  log_msg.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000);

  if(msg.source.filename)
  {
    log_msg.file = msg.source.filename;
  }
  if(msg.source.funcname)
  {
    log_msg.function = msg.source.funcname;
  }
  log_msg.line = msg.source.line;

  pimpl_->log_publisher->publish(log_msg);
}

}  // namespace spdlog_ros