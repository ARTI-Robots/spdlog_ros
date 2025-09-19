//
// Created by Fabian Hirmann on 19.09.2025.
//

#pragma once

#include "spdlog_ros/logger.hpp"
#include "spdlog_ros/ros2_sink.hpp"
#include "spdlog_ros/ros2_get_time_point.h"

#include <rclcpp/rclcpp.hpp>

namespace spdlog_ros
{

inline void SetUpROSLogging(rclcpp::Node::SharedPtr node)
{
  // Set up spdlog_ros to use the ROS time (instead of the default std::chrono time)
  spdlog_ros::UseROSTime(node->get_clock());

  // Create a ROS sink
  auto ros_sink = std::make_shared<spdlog_ros::RosSink>(node);

  // The default sinks are stdout/stderr and file logging
  // When adding here a default sink, all other loggers will have that sink
  spdlog_ros::AddSinkToDefaultSinks(ros_sink);
}

}  // namespace spdlog_ros