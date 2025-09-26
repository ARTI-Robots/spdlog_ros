//
// Created by cmuehlbacher on 20.12.24.
//

#ifndef SPDLOG_ROS_ROS2_GET_TIME_POINT_H
#define SPDLOG_ROS_ROS2_GET_TIME_POINT_H

#include "spdlog_ros/logger_manager.hpp"

#include <rclcpp/clock.hpp>

namespace spdlog_ros
{

inline void UseROSTime(rclcpp::Clock::SharedPtr clock)
{
  spdlog_ros::LoggerManager::GetLoggerManager()->setTimePointCallback(
    [c=clock](spdlog_ros_utils_time_point_value_t * time_point) -> spdlog_ros_utils_ret_t
    {
      try
      {
        *time_point = c->now().nanoseconds();
      }
      catch (...)
      {
        return SPDLOG_ROS_UTILS_RET_ERROR;
      }
      return SPDLOG_ROS_UTILS_RET_OK;
    });
}

}  // namespace spdlog_ros

#endif //SPDLOG_ROS_ROS2_GET_TIME_POINT_H
