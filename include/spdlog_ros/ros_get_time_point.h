//
// Created by cmuehlbacher on 20.12.24.
//

#ifndef SPDLOG_ROS_ROS_GET_TIME_POINT_H
#define SPDLOG_ROS_ROS_GET_TIME_POINT_H

#include "spdlog_ros/logger_manager.hpp"

#include <ros/time.h>

namespace spdlog_ros
{

inline void UseROSTime()
{
  spdlog_ros::LoggerManager::GetLoggerManager()->setTimePointCallback(
    [](spdlog_ros_utils_time_point_value_t * time_point) -> spdlog_ros_utils_ret_t
    {
      try
      {
        *time_point = ros::Time::now().toNSec();
      }
      catch (...)
      {
        return SPDLOG_ROS_UTILS_RET_ERROR;
      }
      return SPDLOG_ROS_UTILS_RET_OK;
    });
}

}  // namespace spdlog_ros

#endif //SPDLOG_ROS_ROS_GET_TIME_POINT_H
