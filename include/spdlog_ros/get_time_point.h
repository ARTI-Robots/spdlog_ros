//
// Created by cmuehlbacher on 20.12.24.
//

#ifndef SPDLOG_ROS_GET_TIME_POINT_H
#define SPDLOG_ROS_GET_TIME_POINT_H

#include <functional>
#include <chrono>

#include <spdlog_ros/logging_util_macros.hpp>

namespace spdlog_ros
{

inline std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)>& GetTimePointCallback()
{
  // The default clock callback uses std::chrono to get the current time in nanoseconds since epoch
  // The clock callback can be overridden by the user to use a different clock source (e.g. ROS time)
  static std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)> clock_callback =
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

  return clock_callback;
}

inline void SetTimePointCallback(
    std::function<spdlog_ros_utils_ret_t(spdlog_ros_utils_time_point_value_t*)> clock_callback)
{
  GetTimePointCallback() = clock_callback;
}

}  // namespace spdlog_ros

#endif //SPDLOG_ROS_GET_TIME_POINT_H
