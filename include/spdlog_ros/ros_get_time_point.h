//
// Created by cmuehlbacher on 20.12.24.
//

#ifndef SPDLOG_ROS_ROS_GET_TIME_POINT_H
#define SPDLOG_ROS_ROS_GET_TIME_POINT_H

#include <ros/time.h>

typedef uint64_t spdlog_ros_utils_time_point_value_t;
typedef uint64_t spdlog_ros_utils_duration_value_t;
typedef uint64_t spdlog_ros_utils_ret_t;

#define GET_TIME_POINT(clock) \
    auto get_time_point = [](spdlog_ros_utils_time_point_value_t * time_point) -> spdlog_ros_utils_ret_t { \
      try { \
        *time_point = clock::now().toNSec(); \
      } catch (...) { \
        return SPDLOG_ROS_UTILS_RET_ERROR; \
      } \
        return SPDLOG_ROS_UTILS_RET_OK; \
    }; \

#endif //SPDLOG_ROS_ROS_GET_TIME_POINT_H
