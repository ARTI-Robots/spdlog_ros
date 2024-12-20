//
// Created by cmuehlbacher on 20.12.24.
//

#ifndef SPDLOG_ROS_GET_TIME_POINT_H
#define SPDLOG_ROS_GET_TIME_POINT_H

#if __ROS_VERSION <= 1
#include <spdlog_ros/ros_get_time_point.h>
#elif __ROS_VERSION == 2
#include <spdlog_ros/ros2_get_time_point.h>
#endif // __ROS_VERSION

#endif //SPDLOG_ROS_GET_TIME_POINT_H
