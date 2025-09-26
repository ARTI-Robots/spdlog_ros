//
// Created by Fabian Hirmann on 28.09.2025.
//


// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef SPDLOG_ROS_LOGGING_UTIL_MACROS_HPP
#define SPDLOG_ROS_LOGGING_UTIL_MACROS_HPP

#include <cstdint>

typedef int64_t spdlog_ros_utils_time_point_value_t;
typedef int64_t spdlog_ros_utils_duration_value_t;
typedef uint64_t spdlog_ros_utils_ret_t;

#define SPDLOG_ROS_LEVEL_DEBUG spdlog::level::debug
#define SPDLOG_ROS_LEVEL_INFO spdlog::level::info
#define SPDLOG_ROS_LEVEL_WARN spdlog::level::warn
#define SPDLOG_ROS_LEVEL_ERROR spdlog::level::err
#define SPDLOG_ROS_LEVEL_FATAL spdlog::level::critical

/// Convenience macro to convert milliseconds to nanoseconds.
#define SPDLOG_ROS_UTILS_MS_TO_NS(milliseconds) ((milliseconds) * (1000LL * 1000LL))

/// Successful operation.
#define SPDLOG_ROS_UTILS_RET_OK 0
/// Generic failure in operation.
#define SPDLOG_ROS_UTILS_RET_ERROR 2

// Provide the compiler with branch prediction information
#ifndef _WIN32
/**
 * \def SPDLOG_ROS_UTILS_LIKELY
 * Instruct the compiler to optimize for the case where the argument equals 1.
 */
# define SPDLOG_ROS_UTILS_LIKELY(x) __builtin_expect((x), 1)
/**
 * \def SPDLOG_ROS_UTILS_UNLIKELY
 * Instruct the compiler to optimize for the case where the argument equals 0.
 */
# define SPDLOG_ROS_UTILS_UNLIKELY(x) __builtin_expect((x), 0)
#else
/**
 * \def SPDLOG_ROS_UTILS_LIKELY
 * No op since Windows doesn't support providing branch prediction information.
 */
# define SPDLOG_ROS_UTILS_LIKELY(x) (x)
/**
 * \def SPDLOG_ROS_UTILS_UNLIKELY
 * No op since Windows doesn't support providing branch prediction information.
 */
# define SPDLOG_ROS_UTILS_UNLIKELY(x) (x)
#endif  // _WIN32

// This is to avoid compilation warnings in C++ with '-Wold-style-cast'.
#ifdef __cplusplus
#define SPDLOG_ROS_UTILS_CAST_DURATION(x) (static_cast < spdlog_ros_utils_duration_value_t > (x))
#else
#define SPDLOG_ROS_UTILS_CAST_DURATION(x) ((spdlog_ros_utils_duration_value_t)x)
#endif

#endif //SPDLOG_ROS_LOGGING_UTIL_MACROS_HPP
