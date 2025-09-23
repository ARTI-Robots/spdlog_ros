//
// Created by cmuehlbacher on 20.12.24.
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



#ifndef SPDLOG_ROS_LOGGING_MACROS_HPP
#define SPDLOG_ROS_LOGGING_MACROS_HPP

#include <spdlog/spdlog.h>
#include "spdlog_ros/logger.hpp"

// The SPDLOG_ROS_UTILS_LOG_COND macro is surrounded by do { .. } while (0) to implement
// the standard C macro idiom to make the macro safe in all contexts; see
// http://c-faq.com/cpp/multistmt.html for more information.
#define SPDLOG_ROS_UTILS_LOG_COND(name, severity, condition_before, condition_after, ...) \
  do { \
      condition_before \
      SPDLOG_LOGGER_CALL((spdlog_ros::Logger::GetInstance()->getLogger(name)), severity, __VA_ARGS__); \
      condition_after \
  } while (0)
///@{
/**
 * \def SPDLOG_ROS_CONDITION_EMPTY
 * An empty macro which can be used as a placeholder for `condition_before`
 * and `condition_after` which doesn't affect the logging call.
 */
#define SPDLOG_ROS_CONDITION_EMPTY
///@}

/** @name Macros for the `once` condition which ignores all subsequent log
 * calls except the first one.
 */
///@{
/**
 * \def SPDLOG_ROS_CONDITION_ONCE_BEFORE
 * A macro initializing and checking the `once` condition.
 */
#define SPDLOG_ROS_CONDITION_ONCE_BEFORE \
  { \
    static int __spdlog_ros_utils_logging_once = 0; \
    if (SPDLOG_ROS_UTILS_UNLIKELY(0 == __spdlog_ros_utils_logging_once)) { \
      __spdlog_ros_utils_logging_once = 1;
/**
 * \def SPDLOG_ROS_CONDITION_ONCE_AFTER
 * A macro finalizing the `once` condition.
 */
#define SPDLOG_ROS_CONDITION_ONCE_AFTER } \
}
///@}

/** @name Macros for the `expression` condition which ignores the log calls
 * when the expression evaluates to false.
 */
///@{
/**
 * \def SPDLOG_ROS_CONDITION_EXPRESSION_BEFORE
 * A macro checking the `expression` condition.
 */
#define SPDLOG_ROS_CONDITION_EXPRESSION_BEFORE(expression) \
  if (expression) {
/**
 * \def SPDLOG_ROS_CONDITION_EXPRESSION_AFTER
 * A macro finalizing the `expression` condition.
 */
#define SPDLOG_ROS_CONDITION_EXPRESSION_AFTER }
///@}

/** @name Macros for the `function` condition which ignores the log calls
 * when the function returns false.
 */
/**
 * \def SPDLOG_ROS_CONDITION_FUNCTION_BEFORE
 * A macro checking the `function` condition.
 */
#define SPDLOG_ROS_CONDITION_FUNCTION_BEFORE(function) \
  if ((*function)()) {
/**
 * \def SPDLOG_ROS_CONDITION_FUNCTION_AFTER
 * A macro finalizing the `function` condition.
 */
#define SPDLOG_ROS_CONDITION_FUNCTION_AFTER }
///@}

/** @name Macros for the `skipfirst` condition which ignores the first log
 * call but processes all subsequent calls.
 */
///@{
/**
 * \def SPDLOG_ROS_CONDITION_SKIPFIRST_BEFORE
 * A macro initializing and checking the `skipfirst` condition.
 */
#define SPDLOG_ROS_CONDITION_SKIPFIRST_BEFORE \
  { \
    static bool __spdlog_ros_utils_logging_first = true; \
    if (SPDLOG_ROS_UTILS_UNLIKELY(true == __spdlog_ros_utils_logging_first)) { \
      __spdlog_ros_utils_logging_first = false; \
    } else {
/**
 * \def SPDLOG_ROS_CONDITION_SKIPFIRST_AFTER
 * A macro finalizing the `skipfirst` condition.
 */
#define SPDLOG_ROS_CONDITION_SKIPFIRST_AFTER } \
}
///@}

/** @name Macros for the `throttle` condition which ignores log calls if the
 * last logged message is not longer ago than the specified duration.
 */
///@{
/**
 * \def SPDLOG_ROS_CONDITION_THROTTLE_BEFORE
 * A macro initializing and checking the `throttle` condition.
 */
#define SPDLOG_ROS_CONDITION_THROTTLE_BEFORE(get_time_point_value, duration) { \
    static spdlog_ros_utils_duration_value_t __spdlog_ros_utils_logging_duration = SPDLOG_ROS_UTILS_MS_TO_NS(SPDLOG_ROS_UTILS_CAST_DURATION(duration)); \
    static spdlog_ros_utils_duration_value_t __spdlog_ros_utils_logging_last_logged = 0; \
    spdlog_ros_utils_time_point_value_t __spdlog_ros_utils_logging_now = 0; \
    bool __spdlog_ros_utils_logging_condition = true; \
    if (get_time_point_value(&__spdlog_ros_utils_logging_now) != SPDLOG_ROS_UTILS_RET_OK) { \
      SPDLOG_ERROR( \
        "%s() at %s:%d getting current steady time failed\n", \
        __func__, __FILE__, __LINE__); \
    } else { \
      __spdlog_ros_utils_logging_condition = __spdlog_ros_utils_logging_now >= __spdlog_ros_utils_logging_last_logged + __spdlog_ros_utils_logging_duration; \
    } \
 \
    if (SPDLOG_ROS_UTILS_LIKELY(__spdlog_ros_utils_logging_condition)) { \
      __spdlog_ros_utils_logging_last_logged = __spdlog_ros_utils_logging_now;

/**
 * \def SPDLOG_ROS_CONDITION_THROTTLE_AFTER
 * A macro finalizing the `throttle` condition.
 */
#define SPDLOG_ROS_CONDITION_THROTTLE_AFTER } \
}
///@}

# define SPDLOG_ROS_UTILS_LOG(name, severity, ...) \
  SPDLOG_ROS_UTILS_LOG_COND( \
    name, \
    severity, \
    SPDLOG_ROS_CONDITION_EMPTY, SPDLOG_ROS_CONDITION_EMPTY, \
    __VA_ARGS__)

# define SPDLOG_ROS_UTILS_LOG_ONCE(name, severity, ...) \
  SPDLOG_ROS_UTILS_LOG_COND( \
    name, \
    severity, \
    SPDLOG_ROS_CONDITION_ONCE_BEFORE, SPDLOG_ROS_CONDITION_ONCE_AFTER, \
    __VA_ARGS__)

# define SPDLOG_ROS_UTILS_LOG_EXPRESSION(name, severity, expression, ...) \
  SPDLOG_ROS_UTILS_LOG_COND( \
    name, \
    severity, \
    SPDLOG_ROS_CONDITION_EXPRESSION_BEFORE(expression), SPDLOG_ROS_CONDITION_EXPRESSION_AFTER, \
    __VA_ARGS__)

# define SPDLOG_ROS_UTILS_LOG_FUNCTION(name, severity, function, ...) \
  SPDLOG_ROS_UTILS_LOG_COND( \
    name, \
    severity, \
    SPDLOG_ROS_CONDITION_FUNCTION_BEFORE(function), SPDLOG_ROS_CONDITION_FUNCTION_AFTER, \
    __VA_ARGS__)

# define SPDLOG_ROS_UTILS_LOG_SKIPFIRST(name, severity, ...) \
  SPDLOG_ROS_UTILS_LOG_COND( \
    name, \
    severity, \
    SPDLOG_ROS_CONDITION_SKIPFIRST_BEFORE, SPDLOG_ROS_CONDITION_SKIPFIRST_AFTER, \
    __VA_ARGS__)

# define SPDLOG_ROS_UTILS_LOG_THROTTLE(name, severity, get_time_point_value, duration, ...) \
  SPDLOG_ROS_UTILS_LOG_COND( \
    name, \
    severity, \
    SPDLOG_ROS_CONDITION_THROTTLE_BEFORE(get_time_point_value, duration), SPDLOG_ROS_CONDITION_THROTTLE_AFTER, \
    __VA_ARGS__)

# define SPDLOG_ROS_UTILS_LOG_SKIPFIRST_THROTTLE(name, severity, get_time_point_value, duration, ...) \
  SPDLOG_ROS_UTILS_LOG_COND( \
    name, \
    severity, \
    SPDLOG_ROS_CONDITION_THROTTLE_BEFORE(get_time_point_value, duration) SPDLOG_ROS_CONDITION_SKIPFIRST_BEFORE, SPDLOG_ROS_CONDITION_THROTTLE_AFTER SPDLOG_ROS_CONDITION_SKIPFIRST_AFTER, \
    __VA_ARGS__)


/** @name Logging macros for all severities.
 */
///@{
// The SPDLOG_ROS_GENERAL macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL
 * Log a message with severity DEBUG.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_GENERAL(severity, ...) \
  do { \
    SPDLOG_ROS_UTILS_LOG( \
       "", \
       severity, \
      __VA_ARGS__); \
  } while (0)

// The SPDLOG_ROS_GENERAL_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_ONCE
 * Log a message with severity DEBUG with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_GENERAL_ONCE(severity, ...) \
  do { \
    SPDLOG_ROS_UTILS_LOG_ONCE(     \
      "", \
      severity, \
      __VA_ARGS__); \
  } while (0)

// The SPDLOG_ROS_GENERAL_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_EXPRESSION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_GENERAL_EXPRESSION(severity, expression, ...) \
  do { \
    SPDLOG_ROS_UTILS_LOG_EXPRESSION( \
      "", \
      severity, \
      expression, \
      __VA_ARGS__); \
  } while (0)

// The SPDLOG_ROS_GENERAL_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_FUNCTION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_GENERAL_FUNCTION(severity, function, ...) \
  do { \
    SPDLOG_ROS_UTILS_LOG_FUNCTION( \
      "", \
      severity, \
      function, \
      __VA_ARGS__); \
  } while (0)

// The SPDLOG_ROS_GENERAL_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_SKIPFIRST
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_GENERAL_SKIPFIRST(severity, ...) \
  do { \
    SPDLOG_ROS_UTILS_LOG_SKIPFIRST( \
      "", \
      severity, \
      __VA_ARGS__); \
  } while (0)

// The SPDLOG_ROS_GENERAL_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_GENERAL_THROTTLE(severity, duration, ...) \
  do { \
    SPDLOG_ROS_UTILS_LOG_THROTTLE( \
      "", \
      severity, \
      spdlog_ros::Logger::GetInstance()->getTimePointCallback(), \
      duration, \
      __VA_ARGS__); \
  } while (0)

// The SPDLOG_ROS_GENERAL_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_SKIPFIRST_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_GENERAL_SKIPFIRST_THROTTLE(severity, duration, ...) \
  do { \
    SPDLOG_ROS_UTILS_LOG_SKIPFIRST_THROTTLE( \
      "", \
      severity, \
      spdlog_ros::Logger::GetInstance()->getTimePointCallback(), \
      duration, \
      __VA_ARGS__); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM
 * Log a message with severity DEBUG.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM(severity, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG(                   \
      "", \
      severity, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_NAMED macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM
 * Log a message with severity DEBUG.
 * \param name name of the logger prepended to the message
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_NAMED(severity, name, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG(                   \
      name, \
      severity, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_ONCE
 * Log a message with severity DEBUG with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_ONCE(severity, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_ONCE( \
      "", \
      severity, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_ONCE_NAMED macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_ONCE
 * Log a message with severity DEBUG with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param name name of the logger prepended to the message
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_ONCE_NAMED(severity, name, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_ONCE( \
      name, \
      severity, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_EXPRESSION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_EXPRESSION(severity, expression, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_EXPRESSION( \
      "", \
      severity, \
      expression, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_EXPRESSION_NAMED macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_EXPRESSION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param name name of the logger prepended to the message
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_EXPRESSION_NAMED(severity, expression, name, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_EXPRESSION( \
      name, \
      severity, \
      expression, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_FUNCTION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_FUNCTION(severity, function, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_FUNCTION( \
      "", \
      severity, \
      function, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_FUNCTION_NAMED macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_FUNCTION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param name name of the logger prepended to the message
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_FUNCTION_NAMED(severity, function, name, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_FUNCTION( \
      name, \
      severity, \
      function, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST(severity, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_SKIPFIRST( \
      "", \
      severity, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_NAMED macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param name name of the logger prepended to the message
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_NAMED(severity, name, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_SKIPFIRST( \
      name, \
      severity, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_THROTTLE(severity, duration, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_THROTTLE( \
      "", \
      severity, \
      spdlog_ros::Logger::GetInstance()->getTimePointCallback(), \
      duration, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_THROTTLE_NAMED macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param name name of the logger prepended to the message
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_THROTTLE_NAMED(severity, duration, name, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_THROTTLE( \
      name, \
      severity, \
      spdlog_ros::Logger::GetInstance()->getTimePointCallback(), \
      duration, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE(severity, duration, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_SKIPFIRST_THROTTLE( \
      "", \
      severity, \
      spdlog_ros::Logger::GetInstance()->getTimePointCallback(), \
      duration, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

// The SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE_NAMED
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param name name of the logger prepended to the message
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE_NAMED(severity, duration, name, stream_arg) \
  do { \
    std::stringstream spllog_ros_stream_ss_; \
    spllog_ros_stream_ss_ << stream_arg; \
    SPDLOG_ROS_UTILS_LOG_SKIPFIRST_THROTTLE( \
      name, \
      severity, \
      spdlog_ros::Logger::GetInstance()->getTimePointCallback(), \
      duration, \
      "{}", spllog_ros_stream_ss_.str().c_str()); \
  } while (0)

///@}
#endif //SPDLOG_ROS_LOGGING_MACROS_HPP
