//
// Created by cmuehlbacher on 20.12.24.
//

#ifndef SPDLOG_ROS_LOGGING_HPP
#define SPDLOG_ROS_LOGGING_HPP

#include <spdlog_ros/logging_macros.hpp>

#undef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

/** @name Logging macros for severity DEBUG.
 */
///@{
#if (SPDLOG_ACTIVE_LEVEL > SPDLOG_LEVEL_DEBUG)
// empty logging macros for severity DEBUG when being disabled at compile time
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_SKIPFIRST_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_STREAM(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_STREAM_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_STREAM_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_STREAM_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_STREAM_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_THROTTLE(...)

#else
// The SPDLOG_ROS_DEBUG macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG
 * Log a message with severity DEBUG.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_DEBUG(...) SPDLOG_ROS_GENERAL(SPDLOG_ROS_LEVEL_DEBUG, __VA_ARGS__)

// The SPDLOG_ROS_DEBUG_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_ONCE
 * Log a message with severity DEBUG with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_DEBUG_ONCE(...) SPDLOG_ROS_GENERAL_ONCE(SPDLOG_ROS_LEVEL_DEBUG, __VA_ARGS__)

// The SPDLOG_ROS_DEBUG_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_EXPRESSION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_DEBUG_EXPRESSION(expression, ...) SPDLOG_ROS_GENERAL_EXPRESSION(SPDLOG_ROS_LEVEL_DEBUG, expression, __VA_ARGS__)

// The SPDLOG_ROS_DEBUG_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_FUNCTION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_DEBUG_FUNCTION(function, ...) SPDLOG_ROS_GENERAL_FUNCTION(SPDLOG_ROS_LEVEL_DEBUG, function, __VA_ARGS__)

// The SPDLOG_ROS_DEBUG_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_SKIPFIRST
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_DEBUG_SKIPFIRST(...) SPDLOG_ROS_GENERAL_SKIPFIRST(SPDLOG_ROS_LEVEL_DEBUG, __VA_ARGS__)

// The SPDLOG_ROS_DEBUG_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_DEBUG_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_THROTTLE(SPDLOG_ROS_LEVEL_DEBUG, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_DEBUG_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_SKIPFIRST_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_DEBUG_SKIPFIRST_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_DEBUG, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_DEBUG_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_STREAM
 * Log a message with severity DEBUG.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_DEBUG_STREAM(stream_arg) SPDLOG_ROS_GENERAL_STREAM(SPDLOG_ROS_LEVEL_DEBUG, stream_arg)

// The SPDLOG_ROS_DEBUG_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_STREAM_ONCE
 * Log a message with severity DEBUG with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_DEBUG_STREAM_ONCE(stream_arg) SPDLOG_ROS_GENERAL_STREAM_ONCE(SPDLOG_ROS_LEVEL_DEBUG, stream_arg)

// The SPDLOG_ROS_DEBUG_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_STREAM_EXPRESSION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_DEBUG_STREAM_EXPRESSION(expression, stream_arg) SPDLOG_ROS_GENERAL_STREAM_EXPRESSION(SPDLOG_ROS_LEVEL_DEBUG, expression, stream_arg)

// The SPDLOG_ROS_DEBUG_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_STREAM_FUNCTION
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_DEBUG_STREAM_FUNCTION(function, stream_arg) SPDLOG_ROS_GENERAL_STREAM_FUNCTION(SPDLOG_ROS_LEVEL_DEBUG, function, stream_arg)

// The SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST(stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST(SPDLOG_ROS_LEVEL_DEBUG, stream_arg)

// The SPDLOG_ROS_DEBUG_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_STREAM_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_DEBUG_STREAM_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_THROTTLE(SPDLOG_ROS_LEVEL_DEBUG, clock, duration, stream_arg)

// The SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity DEBUG with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_DEBUG_STREAM_SKIPFIRST_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_DEBUG, clock, duration, stream_arg)

#endif

/** @name Logging macros for severity INFO.
 */
///@{
#if (SPDLOG_ACTIVE_LEVEL > SPDLOG_LEVEL_INFO)
// empty logging macros for severity INFO when being disabled at compile time
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_SKIPFIRST_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_STREAM(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_STREAM_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_STREAM_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_STREAM_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_STREAM_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_STREAM_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_INFO_STREAM_SKIPFIRST_THROTTLE(...)

#else
// The SPDLOG_ROS_INFO macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO
 * Log a message with severity INFO.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_INFO(...) SPDLOG_ROS_GENERAL(SPDLOG_ROS_LEVEL_INFO, __VA_ARGS__)

// The SPDLOG_ROS_INFO_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_INFO_ONCE(...) SPDLOG_ROS_GENERAL_ONCE(SPDLOG_ROS_LEVEL_INFO, __VA_ARGS__)

// The SPDLOG_ROS_INFO_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_INFO_EXPRESSION(expression, ...) SPDLOG_ROS_GENERAL_EXPRESSION(SPDLOG_ROS_LEVEL_INFO, expression, __VA_ARGS__)

// The SPDLOG_ROS_INFO_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_INFO_FUNCTION(function, ...) SPDLOG_ROS_GENERAL_FUNCTION(SPDLOG_ROS_LEVEL_INFO, function, __VA_ARGS__)

// The SPDLOG_ROS_INFO_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_INFO_SKIPFIRST(...) SPDLOG_ROS_GENERAL_SKIPFIRST(SPDLOG_ROS_LEVEL_INFO, __VA_ARGS__)

// The SPDLOG_ROS_INFO_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_INFO_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_THROTTLE(SPDLOG_ROS_LEVEL_INFO, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_INFO_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_INFO_SKIPFIRST_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_INFO, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_INFO_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_STREAM
 * Log a message with severity INFO.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_INFO_STREAM(stream_arg) SPDLOG_ROS_GENERAL_STREAM(SPDLOG_ROS_LEVEL_INFO, stream_arg)

// The SPDLOG_ROS_INFO_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_STREAM_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_INFO_STREAM_ONCE(stream_arg) SPDLOG_ROS_GENERAL_STREAM_ONCE(SPDLOG_ROS_LEVEL_INFO, stream_arg)

// The SPDLOG_ROS_INFO_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_STREAM_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_INFO_STREAM_EXPRESSION(expression, stream_arg) SPDLOG_ROS_GENERAL_STREAM_EXPRESSION(SPDLOG_ROS_LEVEL_INFO, expression, stream_arg)

// The SPDLOG_ROS_INFO_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_STREAM_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_INFO_STREAM_FUNCTION(function, stream_arg) SPDLOG_ROS_GENERAL_STREAM_FUNCTION(SPDLOG_ROS_LEVEL_INFO, function, stream_arg)

// The SPDLOG_ROS_INFO_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_STREAM_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_INFO_STREAM_SKIPFIRST(stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST(SPDLOG_ROS_LEVEL_INFO, stream_arg)

// The SPDLOG_ROS_INFO_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_STREAM_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_INFO_STREAM_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_THROTTLE(SPDLOG_ROS_LEVEL_INFO, clock, duration, stream_arg)

// The SPDLOG_ROS_INFO_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_INFO_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_INFO_STREAM_SKIPFIRST_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_INFO, clock, duration, stream_arg)

#endif

/** @name Logging macros for severity WARN.
 */
///@{
#if (SPDLOG_ACTIVE_LEVEL > SPDLOG_LEVEL_WARN)
// empty logging macros for severity INFO when being disabled at compile time
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_SKIPFIRST_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_STREAM(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_STREAM_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_STREAM_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_STREAM_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_STREAM_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_STREAM_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_WARN_STREAM_SKIPFIRST_THROTTLE(...)

#else
// The SPDLOG_ROS_WARN macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN
 * Log a message with severity INFO.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_WARN(...) SPDLOG_ROS_GENERAL(SPDLOG_ROS_LEVEL_WARN, __VA_ARGS__)

// The SPDLOG_ROS_WARN_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_WARN_ONCE(...) SPDLOG_ROS_GENERAL_ONCE(SPDLOG_ROS_LEVEL_WARN, __VA_ARGS__)

// The SPDLOG_ROS_WARN_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_WARN_EXPRESSION(expression, ...) SPDLOG_ROS_GENERAL_EXPRESSION(SPDLOG_ROS_LEVEL_WARN, expression, __VA_ARGS__)

// The SPDLOG_ROS_WARN_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_WARN_FUNCTION(function, ...) SPDLOG_ROS_GENERAL_FUNCTION(SPDLOG_ROS_LEVEL_WARN, function, __VA_ARGS__)

// The SPDLOG_ROS_WARN_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_WARN_SKIPFIRST(...) SPDLOG_ROS_GENERAL_SKIPFIRST(SPDLOG_ROS_LEVEL_WARN, __VA_ARGS__)

// The SPDLOG_ROS_WARN_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_WARN_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_THROTTLE(SPDLOG_ROS_LEVEL_WARN, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_WARN_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_WARN_SKIPFIRST_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_WARN, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_WARN_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_STREAM
 * Log a message with severity INFO.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_WARN_STREAM(stream_arg) SPDLOG_ROS_GENERAL_STREAM(SPDLOG_ROS_LEVEL_WARN, stream_arg)

// The SPDLOG_ROS_WARN_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_STREAM_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_WARN_STREAM_ONCE(stream_arg) SPDLOG_ROS_GENERAL_STREAM_ONCE(SPDLOG_ROS_LEVEL_WARN, stream_arg)

// The SPDLOG_ROS_WARN_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_STREAM_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_WARN_STREAM_EXPRESSION(expression, stream_arg) SPDLOG_ROS_GENERAL_STREAM_EXPRESSION(SPDLOG_ROS_LEVEL_WARN, expression, stream_arg)

// The SPDLOG_ROS_WARN_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_STREAM_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_WARN_STREAM_FUNCTION(function, stream_arg) SPDLOG_ROS_GENERAL_STREAM_FUNCTION(SPDLOG_ROS_LEVEL_WARN, function, stream_arg)

// The SPDLOG_ROS_WARN_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_STREAM_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_WARN_STREAM_SKIPFIRST(stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST(SPDLOG_ROS_LEVEL_WARN, stream_arg)

// The SPDLOG_ROS_WARN_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_STREAM_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_WARN_STREAM_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_THROTTLE(SPDLOG_ROS_LEVEL_WARN, clock, duration, stream_arg)

// The SPDLOG_ROS_WARN_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_WARN_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_WARN_STREAM_SKIPFIRST_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_WARN, clock, duration, stream_arg)

#endif

/** @name Logging macros for severity ERROR.
 */
///@{
#if (SPDLOG_ACTIVE_LEVEL > SPDLOG_LEVEL_ERROR)
// empty logging macros for severity INFO when being disabled at compile time
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_SKIPFIRST_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_STREAM(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_STREAM_ONCE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_STREAM_EXPRESSION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_STREAM_FUNCTION(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_STREAM_SKIPFIRST(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_STREAM_THROTTLE(...)
/// Empty logging macro due to the preprocessor definition of RCLCPP_LOG_MIN_SEVERITY.
#define SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_THROTTLE(...)

#else
// The SPDLOG_ROS_ERROR macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR
 * Log a message with severity INFO.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_ERROR(...) SPDLOG_ROS_GENERAL(SPDLOG_ROS_LEVEL_ERROR, __VA_ARGS__)

// The SPDLOG_ROS_ERROR_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_ERROR_ONCE(...) SPDLOG_ROS_GENERAL_ONCE(SPDLOG_ROS_LEVEL_ERROR, __VA_ARGS__)

// The SPDLOG_ROS_ERROR_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_ERROR_EXPRESSION(expression, ...) SPDLOG_ROS_GENERAL_EXPRESSION(SPDLOG_ROS_LEVEL_ERROR, expression, __VA_ARGS__)

// The SPDLOG_ROS_ERROR_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_ERROR_FUNCTION(function, ...) SPDLOG_ROS_GENERAL_FUNCTION(SPDLOG_ROS_LEVEL_ERROR, function, __VA_ARGS__)

// The SPDLOG_ROS_ERROR_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_ERROR_SKIPFIRST(...) SPDLOG_ROS_GENERAL_SKIPFIRST(SPDLOG_ROS_LEVEL_ERROR, __VA_ARGS__)

// The SPDLOG_ROS_ERROR_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_ERROR_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_THROTTLE(SPDLOG_ROS_LEVEL_ERROR, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_ERROR_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_ERROR_SKIPFIRST_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_ERROR, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_ERROR_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_STREAM
 * Log a message with severity INFO.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_ERROR_STREAM(stream_arg) SPDLOG_ROS_GENERAL_STREAM(SPDLOG_ROS_LEVEL_ERROR, stream_arg)

// The SPDLOG_ROS_ERROR_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_STREAM_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_ERROR_STREAM_ONCE(stream_arg) SPDLOG_ROS_GENERAL_STREAM_ONCE(SPDLOG_ROS_LEVEL_ERROR, stream_arg)

// The SPDLOG_ROS_ERROR_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_STREAM_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_ERROR_STREAM_EXPRESSION(expression, stream_arg) SPDLOG_ROS_GENERAL_STREAM_EXPRESSION(SPDLOG_ROS_LEVEL_ERROR, expression, stream_arg)

// The SPDLOG_ROS_ERROR_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_STREAM_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_ERROR_STREAM_FUNCTION(function, stream_arg) SPDLOG_ROS_GENERAL_STREAM_FUNCTION(SPDLOG_ROS_LEVEL_ERROR, function, stream_arg)

// The SPDLOG_ROS_ERROR_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_STREAM_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_ERROR_STREAM_SKIPFIRST(stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST(SPDLOG_ROS_LEVEL_ERROR, stream_arg)

// The SPDLOG_ROS_ERROR_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_STREAM_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_ERROR_STREAM_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_THROTTLE(SPDLOG_ROS_LEVEL_ERROR, clock, duration, stream_arg)

// The SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_ERROR_STREAM_SKIPFIRST_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_ERROR, clock, duration, stream_arg)

#endif

/** @name Logging macros for severity FATAL.
 */
///@{
// The SPDLOG_ROS_FATAL macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL
 * Log a message with severity INFO.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_FATAL(...) SPDLOG_ROS_GENERAL(SPDLOG_ROS_LEVEL_CRITICAL, __VA_ARGS__)

// The SPDLOG_ROS_FATAL_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_FATAL_ONCE(...) SPDLOG_ROS_GENERAL_ONCE(SPDLOG_ROS_LEVEL_CRITICAL, __VA_ARGS__)

// The SPDLOG_ROS_FATAL_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_FATAL_EXPRESSION(expression, ...) SPDLOG_ROS_GENERAL_EXPRESSION(SPDLOG_ROS_LEVEL_CRITICAL, expression, __VA_ARGS__)

// The SPDLOG_ROS_FATAL_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_FATAL_FUNCTION(function, ...) SPDLOG_ROS_GENERAL_FUNCTION(SPDLOG_ROS_LEVEL_CRITICAL, function, __VA_ARGS__)

// The SPDLOG_ROS_FATAL_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_FATAL_SKIPFIRST(...) SPDLOG_ROS_GENERAL_SKIPFIRST(SPDLOG_ROS_LEVEL_CRITICAL, __VA_ARGS__)

// The SPDLOG_ROS_FATAL_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_FATAL_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_THROTTLE(SPDLOG_ROS_LEVEL_CRITICAL, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_FATAL_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param ... The format string, followed by the variable arguments for the format string.
 * It also accepts a single argument of type std::string.
 */
#define SPDLOG_ROS_FATAL_SKIPFIRST_THROTTLE(clock, duration, ...) SPDLOG_ROS_GENERAL_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_CRITICAL, clock, duration, __VA_ARGS__)

// The SPDLOG_ROS_FATAL_STREAM macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_STREAM
 * Log a message with severity INFO.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_FATAL_STREAM(stream_arg) SPDLOG_ROS_GENERAL_STREAM(SPDLOG_ROS_LEVEL_CRITICAL, stream_arg)

// The SPDLOG_ROS_FATAL_STREAM_ONCE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_STREAM_ONCE
 * Log a message with severity INFO with the following conditions:
 * All subsequent log calls except the first one are being ignored.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_FATAL_STREAM_ONCE(stream_arg) SPDLOG_ROS_GENERAL_STREAM_ONCE(SPDLOG_ROS_LEVEL_CRITICAL, stream_arg)

// The SPDLOG_ROS_FATAL_STREAM_EXPRESSION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_STREAM_EXPRESSION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the expression evaluates to false.
 * \param expression The expression determining if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_FATAL_STREAM_EXPRESSION(expression, stream_arg) SPDLOG_ROS_GENERAL_STREAM_EXPRESSION(SPDLOG_ROS_LEVEL_CRITICAL, expression, stream_arg)

// The SPDLOG_ROS_FATAL_STREAM_FUNCTION macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_STREAM_FUNCTION
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored when the function returns false.
 * \param function The functions return value determines if the message should be logged
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_FATAL_STREAM_FUNCTION(function, stream_arg) SPDLOG_ROS_GENERAL_STREAM_FUNCTION(SPDLOG_ROS_LEVEL_CRITICAL, function, stream_arg)

// The SPDLOG_ROS_FATAL_STREAM_SKIPFIRST macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_STREAM_SKIPFIRST
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_FATAL_STREAM_SKIPFIRST(stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST(SPDLOG_ROS_LEVEL_CRITICAL, stream_arg)

// The SPDLOG_ROS_FATAL_STREAM_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_STREAM_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_FATAL_STREAM_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_THROTTLE(SPDLOG_ROS_LEVEL_CRITICAL, clock, duration, stream_arg)

// The SPDLOG_ROS_FATAL_STREAM_SKIPFIRST_THROTTLE macro is surrounded by do { .. } while (0)
// to implement the standard C macro idiom to make the macro safe in all
// contexts; see http://c-faq.com/cpp/multistmt.html for more information.
/**
 * \def SPDLOG_ROS_FATAL_STREAM_SKIPFIRST_THROTTLE
 * Log a message with severity INFO with the following conditions:
 * The first log call is being ignored but all subsequent calls are being processed.
 * Log calls are being ignored if the last logged message is not longer ago than the specified duration.
 * \param clock rclcpp::Clock that will be used to get the time point.
 * \param duration The duration of the throttle interval as an integral value in milliseconds.
 * \param stream_arg The argument << into a stringstream
 */
#define SPDLOG_ROS_FATAL_STREAM_SKIPFIRST_THROTTLE(clock, duration, stream_arg) SPDLOG_ROS_GENERAL_STREAM_SKIPFIRST_THROTTLE(SPDLOG_ROS_LEVEL_CRITICAL, clock, duration, stream_arg)

#endif //SPDLOG_ROS_LOGGING_HPP
