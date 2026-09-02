//
// Created by Fabian Hirmann on 19.09.2025.
//

#pragma once

#include <rclcpp/rclcpp.hpp>

// Note that the target version is ROS 2 Humble which does not yet have integrated services and messages for logging
// Beginning with ROS 2 Iron and upward, the existing service definitions of rcl_interfaces could be used:
// https://github.com/ros2/rcl_interfaces/blob/rolling/rcl_interfaces/README.md#services
#include <spdlog_ros/srv/get_loggers.hpp>
#include <spdlog_ros/srv/set_logger_level.hpp>

namespace spdlog_ros
{

class ROSLoggingManager
{
public:
  ROSLoggingManager(const ROSLoggingManager& other) = delete;
  ROSLoggingManager& operator=(const ROSLoggingManager& other) = delete;

  ROSLoggingManager(rclcpp::Node::SharedPtr node);
  ROSLoggingManager(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface);
  
  template <typename NodeT>
  ROSLoggingManager(NodeT& node)
  // Note(fhirmann): Pass-by-reference is required since NodeT (at least in case of rclcpp::Node and
  // rclcpp_lifecycle::LifecycleNode) does not have a copy-constructor. The unusual non-const reference is required
  // because the getters for the node interfaces are not const and therefore cannot be accessed from a const reference.
  // The getters are also correctly to be non-const although they just return shared pointers to the interfaces (which
  // technically could be const), because using the returned shared pointers, the user can modify the underlying
  // interfaces and therefore the node itself. In the future rclcpp could provide a const variant that is returning a
  // shared pointer to const but in this case this would not help since the ROSLoggingManager requires creating
  // publishers and services and this is in any case non-const.
    : ROSLoggingManager(node.get_node_base_interface(),
                        node.get_node_clock_interface(),
                        node.get_node_topics_interface(),
                        node.get_node_services_interface(),
                        node.get_node_parameters_interface())
  {
  }

  ~ROSLoggingManager();

private:
  bool getLoggersCallback(
    const std::shared_ptr<spdlog_ros::srv::GetLoggers::Request> request,
    const std::shared_ptr<spdlog_ros::srv::GetLoggers::Response> response);

  bool setLoggerLevelCallback(
    const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Request> request,
    const std::shared_ptr<spdlog_ros::srv::SetLoggerLevel::Response> response);

  void setUpROSLogging(rclcpp::Node::SharedPtr node);
  void setUpROSLogging(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_interface,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock_interface,
    rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface
  );

  rclcpp::Service<spdlog_ros::srv::GetLoggers>::SharedPtr get_loggers_srv_;
  rclcpp::Service<spdlog_ros::srv::SetLoggerLevel>::SharedPtr set_logger_level_srv_;

  static size_t reference_count_;
};

}  // namespace spdlog_ros