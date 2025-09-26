cmake_minimum_required(VERSION 3.8)
project(spdlog_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake QUIET)
find_package(spdlog REQUIRED)

# ROS Version
message(STATUS "ROS_VERSION precheck: ENV{ROS_VERSION}=$ENV{ROS_VERSION} ROS_VERSION=${ROS_VERSION} CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}")
if(DEFINED CMAKE_PREFIX_PATH AND "${CMAKE_PREFIX_PATH}" STREQUAL "/opt/ros/humble")
  set(ROS_VERSION 2)
endif()
if(NOT DEFINED ROS_VERSION AND DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
endif()
message(STATUS "ROS_VERSION=${ROS_VERSION}")
add_compile_options(-D__ROS_VERSION=${ROS_VERSION})

if ( ament_cmake_FOUND )

  add_definitions( -DUSING_ROS2 )
  message(STATUS "------------------------------------------")
  message(STATUS "Building using COLCON.")
  message(STATUS "------------------------------------------")

  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(spdlog_vendor REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(msg_files
    "msg/Logger.msg"
  )

  set(srv_files
    "srv/GetLoggers.srv"
    "srv/SetLoggerLevel.srv"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
  )

  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

  add_library(${PROJECT_NAME}_lib src/ros2_sink.cpp src/logger_manager.cpp src/ros2_logging.cpp)

  target_link_libraries(${PROJECT_NAME}_lib
    spdlog::spdlog
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(${PROJECT_NAME}_lib
    spdlog_vendor
    rclcpp
    rcl_interfaces
  )

  target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  add_executable(example_ros2 src/example_ros2.cpp)
  target_link_libraries(example_ros2 ${PROJECT_NAME}_lib "${cpp_typesupport_target}")
  target_include_directories(example_ros2 PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  install(TARGETS
    ${PROJECT_NAME}_lib
    example_ros2
    DESTINATION lib/${PROJECT_NAME})

  ament_export_dependencies(spdlog_vendor rclcpp rcl_interfaces rosidl_default_runtime)
  # ament_export_include_directories(include)  
  # ament_export_libraries(${PROJECT_NAME})
  ament_package()

else()
  add_definitions( -DUSING_ROS )
  message(STATUS "------------------------------------------")
  message(STATUS "Building using CATKIN.")
  message(STATUS "------------------------------------------")

  find_package(catkin REQUIRED COMPONENTS roscpp)

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp
    DEPENDS )

  include_directories(${catkin_INCLUDE_DIRS})

  add_library(${PROJECT_NAME} src/ros_sink.cpp src/logger_manager.cpp src/ros_logging.cpp)
  
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(${PROJECT_NAME}
    spdlog::spdlog
    ${catkin_LIBRARIES}
  )

  add_executable(example_ros src/example_ros.cpp)
  target_link_libraries(example_ros ${PROJECT_NAME} ${catkin_LIBRARIES})
  target_include_directories(example_ros PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

  install(TARGETS ${PROJECT_NAME} example_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
endif()






