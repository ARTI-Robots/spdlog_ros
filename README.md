# spdlog-ros

## Install

### Dependencies

- [spdlog](https://github.com/gabime/spdlog) (version >= 1.6.1)
  - For Ubuntu 22.04 and later, spdlog with the matching version can be installed via `apt`:
    ```bash
    sudo apt install libspdlog-dev
    ```
  - For Ubuntu 20.04, one can install the required version of spdlog from source:
      ```bash
      git clone -b v1.6.1 https://github.com/gabime/spdlog.git
      cd spdlog && mkdir build && cd build
      cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DSPDLOG_BUILD_SHARED=ON
      make -j$(nproc)
      sudo make install
      ```

### Build

Clone this repository into your ROS1 or ROS2 workspace source directory and build as usual:

ROS1:
```bash
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
ROS2:
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

## Quick usage guide

1. Set up logger as defined in `example_ros.cpp` or `example_ros2.cpp`
    - Note that at shutdown of the node the `ROSLoggingManager` should be destructed such that the logging properly shuts down without crash at the end
    - The easiest way to do this is in RAII-style as shown in `example_ros.cpp` and `example_ros2.cpp`
    - Also note that multiple initializations of `ROSLoggingManager` does not harm and only when the last `ROSLoggingManager` is getting destructed, the logging is shut down
2. Log using `SPDLOG_ROS_*` similar to logging with ROS 1
    - Note that format strings are **not** using the ROS 1 / printf syntax but instead the syntax of the {fmt} library (see examples in `example_ros.cpp` and `example_ros2.cpp`)