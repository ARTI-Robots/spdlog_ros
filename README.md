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