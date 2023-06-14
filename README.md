# Veddar VESC Interface

![ROS2 CI Workflow](https://github.com/f1tenth/vesc/workflows/ROS2%20CI%20Workflow/badge.svg)

Packages to interface with Veddar VESC motor controllers. See https://vesc-project.com/ for details

This is a ROS2 implementation of the ROS1 driver using the new serial driver located in [transport drivers](https://github.com/ros-drivers/transport_drivers).

## How to test

1. Clone this repository by `git clone -b humble https://github.com/yoodyui/INC493.git` into `src`.
2. `rosdep install -i --from-path src --rosdistro humble -y`
3. Plug in the VESC with a USB cable.
4. Modify `vesc/vesc_driver/src/vesc_driver.cpp` to reflect any changes on BP's comments.
5. Build the packages `colcon build`
6. `ros2 launch vesc_driver vesc_driver_node.launch.py`
7. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`
