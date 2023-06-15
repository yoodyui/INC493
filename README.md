# Combined VESC_driver node for Humble and joy node

## How to test

1. Clone this repository by `git clone -b master https://github.com/yoodyui/INC493.git` into `src`.
2. `rosdep install -i --from-path src --rosdistro humble -y`
3. Build the packages `colcon build` or `colcon build --packages-select vesc_driver`
4. Source files `source install/setup.bash`
5. `ros2 launch combine_pkg multi_node.launch.py`
6. If prompted "permission denied" on the serial port: `sudo chmod 777 /dev/ttyACM0`
7. `ros2 topic echo motor_speed` to see the command to BLDC
8. `ros2 topic echo servo_angle` to see the command to servo motor
