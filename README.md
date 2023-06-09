# These steps are for students in INC493 class

# 1) To use the vesc, you need to download vesc-foxy to your src folder and colcon build

#connect vesc and run vesc node by:

sudo chmod 777 /dev/ttyACM0

ros2 launch vesc_driver vesc_driver_node.launch.py

#The vesc node is subscribing cmd_vel messages, let us try to publish cmd_vel from the terminal using:

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.1}}'

# 2) you need to install a joy package by:

apt install ros-foxy-joy 

colcon build

. install/setup.bash

#Run joy node and echo joy messages

ros2 run joy joy_node

ros2 topic echo joy

#Check out the list of buttons from http://wiki.ros.org/joy


# 3) you need to convert joy messages to cmd_vel by running teleop_node and echo cmd_vel messages

ros2 run teleop_twist_joy teleop_node

ros2 topic echo cmd_vel 

#Now, teleop_node is publishing cmd_vel messages

# 4) Let us run the 3 nodes, joy node in step 2, teleop node in step 3, and vesc node in step 1, all together. Now, you should see the wheels spin 😁.

#Echo command to BLDC

ros2 topic echo sensors/bldc_speed_command

#Echo command to servo

ros2 topic echo sensors/servo_position_command
![image](https://github.com/yoodyui/INC493/assets/42739618/008d83a9-da21-4c91-aa3e-c4300b42732d)
