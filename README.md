# ssg48_adaptive_electric_gripper_ros2

This is a package for using ros2 to control the Source Robotics SSG48 Gripper. See link https://github.com/PCrnjak/SSG-48-adaptive-electric-gripper

for candleLight USB to CAN adapter:
sudo ip link set dev can0 up type can bitrate 1000000

ROS2 Humble

```Installation
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/Lass6230/ssg48_adaptive_electric_gripper_ros2.git

cd ..

colcon build
source install/setup.bash

ros2 launch ssg48_gripper ssg48_gripper.launch.py
```

```
ros2 launch ssg48_gripper ssg48_gripper.launch.py bustype:="socketcan" channel:="can0" bitrate:=1000000
```


To use the gripper three actions are provided. Homing, Move and Grasp. Grasp have a input called epsilon. its the value the end position are allowed th deviate from the goal position and the grasp is a success. the can be used to evaluate if anything have been grasp.

```
ros2 action send_goal /ssg48_gripper/homing ssg48_gripper_msgs/action/Homing {}

ros2 action send_goal -f /ssg48_gripper/move ssg48_gripper_msgs/action/Move "{width: 0.048, speed: 0.02}"

ros2 action send_goal -f /ssg48_gripper/grasp ssg48_gripper_msgs/action/Grasp "{width: 0.02, speed: 0.02, force: 15.0, epsilon: 0.01}"
```
