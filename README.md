# ssg48_adaptive_electric_gripper_ros2

for candleLight USB to CAN adapter
sudo ip link set dev can0 up type can bitrate 1000000

ros2 launch ssg48_gripper_description ssg48_gripper.launch.py 



ros2 action send_goal /ssg48_gripper/homing ssg48_gripper_msgs/action/Homing {}


ros2 action send_goal -f /ssg48_gripper/move ssg48_gripper_msgs/action/Move "{width: 0.048, speed: 0.02}"


ros2 action send_goal -f /ssg48_gripper/grasp ssg48_gripper_msgs/action/Grasp "{width: 0.02, speed: 0.02, force: 200.0, epsilon: 0.01}"
