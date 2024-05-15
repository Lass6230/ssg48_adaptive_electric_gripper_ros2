import math

from geometry_msgs.msg import TransformStamped

import numpy as np
import can

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose
import Spectral_BLDC as Spectral
import time

from sensor_msgs.msg import JointState
from ssg48_gripper_msgs.action import Grasp, Homing, Move


from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class ssg48Gripper(Node):

    def __init__(self):
        super().__init__('ssg48_gripper')
        # sudo ip link set dev can0 up type can bitrate 1000000
        

        # Initialize the transform broadcaster
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        self._action_server = ActionServer(
            self,
            Grasp,
            'ssg48_gripper/grasp',
            self.execute_grasp_callback)
        
        self._action_server = ActionServer(
            self,
            Homing,
            'ssg48_gripper/homing',
            self.execute_homing_callback)
        
        self._action_server = ActionServer(
            self,
            Move,
            'ssg48_gripper/move',
            self.execute_move_callback)

        connected = 0
        try:

            self.Communication1 = Spectral.CanCommunication(bustype='socketcan', channel='can0', bitrate=1000000)
            print("connected with socketcan")
            connected = 1
        except:
            print("not connected with socketcan")
            connected = 0
        if connected == 0:
            try:
                self.Communication1 = Spectral.CanCommunication(bustype='slcan', channel='/dev/ttyACM0', bitrate=1000000)
                print("connected with slcan")
            except:
                print("not connected with slcan")
        
        # bus = can.Bus(interface="gs_usb", channel=dev.product, index=0, bitrate=250000)
        self.Gripper = Spectral.SpectralCAN(node_id=0, communication=self.Communication1)
        self.Gripper.Send_Clear_Error()

        self.Gripper.Send_gripper_calibrate()

        timer_period = 0.05  # seconds
        self.i = 1
        time.sleep(3.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.timer = self.create_timer(4.0, self.timer_callback2)
        # self.var = 1

        self.max_width = 0.048
        self.position_mm = 0.0
        
    
    def execute_grasp_callback(self, goal_handle):
        self.get_logger().info('Executing grasp goal...')
        

        self.Gripper.Send_gripper_data_pack(int((1-(goal_handle.request.width*255))*self.max_width),int(goal_handle.request.speed),int(goal_handle.request.force),1,1,0,0) 


    def execute_homing_callback(self, goal_handle):
        self.get_logger().info('Executing homing goal...')
        self.Gripper.Send_Clear_Error()

        self.Gripper.Send_gripper_calibrate()


    def execute_move_callback(self, goal_handle):
        self.get_logger().info('Executing move goal...')
        # goal_handle.request
        self.Gripper.Send_gripper_data_pack(((1-(goal_handle.request.width/255))*self.max_width),goal_handle.request.speed,goal_handle.request.force,1,1,0,0) 




    def timer_callback2(self):
        if self.var == 0:
        #Motor1.Send_Clear_Error()
            # self.Gripper.Send_gripper_calibrate()
            self.Gripper.Send_gripper_data_pack(200,20,500,1,1,0,0) 
        # Motor1.Send_gripper_data_pack(50,20,500,1,1,0,0) 
            self.var = 1
        elif self.var == 1:
            self.Gripper.Send_gripper_data_pack(100,20,500,1,1,0,0) 
            self.var = 2
        elif self.var == 2:
            self.Gripper.Send_gripper_data_pack(10,20,500,1,1,0,0) 
            self.var = 0

    def timer_callback(self):
        self.Gripper.Send_gripper_data_pack()
        message, UnpackedMessageID = self.Communication1.receive_can_messages(timeout=0.01)
        if message is not None:
            self.Gripper.UnpackData(message,UnpackedMessageID)
            if(UnpackedMessageID.command_id == 60):

                self.get_logger().info('position: "%s"' % str(self.Gripper.gripper_position))

                self.get_logger().info('number: "%s"' % str(((1-(self.Gripper.gripper_position/255))*self.max_width)))
                
                self.position_mm = ((1-(self.Gripper.gripper_position/255))*self.max_width)
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.header.frame_id = ''

                joint_state.name.append('left_gripper_finger_joint')
                joint_state.position.append(self.position_mm/2)
                # joint_state.velocity[0] =
                # joint_state.effort[0] = 

                joint_state.name.append('right_gripper_finger_joint')
                joint_state.position.append(self.position_mm/2)
                # joint_state.velocity[0] =
                # joint_state.effort[0] = 
                self.publisher_.publish(joint_state)
                
            else:
                self.get_logger().info('UnpackedMessageID.command_id == 60 is not valid')
        else:
            self.get_logger().info('No message after timeout period!')

 


def main():
    rclpy.init()
    
    node = ssg48Gripper()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()