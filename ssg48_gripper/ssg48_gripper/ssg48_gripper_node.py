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
        
        self.declare_parameter('bustype', 'bustype')
        self.declare_parameter('channel', 'channel')
        self.declare_parameter('bitrate', 'bitrate')

        # Initialize the transform broadcaster
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher_ = self.create_publisher(JointState, 'gripper_joint_states', 10)

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
            bustype_param = self.get_parameter('bustype').get_parameter_value().string_value
            channel_param = self.get_parameter('channel').get_parameter_value().string_value
            bitrate_param = self.get_parameter('bitrate').get_parameter_value().integer_value
            self.Communication1 = Spectral.CanCommunication(bustype=bustype_param, channel=channel_param, bitrate=bitrate_param)
            print("connected with socketcan")
            connected = 1
        except:
            print("not connected with socketcan")
            connected = 0
        if connected == 0:
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
        time.sleep(0.2)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.timer = self.create_timer(4.0, self.timer_callback2)
        self.var = 1

        self.max_width = 0.048
        self.position = 0.0 #[m]
        self.pre_position = 0.0 #[tiks]
        self.speed = 0.0 #[m/s]
        self.effort = 0.0 #[N]  #force
        self.radius = 0.006
        self.encoder_resolution = pow(2,14)
        self.effort_factor = (50.0-20.0)/(1100-450)
        print("encoder esolution: ", self.encoder_resolution)
    
    def map(self,x,in_min,in_max,out_min,out_max):
          return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        
    
    def execute_grasp_callback(self, goal_handle):
        self.get_logger().info('Executing grasp goal...')
        
        desired_speed = int((goal_handle.request.speed*self.encoder_resolution)/(2*math.pi*self.radius))
        desired_speed = int(self.map(desired_speed,40,80000,0,255))
        self.Gripper.Send_gripper_data_pack(int(((1-(goal_handle.request.width/self.max_width))*255)),desired_speed,int(goal_handle.request.force),1,1,0,0) 
        self.get_logger().info('position [m]: "%s"' % str(int(((1-(goal_handle.request.width/self.max_width))*255))))
        goal_handle.succeed()

        grasp = Grasp.Result()
        grasp.success = True
        grasp.error = "No error"
        return grasp

    def execute_homing_callback(self, goal_handle):
        self.get_logger().info('Executing homing goal...')
        self.Gripper.Send_Clear_Error()

        self.Gripper.Send_gripper_calibrate()

        goal_handle.succeed()
        return Homing.Result()


    def execute_move_callback(self, goal_handle):
        self.get_logger().info('Executing move goal...')
        # goal_handle.request
        desired_speed = int((goal_handle.request.speed*self.encoder_resolution)/(2*math.pi*self.radius))
        desired_speed = int(self.map(desired_speed,40,80000,0,255))
        # self.get_logger().info('speed [m/s]: "%s"' % str(desired_speed))
        self.Gripper.Send_gripper_data_pack(int(((1-(goal_handle.request.width/self.max_width))*255)),int(desired_speed),300,1,1,0,0) 
        
        self.get_logger().info('position [m]: "%s"' % str(int(((1-(goal_handle.request.width/self.max_width))*255))))
        
        goal_handle.succeed()

        move = Move.Result()
        move.success = True
        move.error = "No error"+str(int(((1-(goal_handle.request.width/self.max_width))*255)))
        return move

    def timer_callback2(self):
        if self.var == 0:
        #Motor1.Send_Clear_Error()
            # self.Gripper.Send_gripper_calibrate()
            self.Gripper.Send_gripper_data_pack(240,20,700,1,1,0,0) 
            
        # Motor1.Send_gripper_data_pack(50,20,500,1,1,0,0) 
            self.var = 1
        elif self.var == 1:
            self.Gripper.Send_gripper_data_pack(0,20,700,1,1,0,0) 
            self.var = 2
        elif self.var == 2:
            self.Gripper.Send_gripper_data_pack(0,20,700,1,1,0,0) 
            self.var = 0

    def timer_callback(self):
        self.Gripper.Send_gripper_data_pack()
        message, UnpackedMessageID = self.Communication1.receive_can_messages(timeout=0.01)
        if message is not None:
            self.Gripper.UnpackData(message,UnpackedMessageID)
            if(UnpackedMessageID.command_id == 60):

                # self.get_logger().info('position [m]: "%s"' % str(self.Gripper.gripper_position))

                
                
                self.position = ((1-(self.Gripper.gripper_position/255))*self.max_width)
                self.speed = ((self.pre_position-self.Gripper.gripper_position)/self.encoder_resolution)*2*math.pi*self.radius
                self.effort = self.Gripper.gripper_current*self.effort_factor
                self.pre_position = self.Gripper.gripper_position
                # self.get_logger().info('number: "%s"' % str(self.position))
                # self.get_logger().info('speed [m/s]: "%s"' % str(self.speed))

                # self.get_logger().info('effort [N]: "%s"' % str(self.effort))



                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.header.frame_id = ''

                joint_state.name.append('left_gripper_finger_joint')
                joint_state.position.append(self.position/2)
                joint_state.velocity.append(self.speed)
                joint_state.effort.append(self.effort)

                joint_state.name.append('right_gripper_finger_joint')
                joint_state.position.append(self.position/2)
                joint_state.velocity.append(self.speed)
                joint_state.effort.append(self.effort)
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