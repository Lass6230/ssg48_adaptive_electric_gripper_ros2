import math

from geometry_msgs.msg import TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose
import Spectral_BLDC as Spectral
import time

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class ssg48Gripper(Node):

    def __init__(self):
        super().__init__('ssg48_gripper')

        

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # Communication1 = Spectral.CanCommunication(bustype='socketcan', channel='can0', bitrate=1000000)
        self.Communication1 = Spectral.CanCommunication(bustype='slcan', channel='/dev/ttyACM0', bitrate=1000000)
        self.Gripper = Spectral.SpectralCAN(node_id=0, communication=self.Communication1)
        self.Gripper.Send_Clear_Error()

        self.Gripper.Send_gripper_calibrate()

        timer_period = 0.1  # seconds
        self.i = 1
        time.sleep(5)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # self.timer = self.create_timer(4.0, self.timer_callback2)
        self.var = 1
    
    def timer_callback2(self):
        if self.var == 0:
        #Motor1.Send_Clear_Error()
            self.Gripper.Send_gripper_calibrate()
        # Motor1.Send_gripper_data_pack(200,20,500,1,1,0,0) 
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
        message, UnpackedMessageID = self.Communication1.receive_can_messages(timeout=0.1)
        if message is not None:
            self.Gripper.UnpackData(message,UnpackedMessageID)
            if(UnpackedMessageID.command_id == 60):

                self.get_logger().info('position: "%s"' % str(self.Gripper.gripper_position))
                self.get_logger().info('number: "%s"' % str(self.i))
                self.i += 1
            else:
                self.get_logger().info('UnpackedMessageID.command_id == 60 is not valid')
        else:
            self.get_logger().info('No message after timeout period!')

    def handle_turtle_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtlename

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ssg48Gripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()