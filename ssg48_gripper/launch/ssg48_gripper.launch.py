import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import xacro
def generate_launch_description():
 
  robot_description_path = os.path.join(
                            get_package_share_directory("ssg48_gripper_description"),
                            'urdf/ssg48.urdf.xacro')
    
  robot_description_content = open(robot_description_path).read()
    
  robot_description = {"robot_description": robot_description_content}
 

  gui = LaunchConfiguration('gui')
  

  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  
  use_sim_time = LaunchConfiguration('use_sim_time')
 
 
  
     
  
     
  declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')
   
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')
 
  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
  


  bustype = LaunchConfiguration('use_sim_time')

  channel = LaunchConfiguration('use_sim_time')

  bitrate = LaunchConfiguration('use_sim_time')
  
  declare_bustype_cmd = DeclareLaunchArgument(
    name='bustype',
    default_value='socketcan',
    description='can bustype, typical socketcan or slcan')
  
  declare_channel_cmd = DeclareLaunchArgument(
    name='channel',
    default_value='can0',
    description='can channel, typical can0 or /dev/ttyACM0')

  declare_bitrate_cmd = DeclareLaunchArgument(
    name='bitrate',
    default_value='1000000',
    description='can channel, typical 1000000 or 500000')
    
  # Specify the actions
 
  # Publish the joint state values for the non-fixed joints in the URDF file.
  start_joint_state_publisher_cmd = Node(
    # condition=UnlessCondition(gui),
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[robot_description,{'source_list':['gripper_joint_states']}],
    
    name='joint_state_publisher')
 
  # # A GUI to manipulate the joint state values
  # start_joint_state_publisher_gui_node = Node(
  #   condition=IfCondition(gui),
  #   package='joint_state_publisher_gui',
  #   executable='joint_state_publisher_gui',
  #   parameters=[robot_description],
  #   name='joint_state_publisher_gui')
 
  # # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[robot_description],)
  
  ssg48 = Node(
    package='ssg48_gripper',
    executable='ssg48_gripper',
    name='ssg48_gripper',
    parameters=[robot_description,bustype,bitrate,channel,{'joint_state_topic':'gripper_joint_states'}],
    output='screen',
  )
 
  # Launch RViz
  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    parameters=[robot_description],
    output='screen',)


   
  # Create the launch description and populate
  ld = LaunchDescription()
 


  ld.add_action(declare_use_joint_state_publisher_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)


  ld.add_action(declare_channel_cmd)
  ld.add_action(declare_bitrate_cmd)
  ld.add_action(declare_bustype_cmd)
 
  # Add any actions
  ld.add_action(start_joint_state_publisher_cmd)
  # ld.add_action(start_joint_state_publisher_gui_node)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(ssg48)
  ld.add_action(start_rviz_cmd)
 
  return ld