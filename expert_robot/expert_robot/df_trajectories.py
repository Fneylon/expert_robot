import collections
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from xarm_msgs.msg import RobotMsg
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
import json
import os
import random
import time as t
import sys
import yaml

import numpy as np
import pickle as pkl

import keyboard
import IPython
import PyKDL
from tf2_ros import TransformException

from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sensor_msgs.msg import Joy
from std_msgs.msg import (
    Int8, Int16, Int32, Int16MultiArray, 
    Float32, Float32MultiArray, 
    MultiArrayDimension, 
    String,
)
from geometry_msgs.msg import (
    Quaternion, Vector3, TwistStamped
)

from std_srvs.srv import SetBool

from visualization_msgs.msg import (
    Marker, MarkerArray
)


from xarm_msgs.srv import (MoveCartesian, SetInt16) 
from xarm_msgs.msg import RobotMsg

from enum import Enum, auto

from .submodules.xarm_moveit_wrapper import XARM, XarmWrapper

class State_(Enum):
    START = auto()
    REACHING = auto()
    TRANSITION = auto()
    HOMING = auto()
    DONE = auto()
    EXECUTING = auto()
    PLANNING = auto()

class ExpertTrajectories(Node):
    def __init__(self):
        super().__init__('expert_trajectories')
        self.get_logger().info('Initializing Expert Trajectories Node...')

        # Initialize the xArm Moveit Wrapper: 
        self.XarmMoveIt = XarmWrapper(self) # NOTE: this is the Moveit Wrapper for the xArm which inherits this node

        # Initialize the Xarm Moveit State Machine: 
        self.moveit_state = XARM

        # self.param_flag = False
        self.state = State_.START
        self.print_input = True

         # Initialize Publishers: 
        self._initialize_publishers()

        # Initialize Subscribers: 
        self._initialize_subscribers()

        # Initialize Clients: 
        # self._initialize_clients()

        # self.state_ = State_.START
        self.local_param_init()

        # Timer Frequency:
        self.timer_freq = 50.0

        # Init timers
        self.timer = self.create_timer(self.timer_freq, self.timer_callback) # NOTE: this has to be at 0.02 for homing purposes

    def timer_callback(self):

        if self.state == State_.HOMING:
            future = self.stop_client.call_async(self.stop_client_req)
            future.add_done_callback(self.stop_servo_cb)
            t.sleep(5)
            self.state = State_.PLANNING
            self.get_logger().info(f"Homing to {self.task} r01-Home Position")
            self.XarmMoveIt.plan_to_joint_states(self.home_joint_positions)

        if self.state == State_.PLANNING:
            if self.XarmMoveIt.state == self.moveit_state.EXECUTING: 
                self.state = State_.EXECUTING
                self.get_logger().info("Moving Robot to Request Joint Positions")

        elif self.state == State_.EXECUTING:
            if self.XarmMoveIt.state == self.moveit_state.DONE:
                self.get_logger().info("Done Planning Robot--switching state to TRANSITION")
                self.state = State_.TRANSITION

        if keyboard.is_pressed('alt+k'):
            self.get_logger().warn("ROBOT REHOME CALLED")
            self.state = State_.HOMING

        if self.state == State_.DONE: 
            self.shutdown_hook("Reached End of Trial")

        if self.state == State_.TRANSITION:
            if self.print_input: 
                self.get_logger().info("In TRANSITION State")
                self.get_logger().info(f'\n\tNOTE[{self.task}]:Begin reaching for target {self.target_idx}, press alt + s')
                self.print_input = False
                future = self.start_client.call_async(self.start_client_req)
                future.add_done_callback(self.start_servo_cb)
                self.current_dim_pub.publish(String(data=self.curr_dim))

        if keyboard.is_pressed('alt + s'):
            self.state = State_.REACHING
            self.start_time = t.time()
            self.reaching_time = t.time() - self.start_time
    
        if self.state == State_.REACHING:

            if self.reaching_time <= self.reach_duration:
                # Publish the control signal: 
                self.publish_robot_cmd(self.signal[self.t_idx])
                self.t_idx += 1
                self.reaching_time = t.time() - self.start_time

            else:
                # Stop the robot:
                self.publish_robot_cmd(0.0)

                # Reset the timer:
                self.start_time = None
                self.reaching_time = None

                # Iterate to the next dimension:
                self.dim_list = self.dim_list.pop(0)
                self.curr_dim = self.dim_list[0]

                # Prepare for homing:
                self.state = State_.HOMING
                self.print_input = True
                t.sleep(2)

    def publish_robot_cmd(self, signal):

        # Publish the control signal: 
        msg = TwistStamped()
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        if self.curr_dim == 'x':
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = signal
            msg.twist.linear.z = 0.0

        elif self.curr_dim == 'y':
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = -signal
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = 0.0

        elif self.curr_dim == 'z': 
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = signal

        self.robot_cmd_pub.publish(msg)

    def local_param_init(self):
        self.get_logger().info("Initializing Local Parameters")
        self.list_of_dims = ['x', 'y', 'z']
        self.curr_dim = 'x'
        self.initialize_msgs()

        # Parameters for defining the synthetic user signal:
        self.phase = 0
        self.amplitude = 1.0
        self.period = 20.0 # Seconds
        self.freq = 1.0 / self.period
        self.duration = 1.0 # For step signal at max amplitude

        # Initialize the stopwatch: 
        self.start_time = None
        self.reaching_time = None
        self.reach_duration = 20.0 # Seconds

        # Time Series Tracker: 
        self.t_idx = 0

        # load in the synthetic signal:
        self.signal = np.loadtxt('/home/r01_ros2_ws/src/expert_robot/expert_robot/expert_robot/submodules/synthetic_signal.txt')
        
        # Home Position: 
        self.home_joint_positions = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0, 0.0]


    def _initialize_publishers(self):
        self.get_logger().info("Initializing Publishers")

        # Robot Command Publisher
        self.robot_cmd_pub = self.create_publisher(
            TwistStamped, '/servo_server/delta_twist_cmds', 10)
        
        # Current Dimension Publisher
        self.current_dim_pub = self.create_publisher(
            String, '/current_dim', 10)

    def _initialize_subscribers(self):
        self.get_logger().info("Initializing Subscribers")

    def _initialize_clients(self):
        self.get_logger().info("Initializing Clients")
        
        ### Create Clients to Control the xArm: 
        # start servo client
        self.start_client = self.create_client(Trigger, '/servo_server/start_servo')
        self.start_client_req = Trigger.Request()
        while not self.start_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("start service not found")

        # stop servo client
        self.stop_client = self.create_client(Trigger, '/servo_server/stop_servo')
        self.stop_client_req = Trigger.Request()
        while not self.stop_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("stop service not found")

        start = self.start_client.call_async(self.start_client_req)
        rclpy.spin_until_future_complete(self, start)

                
        # Create the Set Position Client
        self.set_position_srv = self.create_client(MoveCartesian, '/xarm/set_position')
        self.set_position_req = MoveCartesian.Request()
        self.set_position_req.mvtime = 0.0 # Move Time
        self.set_position_req.speed = 50.0 # Move Speed
        self.set_position_req.acc = 500.0

        while not self.set_position_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Set Position Service not available, waiting again...')



def main():
    rclpy.init()
    expert_traj = ExpertTrajectories()
    expert_traj.get_logger().info("Starting XarmController node on executor...\n\n")
    rclpy.spin(expert_traj)
    expert_traj.get_logger().info("Shutting down xarmController node...\n\n")
    rclpy.shutdown()
    

