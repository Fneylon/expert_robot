import rclpy
from rclpy.node import Node
from xarm_msgs.msg import RobotMsg
import json
import os
import random
import time as t
import sys
import yaml

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

from xarm_msgs.msg import RobotMsg


import csv
from datetime import datetime


class RecordTrajectories(Node):
    def __init__(self):
        super().__init__('record_trajectories')

        self.save_file_path = '/home/r01_ros2_ws/src/expert_robot/expert_robot/trajectories/'

        # subscribe to the robot state
        self.robot_state_sub = self.create_subscription(
            RobotMsg,
            '/xarm/robot_msg',
            self.robot_state_callback,
            10
        )

        self.current_dim_sub = self.create_subscription(
            String,
            '/current_dim',
            self.current_dim_callback,
            10
        )

        # Open the CSV file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file = open(f'/home/r01_ros2_ws/src/expert_robot/expert_robot/trajectories/tool_pose_{timestamp}.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'position_x', 'position_y', 'position_z', 'orientation_x', 'orientation_y', 'orientation_z', 'current_dim'])

    def robot_state_callback(self, msg):
        # Get robot pose
        pose = msg.pose

        # Get current timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Write to CSV
        self.csv_writer.writerow([
            timestamp.sec + timestamp.nanosec * 1e-9,
            pose[0], pose[1], pose[2],
            pose[3], pose[4], pose[5], self.current_dim
        ])

    def current_dim_callback(self, msg):
        self.current_dim = msg.data

    def destroy_node(self):
        super().destroy_node()
        self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    record_trajectories = RecordTrajectories()

    try: 
        rclpy.spin(record_trajectories)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    record_trajectories.destroy_node()
    rclpy.shutdown()



