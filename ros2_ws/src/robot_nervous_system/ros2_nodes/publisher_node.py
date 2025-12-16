#!/usr/bin/env python3

"""
Publisher node for sensor data in the ROS 2 nervous system.
Implements publisher/subscriber nodes for real-time message passing (FR-001).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
import random
import math
from robot_nervous_system.msg import SensorDataMessage, JointCommandMessage
from robot_nervous_system.srv import LoadURDF


class SensorPublisherNode(Node):
    def __init__(self):
        super().__init__('sensor_publisher_node')
        
        # Define QoS profile for real-time critical data (from API contract)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(seconds=0.1)  # 100ms deadline
        )
        
        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            '/joint_states', 
            qos_profile
        )
        
        # Publisher for sensor data
        self.sensor_data_publisher = self.create_publisher(
            SensorDataMessage, 
            '/sensor_data', 
            qos_profile
        )
        
        # Timer to publish data at 100Hz (per performance goals)
        self.timer = self.create_timer(0.01, self.publish_sensor_data)  # 100Hz = 0.01s
        
        self.get_logger().info('Sensor Publisher Node initialized')
        
        # Initialize joint names for our 24+ DOF humanoid model
        self.joint_names = [
            'neck_joint',
            'left_shoulder_yaw', 'left_shoulder_pitch', 'left_elbow',
            'right_shoulder_yaw', 'right_shoulder_pitch', 'right_elbow',
            'left_hip_yaw', 'left_hip_pitch', 'left_knee', 'left_ankle',
            'right_hip_yaw', 'right_hip_pitch', 'right_knee', 'right_ankle'
        ]
        
        # Initialize position values
        self.joint_positions = [0.0] * len(self.joint_names)
        self.joint_velocities = [0.0] * len(self.joint_names)
        self.joint_efforts = [0.0] * len(self.joint_names)

    def publish_sensor_data(self):
        """Publish joint states and sensor data"""
        # Update joint positions with simulated movement
        for i in range(len(self.joint_positions)):
            # Simulate small random movements
            self.joint_positions[i] = math.sin(
                self.get_clock().now().nanoseconds * 0.000000001 + i
            ) * 0.2
        
        # Publish joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = 'base_link'
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        joint_state_msg.velocity = self.joint_velocities
        joint_state_msg.effort = self.joint_efforts
        
        self.joint_state_publisher.publish(joint_state_msg)
        
        # Publish sensor data
        sensor_msg = SensorDataMessage()
        sensor_msg.header_stamp = self.get_clock().now().to_msg()
        sensor_msg.sensor_type = 'imu'
        sensor_msg.sensor_name = 'imu_sensor_1'
        sensor_msg.data = [random.uniform(-1.0, 1.0) for _ in range(9)]  # 9 DOF data: orientation, angular velocity, linear acceleration
        sensor_msg.frame_id = 'imu_link'
        
        self.sensor_data_publisher.publish(sensor_msg)


def main(args=None):
    rclpy.init(args=args)
    
    publisher_node = SensorPublisherNode()
    
    try:
        rclpy.spin(publisher_node)
    except KeyboardInterrupt:
        publisher_node.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()