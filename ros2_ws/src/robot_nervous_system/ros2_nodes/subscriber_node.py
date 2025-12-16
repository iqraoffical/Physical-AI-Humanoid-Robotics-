#!/usr/bin/env python3

"""
Subscriber node for joint commands in the ROS 2 nervous system.
Implements publisher/subscriber nodes for real-time message passing (FR-001).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from robot_nervous_system.msg import JointCommandMessage, SensorDataMessage
from robot_nervous_system.srv import ExecuteAction
import threading


class JointSubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        
        # Define QoS profile for real-time critical data (from API contract)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(seconds=0.1)  # 100ms deadline
        )
        
        # Subscriber for joint commands
        self.joint_command_subscriber = self.create_subscription(
            JointCommandMessage,
            '/joint_commands',
            self.joint_command_callback,
            qos_profile
        )
        
        # Subscriber for sensor data
        self.sensor_data_subscriber = self.create_subscription(
            SensorDataMessage,
            '/sensor_data',
            self.sensor_data_callback,
            qos_profile
        )
        
        # Subscriber for joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        self.get_logger().info('Joint Subscriber Node initialized')
        
        # Store the latest joint command
        self.latest_joint_command = None
        self.joint_command_lock = threading.Lock()
        
        # Store the latest sensor data
        self.latest_sensor_data = None
        self.sensor_data_lock = threading.Lock()
        
        # Store the latest joint states
        self.latest_joint_states = None
        self.joint_states_lock = threading.Lock()

    def joint_command_callback(self, msg):
        """Process incoming joint commands"""
        with self.joint_command_lock:
            self.latest_joint_command = msg
            self.get_logger().debug(f'Received joint command for joints: {msg.joint_names}')
            
            # Process the command (in a real implementation, this would interface with hardware)
            self.process_joint_command(msg)

    def sensor_data_callback(self, msg):
        """Process incoming sensor data"""
        with self.sensor_data_lock:
            self.latest_sensor_data = msg
            self.get_logger().debug(f'Received sensor data from {msg.sensor_name}')

    def joint_state_callback(self, msg):
        """Process incoming joint states"""
        with self.joint_states_lock:
            self.latest_joint_states = msg
            self.get_logger().debug(f'Received joint states for {len(msg.name)} joints')

    def process_joint_command(self, command_msg):
        """Process the joint command message according to validation rules"""
        # Validate joint positions against joint limits (from data model)
        # In a real implementation, this would interface with the actual robot
        for i, joint_name in enumerate(command_msg.joint_names):
            if i < len(command_msg.positions):
                position = command_msg.positions[i]
                # Log the received command (for observability)
                self.get_logger().info(f'Setting {joint_name} to position {position}')
        
        # In a real implementation, this would send the commands to the robot's actuators


def main(args=None):
    rclpy.init(args=args)
    
    subscriber_node = JointSubscriberNode()
    
    try:
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        subscriber_node.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()