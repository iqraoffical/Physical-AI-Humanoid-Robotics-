#!/usr/bin/env python3

"""
Python agent bridge for the ROS 2 nervous system.
Implements the bridge between Python AI agents and ROS controllers (FR-004).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from robot_nervous_system.msg import JointCommandMessage, SensorDataMessage
from robot_nervous_system.srv import LoadURDF, ExecuteAction
from robot_nervous_system.action import MoveRobot

import rclpy.action
import threading
import time
import math
from typing import List, Dict, Any, Optional

# Import security context
from robot_nervous_system.security.security_context import (
    validate_access_for_node, 
    AccessLevel,
    encrypt_message,
    decrypt_message
)


class PythonAgentBridge(Node):
    """
    Bridge between Python AI agents and ROS controllers for joint control.
    Enables Python agents to send joint commands to robot controllers via ROS 2 (FR-004).
    """
    
    def __init__(self):
        super().__init__('python_agent_bridge')
        
        # Validate access to this node
        if not validate_access_for_node('python_agent_bridge', AccessLevel.USER):
            self.get_logger().error("Access denied for python_agent_bridge")
            return
        
        # Define QoS profile for real-time critical data (from API contract)
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(seconds=0.05)  # 50ms deadline for commands
        )
        
        # Publishers for sending commands to the robot
        self.joint_command_publisher = self.create_publisher(
            JointCommandMessage,
            '/joint_commands',
            qos_profile
        )
        
        # Subscribers for receiving feedback from the robot
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos_profile
        )
        
        self.sensor_data_subscriber = self.create_subscription(
            SensorDataMessage,
            '/sensor_data',
            self.sensor_data_callback,
            qos_profile
        )
        
        # Clients for services
        self.load_urdf_client = self.create_client(LoadURDF, 'load_urdf')
        self.execute_action_client = self.create_client(ExecuteAction, 'execute_action')
        
        # Action client for robot movement
        self.move_robot_action_client = rclpy.action.ActionClient(
            self, 
            MoveRobot, 
            'move_robot'
        )
        
        # Store latest sensor data and joint states
        self.latest_joint_states = None
        self.latest_sensor_data = None
        self.joint_state_lock = threading.Lock()
        self.sensor_data_lock = threading.Lock()
        
        # Initialize joint names for our 24+ DOF humanoid model
        self.joint_names = [
            'neck_joint',
            'left_shoulder_yaw', 'left_shoulder_pitch', 'left_elbow',
            'right_shoulder_yaw', 'right_shoulder_pitch', 'right_elbow',
            'left_hip_yaw', 'left_hip_pitch', 'left_knee', 'left_ankle',
            'right_hip_yaw', 'right_hip_pitch', 'right_knee', 'right_ankle'
        ]
        
        # Initialize with default positions
        self.default_joint_positions = [0.0] * len(self.joint_names)
        
        self.get_logger().info('Python Agent Bridge initialized')

    def joint_state_callback(self, msg):
        """Callback to handle incoming joint states."""
        with self.joint_state_lock:
            self.latest_joint_states = msg
            # self.get_logger().debug(f'Updated joint states for {len(msg.name)} joints')

    def sensor_data_callback(self, msg):
        """Callback to handle incoming sensor data."""
        with self.sensor_data_lock:
            self.latest_sensor_data = msg
            # self.get_logger().debug(f'Updated sensor data from {msg.sensor_name}')

    def get_joint_states(self) -> Optional[JointState]:
        """Get the latest joint states from the robot."""
        with self.joint_state_lock:
            return self.latest_joint_states
    
    def get_sensor_data(self) -> Optional[SensorDataMessage]:
        """Get the latest sensor data from the robot."""
        with self.sensor_data_lock:
            return self.latest_sensor_data

    def send_joint_commands(self, joint_names: List[str], positions: List[float], 
                           velocities: Optional[List[float]] = None, 
                           efforts: Optional[List[float]] = None):
        """
        Send joint commands to the robot.
        
        Args:
            joint_names: Names of the joints to command
            positions: Target positions for the joints
            velocities: Target velocities for the joints (optional)
            efforts: Target efforts for the joints (optional)
        """
        if velocities is None:
            velocities = [0.0] * len(positions)
        if efforts is None:
            efforts = [0.0] * len(positions)
        
        # Validate command against joint limits (from data model and clarifications)
        for i, pos in enumerate(positions):
            if abs(pos) > math.pi:  # Example limit check
                self.get_logger().warning(f"Joint command for {joint_names[i]} exceeds limits: {pos}")
                # Adjust to valid range or reject command
        
        # Create and send the command message
        cmd_msg = JointCommandMessage()
        cmd_msg.stamp = self.get_clock().now().to_msg()
        cmd_msg.joint_names = joint_names
        cmd_msg.positions = positions
        cmd_msg.velocities = velocities
        cmd_msg.efforts = efforts
        
        # Apply security validation (from clarifications)
        if not self.validate_joint_command(cmd_msg):
            self.get_logger().error("Joint command failed security validation")
            return False
        
        self.joint_command_publisher.publish(cmd_msg)
        self.get_logger().info(f'Sent joint commands to {len(joint_names)} joints')
        return True

    def validate_joint_command(self, cmd_msg: JointCommandMessage) -> bool:
        """
        Validate a joint command message according to security and validation rules.
        
        Args:
            cmd_msg: Joint command message to validate
            
        Returns:
            True if command is valid, False otherwise
        """
        # Check that all joint names are valid
        for joint_name in cmd_msg.joint_names:
            if joint_name not in self.joint_names:
                self.get_logger().error(f"Invalid joint name: {joint_name}")
                return False
        
        # Check that position values are within reasonable limits
        for pos in cmd_msg.positions:
            if abs(pos) > 10.0:  # Example limit
                self.get_logger().error(f"Position command exceeds reasonable limit: {pos}")
                return False
        
        # Additional validation could go here based on the specific humanoid model
        return True

    def execute_action(self, action_name: str, parameters: Dict[str, str] = None) -> bool:
        """
        Execute a predefined robot action.
        
        Args:
            action_name: Name of the action to execute
            parameters: Parameters for the action (optional)
            
        Returns:
            True if the action was successfully sent, False otherwise
        """
        if parameters is None:
            parameters = {}
        
        while not self.execute_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ExecuteAction service not available, waiting again...')
        
        request = ExecuteAction.Request()
        request.action_name = action_name
        request.param_names = list(parameters.keys())
        request.param_values = list(parameters.values())
        
        future = self.execute_action_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response is not None and response.success:
            self.get_logger().info(f'Successfully executed action: {action_name}')
            return True
        else:
            self.get_logger().error(f'Failed to execute action: {action_name}')
            return False

    def move_robot_to_pose(self, target_pose, timeout=30.0):
        """
        Move the robot to a target pose using the action interface.
        
        Args:
            target_pose: Target pose for the robot
            timeout: Timeout for the action in seconds
            
        Returns:
            Result of the action execution
        """
        goal_msg = MoveRobot.Goal()
        goal_msg.target_pose = target_pose
        
        self.get_logger().info('Waiting for action server...')
        self.move_robot_action_client.wait_for_server()
        
        self.get_logger().info('Sending goal to move robot...')
        send_goal_future = self.move_robot_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.move_robot_feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return None
        
        self.get_logger().info('Goal accepted by server, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=timeout)
        
        if get_result_future.result() is None:
            self.get_logger().error(f'Timeout waiting for result after {timeout} seconds')
            return None
        
        result = get_result_future.result().result
        self.get_logger().info(f'Action completed with success={result.success}')
        
        return result
    
    def move_robot_feedback_callback(self, feedback_msg):
        """Callback for move robot action feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Robot progress: {feedback.trajectory_progress:.2f}, '
                              f'Distance to target: {feedback.distance_remaining:.2f}')

    def execute_simple_movement(self):
        """Example function to demonstrate a simple movement pattern."""
        # Example: Move a joint back and forth
        import time
        
        for i in range(10):
            # Move left shoulder joint
            positions = [math.sin(i * 0.5) * 0.5]  # Oscillating movement
            self.send_joint_commands(['left_shoulder_pitch'], positions)
            time.sleep(0.5)
        
        self.get_logger().info('Completed simple movement demonstration')


def main(args=None):
    rclpy.init(args=args)
    
    agent_bridge = PythonAgentBridge()
    
    # Give some time for the node to initialize
    time.sleep(1)
    
    try:
        # Example: Run a simple movement demonstration
        agent_bridge.execute_simple_movement()
        
        # Spin to keep the node running and handling callbacks
        rclpy.spin(agent_bridge)
    except KeyboardInterrupt:
        agent_bridge.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        agent_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()