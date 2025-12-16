#!/usr/bin/env python3

"""
Simulation Controller for Digital Twin
Provides control interface for the humanoid robot in simulation
Based on the simulation control requirements from the specification
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Time
import numpy as np
from typing import Dict, List, Optional


class SimulationController(Node):
    """
    Controller for simulation environment based on humanoid robot requirements.
    Implements control and validation for the digital twin simulation.
    """
    
    def __init__(self):
        super().__init__('simulation_controller')
        
        # Configuration parameters
        self.declare_parameter('robot_name', 'humanoid_robot')
        self.declare_parameter('control_frequency', 50.0)  # Hz
        self.declare_parameter('max_joint_velocity', 2.0)  # rad/s
        
        self.robot_name = self.get_parameter('robot_name').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        
        # Define QoS profile for control commands
        control_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(seconds=0.02)  # 20ms deadline for control
        )
        
        # Publishers for control commands
        self.joint_command_publisher = self.create_publisher(
            JointState,
            '/joint_commands',
            control_qos_profile
        )
        
        self.velocity_command_publisher = self.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            control_qos_profile
        )
        
        # Subscribers for robot state feedback
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # Keep last 10 joint states
        )
        
        # Internal state
        self.current_joint_states = JointState()
        self.desired_joint_positions = {}
        self.control_timer = self.create_timer(1.0/self.control_frequency, self.control_loop)
        
        # Initialize joint position targets
        self.initialize_joint_targets()
        
        self.get_logger().info(
            f'Simulation Controller initialized for {self.robot_name}\n'
            f'  Control Frequency: {self.control_frequency} Hz\n'
            f'  Max Joint Velocity: {self.max_joint_velocity} rad/s'
        )

    def initialize_joint_targets(self):
        """Initialize desired joint positions to current positions."""
        # In a real implementation, this would get initial positions from the robot
        # For now, we'll set default positions
        default_joints = [
            'head_joint',
            'left_shoulder_yaw', 'left_shoulder_pitch', 'left_elbow',
            'right_shoulder_yaw', 'right_shoulder_pitch', 'right_elbow',
            'left_hip_yaw', 'left_hip_pitch', 'left_knee', 'left_ankle',
            'right_hip_yaw', 'right_hip_pitch', 'right_knee', 'right_ankle'
        ]
        
        for joint_name in default_joints:
            self.desired_joint_positions[joint_name] = 0.0  # Default to zero position

    def joint_state_callback(self, msg: JointState):
        """Update current joint states from simulation feedback."""
        self.current_joint_states = msg
        
        # Update internal position tracking
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                if name in self.desired_joint_positions:
                    # Update only the position, keep other control targets
                    pass
                else:
                    # Add new joint to our tracking
                    self.desired_joint_positions[name] = msg.position[i]

    def control_loop(self):
        """Main control loop running at control_frequency."""
        # Create joint command message
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = f'{self.robot_name}_base_link'
        
        # Set joint names
        cmd_msg.name = list(self.desired_joint_positions.keys())
        
        # Set desired positions
        cmd_msg.position = list(self.desired_joint_positions.values())
        
        # Set velocities to zero (for position control)
        cmd_msg.velocity = [0.0] * len(cmd_msg.position)
        
        # Set efforts to zero
        cmd_msg.effort = [0.0] * len(cmd_msg.position)
        
        # Publish the command
        self.joint_command_publisher.publish(cmd_msg)
        
        self.get_logger().debug(f'Published joint command for {len(cmd_msg.name)} joints')

    def set_joint_positions(self, joint_positions: Dict[str, float]):
        """Set desired joint positions for the robot."""
        for joint_name, position in joint_positions.items():
            if joint_name in self.desired_joint_positions:
                # Check joint limits (simplified)
                limited_position = max(min(position, np.pi), -np.pi)  # Limit to ±π
                self.desired_joint_positions[joint_name] = limited_position
            else:
                self.get_logger().warn(f'Joint {joint_name} not found in robot model')
    
    def set_trajectory(self, joint_trajectories: List[Dict[str, float]], duration: float):
        """Set a trajectory of joint positions to be executed over a duration."""
        # This would implement smooth trajectory following
        # For now, we'll just set the final positions
        if len(joint_trajectories) > 0:
            # Use the last trajectory point as the target
            final_positions = joint_trajectories[-1]
            self.set_joint_positions(final_positions)

    def move_to_pose(self, x: float, y: float, theta: float):
        """Command the robot to move to a specific 2D pose."""
        # This would implement differential drive motion for the humanoid base
        cmd_msg = Twist()
        cmd_msg.linear.x = x
        cmd_msg.linear.y = y
        cmd_msg.angular.z = theta
        
        self.velocity_command_publisher.publish(cmd_msg)
        self.get_logger().info(f'Moving to pose: ({x}, {y}, {theta})')

    def stop_motion(self):
        """Stop all robot motion immediately."""
        # Set all joint targets to current positions
        for i, name in enumerate(self.current_joint_states.name):
            if i < len(self.current_joint_states.position):
                self.desired_joint_positions[name] = self.current_joint_states.position[i]
        
        # Publish zero velocity command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0
        
        self.velocity_command_publisher.publish(stop_cmd)
        self.get_logger().info('Motion stopped')

    def validate_simulation_state(self) -> bool:
        """Validate that the simulation is behaving as expected."""
        # Check if we're receiving joint states
        if len(self.current_joint_states.name) == 0:
            self.get_logger().error('No joint states received from simulation')
            return False
        
        # Check if joint positions are within reasonable bounds
        for pos in self.current_joint_states.position:
            if abs(pos) > 100:  # Unreasonable position
                self.get_logger().error(f'Joint position out of bounds: {pos}')
                return False
        
        return True


def main(args=None):
    rclpy.init(args=args)
    
    controller = SimulationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Keyboard interrupt received, stopping motion')
        controller.stop_motion()
    finally:
        controller.stop_motion()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()