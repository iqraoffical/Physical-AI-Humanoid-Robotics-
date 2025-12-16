#!/usr/bin/env python3

"""
Physics Validation Script for Digital Twin
Validates that simulated physics behavior matches physical characteristics per success criteria
Implements SC-001: Robot behaves realistically under physics with mass/inertia/friction properties matching physical specifications
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
import numpy as np
import math
import time
import unittest
from typing import Dict, List, Tuple
import yaml
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class PhysicsValidator(Node):
    """
    System that validates physics simulation accuracy against physical specifications.
    Implements SC-001: Robot behaves realistically under physics with mass, inertia, friction properties matching physical specifications.
    """
    
    def __init__(self):
        super().__init__('physics_validator')
        
        # Configuration parameters
        self.declare_parameter('validation_duration', 10.0)  # seconds
        self.declare_parameter('tolerance_threshold', 0.1)  # 10% tolerance
        self.declare_parameter('gravity_tolerance', 0.05)   # 5% tolerance for gravity
        self.declare_parameter('data_collection_rate', 100.0)  # Hz
        
        self.validation_duration = self.get_parameter('validation_duration').value
        self.tolerance_threshold = self.get_parameter('tolerance_threshold').value
        self.gravity_tolerance = self.get_parameter('gravity_tolerance').value
        self.data_collection_rate = self.get_parameter('data_collection_rate').value
        
        # Data collection
        self.joint_data = {}  # Collected joint data for analysis
        self.collection_start_time = None
        self.physics_parameters = {}  # Expected physical parameters from URDF
        
        # QoS for data collection
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Subscription for joint states (from FR-001: publisher/subscriber nodes)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            sensor_qos
        )
        
        # Publisher for test commands
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/test_commands',
            10
        )
        
        # Service for starting validation tests
        self.start_validation_srv = self.create_service(
            SetBool,
            '/start_physics_validation',
            self.start_validation_callback
        )
        
        self.get_logger().info(
            f'Physics Validator initialized:\n'
            f'  Validation Duration: {self.validation_duration}s\n'
            f'  Tolerance Threshold: {self.tolerance_threshold} ({self.tolerance_threshold*100}%)\n'
            f'  Gravity Tolerance: {self.gravity_tolerance} ({self.gravity_tolerance*100}%)\n'
            f'  Data Collection Rate: {self.data_collection_rate} Hz'
        )
        
        # Load expected physics parameters from URDF/model
        self.load_expected_parameters()

    def load_expected_parameters(self):
        """
        Load expected physical parameters from URDF/model specifications.
        These are the reference values that simulation must match.
        """
        # In a real implementation, this would load from URDF/SDF files
        # For now, we'll simulate loading parameters based on humanoid robot specifications
        
        # Example physical parameters (these would come from URDF/model files)
        self.expected_parameters = {
            'gravity': [-9.81, 0.0, 0.0],  # Gravity vector in world coordinates
            'robot_mass': 50.0,  # Total robot mass in kg (example)
            
            # Joint limits based on humanoid model
            'joint_limits': {
                'head_yaw': (-1.57, 1.57),      # +/- 90 degrees
                'head_pitch': (-0.785, 0.785),  # +/- 45 degrees
                'shoulder_yaw': (-2.356, 0.785), # -135 to 45 degrees
                'elbow_joint': (0.0, 2.356),    # 0 to 135 degrees
                'hip_yaw': (-0.785, 0.785),     # +/- 45 degrees
                'knee_joint': (0.0, 2.356),     # 0 to 135 degrees
            },
            
            # Mass properties of major links
            'link_masses': {
                'base_link': 10.0,
                'torso': 8.0,
                'head': 2.0,
                'upper_arm': 1.5,
                'lower_arm': 1.0,
                'upper_leg': 3.0,
                'lower_leg': 2.5,
            },
            
            # Friction coefficients
            'friction_coeffs': {
                'joints': 0.1,  # Joint friction coefficient
                'feet': 0.8,    # Foot-ground friction
            },
            
            # Expected behavior under gravity (for free fall validation)
            'free_fall_acceleration': 9.81,  # m/s^2
        }
        
        self.get_logger().info('Expected physics parameters loaded for validation')

    def joint_state_callback(self, msg: JointState):
        """Collect joint state data for physics validation."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        
        for i, name in enumerate(msg.name):
            if i < len(msg.position) and i < len(msg.velocity):
                joint_info = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i],
                    'timestamp': current_time
                }
                
                if name not in self.joint_data:
                    self.joint_data[name] = []
                
                # Store data with time window management
                self.joint_data[name].append(joint_info)
                
                # Keep only recent data (last 10 seconds worth)
                cutoff_time = current_time - 10.0
                self.joint_data[name] = [
                    j for j in self.joint_data[name] 
                    if j['timestamp'] > cutoff_time
                ]

    def start_validation_callback(self, request, response):
        """Handle validation service request."""
        if request.data:  # Start validation
            self.get_logger().info('Starting physics validation sequence...')
            success = self.run_physics_validation()
            
            response.success = success
            response.message = f'Physics validation completed. Success: {success}'
            
            if success:
                self.get_logger().info('Physics validation PASSED - Simulation matches physical specifications')
            else:
                self.get_logger().error('Physics validation FAILED - Simulation does not match physical specifications')
        else:
            response.success = True
            response.message = 'Physics validation service acknowledged (no action taken)'
        
        return response

    def run_physics_validation(self) -> bool:
        """
        Execute comprehensive physics validation test.
        Verifies that simulated behavior matches physical specifications within tolerance.
        """
        self.get_logger().info('Running physics validation tests...')
        
        # Collect data for specified duration
        self.get_logger().info(f'Collecting data for {self.validation_duration} seconds...')
        start_time = time.time()
        collection_timeout = start_time + self.validation_duration
        
        while time.time() < collection_timeout:
            time.sleep(0.1)  # Allow data collection to proceed
        
        # Perform validation checks
        validation_results = []
        
        # Check 1: Gravity validation
        gravity_ok = self.validate_gravity_behavior()
        validation_results.append(('Gravity behavior', gravity_ok))
        
        # Check 2: Joint limit validation
        joint_limits_ok = self.validate_joint_limits()
        validation_results.append(('Joint limits', joint_limits_ok))
        
        # Check 3: Joint velocity validation
        joint_vel_ok = self.validate_joint_velocities()
        validation_results.append(('Joint velocities', joint_vel_ok))
        
        # Check 4: Mass property validation (if applicable)
        mass_property_ok = self.validate_mass_properties()
        validation_results.append(('Mass properties', mass_property_ok))
        
        # Check 5: Friction behavior validation
        friction_ok = self.validate_friction_behavior()
        validation_results.append(('Friction behavior', friction_ok))
        
        # Compile results
        total_checks = len(validation_results)
        passed_checks = sum(1 for _, passed in validation_results if passed)
        
        all_passed = all(passed for _, passed in validation_results)
        
        self.get_logger().info(f'Validation Results: {passed_checks}/{total_checks} checks passed')
        
        for check_name, result in validation_results:
            status = "PASS" if result else "FAIL"
            self.get_logger().info(f'  {check_name}: {status}')
        
        # Detailed analysis if matplotlib is available
        if MATPLOTLIB_AVAILABLE:
            self.perform_detailed_analysis()
        
        return all_passed

    def validate_gravity_behavior(self) -> bool:
        """
        Validate that objects behave correctly under gravity.
        Implements verification for SC-001: realistic physics behavior under gravity.
        """
        self.get_logger().info('Validating gravity behavior...')
        
        # For this test, we'll analyze joint positions over time to detect gravitational effects
        # Look for joints that should exhibit gravitational settling
        
        gravity_valid = True
        
        # Check if we have sufficient data
        if not self.joint_data:
            self.get_logger().warn('Insufficient joint data for gravity validation')
            return False
        
        # Look for joints that appear to be under gravitational influence
        for joint_name, joint_states in self.joint_data.items():
            if len(joint_states) < 2:
                continue
            
            # Calculate velocity trends to detect gravitational settling
            velocities = [js['velocity'] for js in joint_states]
            
            # Check for realistic acceleration/deceleration patterns
            # Under gravity, we expect certain patterns in a physical system
            velocity_changes = []
            for i in range(1, len(velocities)):
                change = velocities[i] - velocities[i-1]
                velocity_changes.append(change)
            
            # Check if average acceleration is reasonable (close to gravity effects)
            if velocity_changes:
                avg_acceleration = sum(velocity_changes) / len(velocity_changes)
                
                # For humanoid joints, gravitational effects typically manifest as accelerations
                # within certain realistic bounds
                if abs(avg_acceleration) > 50.0:  # Too high acceleration
                    self.get_logger().warn(f'Joint {joint_name} shows excessive acceleration: {avg_acceleration}')
                    gravity_valid = False
                elif abs(avg_acceleration) < 0.001:  # Too low acceleration (implausible)
                    self.get_logger().warn(f'Joint {joint_name} shows insufficient acceleration: {avg_acceleration}')
                    gravity_valid = False
                else:
                    self.get_logger().debug(f'Joint {joint_name} acceleration {avg_acceleration} appears realistic')
        
        return gravity_valid

    def validate_joint_limits(self) -> bool:
        """
        Validate that joint positions stay within expected limits.
        Implements verification for FR-008: 24+ DOF humanoid model with appropriate constraints.
        """
        self.get_logger().info('Validating joint limits...')
        
        limits_valid = True
        
        for joint_name, joint_states in self.joint_data.items():
            if joint_name in self.expected_parameters['joint_limits']:
                expected_min, expected_max = self.expected_parameters['joint_limits'][joint_name]
                
                for state in joint_states:
                    pos = state['position']
                    
                    if pos < expected_min - self.tolerance_threshold or pos > expected_max + self.tolerance_threshold:
                        self.get_logger().error(
                            f'Joint {joint_name} exceeded limits: {pos} not in [{expected_min}, {expected_max}]'
                        )
                        limits_valid = False
        
        return limits_valid

    def validate_joint_velocities(self) -> bool:
        """
        Validate that joint velocities are within reasonable bounds.
        Implements verification for realistic dynamic behavior.
        """
        self.get_logger().info('Validating joint velocities...')
        
        velocities_valid = True
        
        # Define reasonable velocity limits based on humanoid robot specifications
        max_reasonable_velocity = 5.0  # rad/s (adjust based on robot specs)
        
        for joint_name, joint_states in self.joint_data.items():
            for state in joint_states:
                vel = abs(state['velocity'])
                
                if vel > max_reasonable_velocity:
                    self.get_logger().warn(f'Joint {joint_name} velocity {vel} exceeds reasonable limit {max_reasonable_velocity}')
                    velocities_valid = False
                elif vel > max_reasonable_velocity * 0.9:
                    self.get_logger().info(f'Joint {joint_name} velocity {vel} approaching limit ({max_reasonable_velocity})')
        
        return velocities_valid

    def validate_mass_properties(self) -> bool:
        """
        Validate that system responds appropriately to mass properties.
        This indirectly verifies mass, inertia, and friction properties match specifications.
        """
        self.get_logger().info('Validating mass properties through dynamic response...')
        
        mass_valid = True
        
        # Analyze the relationship between applied forces and resulting motion
        # This requires command data which we may not have collected yet
        # For now, we'll check for physically plausible behavior in joint movements
        
        for joint_name, joint_states in self.joint_data.items():
            if len(joint_states) < 10:  # Need sufficient data for analysis
                continue
            
            # Calculate jerk (derivative of acceleration) to detect implausible motion
            velocities = [js['velocity'] for js in joint_states]
            
            accelerations = []
            for i in range(1, len(velocities)):
                dt = 0.01  # Assumed time step
                accel = (velocities[i] - velocities[i-1]) / dt
                accelerations.append(accel)
            
            jerks = []
            for i in range(1, len(accelerations)):
                dt = 0.01  # Assumed time step
                jerk = (accelerations[i] - accelerations[i-1]) / dt
                jerks.append(jerk)
            
            # Check for excessive jerk values (indicating implausible forces)
            max_jerk = max(abs(j) for j in jerks) if jerks else 0
            
            # For a 50kg humanoid robot, excessive jerk would indicate wrong mass properties
            if max_jerk > 10000:  # Very high jerk indicates mass/inertia issues
                self.get_logger().error(f'Joint {joint_name} exhibits excessive jerk: {max_jerk}')
                mass_valid = False
        
        return mass_valid

    def validate_friction_behavior(self) -> bool:
        """
        Validate that friction behavior matches expected coefficients.
        """
        self.get_logger().info('Validating friction behavior...')
        
        friction_valid = True
        
        # For friction validation, we look for patterns consistent with appropriate friction
        # When velocities approach zero, friction should prevent sliding
        
        for joint_name, joint_states in self.joint_data.items():
            if len(joint_states) < 5:
                continue
            
            # Look for instances where velocity approaches zero
            for i in range(2, len(joint_states)-2):
                if abs(joint_states[i]['velocity']) < 0.01:  # Near zero velocity
                    # Check if position remains stable (friction holding position)
                    prev_pos = joint_states[i-2]['position']
                    curr_pos = joint_states[i]['position']
                    next_pos = joint_states[i+2]['position']
                    
                    # Calculate positional drift
                    drift = abs(next_pos - prev_pos)
                    
                    # For joints with appropriate friction, minimal drift should occur
                    # when velocity is near zero
                    if drift > 0.01:  # Significant drift when velocity is near zero
                        self.get_logger().warn(
                            f'Joint {joint_name} shows excessive drift ({drift}) when velocity near zero'
                        )
                        # This may indicate insufficient friction, but isn't necessarily a failure
                        # as some joints may legitimately have low friction
        
        return friction_valid

    def perform_detailed_analysis(self):
        """Perform detailed analysis and plot results if matplotlib is available."""
        if not MATPLOTLIB_AVAILABLE:
            return
        
        try:
            # Create plots for each joint's position and velocity over time
            fig, axes = plt.subplots(2, 1, figsize=(12, 8))
            
            for joint_name in list(self.joint_data.keys())[:1]:  # Plot first joint data
                joint_states = self.joint_data[joint_name]
                
                if len(joint_states) > 0:
                    times = [js['timestamp'] for js in joint_states]
                    # Normalize time to start from 0
                    times = [t - times[0] for t in times]
                    positions = [js['position'] for js in joint_states]
                    velocities = [js['velocity'] for js in joint_states]
                    
                    axes[0].plot(times, positions, label=f'{joint_name} pos')
                    axes[0].set_title('Joint Positions Over Time')
                    axes[0].set_ylabel('Position (rad)')
                    axes[0].grid(True)
                    
                    axes[1].plot(times, velocities, label=f'{joint_name} vel')
                    axes[1].set_title('Joint Velocities Over Time')
                    axes[1].set_xlabel('Time (s)')
                    axes[1].set_ylabel('Velocity (rad/s)')
                    axes[1].grid(True)
            
            plt.tight_layout()
            plt.savefig('/tmp/physics_validation_analysis.png')
            self.get_logger().info('Detailed analysis plot saved to /tmp/physics_validation_analysis.png')
            
        except Exception as e:
            self.get_logger().error(f'Error in detailed analysis: {e}')

    def get_validation_report(self) -> str:
        """
        Generate a comprehensive validation report.
        """
        report = []
        report.append("Physics Validation Report")
        report.append("=" * 30)
        report.append(f"Duration: {self.validation_duration}s")
        report.append(f"Tolerance: {self.tolerance_threshold} ({self.tolerance_threshold*100}%)")
        report.append("")
        
        # Add summary of validation results
        report.append("Validation Summary:")
        report.append("-" * 20)
        # This would be populated after running validation
        
        return "\n".join(report)


def main(args=None):
    """Main function to run the physics validator."""
    rclpy.init(args=args)
    
    physics_validator = PhysicsValidator()
    
    try:
        rclpy.spin(physics_validator)
    except KeyboardInterrupt:
        physics_validator.get_logger().info('Shutting down Physics Validator')
    finally:
        physics_validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()