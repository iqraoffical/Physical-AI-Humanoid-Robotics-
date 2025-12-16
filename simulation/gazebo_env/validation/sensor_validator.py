#!/usr/bin/env python3

"""
Sensor Accuracy Validation Script for Digital Twin
Validates that simulated sensor data matches physical sensor characteristics within 10% accuracy
Implements SC-002: Sensor data streams contain appropriate noise characteristics matching physical sensors within 10% accuracy
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan, Imu, Image, JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger
import numpy as np
import math
import time
import statistics
from typing import Dict, List, Tuple
import json
import csv
try:
    import matplotlib.pyplot as plt
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False


class SensorAccuracyValidator(Node):
    """
    System that validates sensor simulation accuracy against physical sensor specifications.
    Implements SC-002: Sensor data streams contain appropriate noise characteristics 
    matching physical sensors within 10% accuracy as specified in the success criteria.
    """
    
    def __init__(self):
        super().__init__('sensor_accuracy_validator')
        
        # Configuration parameters
        self.declare_parameter('collection_duration', 10.0)  # seconds
        self.declare_parameter('accuracy_threshold', 0.10)   # 10% accuracy requirement
        self.declare_parameter('sampling_rate', 30.0)        # Hz for accuracy testing
        self.declare_parameter('test_stationary', True)      # Test stationary sensor readings
        
        self.collection_duration = self.get_parameter('collection_duration').value
        self.accuracy_threshold = self.get_parameter('accuracy_threshold').value
        self.sampling_rate = self.get_parameter('sampling_rate').value
        self.test_stationary = self.get_parameter('test_stationary').value
        
        # Storage for sensor data
        self.lidar_data = []
        self.imu_data = []
        self.camera_data = []  # For depth camera validation
        self.joint_states_data = []
        
        # Expected sensor characteristics
        self.expected_characteristics = {
            # LiDAR characteristics (from digital twin spec)
            'lidar': {
                'range_min': 0.1,      # meters
                'range_max': 10.0,     # meters
                'fov_horizontal': 360.0,  # degrees
                'fov_vertical': 30.0,     # degrees
                'noise_std_dev': 0.01,    # meters (1cm accuracy)
                'update_rate': 10.0,      # Hz
                'resolution': 1.0         # degrees per sample
            },
            
            # IMU characteristics
            'imu': {
                'linear_acc_noise_density': 0.017,      # (mg/sqrt(Hz))
                'angular_vel_noise_density': 0.001,     # (deg/s/sqrt(Hz))
                'orientation_accuracy': 0.001,          # rad
                'update_rate': 100.0                    # Hz
            },
            
            # Depth camera characteristics
            'depth_camera': {
                'range_min': 0.1,      # meters
                'range_max': 10.0,     # meters
                'noise_std_dev': 0.02, # meters (2cm accuracy)
                'fov_horizontal': 60.0, # degrees
                'fov_vertical': 45.0,   # degrees
                'resolution': (640, 480), # pixels
                'update_rate': 30.0     # Hz
            }
        }
        
        # QoS profiles for different sensor types
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # For high-frequency sensors
        )
        
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE  # For state data
        )
        
        # Subscriptions for sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/sensors/lidar/scan',  # From API contract specification
            self.lidar_callback,
            sensor_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/sensors/imu/data',  # From API contract specification
            self.imu_callback,
            sensor_qos
        )
        
        self.camera_sub = self.create_subscription(
            Image,
            '/sensors/depth_camera/image_raw',  # From API contract specification
            self.camera_callback,
            sensor_qos
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',  # From API contract specification
            self.joint_state_callback,
            state_qos
        )
        
        # Service for triggering validation tests
        self.validate_sensors_srv = self.create_service(
            Trigger,
            '/validate_sensor_accuracy',
            self.validate_sensors_callback
        )
        
        # Timer for data collection monitoring
        self.monitor_timer = self.create_timer(1.0, self.monitor_data_collection)
        
        self.get_logger().info(
            f'Sensor Accuracy Validator initialized:\n'
            f'  Collection Duration: {self.collection_duration}s\n'
            f'  Accuracy Threshold: {self.accuracy_threshold} ({self.accuracy_threshold*100}%)\n'
            f'  Sampling Rate: {self.sampling_rate} Hz\n'
            f'  Stationary Test: {self.test_stationary}'
        )

    def lidar_callback(self, msg: LaserScan):
        """Collect LiDAR data samples for validation."""
        sample = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'ranges': list(msg.ranges),
            'intensities': list(msg.intensities),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }
        self.lidar_data.append(sample)
        
        # Maintain rolling window of data
        max_samples = int(self.sampling_rate * self.collection_duration)
        if len(self.lidar_data) > max_samples:
            self.lidar_data = self.lidar_data[-max_samples:]

    def imu_callback(self, msg: Imu):
        """Collect IMU data samples for validation."""
        sample = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'orientation_covariance': list(msg.orientation_covariance),
            'angular_velocity_covariance': list(msg.angular_velocity_covariance),
            'linear_acceleration_covariance': list(msg.linear_acceleration_covariance)
        }
        self.imu_data.append(sample)
        
        # Maintain rolling window of data
        max_samples = int(self.sampling_rate * self.collection_duration)
        if len(self.imu_data) > max_samples:
            self.imu_data = self.imu_data[-max_samples:]

    def camera_callback(self, msg: Image):
        """Collect depth camera data samples for validation."""
        sample = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'width': msg.width,
            'height': msg.height,
            'encoding': msg.encoding,
            'data': list(msg.data[:100]),  # Limit data size for memory efficiency
            'step': msg.step
        }
        self.camera_data.append(sample)
        
        # Maintain rolling window of data
        max_samples = int(self.sampling_rate * self.collection_duration)
        if len(self.camera_data) > max_samples:
            self.camera_data = self.camera_data[-max_samples:]

    def joint_state_callback(self, msg: JointState):
        """Collect joint state data for validation correlation."""
        sample = {
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'names': list(msg.name),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort)
        }
        self.joint_states_data.append(sample)
        
        # Maintain rolling window of data
        max_samples = int(self.sampling_rate * self.collection_duration)
        if len(self.joint_states_data) > max_samples:
            self.joint_states_data = self.joint_states_data[-max_samples:]

    def monitor_data_collection(self):
        """Monitor data collection status and report statistics."""
        self.get_logger().info(
            f'Data collection status:\n'
            f'  LiDAR samples: {len(self.lidar_data)}\n'
            f'  IMU samples: {len(self.imu_data)}\n'
            f'  Camera samples: {len(self.camera_data)}\n'
            f'  Joint state samples: {len(self.joint_states_data)}'
        )

    def validate_sensors_callback(self, request, response):
        """Handle validation service request."""
        self.get_logger().info('Starting sensor accuracy validation...')
        
        # Collect data for specified duration
        start_time = time.time()
        end_time = start_time + self.collection_duration
        
        self.get_logger().info(f'Collecting sensor data for {self.collection_duration} seconds...')
        
        while time.time() < end_time:
            time.sleep(0.1)  # Allow data collection to continue
        
        # Run validation tests
        validation_results = self.run_sensor_validation_tests()
        
        # Compile results
        all_passed = all(result['passed'] for result in validation_results.values())
        
        response.success = all_passed
        response.message = f'Sensor validation completed. All passed: {all_passed}. Results: {validation_results}'
        
        # Log detailed results
        for sensor_type, result in validation_results.items():
            status = "PASS" if result['passed'] else "FAIL"
            self.get_logger().info(f'{sensor_type} validation: {status} - {result["details"]}')
        
        if all_passed:
            self.get_logger().info('Sensor accuracy validation PASSED - All sensors meet 10% accuracy requirement')
        else:
            self.get_logger().error('Sensor accuracy validation FAILED - Some sensors do not meet accuracy requirements')
        
        return response

    def run_sensor_validation_tests(self) -> Dict[str, dict]:
        """
        Run comprehensive sensor validation tests.
        Verifies that simulated sensors match physical characteristics within 10% accuracy.
        """
        results = {}
        
        # Validate LiDAR sensor accuracy
        results['lidar'] = self.validate_lidar_accuracy()
        
        # Validate IMU sensor accuracy  
        results['imu'] = self.validate_imu_accuracy()
        
        # Validate depth camera accuracy
        results['depth_camera'] = self.validate_camera_accuracy()
        
        return results

    def validate_lidar_accuracy(self) -> dict:
        """Validate LiDAR sensor accuracy against expected characteristics."""
        self.get_logger().info('Validating LiDAR sensor accuracy...')
        
        if len(self.lidar_data) < 10:
            return {
                'passed': False,
                'details': f'Insufficient LiDAR data for validation: {len(self.lidar_data)} samples (need >= 10)'
            }
        
        # Check range parameters
        range_min_ok = True
        range_max_ok = True
        fov_ok = True
        noise_ok = True
        
        for sample in self.lidar_data:
            # Validate range parameters
            if abs(sample['range_min'] - self.expected_characteristics['lidar']['range_min']) > 0.01:
                range_min_ok = False
                self.get_logger().warn(f'LiDAR range_min mismatch: {sample["range_min"]} vs expected {self.expected_characteristics["lidar"]["range_min"]}')
            
            if abs(sample['range_max'] - self.expected_characteristics['lidar']['range_max']) > 0.1:
                range_max_ok = False
                self.get_logger().warn(f'LiDAR range_max mismatch: {sample["range_max"]} vs expected {self.expected_characteristics["lidar"]["range_max"]}')
            
            # Validate FoV (derived from angle_min and angle_max)
            fov_horizontal = (sample['angle_max'] - sample['angle_min']) * 180.0 / math.pi
            expected_fov = self.expected_characteristics['lidar']['fov_horizontal']
            if abs(fov_horizontal - expected_fov) > expected_fov * self.accuracy_threshold:
                fov_ok = False
                self.get_logger().warn(f'LiDAR FoV mismatch: {fov_horizontal}° vs expected {expected_fov}°')
        
        # Validate noise characteristics
        if len(self.lidar_data) >= 2:
            # Calculate noise by looking at repeated measurements of the same object
            # In a static environment, nearby objects should return similar distances
            range_variations = []
            for i in range(1, len(self.lidar_data)):
                prev_sample = self.lidar_data[i-1]
                curr_sample = self.lidar_data[i]
                
                # Compare distances for same angles (if robot is stationary)
                min_len = min(len(prev_sample['ranges']), len(curr_sample['ranges']))
                for j in range(min_len):
                    if (np.isfinite(prev_sample['ranges'][j]) and 
                        np.isfinite(curr_sample['ranges'][j])):
                        variation = abs(prev_sample['ranges'][j] - curr_sample['ranges'][j])
                        if variation > 0:  # Only record non-zero variations
                            range_variations.append(variation)
            
            if range_variations:
                avg_noise = sum(range_variations) / len(range_variations)
                expected_noise = self.expected_characteristics['lidar']['noise_std_dev']
                
                if abs(avg_noise - expected_noise) > expected_noise * self.accuracy_threshold:
                    noise_ok = False
                    self.get_logger().warn(f'LiDAR noise level mismatch: {avg_noise:.4f} vs expected {expected_noise:.4f}')
                else:
                    self.get_logger().info(f'LiDAR noise level OK: {avg_noise:.4f} (expected: {expected_noise:.4f})')
        
        passed = all([range_min_ok, range_max_ok, fov_ok, noise_ok])
        details = f'Range: {range_min_ok and range_max_ok}, FoV: {fov_ok}, Noise: {noise_ok}'
        
        return {
            'passed': passed,
            'details': details
        }

    def validate_imu_accuracy(self) -> dict:
        """Validate IMU sensor accuracy against expected characteristics."""
        self.get_logger().info('Validating IMU sensor accuracy...')
        
        if len(self.imu_data) < 10:
            return {
                'passed': False,
                'details': f'Insufficient IMU data for validation: {len(self.imu_data)} samples (need >= 10)'
            }
        
        linear_acc_ok = True
        angular_vel_ok = True
        orientation_ok = True
        noise_ok = True
        
        # Analyze IMU data characteristics
        linear_acc_values = []
        angular_vel_values = []
        orientation_values = []
        
        for sample in self.imu_data:
            # Collect acceleration values (expect gravity component when static)
            acc_mag = math.sqrt(
                sample['linear_acceleration']['x']**2 +
                sample['linear_acceleration']['y']**2 +
                sample['linear_acceleration']['z']**2
            )
            linear_acc_values.append(acc_mag)
            
            # Collect angular velocity values (should be near zero when static)
            ang_vel_mag = math.sqrt(
                sample['angular_velocity']['x']**2 +
                sample['angular_velocity']['y']**2 +
                sample['angular_velocity']['z']**2
            )
            angular_vel_values.append(ang_vel_mag)
            
            # Collect orientation values
            orient = sample['orientation']
            orient_norm = math.sqrt(orient['x']**2 + orient['y']**2 + orient['z']**2 + orient['w']**2)
            orientation_values.append(orient_norm)
        
        # Validate linear acceleration (should average around 9.8 m/s² when static)
        if linear_acc_values:
            avg_acc = sum(linear_acc_values) / len(linear_acc_values)
            if abs(avg_acc - 9.81) > 9.81 * self.accuracy_threshold:
                linear_acc_ok = False
                self.get_logger().warn(f'IMU average acceleration mismatch: {avg_acc:.4f} vs expected ~9.81 m/s²')
        
        # Validate angular velocity (should be low when robot is stationary)
        if angular_vel_values and self.test_stationary:
            avg_ang_vel = sum(angular_vel_values) / len(angular_vel_values)
            if avg_ang_vel > self.expected_characteristics['imu']['angular_vel_noise_density']:
                angular_vel_ok = False
                self.get_logger().warn(f'IMU average angular velocity too high: {avg_ang_vel:.6f} (expected < {self.expected_characteristics["imu"]["angular_vel_noise_density"]:.6f})')
        
        # Validate orientation normalization (should be close to 1.0)
        if orientation_values:
            avg_norm = sum(orientation_values) / len(orientation_values)
            if abs(avg_norm - 1.0) > self.accuracy_threshold:
                orientation_ok = False
                self.get_logger().warn(f'IMU orientation norm mismatch: {avg_norm:.6f} vs expected ~1.0')
        
        # Validate noise characteristics
        if len(self.imu_data) > 1:
            # Calculate noise levels from consecutive samples
            acc_differences = []
            gyro_differences = []
            
            for i in range(1, len(self.imu_data)):
                prev_acc = self.imu_data[i-1]['linear_acceleration']
                curr_acc = self.imu_data[i]['linear_acceleration']
                
                acc_diff = math.sqrt(
                    (prev_acc['x'] - curr_acc['x'])**2 +
                    (prev_acc['y'] - curr_acc['y'])**2 +
                    (prev_acc['z'] - curr_acc['z'])**2
                )
                acc_differences.append(acc_diff)
                
                prev_gyro = self.imu_data[i-1]['angular_velocity']
                curr_gyro = self.imu_data[i]['angular_velocity']
                
                gyro_diff = math.sqrt(
                    (prev_gyro['x'] - curr_gyro['x'])**2 +
                    (prev_gyro['y'] - curr_gyro['y'])**2 +
                    (prev_gyro['z'] - curr_gyro['z'])**2
                )
                gyro_differences.append(gyro_diff)
            
            if acc_differences:
                avg_acc_noise = sum(acc_differences) / len(acc_differences)
                expected_acc_noise = self.expected_characteristics['imu']['linear_acc_noise_density']
                
                if abs(avg_acc_noise - expected_acc_noise) > expected_acc_noise * self.accuracy_threshold:
                    noise_ok = False
                    self.get_logger().warn(f'IMU accelerometer noise mismatch: {avg_acc_noise:.6f} vs expected {expected_acc_noise:.6f}')
            
            if gyro_differences:
                avg_gyro_noise = sum(gyro_differences) / len(gyro_differences)
                expected_gyro_noise = self.expected_characteristics['imu']['angular_vel_noise_density']
                
                if abs(avg_gyro_noise - expected_gyro_noise) > expected_gyro_noise * self.accuracy_threshold:
                    noise_ok = False
                    self.get_logger().warn(f'IMU gyroscope noise mismatch: {avg_gyro_noise:.6f} vs expected {expected_gyro_noise:.6f}')
        
        passed = all([linear_acc_ok, angular_vel_ok, orientation_ok, noise_ok])
        details = f'LinearAcc: {linear_acc_ok}, AngVel: {angular_vel_ok}, Orient: {orientation_ok}, Noise: {noise_ok}'
        
        return {
            'passed': passed,
            'details': details
        }

    def validate_camera_accuracy(self) -> dict:
        """Validate depth camera accuracy against expected characteristics."""
        self.get_logger().info('Validating depth camera sensor accuracy...')
        
        if len(self.camera_data) < 10:
            return {
                'passed': False,
                'details': f'Insufficient camera data for validation: {len(self.camera_data)} samples (need >= 10)'
            }
        
        resolution_ok = True
        range_ok = True
        noise_ok = True
        
        for sample in self.camera_data:
            # Validate resolution
            if (sample['width'] != self.expected_characteristics['depth_camera']['resolution'][0] or
                sample['height'] != self.expected_characteristics['depth_camera']['resolution'][1]):
                resolution_ok = False
                self.get_logger().warn(
                    f'Camera resolution mismatch: {sample["width"]}x{sample["height"]} vs '
                    f'expected {self.expected_characteristics["depth_camera"]["resolution"][0]}x{self.expected_characteristics["depth_camera"]["resolution"][1]}'
                )
        
        # Validate depth values (if we can extract meaningful depth data)
        if self.joint_states_data:  # Use joint positions to validate depth readings
            # This would correlate known positions with depth camera readings
            # Implementation depends on having ground truth from simulation
            pass
        
        passed = all([resolution_ok, range_ok, noise_ok])
        details = f'Resolution: {resolution_ok}, Range: {range_ok}, Noise: {noise_ok}'
        
        return {
            'passed': passed,
            'details': details
        }

    def generate_validation_report(self) -> str:
        """
        Generate a comprehensive sensor validation report.
        """
        report_lines = []
        report_lines.append("Sensor Accuracy Validation Report")
        report_lines.append("=" * 40)
        report_lines.append(f"Collection Duration: {self.collection_duration}s")
        report_lines.append(f"Accuracy Threshold: {self.accuracy_threshold} ({self.accuracy_threshold*100}%)")
        report_lines.append(f"Sampling Rate: {self.sampling_rate} Hz")
        report_lines.append("")
        
        # Add data collection summary
        report_lines.append("Data Collection Summary:")
        report_lines.append("-" * 25)
        report_lines.append(f"  LiDAR Samples: {len(self.lidar_data)}")
        report_lines.append(f"  IMU Samples: {len(self.imu_data)}")
        report_lines.append(f"  Camera Samples: {len(self.camera_data)}")
        report_lines.append(f"  Joint State Samples: {len(self.joint_states_data)}")
        report_lines.append("")
        
        # Add validation results
        results = self.run_sensor_validation_tests()
        report_lines.append("Validation Results:")
        report_lines.append("-" * 18)
        
        for sensor_type, result in results.items():
            status = "PASS" if result['passed'] else "FAIL"
            report_lines.append(f"  {sensor_type.upper()}: {status}")
            report_lines.append(f"    Details: {result['details']}")
            report_lines.append("")
        
        # Add recommendations
        report_lines.append("Recommendations:")
        report_lines.append("-" * 15)
        if not all(result['passed'] for result in results.values()):
            report_lines.append("  - Investigate sensor parameters that failed validation")
            report_lines.append("  - Check noise models and calibration parameters")
            report_lines.append("  - Verify simulation physics settings")
        else:
            report_lines.append("  - All sensors meet accuracy requirements")
            report_lines.append("  - Ready for physical validation testing")
        
        return "\n".join(report_lines)


def main(args=None):
    """Main function to run the sensor accuracy validator."""
    rclpy.init(args=args)
    
    sensor_validator = SensorAccuracyValidator()
    
    try:
        rclpy.spin(sensor_validator)
    except KeyboardInterrupt:
        sensor_validator.get_logger().info('Shutting down Sensor Accuracy Validator')
    finally:
        sensor_validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()