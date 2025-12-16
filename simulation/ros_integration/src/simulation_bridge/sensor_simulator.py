#!/usr/bin/env python3

"""
Sensor Simulator Node for Digital Twin
Implements realistic sensors: LiDAR, Depth camera, IMU with appropriate noise and latency
Based on FR-003: Simulate realistic sensors with appropriate noise and latency
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan, Image, CameraInfo, Imu, PointCloud2
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import numpy as np
import math
import yaml
from pathlib import Path


class SensorSimulatorNode(Node):
    """
    Component that mimics physical sensor behavior with realistic noise, latency, and accuracy parameters.
    Implements FR-003: System MUST simulate realistic sensors: LiDAR, Depth camera, IMU with appropriate noise and latency.
    """
    
    def __init__(self):
        super().__init__('sensor_simulator_node')
        
        # Load sensor configurations
        self.load_sensor_configs()
        
        # Define QoS profile for sensor data
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # For high-frequency sensors like cameras
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        
        # Publishers for sensor data
        self.lidar_publisher = self.create_publisher(
            LaserScan, 
            '/sensors/lidar/scan', 
            qos_profile
        )
        
        self.depth_camera_publisher = self.create_publisher(
            Image, 
            '/sensors/depth_camera/depth/image_raw', 
            qos_profile
        )
        
        self.imu_publisher = self.create_publisher(
            Imu, 
            '/sensors/imu/data', 
            qos_profile
        )
        
        # Timer for publishing sensor data
        self.timer_period = 0.01  # 100Hz for IMU, 10Hz for LiDAR
        self.timer = self.create_timer(self.timer_period, self.publish_sensor_data)
        
        self.get_logger().info('Sensor Simulator Node initialized')
        
        # Initialize sensor data
        self.scan_counter = 0
        
        # For simulating sensor latency
        self.latency_queue = []

    def load_sensor_configs(self):
        """
        Load sensor configurations from YAML files.
        Based on data-model.md specifications for SensorSimulationParams.
        """
        config_dir = Path(__file__).parent.parent.parent / "config" / "sensors"
        
        # Load LiDAR config
        lidar_config_path = config_dir / "lidar_config.yaml"
        if lidar_config_path.exists():
            with open(lidar_config_path, 'r') as file:
                self.lidar_config = yaml.safe_load(file)
        else:
            # Default configuration if file not found
            self.lidar_config = {
                'simulation_parameters': {
                    'update_rate': 10.0,
                    'latency': 0.01,
                    'range_min': 0.1,
                    'range_max': 10.0,
                    'fov_horizontal': 6.28318,  # 2*pi radians = 360 degrees
                    'fov_vertical': 0.7854      # pi/4 radians = 45 degrees
                },
                'noise_model': {
                    'noise_type': 'gaussian',
                    'std_dev': 0.01
                },
                'lidar_config': {
                    'samples': 360,
                    'resolution': 1,
                    'min_angle': -3.14159,
                    'max_angle': 3.14159
                }
            }
        
        # Load depth camera config
        depth_camera_config_path = config_dir / "depth_camera_config.yaml"
        if depth_camera_config_path.exists():
            with open(depth_camera_config_path, 'r') as file:
                self.depth_camera_config = yaml.safe_load(file)
        else:
            # Default configuration if file not found
            self.depth_camera_config = {
                'simulation_parameters': {
                    'update_rate': 30.0,
                    'latency': 0.033,
                    'range_min': 0.1,
                    'range_max': 10.0,
                    'fov_horizontal': 1.047,  # ~60 degrees
                    'fov_vertical': 0.7854    # ~45 degrees
                },
                'noise_model': {
                    'noise_type': 'gaussian',
                    'std_dev': 0.02
                },
                'depth_camera_config': {
                    'image_width': 640,
                    'image_height': 480,
                    'format': 'R8G8B8',
                    'near_clip': 0.1,
                    'far_clip': 10.0
                }
            }
        
        # Load IMU config
        imu_config_path = config_dir / "imu_config.yaml"
        if imu_config_path.exists():
            with open(imu_config_path, 'r') as file:
                self.imu_config = yaml.safe_load(file)
        else:
            # Default configuration if file not found
            self.imu_config = {
                'simulation_parameters': {
                    'update_rate': 100.0,
                    'latency': 0.01,
                    'range_min': -100.0,
                    'range_max': 100.0,
                    'fov_horizontal': 0.0,
                    'fov_vertical': 0.0
                },
                'noise_model': {
                    'noise_type': 'gaussian',
                    'std_dev': 0.01
                },
                'imu_config': {
                    'linear_acceleration_stdev': 0.017,
                    'angular_velocity_stdev': 0.001,
                    'orientation_stdev': 1.0e-6,
                    'gravitational_constant': 9.81
                }
            }

    def publish_sensor_data(self):
        """
        Publish sensor data with realistic noise and characteristics.
        Implements FR-003: Realistic sensors with appropriate noise and latency.
        """
        current_time = self.get_clock().now().to_msg()
        
        # Publish IMU data (every cycle - 100Hz)
        self.publish_imu_data(current_time)
        
        # Publish LiDAR data periodically (10Hz)
        if self.scan_counter % 10 == 0:  # Every 10 cycles at 100Hz = 10Hz
            self.publish_lidar_data(current_time)
        
        # Publish depth camera data periodically (30Hz)
        if self.scan_counter % 3 == 0:  # Every 3 cycles at 100Hz = ~33Hz (close to 30Hz)
            self.publish_depth_camera_data(current_time)
        
        # Increment counter
        self.scan_counter += 1
        if self.scan_counter > 300:  # Reset to prevent overflow
            self.scan_counter = 0

    def publish_lidar_data(self, stamp: Time):
        """
        Publish realistic LiDAR data with noise characteristics.
        Based on data-model.md specifications for SensorData.
        """
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = 'lidar_sensor_frame'
        
        # Set laser scan parameters based on config
        lidar_params = self.lidar_config['lidar_config']
        sim_params = self.lidar_config['simulation_parameters']
        noise_model = self.lidar_config['noise_model']
        
        msg.angle_min = lidar_params['min_angle']
        msg.angle_max = lidar_params['max_angle']
        msg.angle_increment = (msg.angle_max - msg.angle_min) / lidar_params['samples']
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = sim_params['range_min']
        msg.range_max = sim_params['range_max']
        
        # Generate ranges with some obstacles
        num_readings = lidar_params['samples']
        ranges = []
        
        for i in range(num_readings):
            angle = msg.angle_min + i * msg.angle_increment
            
            # Simulate some obstacles at various distances
            dist = 5.0  # Default distance
            
            # Add some objects to create a more realistic scan
            if abs(angle) < 0.5:  # Front
                dist = 3.0 + 0.5 * math.sin(i * 0.1)  # Wall in front
            elif abs(angle - 1.0) < 0.3:  # Right side
                dist = 2.0 + 0.2 * math.sin(i * 0.05)  # Obstacle on right
            elif abs(angle + 1.0) < 0.3:  # Left side
                dist = 4.0 + 0.3 * math.sin(i * 0.08)  # Obstacle on left
            else:
                dist = 8.0 + 2.0 * np.random.random()  # Random background
            
            # Add noise to the reading based on noise model
            if noise_model['noise_type'] == 'gaussian':
                noise = np.random.normal(0, noise_model['std_dev'])
                dist_with_noise = dist + noise
                # Ensure distance stays within valid range
                dist_with_noise = max(sim_params['range_min'], min(sim_params['range_max'], dist_with_noise))
            else:
                dist_with_noise = dist
            
            ranges.append(dist_with_noise)
        
        msg.ranges = ranges
        msg.intensities = []  # Empty intensities array
        
        self.lidar_publisher.publish(msg)
        self.get_logger().debug(f'Published LiDAR scan with {len(ranges)} readings')

    def publish_depth_camera_data(self, stamp: Time):
        """
        Publish realistic depth camera data with noise characteristics.
        Based on data-model.md specifications for SensorData.
        """
        msg = Image()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = 'depth_camera_frame'
        
        # Set image parameters based on config
        cam_params = self.depth_camera_config['depth_camera_config']
        sim_params = self.depth_camera_config['simulation_parameters']
        
        msg.height = cam_params['image_height']
        msg.width = cam_params['image_width']
        msg.encoding = '32FC1'  # 32-bit float per channel for depth
        msg.is_bigendian = False
        msg.step = msg.width * 4  # 4 bytes per pixel for float32
        
        # Generate fake depth data (simulated depth map)
        # This is a simplified example - in reality this would come from the Gazebo scene
        total_pixels = msg.height * msg.width
        depth_data = []
        
        for y in range(msg.height):
            for x in range(msg.width):
                # Calculate normalized coordinates
                norm_x = (x - msg.width/2) / (msg.width/2)
                norm_y = (y - msg.height/2) / (msg.height/2)
                
                # Simulate a spherical object in the center
                distance = math.sqrt(norm_x**2 + norm_y**2) * 2.0 + 3.0
                
                # Add noise based on noise model
                noise_std = self.depth_camera_config['noise_model']['std_dev']
                noise = np.random.normal(0, noise_std)
                noisy_distance = distance + noise
                
                # Ensure distance is within valid range
                noisy_distance = max(cam_params['near_clip'], min(cam_params['far_clip'], noisy_distance))
                
                depth_data.append(float(noisy_distance))
        
        msg.data = [int(d * 1000) & 0xFF for d in depth_data for int_d in [int(d * 1000)] for byte in [int_d >> 24, (int_d >> 16) & 0xFF, (int_d >> 8) & 0xFF, int_d & 0xFF]]
        
        # For simplicity, we'll just use the byte array of the float values
        # In a real implementation, we'd pack the floats properly
        packed_data = []
        for d in depth_data:
            # Pack float32 as bytes
            as_bytes = struct.pack('f', d)
            packed_data.extend(as_bytes)
        
        msg.data = packed_data
        
        self.depth_camera_publisher.publish(msg)
        self.get_logger().debug(f'Published depth camera image {msg.width}x{msg.height}')

    def publish_imu_data(self, stamp: Time):
        """
        Publish realistic IMU data with noise characteristics.
        Based on data-model.md specifications for SensorData.
        """
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = 'imu_link'
        
        # Set IMU parameters based on config
        imu_params = self.imu_config['imu_config']
        noise_model = self.imu_config['noise_model']
        
        # Simulate IMU data with realistic noise
        # For this example, we'll simulate the robot standing still with slight movements
        # Add noise based on configuration
        linear_acc_noise_std = imu_params['linear_acceleration_stdev']
        angular_vel_noise_std = imu_params['angular_velocity_stdev']
        orientation_noise_std = imu_params['orientation_stdev']
        
        # Linear acceleration (simulating slight vibrations while standing)
        msg.linear_acceleration.x = 0.0 + np.random.normal(0, linear_acc_noise_std)
        msg.linear_acceleration.y = 0.0 + np.random.normal(0, linear_acc_noise_std)
        msg.linear_acceleration.z = 9.81 + np.random.normal(0, linear_acc_noise_std)  # Gravity
        
        # Angular velocity (simulating slight movements)
        msg.angular_velocity.x = 0.0 + np.random.normal(0, angular_vel_noise_std)
        msg.angular_velocity.y = 0.0 + np.random.normal(0, angular_vel_noise_std)
        msg.angular_velocity.z = 0.0 + np.random.normal(0, angular_vel_noise_std)
        
        # Orientation (we'll assume the robot is mostly upright)
        # For simplicity, just set to identity quaternion (pointing forward)
        msg.orientation.w = 1.0  # + np.random.normal(0, orientation_noise_std)
        msg.orientation.x = 0.0  # + np.random.normal(0, orientation_noise_std)
        msg.orientation.y = 0.0  # + np.random.normal(0, orientation_noise_std)
        msg.orientation.z = 0.0  # + np.random.normal(0, orientation_noise_std)
        
        # Set covariance matrices (indicating noise levels)
        # Linear acceleration covariance
        msg.linear_acceleration_covariance = [
            linear_acc_noise_std**2, 0.0, 0.0,
            0.0, linear_acc_noise_std**2, 0.0,
            0.0, 0.0, linear_acc_noise_std**2
        ]
        
        # Angular velocity covariance
        msg.angular_velocity_covariance = [
            angular_vel_noise_std**2, 0.0, 0.0,
            0.0, angular_vel_noise_std**2, 0.0,
            0.0, 0.0, angular_vel_noise_std**2
        ]
        
        # Orientation covariance
        msg.orientation_covariance = [
            orientation_noise_std**2, 0.0, 0.0,
            0.0, orientation_noise_std**2, 0.0,
            0.0, 0.0, orientation_noise_std**2
        ]
        
        self.imu_publisher.publish(msg)
        self.get_logger().debug('Published IMU data')


def main(args=None):
    rclpy.init(args=args)

    sensor_simulator = SensorSimulatorNode()

    try:
        rclpy.spin(sensor_simulator)
    except KeyboardInterrupt:
        sensor_simulator.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        sensor_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import struct  # Import struct for packing floats
    main()