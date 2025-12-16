#!/usr/bin/env python3

"""
Unity ROS Connector for Digital Twin
Implements real-time synchronization between Gazebo simulation and Unity visualization
Based on FR-004: System MUST provide Unity scene with real-time humanoid visualization synchronized with simulation
and FR-009: System MUST use ROS 2 topics with clock synchronization via /clock
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan, Imu, Image
from builtin_interfaces.msg import Time
import socket
import json
import threading
import time
import struct
from typing import Dict, List, Optional


class UnityROSConnectorNode(Node):
    """
    System that synchronizes simulation state with Unity visualization.
    Implements FR-004: Unity scene with real-time humanoid visualization synchronized with simulation
    and FR-009: ROS 2 topics with clock synchronization via /clock.
    """
    
    def __init__(self):
        super().__init__('unity_ros_connector')
        
        # Configuration parameters
        self.declare_parameter('unity_ip', '127.0.0.1')
        self.declare_parameter('unity_port', 5005)
        self.declare_parameter('sync_frequency', 30.0)  # Hz
        self.declare_parameter('max_desync_threshold', 0.1)  # seconds
        
        self.unity_ip = self.get_parameter('unity_ip').value
        self.unity_port = self.get_parameter('unity_port').value
        self.sync_frequency = self.get_parameter('sync_frequency').value
        self.max_desync_threshold = self.get_parameter('max_desync_threshold').value
        
        # Socket for Unity communication
        self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.unity_socket.settimeout(1.0)
        
        # Connection status
        self.connected_to_unity = False
        self.connect_thread = None
        self.sync_thread = None
        
        # Data storage for synchronization
        self.robot_states = {}  # Dictionary to store states of multiple robots
        self.environment_state = {}
        self.last_sync_time = None
        
        # Define QoS profiles based on requirements
        # From QoS considerations in data-model.md
        sensor_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # For high-frequency sensors like cameras
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        state_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(seconds=0.05)  # 50ms deadline for real-time performance
        )
        
        # Publishers and subscribers
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            state_qos_profile
        )
        
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            state_qos_profile
        )
        
        # Sensor subscribers
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/sensors/lidar/scan',
            self.lidar_callback,
            sensor_qos_profile
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_callback,
            sensor_qos_profile
        )
        
        # Clock synchronization via /clock topic
        self.clock_subscriber = self.create_subscription(
            Time,
            '/clock',
            self.clock_callback,
            10  # Simple QoS for clock
        )
        
        # Timer for synchronization with Unity
        self.sync_timer = self.create_timer(1.0/self.sync_frequency, self.sync_with_unity)
        
        self.get_logger().info(
            f'Unity ROS Connector initialized:\n'
            f'  Unity IP: {self.unity_ip}\n'
            f'  Unity Port: {self.unity_port}\n'
            f'  Sync Frequency: {self.sync_frequency} Hz\n'
            f'  Max Desync Threshold: {self.max_desync_threshold} sec\n'
            f'  Starting connection thread...'
        )
        
        # Start connection thread to Unity
        self.connect_thread = threading.Thread(target=self.connect_to_unity, daemon=True)
        self.connect_thread.start()

    def connect_to_unity(self):
        """Attempt to connect to Unity application."""
        while rclpy.ok():
            try:
                self.unity_socket.connect((self.unity_ip, self.unity_port))
                self.connected_to_unity = True
                self.get_logger().info(f'Connected to Unity at {self.unity_ip}:{self.unity_port}')
                break
            except socket.error as e:
                self.get_logger().warn(f'Could not connect to Unity: {e}. Retrying in 5 seconds...')
                time.sleep(5)
    
    def clock_callback(self, msg: Time):
        """Handle clock messages for synchronization."""
        # Use ROS clock for synchronization between Gazebo and Unity
        pass  # Clock information is used implicitly by ROS time
    
    def joint_state_callback(self, msg: JointState):
        """Process incoming joint state messages."""
        robot_name = "default_humanoid"  # Default robot name
        
        if robot_name not in self.robot_states:
            self.robot_states[robot_name] = {
                'joint_names': [],
                'positions': [],
                'velocities': [],
                'efforts': [],
                'timestamp': msg.header.stamp
            }
        
        # Update joint state data
        self.robot_states[robot_name]['joint_names'] = list(msg.name)
        self.robot_states[robot_name]['positions'] = list(msg.position)
        self.robot_states[robot_name]['velocities'] = list(msg.velocity)
        self.robot_states[robot_name]['efforts'] = list(msg.effort)
        self.robot_states[robot_name]['timestamp'] = msg.header.stamp
        
        self.get_logger().debug(f'Updated joint states for {robot_name} with {len(msg.name)} joints')
    
    def odometry_callback(self, msg: Odometry):
        """Process incoming odometry messages."""
        robot_name = "default_humanoid"  # Default robot name
        
        if robot_name not in self.robot_states:
            self.robot_states[robot_name] = {
                'pose': None,
                'twist': None,
                'timestamp': msg.header.stamp
            }
        
        # Update pose and twist data
        self.robot_states[robot_name]['pose'] = msg.pose.pose
        self.robot_states[robot_name]['twist'] = msg.twist.twist
        self.robot_states[robot_name]['timestamp'] = msg.header.stamp
    
    def lidar_callback(self, msg: LaserScan):
        """Process incoming LiDAR data."""
        # Store LiDAR data for Unity synchronization
        self.environment_state['lidar'] = {
            'ranges': list(msg.ranges),
            'intensities': list(msg.intensities),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'time_increment': msg.time_increment,
            'scan_time': msg.scan_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'timestamp': msg.header.stamp
        }
    
    def imu_callback(self, msg: Imu):
        """Process incoming IMU data."""
        # Store IMU data for Unity synchronization
        self.environment_state['imu'] = {
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
            'timestamp': msg.header.stamp
        }

    def sync_with_unity(self):
        """Synchronize robot states and sensor data with Unity."""
        if not self.connected_to_unity:
            return
        
        try:
            # Prepare synchronization data
            sync_data = {
                'gazebo_time': self.get_clock().now().seconds_nanoseconds(),
                'robots': {},
                'environment': self.environment_state.copy()
            }
            
            # Add all robot states to sync data
            for robot_name, state in self.robot_states.items():
                sync_data['robots'][robot_name] = state.copy()
            
            # Serialize data to JSON
            json_data = json.dumps(sync_data, separators=(',', ':'))
            
            # Prefix with length to handle TCP packet boundaries
            data_with_length = struct.pack('!I', len(json_data)) + json_data.encode('utf-8')
            
            # Send data to Unity
            self.unity_socket.send(data_with_length)
            
            self.get_logger().debug(f'Sent sync data to Unity with {len(self.robot_states)} robots')
            
        except socket.error as e:
            self.get_logger().warn(f'Error sending data to Unity: {e}')
            self.connected_to_unity = False
            
            # Try to reconnect
            try:
                self.unity_socket.close()
                self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.unity_socket.settimeout(1.0)
            except:
                pass  # Ignore errors during socket reconnection
            
            # Restart connection thread
            if self.connect_thread is None or not self.connect_thread.is_alive():
                self.connect_thread = threading.Thread(target=self.connect_to_unity, daemon=True)
                self.connect_thread.start()
        except Exception as e:
            self.get_logger().error(f'Unexpected error in sync_with_unity: {e}')

    def send_command_to_robot(self, robot_name: str, joint_commands: Dict[str, float]):
        """Send commands to a specific robot in the simulation."""
        # This would implement sending commands back to Gazebo/ROS
        # For now, it's just a placeholder
        self.get_logger().info(f'Received command for {robot_name}: {joint_commands}')

    def destroy_node(self):
        """Clean up resources before destroying the node."""
        if self.unity_socket:
            self.unity_socket.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    unity_connector = UnityROSConnectorNode()
    
    try:
        rclpy.spin(unity_connector)
    except KeyboardInterrupt:
        unity_connector.get_logger().info('Keyboard interrupt received, shutting down')
    finally:
        unity_connector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()