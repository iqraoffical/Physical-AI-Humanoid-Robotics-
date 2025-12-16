#!/usr/bin/env python3

"""
Digital Twin Synchronization Controller
Implements real-time synchronization between Gazebo simulation and Unity visualization
Satisfies FR-009: Unity scene with real-time humanoid visualization synchronized with simulation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState, LaserScan, Imu, Image
from nav_msgs.msg import Odometry
import socket
import json
import struct
import threading
from typing import Dict, List, Any
import numpy as np
import time


class DigitalTwinController(Node):
    """
    System that synchronizes simulation state between Gazebo and Unity environments.
    Implements the synchronization requirements from FR-009.
    """
    
    def __init__(self):
        super().__init__('digital_twin_controller')
        
        # Configuration parameters
        self.declare_parameter('unity_ip', '127.0.0.1')
        self.declare_parameter('unity_port', 5005)
        self.declare_parameter('sync_frequency', 60.0)  # Hz
        self.declare_parameter('max_desync_threshold', 0.05)  # seconds
        
        self.unity_ip = self.get_parameter('unity_ip').value
        self.unity_port = self.get_parameter('unity_port').value
        self.sync_frequency = self.get_parameter('sync_frequency').value
        self.max_desync_threshold = self.get_parameter('max_desync_threshold').value
        
        # Socket for Unity communication
        self.unity_socket = None
        self.socket_lock = threading.Lock()
        
        # Connection management
        self.connected_to_unity = False
        self.connection_thread = None
        self.sync_thread = None
        self.should_run = True
        
        # Data storage
        self.robot_states = {}  # Multiple robots support
        self.environment_state = {}
        self.sensor_data = {}
        self.simulation_time = 0.0
        self.last_sync_time = self.get_clock().now().nanoseconds * 1e-9
        
        # QoS profiles for different data types
        # High-frequency sensor data
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Critical state data
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscriptions for simulation data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            state_qos
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            state_qos
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            sensor_qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )
        
        # Clock synchronization from simulation
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10
        )
        
        # Timer for Unity synchronization
        self.sync_timer = self.create_timer(1.0/self.sync_frequency, self.sync_with_unity)
        
        # Connect to Unity in a separate thread
        self.connection_thread = threading.Thread(target=self.connect_to_unity, daemon=True)
        self.connection_thread.start()
        
        self.get_logger().info(
            f'Digital Twin Controller initialized.\n'
            f'Unity IP: {self.unity_ip}:{self.unity_port}\n'
            f'Sync Frequency: {self.sync_frequency} Hz\n'
            f'Max Desync Threshold: {self.max_desync_threshold} s'
        )

    def connect_to_unity(self):
        """Establish connection to Unity application."""
        while self.should_run and not self.connected_to_unity:
            try:
                self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.unity_socket.settimeout(5.0)
                self.unity_socket.connect((self.unity_ip, self.unity_port))
                self.connected_to_unity = True
                self.get_logger().info(f'Connected to Unity at {self.unity_ip}:{self.unity_port}')
                
                # Start sync thread once connected
                self.sync_thread = threading.Thread(target=self.unity_sync_loop, daemon=True)
                self.sync_thread.start()
                
            except socket.error as e:
                self.get_logger().warn(f'Connection failed: {e}. Retrying in 2 seconds...')
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f'Unexpected error during connection: {e}')
                time.sleep(5)

    def unity_sync_loop(self):
        """Main synchronization loop with Unity."""
        while self.should_run and self.connected_to_unity:
            try:
                # Receive any commands from Unity
                command = self.receive_from_unity()
                if command:
                    self.handle_unity_command(command)
                
            except socket.error:
                self.connected_to_unity = False
                self.get_logger().warn('Lost connection to Unity. Attempting to reconnect...')
                time.sleep(2)
                if self.unity_socket:
                    try:
                        self.unity_socket.close()
                    except:
                        pass
                self.attempt_reconnection()
            except Exception as e:
                self.get_logger().error(f'Error in Unity sync loop: {e}')
                time.sleep(0.1)

    def sync_with_unity(self):
        """Send current simulation state to Unity at regular intervals."""
        if not self.connected_to_unity:
            return
            
        try:
            # Create synchronization data
            sync_data = {
                'timestamp': self.get_clock().now().nanoseconds * 1e-9,
                'simulation_time': self.simulation_time,
                'robot_states': dict(self.robot_states),
                'environment_state': dict(self.environment_state),
                'sensor_data': dict(self.sensor_data)
            }
            
            # Convert to JSON and send to Unity
            json_data = json.dumps(sync_data)
            self.send_to_unity(json_data)
            
            # Check for synchronization accuracy
            current_time = self.get_clock().now().nanoseconds * 1e-9
            time_diff = abs(current_time - self.last_sync_time - 1.0/self.sync_frequency)
            
            if time_diff > self.max_desync_threshold:
                self.get_logger().warn(
                    f'Sync desynchronization detected: {time_diff:.4f}s '
                    f'(threshold: {self.max_desync_threshold}s)'
                )
            
            self.last_sync_time = current_time
            
        except Exception as e:
            self.get_logger().error(f'Error during unity sync: {e}')

    def send_to_unity(self, data: str):
        """Send data to Unity with length prefix."""
        with self.socket_lock:
            if self.unity_socket:
                try:
                    # Prefix with length to handle message boundaries
                    length_prefix = struct.pack('!I', len(data))
                    self.unity_socket.send(length_prefix + data.encode('utf-8'))
                except socket.error as e:
                    self.connected_to_unity = False
                    self.get_logger().error(f'Socket error during send: {e}')

    def receive_from_unity(self) -> str:
        """Receive data from Unity."""
        with self.socket_lock:
            if self.unity_socket:
                try:
                    # First receive length prefix
                    length_data = self.unity_socket.recv(4)
                    if len(length_data) == 4:
                        length = struct.unpack('!I', length_data)[0]
                        # Then receive the actual message
                        message_data = b''
                        while len(message_data) < length:
                            chunk = self.unity_socket.recv(length - len(message_data))
                            if not chunk:
                                break
                            message_data += chunk
                        
                        return message_data.decode('utf-8')
                except socket.timeout:
                    # No data available, this is normal
                    return None
                except socket.error as e:
                    self.connected_to_unity = False
                    self.get_logger().error(f'Error receiving from Unity: {e}')
                    return None
        return None

    def attempt_reconnection(self):
        """Try to reconnect to Unity."""
        time.sleep(2)  # Brief pause before reconnection attempt
        try:
            if self.unity_socket:
                self.unity_socket.close()
        except:
            pass
        
        # Restart connection process
        self.connection_thread = threading.Thread(target=self.connect_to_unity, daemon=True)
        self.connection_thread.start()

    def joint_state_callback(self, msg: JointState):
        """Process joint state messages from Gazebo."""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        for i, name in enumerate(msg.name):
            if 'robot' not in self.robot_states:
                self.robot_states['robot'] = {}
            
            self.robot_states['robot'][name] = {
                'position': float(msg.position[i]) if i < len(msg.position) else 0.0,
                'velocity': float(msg.velocity[i]) if i < len(msg.velocity) else 0.0,
                'effort': float(msg.effort[i]) if i < len(msg.effort) else 0.0,
                'timestamp': timestamp
            }

    def odometry_callback(self, msg: Odometry):
        """Process odometry messages from Gazebo."""
        robot_name = 'robot'  # Default name
        
        if robot_name not in self.robot_states:
            self.robot_states[robot_name] = {}
        
        self.robot_states[robot_name]['pose'] = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
        }
        
        self.robot_states[robot_name]['twist'] = {
            'linear': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            },
            'angular': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            }
        }

    def lidar_callback(self, msg: LaserScan):
        """Process LiDAR data from simulation."""
        self.sensor_data['lidar'] = {
            'ranges': [float(r) for r in msg.ranges],
            'intensities': [float(i) for i in msg.intensities],
            'angle_min': float(msg.angle_min),
            'angle_max': float(msg.angle_max),
            'angle_increment': float(msg.angle_increment),
            'range_min': float(msg.range_min),
            'range_max': float(msg.range_max)
        }

    def imu_callback(self, msg: Imu):
        """Process IMU data from simulation."""
        self.sensor_data['imu'] = {
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
            }
        }

    def clock_callback(self, msg: Clock):
        """Process simulation clock messages."""
        self.simulation_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def handle_unity_command(self, command_json: str):
        """Process commands received from Unity."""
        try:
            command = json.loads(command_json)
            cmd_type = command.get('type')
            
            if cmd_type == 'set_joint_positions':
                # Forward joint commands to robot
                joint_positions = command.get('positions', {})
                self.execute_joint_command(joint_positions)
            elif cmd_type == 'request_robot_state':
                # Respond with current robot state
                self.send_robot_state_to_unity()
            else:
                self.get_logger().warn(f'Unknown command type from Unity: {cmd_type}')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON received from Unity: {command_json}')
        except Exception as e:
            self.get_logger().error(f'Error handling Unity command: {e}')

    def execute_joint_command(self, joint_positions: Dict[str, float]):
        """Forward joint commands to the robot."""
        # In a real implementation, this would publish to joint command topics
        self.get_logger().debug(f'Executing joint command: {joint_positions}')

    def send_robot_state_to_unity(self):
        """Send current robot state to Unity."""
        state_data = {
            'type': 'robot_state_response',
            'robot_states': dict(self.robot_states),
            'timestamp': self.get_clock().now().nanoseconds * 1e-9
        }
        self.send_to_unity(json.dumps(state_data))

    def destroy_node(self):
        """Clean up resources before destroying the node."""
        self.should_run = False
        if self.unity_socket:
            try:
                self.unity_socket.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    """Main function to run the digital twin controller."""
    rclpy.init(args=args)
    
    controller = DigitalTwinController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Digital Twin Controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()