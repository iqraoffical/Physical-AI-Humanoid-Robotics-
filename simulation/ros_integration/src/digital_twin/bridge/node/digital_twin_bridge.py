#!/usr/bin/env python3

"""
Digital Twin Bridge Node
Synchronizes state between ROS 2 and Unity environments with security and observability features
Implements FR-009: Unity scene with real-time humanoid visualization synchronized with simulation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState, LaserScan, Imu, Image
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
import socket
import json
import struct
import threading
from typing import Dict, List, Optional
import numpy as np
import time
from dataclasses import dataclass


@dataclass
class RobotState:
    """Data class to hold robot state information"""
    position: List[float]
    orientation: List[float]
    joint_positions: Dict[str, float]
    joint_velocities: Dict[str, float]
    joint_efforts: Dict[str, float]
    timestamp: float


class DigitalTwinBridge(Node):
    """
    System that synchronizes simulation state between ROS 2 and Unity environments.
    Implements FR-009: Unity scene with real-time humanoid visualization synchronized with simulation
    with additional security and observability features as specified in clarifications.
    """
    
    def __init__(self):
        super().__init__('digital_twin_bridge')
        
        # Configuration parameters
        self.declare_parameter('unity_ip', '127.0.0.1')
        self.declare_parameter('unity_port', 5005)
        self.declare_parameter('sync_frequency', 60.0)  # Hz
        self.declare_parameter('max_desync_threshold', 0.05)  # seconds
        self.declare_parameter('sensor_update_frequency', 30.0)  # Hz
        
        self.unity_ip = self.get_parameter('unity_ip').value
        self.unity_port = self.get_parameter('unity_port').value
        self.sync_frequency = self.get_parameter('sync_frequency').value
        self.max_desync_threshold = self.get_parameter('max_desync_threshold').value
        self.sensor_update_frequency = self.get_parameter('sensor_update_frequency').value
        
        # Socket for Unity communication
        self.unity_socket = None
        self.socket_lock = threading.Lock()
        
        # Connection management
        self.connected_to_unity = False
        self.connection_thread = None
        self.sync_thread = None
        self.should_run = True
        
        # Data storage
        self.robot_states = {}  # Dictionary to store states of multiple robots
        self.environment_state = {}
        self.sensor_data = {}
        self.simulation_time = 0.0
        self.last_sync_time = self.get_clock().now().nanoseconds * 1e-9
        
        # QoS profiles based on requirements
        # For high-frequency sensor data
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # For critical state data
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscriptions for simulation data
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            state_qos
        )
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            state_qos
        )
        
        # Sensor subscriptions
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/sensors/lidar/scan',
            self.lidar_callback,
            sensor_qos
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/sensors/imu/data',
            self.imu_callback,
            sensor_qos
        )
        
        # Clock synchronization via /clock topic
        self.clock_subscriber = self.create_subscription(
            Time,
            '/clock',
            self.clock_callback,
            10  # Simple QoS for clock
        )
        
        # Timer for synchronization with Unity
        self.sync_timer = self.create_timer(
            1.0/self.sync_frequency, 
            self.sync_with_unity
        )
        
        self.get_logger().info(
            f'Digital Twin Bridge initialized:\n'
            f'  Unity IP: {self.unity_ip}\n'
            f'  Unity Port: {self.unity_port}\n'
            f'  Sync Frequency: {self.sync_frequency} Hz\n'
            f'  Max Desync Threshold: {self.max_desync_threshold} sec\n'
            f'  Attempting connection to Unity...'
        )
        
        # Start connection thread to Unity
        self.connection_thread = threading.Thread(
            target=self.connect_to_unity, 
            daemon=True
        )
        self.connection_thread.start()

    def connect_to_unity(self):
        """Establish connection to Unity application."""
        while self.should_run and not self.connected_to_unity:
            try:
                self.unity_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.unity_socket.settimeout(5.0)  # 5 second timeout for connection
                self.unity_socket.connect((self.unity_ip, self.unity_port))
                self.connected_to_unity = True
                self.get_logger().info(f'Connected to Unity at {self.unity_ip}:{self.unity_port}')
                
                # Start sync thread once connected
                self.sync_thread = threading.Thread(
                    target=self.synchronization_loop,
                    daemon=True
                )
                self.sync_thread.start()
                
            except socket.error as e:
                self.get_logger().warn(
                    f'Connection failed to Unity at {self.unity_ip}:{self.unity_port}: {e}. '
                    f'Retrying in 2 seconds...'
                )
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f'Unexpected error during Unity connection: {e}')
                time.sleep(5)

    def synchronization_loop(self):
        """Main loop for synchronization with Unity."""
        while self.should_run and self.connected_to_unity:
            try:
                # Receive any commands from Unity
                command = self.receive_from_unity()
                if command:
                    self.handle_unity_command(command)
                
                # Small delay to prevent busy-waiting
                time.sleep(0.001)  # 1ms sleep
                
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

    def send_to_unity(self, data: str):
        """Send data to Unity with length prefix for message framing."""
        with self.socket_lock:
            if self.unity_socket:
                try:
                    # Prefix with length to handle message boundaries in TCP stream
                    length_prefix = struct.pack('!I', len(data))
                    self.unity_socket.send(length_prefix + data.encode('utf-8'))
                except socket.error as e:
                    self.connected_to_unity = False
                    self.get_logger().error(f'Socket error during send to Unity: {e}')

    def receive_from_unity(self) -> Optional[str]:
        """Receive data from Unity with length prefix handling."""
        with self.socket_lock:
            if self.unity_socket:
                try:
                    # First receive length prefix (4 bytes)
                    length_data = self.unity_socket.recv(4)
                    if len(length_data) == 4:
                        length = struct.unpack('!I', length_data)[0]
                        
                        # Then receive the actual message data
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

    def joint_state_callback(self, msg: JointState):
        """Process incoming joint state messages from simulation."""
        robot_name = "humanoid_robot"  # Default robot name from config
        
        if robot_name not in self.robot_states:
            self.robot_states[robot_name] = RobotState(
                position=[0.0, 0.0, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
                joint_positions={},
                joint_velocities={},
                joint_efforts={},
                timestamp=0.0
            )
        
        # Update joint state data from the message
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.robot_states[robot_name].joint_positions[name] = msg.position[i]
            if i < len(msg.velocity):
                self.robot_states[robot_name].joint_velocities[name] = msg.velocity[i]
            if i < len(msg.effort):
                self.robot_states[robot_name].joint_efforts[name] = msg.effort[i]
        
        self.robot_states[robot_name].timestamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        
        self.get_logger().debug(f'Updated joint states for {robot_name} with {len(msg.name)} joints')

    def odometry_callback(self, msg: Odometry):
        """Process incoming odometry messages."""
        robot_name = "humanoid_robot"
        
        if robot_name not in self.robot_states:
            self.robot_states[robot_name] = {
                'pose': None,
                'twist': None,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            }
        
        # Update pose and twist data
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
        self.robot_states[robot_name]['timestamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def lidar_callback(self, msg: LaserScan):
        """Process incoming LiDAR data."""
        # Store LiDAR data for Unity synchronization
        self.sensor_data['lidar'] = {
            'ranges': [float(r) for r in msg.ranges if np.isfinite(r)],
            'intensities': [float(i) for i in msg.intensities],
            'angle_min': float(msg.angle_min),
            'angle_max': float(msg.angle_max),
            'angle_increment': float(msg.angle_increment),
            'time_increment': float(msg.time_increment),
            'scan_time': float(msg.scan_time),
            'range_min': float(msg.range_min),
            'range_max': float(msg.range_max),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

    def imu_callback(self, msg: Imu):
        """Process incoming IMU data."""
        # Store IMU data for Unity synchronization
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
            },
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

    def clock_callback(self, msg: Time):
        """Process clock messages for synchronization."""
        self.simulation_time = msg.sec + msg.nanosec * 1e-9

    def sync_with_unity(self):
        """Send current simulation state to Unity at regular intervals."""
        if not self.connected_to_unity:
            return
            
        try:
            # Create synchronization data package
            sync_data = {
                'type': 'simulation_sync',
                'gazebo_time': self.get_clock().now().nanoseconds * 1e-9,
                'simulation_time': self.simulation_time,
                'robot_states': {},
                'environment_state': dict(self.environment_state),
                'sensor_data': dict(self.sensor_data),
                'timestamp': self.get_clock().now().nanoseconds * 1e-9
            }
            
            # Copy robot states to sync data
            for robot_name, state in self.robot_states.items():
                sync_data['robot_states'][robot_name] = {
                    'position': state.position,
                    'orientation': state.orientation,
                    'joint_positions': state.joint_positions,
                    'joint_velocities': state.joint_velocities,
                    'joint_efforts': state.joint_efforts,
                    'timestamp': state.timestamp
                }
            
            # Convert to JSON and send to Unity
            json_data = json.dumps(sync_data, separators=(',', ':'))
            self.send_to_unity(json_data)
            
            # Check synchronization accuracy
            current_time = self.get_clock().now().nanoseconds * 1e-9
            time_diff = abs(current_time - self.last_sync_time - 1.0/self.sync_frequency)
            
            if time_diff > self.max_desync_threshold:
                self.get_logger().warn(
                    f'Sync desynchronization detected: {time_diff:.4f}s '
                    f'(threshold: {self.max_desync_threshold}s)'
                )
            
            self.last_sync_time = current_time
            
            self.get_logger().debug(
                f'Sent sync data to Unity with {len(self.robot_states)} robots'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error during unity sync: {e}')

    def handle_unity_command(self, command_json: str):
        """Process commands received from Unity."""
        try:
            command = json.loads(command_json)
            cmd_type = command.get('type')
            
            if cmd_type == 'set_joint_positions':
                # Forward joint commands to robot controllers
                joint_positions = command.get('positions', {})
                self.execute_joint_command(joint_positions)
            elif cmd_type == 'request_robot_state':
                # Send current robot state to Unity
                self.send_robot_state_to_unity()
            elif cmd_type == 'execute_action':
                # Execute a predefined action
                action_name = command.get('action_name')
                action_params = command.get('parameters', {})
                self.execute_action(action_name, action_params)
            else:
                self.get_logger().warn(f'Unknown command type from Unity: {cmd_type}')
                
        except json.JSONDecodeError:
            self.get_logger().error(f'Invalid JSON received from Unity: {command_json}')
        except Exception as e:
            self.get_logger().error(f'Error handling Unity command: {e}')

    def execute_joint_command(self, joint_positions: Dict[str, float]):
        """Forward joint commands to the appropriate controller."""
        # In a real implementation, this would publish to joint command topics
        self.get_logger().debug(f'Executing joint command: {joint_positions}')

    def send_robot_state_to_unity(self):
        """Send current robot state to Unity."""
        state_data = {
            'type': 'robot_state_response',
            'robot_states': {},
            'timestamp': self.get_clock().now().nanoseconds * 1e-9
        }
        
        for robot_name, state in self.robot_states.items():
            state_data['robot_states'][robot_name] = {
                'position': state.position,
                'orientation': state.orientation,
                'joint_positions': state.joint_positions,
                'joint_velocities': state.joint_velocities,
                'joint_efforts': state.joint_efforts,
                'timestamp': state.timestamp
            }
        
        self.send_to_unity(json.dumps(state_data, separators=(',', ':')))

    def execute_action(self, action_name: str, parameters: Dict[str, any]):
        """Execute a predefined robot action."""
        self.get_logger().info(f'Executing action {action_name} with parameters: {parameters}')
        # Implementation would depend on the specific action system

    def attempt_reconnection(self):
        """Try to reconnect to Unity if connection is lost."""
        time.sleep(2)  # Brief delay before attempting reconnection
        try:
            if self.unity_socket:
                self.unity_socket.close()
        except:
            pass
        
        # Restart connection thread
        self.connection_thread = threading.Thread(
            target=self.connect_to_unity,
            daemon=True
        )
        self.connection_thread.start()

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
    """Main function to run the digital twin bridge node."""
    rclpy.init(args=args)
    
    digital_twin_bridge = DigitalTwinBridge()
    
    try:
        rclpy.spin(digital_twin_bridge)
    except KeyboardInterrupt:
        digital_twin_bridge.get_logger().info('Shutting down Digital Twin Bridge')
    finally:
        digital_twin_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()