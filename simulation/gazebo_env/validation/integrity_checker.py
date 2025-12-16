#!/usr/bin/env python3

"""
Simulation Integrity Checker for Digital Twin
Validates real-time performance and synchronization between Gazebo and Unity
Implements SC-004: Simulation maintains real-time performance suitable for physical deployment validation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
import time
import statistics
from typing import Dict, List, Tuple
import numpy as np
from collections import deque
import threading
import psutil  # System performance monitoring


class SimulationIntegrityChecker(Node):
    """
    System that validates simulation integrity in terms of real-time performance and synchronization.
    Implements SC-004: Simulation maintains real-time performance suitable for physical deployment validation.
    """
    
    def __init__(self):
        super().__init__('simulation_integrity_checker')
        
        # Configuration parameters
        self.declare_parameter('monitoring_duration', 60.0)      # seconds
        self.declare_parameter('real_time_threshold', 0.95)      # 95% real-time factor
        self.declare_parameter('sync_threshold', 0.05)           # 50ms max desync
        self.declare_parameter('monitoring_rate', 10.0)          # Hz
        
        self.monitoring_duration = self.get_parameter('monitoring_duration').value
        self.real_time_threshold = self.get_parameter('real_time_threshold').value
        self.sync_threshold = self.get_parameter('sync_threshold').value
        self.monitoring_rate = self.get_parameter('monitoring_rate').value
        
        # Data collection
        self.simulation_times = deque(maxlen=1000)  # Keep last 1000 timestamps
        self.wall_clock_times = deque(maxlen=1000)  # Keep last 1000 timestamps
        self.performance_metrics = {
            'avg_real_time_factor': 0.0,
            'min_real_time_factor': float('inf'),
            'max_real_time_factor': 0.0,
            'std_real_time_factor': 0.0,
            'avg_sync_error': 0.0,
            'max_sync_error': 0.0,
            'cpu_usage': [],
            'ram_usage': [],
            'gpu_usage': []
        }
        
        # Synchronization validation
        self.unity_sync_errors = deque(maxlen=1000)
        self.ros_time_drift = 0.0
        self.last_time_check = None
        
        # Monitoring flags
        self.monitoring_active = False
        self.monitoring_start_time = None
        self.monitoring_data_lock = threading.Lock()
        
        # QoS profiles
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Subscriptions for monitoring
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10  # Simple QoS for clock
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            state_qos
        )
        
        # Additional performance metrics publishers
        self.performance_pub = self.create_publisher(
            Float64MultiArray,
            '/simulation/performance_metrics',
            10
        )
        
        # Service for starting integrity validation
        self.validate_integrity_srv = self.create_service(
            Trigger,
            '/validate_simulation_integrity',
            self.validate_integrity_callback
        )
        
        # Timer for continuous monitoring
        self.monitor_timer = self.create_timer(
            1.0/self.monitoring_rate,
            self.monitor_performance
        )
        
        self.get_logger().info(
            f'Simulation Integrity Checker initialized:\n'
            f'  Monitoring Duration: {self.monitoring_duration}s\n'
            f'  Real-time Threshold: {self.real_time_threshold} ({self.real_time_threshold*100}%)\n'
            f'  Sync Threshold: {self.sync_threshold}s ({self.sync_threshold*1000}ms)\n'
            f'  Monitoring Rate: {self.monitoring_rate} Hz'
        )

    def clock_callback(self, msg: Clock):
        """Capture simulation time and wall clock time for RTF calculation."""
        sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
        wall_time = time.time()
        
        with self.data_lock:
            self.simulation_times.append(sim_time)
            self.wall_clock_times.append(wall_time)
            
            # Calculate real-time factor based on recent data
            if len(self.simulation_times) >= 2:
                sim_dt = self.simulation_times[-1] - self.simulation_times[0]
                wall_dt = self.wall_clock_times[-1] - self.wall_clock_times[0]
                
                if wall_dt > 0:
                    current_rtf = sim_dt / wall_dt
                    
                    # Update performance metrics
                    self.performance_metrics['avg_real_time_factor'] = (
                        len(self.simulation_times) * self.performance_metrics['avg_real_time_factor'] + current_rtf
                    ) / (len(self.simulation_times) + 1)
                    
                    if current_rtf < self.performance_metrics['min_real_time_factor']:
                        self.performance_metrics['min_real_time_factor'] = current_rtf
                    
                    if current_rtf > self.performance_metrics['max_real_time_factor']:
                        self.performance_metrics['max_real_time_factor'] = current_rtf

    def joint_state_callback(self, msg: JointState):
        """Monitor joint state messages for timing consistency."""
        with self.data_lock:
            # Record timing information for synchronization validation
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            wall_time = time.time()
            
            # Calculate sync error (difference between message timestamp and wall time)
            sync_error = abs(msg_time - wall_time)
            self.unity_sync_errors.append(sync_error)
            
            # Track drift over time
            if self.last_time_check is not None:
                expected_dt = msg_time - self.last_time_check[0]
                actual_dt = wall_time - self.last_time_check[1]
                drift = abs(expected_dt - actual_dt)
                self.ros_time_drift += drift
            
            self.last_time_check = (msg_time, wall_time)

    def monitor_performance(self):
        """Monitor system performance continuously."""
        # Collect system performance metrics
        with self.data_lock:
            self.performance_metrics['cpu_usage'].append(psutil.cpu_percent())
            self.performance_metrics['ram_usage'].append(psutil.virtual_memory().percent)
            
            # Note: GPU usage would require additional libraries like GPUtil or pyNVML
            # For now, we'll add a placeholder value
            self.performance_metrics['gpu_usage'].append(0.0)  # Placeholder
        
        # Keep usage data lists at reasonable size
        max_performance_samples = 1000
        if len(self.performance_metrics['cpu_usage']) > max_performance_samples:
            self.performance_metrics['cpu_usage'] = self.performance_metrics['cpu_usage'][-max_performance_samples:]
            self.performance_metrics['ram_usage'] = self.performance_metrics['ram_usage'][-max_performance_samples:]
            self.performance_metrics['gpu_usage'] = self.performance_metrics['gpu_usage'][-max_performance_samples:]

    def validate_integrity_callback(self, request, response):
        """Handle validation service request."""
        self.get_logger().info('Starting simulation integrity validation...')
        
        # Start monitoring if not active
        if not self.monitoring_active:
            self.start_monitoring()
        
        # Wait for monitoring duration
        start_time = time.time()
        while time.time() - start_time < self.monitoring_duration:
            time.sleep(0.1)
        
        # Perform validation analysis
        validation_results = self.analyze_integrity()
        
        # Compile response
        all_passed = all(result['pass'] for result in validation_results.values())
        
        response.success = all_passed
        response.message = f'Simulation integrity validation completed. All passed: {all_passed}. Results: {validation_results}'
        
        # Log detailed results
        for check_name, result in validation_results.items():
            status = "PASS" if result['pass'] else "FAIL"
            self.get_logger().info(f'{check_name} validation: {status} - {result["details"]}')
        
        if all_passed:
            self.get_logger().info('Simulation integrity validation PASSED - All checks meet real-time requirements')
        else:
            self.get_logger().error('Simulation integrity validation FAILED - Some checks do not meet real-time requirements')
        
        return response

    def start_monitoring(self):
        """Start the monitoring process."""
        with self.monitoring_lock:
            self.monitoring_active = True
            self.monitoring_start_time = time.time()
            self.get_logger().info('Simulation integrity monitoring started')

    def stop_monitoring(self):
        """Stop the monitoring process."""
        with self.monitoring_lock:
            self.monitoring_active = False
            self.get_logger().info('Simulation integrity monitoring stopped')

    def analyze_integrity(self) -> Dict[str, dict]:
        """
        Analyze collected monitoring data to validate simulation integrity.
        Verifies real-time performance metrics meet success criteria.
        """
        self.get_logger().info('Analyzing simulation integrity...')
        
        results = {}
        
        # Get current performance data
        with self.data_lock:
            # Calculate real-time performance metrics
            if len(self.simulation_times) > 10:  # Need sufficient data points
                sim_times = list(self.simulation_times)
                wall_times = list(self.wall_clock_times)
                
                # Calculate real-time factors for each interval
                rt_factors = []
                for i in range(1, len(sim_times)):
                    sim_dt = sim_times[i] - sim_times[i-1]
                    wall_dt = wall_times[i] - wall_times[i-1]
                    
                    if wall_dt > 0:
                        rtf = sim_dt / wall_dt
                        rt_factors.append(rtf)
                
                if rt_factors:
                    avg_rtf = statistics.mean(rt_factors)
                    min_rtf = min(rt_factors)
                    max_rtf = max(rt_factors)
                    std_rtf = statistics.stdev(rt_factors) if len(rt_factors) > 1 else 0.0
                    
                    # Validate real-time factor requirements
                    rtf_pass = avg_rtf >= self.real_time_threshold
                    results['real_time_performance'] = {
                        'pass': rtf_pass,
                        'details': f'Avg RTF: {avg_rtf:.3f}, Min: {min_rtf:.3f}, Max: {max_rtf:.3f}, StdDev: {std_rtf:.3f}',
                        'metrics': {
                            'avg_rtf': avg_rtf,
                            'min_rtf': min_rtf,
                            'max_rtf': max_rtf,
                            'std_rtf': std_rtf
                        }
                    }
                    
                    if not rtf_pass:
                        self.get_logger().error(
                            f'Real-time performance failed: {avg_rtf:.3f} < {self.real_time_threshold:.3f} required'
                        )
                    else:
                        self.get_logger().info(
                            f'Real-time performance OK: {avg_rtf:.3f} >= {self.real_time_threshold:.3f} required'
                        )
                else:
                    results['real_time_performance'] = {
                        'pass': False,
                        'details': 'Insufficient data for RTF calculation',
                        'metrics': {}
                    }
            else:
                results['real_time_performance'] = {
                    'pass': False,
                    'details': f'Insufficient data points: {len(self.simulation_times)} < 10 required',
                    'metrics': {}
                }
            
            # Validate synchronization accuracy
            if self.unity_sync_errors:
                avg_sync_error = sum(self.unity_sync_errors) / len(self.unity_sync_errors)
                max_sync_error = max(self.unity_sync_errors)
                
                sync_pass = avg_sync_error <= self.sync_threshold
                results['synchronization'] = {
                    'pass': sync_pass,
                    'details': f'Avg sync error: {avg_sync_error:.4f}s, Max: {max_sync_error:.4f}s, Threshold: {self.sync_threshold:.4f}s',
                    'metrics': {
                        'avg_sync_error': avg_sync_error,
                        'max_sync_error': max_sync_error,
                        'sync_threshold': self.sync_threshold
                    }
                }
                
                if not sync_pass:
                    self.get_logger().error(
                        f'Synchronization failed: {avg_sync_error:.4f}s > {self.sync_threshold:.4f}s threshold'
                    )
                else:
                    self.get_logger().info(
                        f'Synchronization OK: {avg_sync_error:.4f}s <= {self.sync_threshold:.4f}s threshold'
                    )
            else:
                results['synchronization'] = {
                    'pass': False,
                    'details': 'No synchronization data available',
                    'metrics': {}
                }
            
            # Validate system resource usage
            if self.performance_metrics['cpu_usage']:
                avg_cpu = sum(self.performance_metrics['cpu_usage']) / len(self.performance_metrics['cpu_usage'])
                max_cpu = max(self.performance_metrics['cpu_usage'])
                
                # Consider performance acceptable if CPU usage under 90% (leaves headroom for simulation)
                cpu_pass = max_cpu < 90.0
                results['resource_utilization'] = {
                    'pass': cpu_pass,
                    'details': f'Avg CPU: {avg_cpu:.2f}%, Max CPU: {max_cpu:.2f}% (target < 90%)',
                    'metrics': {
                        'avg_cpu': avg_cpu,
                        'max_cpu': max_cpu,
                        'avg_ram': sum(self.performance_metrics['ram_usage']) / len(self.performance_metrics['ram_usage']) if self.performance_metrics['ram_usage'] else 0.0
                    }
                }
                
                if not cpu_pass:
                    self.get_logger().warn(f'High CPU usage detected: {max_cpu:.2f}%')
                else:
                    self.get_logger().info(f'CPU usage OK: {max_cpu:.2f}% < 90% threshold')
            else:
                results['resource_utilization'] = {
                    'pass': False,
                    'details': 'No resource usage data available',
                    'metrics': {}
                }
        
        # Calculate overall result
        total_checks = len(results)
        passed_checks = sum(1 for result in results.values() if result['pass'])
        
        results['overall'] = {
            'pass': passed_checks == total_checks,
            'details': f'{passed_checks}/{total_checks} checks passed',
            'metrics': {
                'total_checks': total_checks,
                'passed_checks': passed_checks,
                'failed_checks': total_checks - passed_checks
            }
        }
        
        return results

    def get_integrity_report(self) -> str:
        """
        Generate a comprehensive simulation integrity report.
        """
        report_lines = []
        report_lines.append("Simulation Integrity Validation Report")
        report_lines.append("=" * 40)
        report_lines.append(f"Monitoring Duration: {self.monitoring_duration}s")
        report_lines.append(f"Real-time Threshold: {self.real_time_threshold} ({self.real_time_threshold*100}%)")
        report_lines.append(f"Sync Threshold: {self.sync_threshold}s ({self.sync_threshold*1000}ms)")
        report_lines.append(f"Monitoring Rate: {self.monitoring_rate} Hz")
        report_lines.append("")
        
        # Add data collection summary
        with self.data_lock:
            report_lines.append("Data Collection Summary:")
            report_lines.append("-" * 25)
            report_lines.append(f"  Simulation time samples: {len(self.simulation_times)}")
            report_lines.append(f"  Wall clock samples: {len(self.wall_clock_times)}")
            report_lines.append(f"  Sync error samples: {len(self.unity_sync_errors)}")
            report_lines.append(f"  Performance samples: {len(self.performance_metrics['cpu_usage'])}")
            report_lines.append("")
        
        # Add validation results
        results = self.analyze_integrity()
        
        report_lines.append("Validation Results:")
        report_lines.append("-" * 18)
        
        for check_name, result in results.items():
            if check_name != 'overall':
                status = "PASS" if result['pass'] else "FAIL"
                report_lines.append(f"  {check_name.replace('_', ' ').title()}: {status}")
                report_lines.append(f"    Details: {result['details']}")
                report_lines.append("")
        
        # Add overall result
        overall_result = results.get('overall', {})
        status = "PASS" if overall_result.get('pass', False) else "FAIL"
        report_lines.append(f"Overall Integrity: {status}")
        report_lines.append(f"  Summary: {overall_result.get('details', '')}")
        report_lines.append("")
        
        # Add recommendations
        report_lines.append("Recommendations:")
        report_lines.append("-" * 15)
        
        if not all(result['pass'] for name, result in results.items() if name != 'overall'):
            report_lines.append("  - Investigate performance bottlenecks causing real-time factor degradation")
            report_lines.append("  - Check GPU/CPU utilization and resource allocation")
            report_lines.append("  - Verify system meets minimum hardware requirements")
        else:
            report_lines.append("  - Simulation maintains real-time performance requirements")
            report_lines.append("  - Ready for physical deployment validation")
        
        return "\n".join(report_lines)


def main(args=None):
    """Main function to run the simulation integrity checker."""
    rclpy.init(args=args)
    
    integrity_checker = SimulationIntegrityChecker()
    
    try:
        rclpy.spin(integrity_checker)
    except KeyboardInterrupt:
        integrity_checker.get_logger().info('Shutting down Simulation Integrity Checker')
    finally:
        integrity_checker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()