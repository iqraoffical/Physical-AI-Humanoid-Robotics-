#!/usr/bin/env python3

"""
Metrics collection module for the ROS 2 nervous system observability.
Implements custom metrics collection for performance monitoring (from clarifications).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import json
import time
from datetime import datetime
from typing import Dict, List, Any, Optional, Callable
import threading
import psutil  # For system metrics
import os


class MetricsCollector(Node):
    """
    Custom metrics collection system for performance monitoring in the ROS 2 nervous system.
    Collects and aggregates various types of metrics from the robotic system (from clarifications).
    """
    
    def __init__(self, node_name: str = 'metrics_collector'):
        super().__init__(node_name)
        
        # Publisher for metrics data
        self.metrics_publisher = self.create_publisher(
            String,
            '/system_metrics',
            QoSProfile(depth=100)
        )
        
        # Subscriber for joint state metrics
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            QoSProfile(depth=10)
        )
        
        # Storage for metrics
        self.metrics_data: Dict[str, List[Dict[str, Any]]] = {}
        self.metrics_lock = threading.Lock()
        
        # Store node-specific metrics
        self.node_metrics: Dict[str, Dict[str, Any]] = {}
        
        # Performance counters
        self.message_counts = {}
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        
        # Timer for periodic metrics collection
        self.metrics_timer = self.create_timer(1.0, self.publish_system_metrics)  # Every 1 second
        
        self.get_logger().info('Metrics collector initialized')
    
    def record_metric(self, name: str, value: Any, unit: str = "", 
                     tags: Optional[Dict[str, str]] = None,
                     node_name: str = ""):
        """
        Record a metric value with optional tags and node association.
        
        Args:
            name: Name of the metric
            value: Value of the metric
            unit: Unit of measurement
            tags: Additional tags for categorization
            node_name: Name of the node recording this metric
        """
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        
        metric_entry = {
            "name": name,
            "value": value,
            "unit": unit,
            "timestamp": timestamp,
            "node": node_name or self.get_name(),
            "tags": tags or {}
        }
        
        with self.metrics_lock:
            if name not in self.metrics_data:
                self.metrics_data[name] = []
            
            self.metrics_data[name].append(metric_entry)
            
            # Limit history to prevent memory issues
            if len(self.metrics_data[name]) > 1000:
                self.metrics_data[name] = self.metrics_data[name][-1000:]
        
        # Publish the metric immediately
        self.publish_metric(metric_entry)
    
    def publish_metric(self, metric_entry: Dict[str, Any]):
        """Publish a single metric to the ROS 2 network."""
        metrics_msg = String()
        metrics_msg.data = json.dumps(metric_entry)
        self.metrics_publisher.publish(metrics_msg)
    
    def publish_system_metrics(self):
        """Publish system-level metrics periodically."""
        # Collect system metrics
        self.collect_cpu_metrics()
        self.collect_memory_metrics()
        self.collect_network_metrics()
        self.collect_uptime_metrics()
    
    def collect_cpu_metrics(self):
        """Collect CPU utilization metrics."""
        cpu_percent = psutil.cpu_percent(interval=None)
        cpu_count = psutil.cpu_count()
        
        self.record_metric(
            name="cpu.utilization",
            value=cpu_percent,
            unit="percent",
            tags={"type": "system"}
        )
        
        self.record_metric(
            name="cpu.count",
            value=cpu_count,
            unit="count",
            tags={"type": "system"}
        )
    
    def collect_memory_metrics(self):
        """Collect memory utilization metrics."""
        memory = psutil.virtual_memory()
        
        self.record_metric(
            name="memory.utilization",
            value=memory.percent,
            unit="percent",
            tags={"type": "system", "memory_type": "virtual"}
        )
        
        self.record_metric(
            name="memory.available",
            value=memory.available / (1024**2),  # Convert to MB
            unit="MB",
            tags={"type": "system", "memory_type": "available"}
        )
        
        self.record_metric(
            name="memory.total",
            value=memory.total / (1024**2),  # Convert to MB
            unit="MB",
            tags={"type": "system", "memory_type": "total"}
        )
    
    def collect_network_metrics(self):
        """Collect network metrics."""
        net_io = psutil.net_io_counters()
        
        self.record_metric(
            name="network.bytes_sent",
            value=net_io.bytes_sent,
            unit="bytes",
            tags={"type": "system", "direction": "out"}
        )
        
        self.record_metric(
            name="network.bytes_recv",
            value=net_io.bytes_recv,
            unit="bytes",
            tags={"type": "system", "direction": "in"}
        )
    
    def collect_uptime_metrics(self):
        """Collect system uptime metrics."""
        current_time = self.get_clock().now().nanoseconds * 1e-9
        uptime = current_time - self.start_time
        
        self.record_metric(
            name="system.uptime",
            value=uptime,
            unit="seconds",
            tags={"type": "system"}
        )
    
    def joint_state_callback(self, msg: JointState):
        """Handle incoming joint state messages and extract metrics."""
        # Count the number of joints
        self.record_metric(
            name="robot.joint_count",
            value=len(msg.name),
            unit="count",
            tags={"type": "robot"}
        )
        
        # Record joint position metrics
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position):
                self.record_metric(
                    name=f"joint.{joint_name}.position",
                    value=msg.position[i],
                    unit="radians",
                    tags={"type": "joint", "joint_name": joint_name}
                )
        
        # Record message rate metrics
        topic_name = '/joint_states'
        if topic_name not in self.message_counts:
            self.message_counts[topic_name] = {
                'count': 0,
                'last_reset': time.time()
            }
        
        self.message_counts[topic_name]['count'] += 1
        
        # Calculate and publish message rate every 5 seconds
        current_time = time.time()
        if current_time - self.message_counts[topic_name]['last_reset'] >= 5.0:
            elapsed_time = current_time - self.message_counts[topic_name]['last_reset']
            rate = self.message_counts[topic_name]['count'] / elapsed_time
            
            self.record_metric(
                name="topic.joint_states.rate",
                value=rate,
                unit="Hz",
                tags={"type": "topic", "topic_name": topic_name}
            )
            
            # Reset counter
            self.message_counts[topic_name]['count'] = 0
            self.message_counts[topic_name]['last_reset'] = current_time
    
    def get_metrics_by_name(self, name: str) -> List[Dict[str, Any]]:
        """Get all recorded values for a specific metric name."""
        with self.metrics_lock:
            return self.metrics_data.get(name, [])
    
    def get_all_metrics(self) -> Dict[str, List[Dict[str, Any]]]:
        """Get all recorded metrics."""
        with self.metrics_lock:
            return self.metrics_data.copy()
    
    def calculate_rate(self, metric_name: str, time_window: int = 60) -> float:
        """
        Calculate the rate of a counter metric over a specified time window.
        
        Args:
            metric_name: Name of the metric to calculate rate for
            time_window: Time window in seconds
            
        Returns:
            Rate of the metric per second
        """
        current_time = self.get_clock().now().nanoseconds * 1e-9
        cutoff_time = current_time - time_window
        
        with self.metrics_lock:
            metric_history = self.metrics_data.get(metric_name, [])
        
        # Filter metrics within the time window
        recent_metrics = [
            m for m in metric_history
            if m["timestamp"] >= cutoff_time
        ]
        
        if len(recent_metrics) < 2:
            return 0.0
        
        # Calculate rate (change in value over time)
        first_metric = recent_metrics[0]
        last_metric = recent_metrics[-1]
        
        time_diff = last_metric["timestamp"] - first_metric["timestamp"]
        value_diff = last_metric["value"] - first_metric["value"]
        
        if time_diff > 0:
            return value_diff / time_diff
        else:
            return 0.0
    
    def calculate_average(self, metric_name: str, time_window: int = 300) -> float:
        """
        Calculate the average value of a metric over a specified time window.
        
        Args:
            metric_name: Name of the metric to calculate average for
            time_window: Time window in seconds
            
        Returns:
            Average value of the metric
        """
        current_time = self.get_clock().now().nanoseconds * 1e-9
        cutoff_time = current_time - time_window
        
        with self.metrics_lock:
            metric_history = self.metrics_data.get(metric_name, [])
        
        # Filter metrics within the time window
        recent_metrics = [
            m for m in metric_history
            if m["timestamp"] >= cutoff_time
        ]
        
        if not recent_metrics:
            return 0.0
        
        # Calculate average
        total = sum(m["value"] for m in recent_metrics)
        return total / len(recent_metrics)
    
    def register_custom_collector(self, name: str, collector_func: Callable[[], Any], 
                                 interval: float = 10.0):
        """
        Register a custom metrics collector function that runs at a specified interval.
        
        Args:
            name: Name of the custom collector
            collector_func: Function that returns a metric value
            interval: Interval in seconds to run the collector
        """
        def wrapper():
            try:
                value = collector_func()
                self.record_metric(name, value, tags={"type": "custom"})
            except Exception as e:
                self.get_logger().error(f"Error in custom collector {name}: {e}")
        
        # Create a timer for the custom collector
        self.create_timer(interval, wrapper)


# Global metrics collector instance
_metrics_collector = None


def get_metrics_collector(node: Node = None) -> MetricsCollector:
    """Get the global metrics collector instance."""
    global _metrics_collector
    if _metrics_collector is None:
        _metrics_collector = MetricsCollector()
    return _metrics_collector


def record_metric(name: str, value: Any, unit: str = "", 
                 tags: Optional[Dict[str, str]] = None,
                 node_name: str = "") -> None:
    """Record a metric using the global metrics collector."""
    collector = get_metrics_collector()
    collector.record_metric(name, value, unit, tags, node_name)


def calculate_rate(metric_name: str, time_window: int = 60) -> float:
    """Calculate the rate of a metric using the global metrics collector."""
    collector = get_metrics_collector()
    return collector.calculate_rate(metric_name, time_window)


def calculate_average(metric_name: str, time_window: int = 300) -> float:
    """Calculate the average of a metric using the global metrics collector."""
    collector = get_metrics_collector()
    return collector.calculate_average(metric_name, time_window)


if __name__ == "__main__":
    # Example usage
    rclpy.init()
    
    metrics_node = MetricsCollector("test_metrics")
    
    # Record some example metrics
    import random
    for i in range(10):
        metrics_node.record_metric(
            name="test.metric",
            value=random.uniform(0, 100),
            unit="units",
            tags={"type": "test", "iteration": str(i)}
        )
        time.sleep(0.1)  # Small delay
    
    # Get metrics
    test_metrics = metrics_node.get_metrics_by_name("test.metric")
    print(f"Recorded {len(test_metrics)} test metrics")
    
    # Calculate rate and average
    rate = metrics_node.calculate_rate("test.metric", 30)
    avg = metrics_node.calculate_average("test.metric", 30)
    print(f"Rate: {rate}, Average: {avg}")
    
    metrics_node.destroy_node()
    rclpy.shutdown()