#!/usr/bin/env python3

"""
Logging module for the ROS 2 nervous system observability.
Implements comprehensive logging using ROS 2's built-in logging infrastructure (from clarifications).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from typing import Any, Dict, Optional
import json
import time
from datetime import datetime
import traceback
import sys


class LoggingModule(Node):
    """
    Comprehensive logging module that follows ROS 2's built-in logging infrastructure.
    Provides logging, metrics collection, and tracing capabilities (from clarifications).
    """
    
    def __init__(self, node_name: str = 'logging_module'):
        super().__init__(node_name)
        
        # Publisher for log messages (for external monitoring)
        self.log_publisher = self.create_publisher(
            String,
            '/system_logs',
            QoSProfile(depth=100)  # Keep last 100 log messages
        )
        
        # Initialize logging configuration
        self.node_name = node_name
        self.log_buffer = []
        self.max_buffer_size = 1000
        
        # Metrics storage
        self.metrics = {}
        
        self.get_logger().info('Logging module initialized')
    
    def log_debug(self, message: str, component: str = "general"):
        """Log a debug message."""
        self._log_message("DEBUG", message, component)
    
    def log_info(self, message: str, component: str = "general"):
        """Log an info message."""
        self._log_message("INFO", message, component)
    
    def log_warn(self, message: str, component: str = "general"):
        """Log a warning message."""
        self._log_message("WARN", message, component)
    
    def log_error(self, message: str, component: str = "general"):
        """Log an error message."""
        self._log_message("ERROR", message, component)
    
    def log_critical(self, message: str, component: str = "general"):
        """Log a critical message."""
        self._log_message("CRITICAL", message, component)
    
    def _log_message(self, level: str, message: str, component: str):
        """Internal method to create and publish log messages."""
        # Create timestamp
        timestamp = datetime.fromtimestamp(self.get_clock().now().nanoseconds * 1e-9).isoformat()
        
        # Create log entry
        log_entry = {
            "timestamp": timestamp,
            "level": level,
            "node": self.node_name,
            "component": component,
            "message": message,
            "source_info": self._get_caller_info()
        }
        
        # Add to internal buffer
        self.log_buffer.append(log_entry)
        if len(self.log_buffer) > self.max_buffer_size:
            self.log_buffer.pop(0)  # Remove oldest entry
        
        # Publish to ROS 2 topic
        log_msg = String()
        log_msg.data = json.dumps(log_entry)
        self.log_publisher.publish(log_msg)
        
        # Also log through ROS 2's standard logger
        if level == "DEBUG":
            self.get_logger().debug(f"[{component}] {message}")
        elif level == "INFO":
            self.get_logger().info(f"[{component}] {message}")
        elif level == "WARN":
            self.get_logger().warn(f"[{component}] {message}")
        elif level == "ERROR":
            self.get_logger().error(f"[{component}] {message}")
        elif level == "CRITICAL":
            self.get_logger().fatal(f"[{component}] {message}")
    
    def _get_caller_info(self) -> Dict[str, str]:
        """Get information about the caller function."""
        frame = sys._getframe(3)  # Go back 3 frames to get the actual caller
        return {
            "filename": frame.f_code.co_filename,
            "function": frame.f_code.co_name,
            "line": frame.f_lineno
        }
    
    def record_metric(self, name: str, value: Any, unit: str = "", tags: Optional[Dict[str, str]] = None):
        """Record a metric value."""
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        
        metric_entry = {
            "name": name,
            "value": value,
            "unit": unit,
            "timestamp": timestamp,
            "tags": tags or {},
            "node": self.node_name
        }
        
        # Store in metrics dictionary (keyed by name)
        if name not in self.metrics:
            self.metrics[name] = []
        
        self.metrics[name].append(metric_entry)
        
        # Keep only the last 1000 metric entries per name to prevent memory issues
        if len(self.metrics[name]) > 1000:
            self.metrics[name] = self.metrics[name][-1000:]
    
    def get_metrics(self, name: str = None) -> Dict[str, Any]:
        """Get recorded metrics."""
        if name:
            return self.metrics.get(name, [])
        return self.metrics
    
    def get_log_entries(self, component: str = None, level: str = None, limit: int = 50) -> list:
        """Get log entries, optionally filtered by component or level."""
        filtered_logs = self.log_buffer
        
        if component:
            filtered_logs = [log for log in filtered_logs if log["component"] == component]
        
        if level:
            filtered_logs = [log for log in filtered_logs if log["level"] == level]
        
        # Return the last 'limit' entries
        return filtered_logs[-limit:]
    
    def log_exception(self, e: Exception, context: str = ""):
        """Log an exception with its traceback."""
        tb_str = traceback.format_exception(type(e), e, e.__traceback__)
        tb_formatted = "".join(tb_str)
        
        message = f"Exception in {context}: {str(e)}\nTraceback:\n{tb_formatted}"
        self.log_error(message, "exception")
    
    def start_timer(self, name: str) -> None:
        """Start a timer for performance measurement."""
        setattr(self, f"timer_{name}", time.time())
    
    def stop_timer(self, name: str, message: str = "Operation took") -> float:
        """Stop a timer and log the elapsed time."""
        timer_attr = f"timer_{name}"
        if hasattr(self, timer_attr):
            elapsed = time.time() - getattr(self, timer_attr)
            delattr(self, timer_attr)
            self.log_info(f"{message}: {elapsed:.4f}s", "timer")
            self.record_metric(f"{name}_duration", elapsed, "seconds")
            return elapsed
        else:
            self.log_warn(f"Timer {name} not found", "timer")
            return 0.0


# Global logging module instance
_logging_module = None


def get_logging_module(node: Node = None) -> LoggingModule:
    """Get the global logging module instance."""
    global _logging_module
    if _logging_module is None:
        # If we're called from within a node, we need to create a separate logging node
        # For this implementation, we'll handle it when needed
        _logging_module = LoggingModule()
    return _logging_module


def log_debug(message: str, component: str = "general") -> None:
    """Log a debug message using the global logging module."""
    logger = get_logging_module()
    logger.log_debug(message, component)


def log_info(message: str, component: str = "general") -> None:
    """Log an info message using the global logging module."""
    logger = get_logging_module()
    logger.log_info(message, component)


def log_warn(message: str, component: str = "general") -> None:
    """Log a warning message using the global logging module."""
    logger = get_logging_module()
    logger.log_warn(message, component)


def log_error(message: str, component: str = "general") -> None:
    """Log an error message using the global logging module."""
    logger = get_logging_module()
    logger.log_error(message, component)


def log_critical(message: str, component: str = "general") -> None:
    """Log a critical message using the global logging module."""
    logger = get_logging_module()
    logger.log_critical(message, component)


def record_metric(name: str, value: Any, unit: str = "", tags: Optional[Dict[str, str]] = None) -> None:
    """Record a metric using the global logging module."""
    logger = get_logging_module()
    logger.record_metric(name, value, unit, tags)


def log_exception(e: Exception, context: str = "") -> None:
    """Log an exception using the global logging module."""
    logger = get_logging_module()
    logger.log_exception(e, context)


def start_timer(name: str) -> None:
    """Start a timer using the global logging module."""
    logger = get_logging_module()
    logger.start_timer(name)


def stop_timer(name: str, message: str = "Operation took") -> float:
    """Stop a timer using the global logging module."""
    logger = get_logging_module()
    return logger.stop_timer(name, message)


if __name__ == "__main__":
    # Example usage
    rclpy.init()
    
    logger_node = LoggingModule("test_logger")
    
    # Log some messages
    logger_node.log_info("This is an info message", "test_component")
    logger_node.log_warn("This is a warning message", "test_component")
    logger_node.log_error("This is an error message", "test_component")
    
    # Record some metrics
    logger_node.record_metric("cpu_usage", 45.2, "percent", {"host": "robot1"})
    logger_node.record_metric("memory_usage", 1024.5, "MB", {"host": "robot1"})
    
    # Show last 5 log entries
    recent_logs = logger_node.get_log_entries(limit=5)
    print(f"Recent log entries: {len(recent_logs)}")
    for log in recent_logs:
        print(f"  {log['timestamp']} - {log['level']} - {log['message']}")
    
    # Show metrics
    all_metrics = logger_node.get_metrics()
    print(f"Recorded metrics: {list(all_metrics.keys())}")
    
    logger_node.destroy_node()
    rclpy.shutdown()