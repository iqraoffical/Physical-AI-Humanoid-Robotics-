#!/usr/bin/env python3

"""
Tracing system for the ROS 2 nervous system observability.
Implements tracing capabilities to track requests across nodes (from clarifications).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from typing import Dict, List, Optional, Any, Callable
import uuid
import time
import json
from datetime import datetime
import threading
from contextlib import contextmanager
import traceback


class TraceSpan:
    """
    Represents a single span in a trace, tracking a unit of work or operation.
    """
    
    def __init__(self, trace_id: str, span_id: str, name: str, parent_span_id: Optional[str] = None):
        self.trace_id = trace_id
        self.span_id = span_id
        self.name = name
        self.parent_span_id = parent_span_id
        self.start_time = time.time()
        self.end_time = None
        self.attributes = {}
        self.events = []
        self.status = "OK"
    
    def set_attribute(self, key: str, value: Any):
        """Set an attribute on this span."""
        self.attributes[key] = value
    
    def add_event(self, name: str, timestamp: Optional[float] = None, attributes: Optional[Dict[str, Any]] = None):
        """Add an event to this span."""
        if timestamp is None:
            timestamp = time.time()
        
        event = {
            "name": name,
            "timestamp": timestamp,
            "attributes": attributes or {}
        }
        self.events.append(event)
    
    def end(self, status: str = "OK"):
        """Mark the span as ended."""
        self.end_time = time.time()
        self.status = status
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert the span to a dictionary for serialization."""
        return {
            "trace_id": self.trace_id,
            "span_id": self.span_id,
            "name": self.name,
            "parent_span_id": self.parent_span_id,
            "start_time": self.start_time,
            "end_time": self.end_time,
            "attributes": self.attributes,
            "events": self.events,
            "status": self.status
        }


class TracingSystem(Node):
    """
    Tracing system to track requests across nodes in the ROS 2 nervous system.
    Provides tracing capabilities to track requests across nodes (from clarifications).
    """
    
    def __init__(self, node_name: str = 'tracing_system'):
        super().__init__(node_name)
        
        # Publisher for trace data
        self.trace_publisher = self.create_publisher(
            String,
            '/system_traces',
            QoSProfile(depth=1000)  # Higher depth for trace data
        )
        
        # Storage for active traces and spans
        self.active_traces: Dict[str, List[TraceSpan]] = {}
        self.active_spans: Dict[str, TraceSpan] = {}
        self.trace_lock = threading.Lock()
        
        # Store trace context (current trace/span for this node)
        self.trace_context: Dict[str, str] = {}
        
        # Performance tracking
        self.span_counts = {}
        
        self.get_logger().info('Tracing system initialized')
    
    def start_trace(self, name: str, parent_trace_id: Optional[str] = None) -> str:
        """
        Start a new trace.
        
        Args:
            name: Name of the trace
            parent_trace_id: ID of parent trace if this is a child trace
            
        Returns:
            Trace ID
        """
        trace_id = str(uuid.uuid4())
        
        with self.trace_lock:
            if trace_id not in self.active_traces:
                self.active_traces[trace_id] = []
        
        # Create root span for the trace
        root_span = self.start_span(name, trace_id)
        
        return trace_id
    
    def start_span(self, name: str, trace_id: str, parent_span_id: Optional[str] = None) -> TraceSpan:
        """
        Start a new span within a trace.
        
        Args:
            name: Name of the span
            trace_id: ID of the trace this span belongs to
            parent_span_id: ID of the parent span (if any)
            
        Returns:
            TraceSpan object
        """
        span_id = str(uuid.uuid4())
        
        span = TraceSpan(trace_id, span_id, name, parent_span_id)
        
        with self.trace_lock:
            self.active_spans[span_id] = span
            
            if trace_id not in self.active_traces:
                self.active_traces[trace_id] = []
            self.active_traces[trace_id].append(span)
        
        # Track span creation for performance metrics
        if name not in self.span_counts:
            self.span_counts[name] = 0
        self.span_counts[name] += 1
        
        return span
    
    def end_span(self, span: TraceSpan, status: str = "OK"):
        """End a span and publish its data."""
        span.end(status)
        
        # Publish the completed span
        self.publish_span(span)
        
        with self.trace_lock:
            # Remove from active spans
            if span.span_id in self.active_spans:
                del self.active_spans[span.span_id]
    
    def publish_span(self, span: TraceSpan):
        """Publish a span to the ROS 2 network."""
        span_msg = String()
        span_msg.data = json.dumps(span.to_dict())
        self.trace_publisher.publish(span_msg)
    
    def link_span_to_context(self, span: TraceSpan):
        """Link a span to the current trace context."""
        self.trace_context['current_trace_id'] = span.trace_id
        self.trace_context['current_span_id'] = span.span_id
    
    def get_active_spans_for_trace(self, trace_id: str) -> List[TraceSpan]:
        """Get all active spans for a specific trace."""
        with self.trace_lock:
            return [span for span in self.active_spans.values() if span.trace_id == trace_id]
    
    def get_trace_spans(self, trace_id: str) -> List[TraceSpan]:
        """Get all spans for a specific trace (active and completed)."""
        with self.trace_lock:
            return self.active_traces.get(trace_id, [])
    
    def get_all_traces(self) -> List[str]:
        """Get IDs of all active traces."""
        with self.trace_lock:
            return list(self.active_traces.keys())
    
    @contextmanager
    def trace_operation(self, name: str, trace_id: Optional[str] = None):
        """
        Context manager for tracing an operation.
        
        Args:
            name: Name of the operation to trace
            trace_id: Existing trace ID to use, or None to create a new one
        """
        if trace_id is None:
            trace_id = self.start_trace(name)
        
        span = self.start_span(name, trace_id)
        self.link_span_to_context(span)
        
        try:
            yield span
            self.end_span(span, "OK")
        except Exception as e:
            span.add_event("exception", attributes={
                "exception_type": type(e).__name__,
                "message": str(e),
                "traceback": traceback.format_exc()
            })
            self.end_span(span, "ERROR")
            raise
    
    def continue_trace(self, trace_id: str, span_name: str) -> TraceSpan:
        """
        Continue an existing trace in this node.
        
        Args:
            trace_id: ID of the trace to continue
            span_name: Name of the new span in this trace
            
        Returns:
            TraceSpan object
        """
        return self.start_span(span_name, trace_id)
    
    def add_attribute_to_current_span(self, key: str, value: Any):
        """Add an attribute to the currently active span."""
        current_span_id = self.trace_context.get('current_span_id')
        if current_span_id and current_span_id in self.active_spans:
            self.active_spans[current_span_id].set_attribute(key, value)
    
    def add_event_to_current_span(self, name: str, attributes: Optional[Dict[str, Any]] = None):
        """Add an event to the currently active span."""
        current_span_id = self.trace_context.get('current_span_id')
        if current_span_id and current_span_id in self.active_spans:
            self.active_spans[current_span_id].add_event(name, attributes=attributes)
    
    def get_trace_info(self, trace_id: str) -> Dict[str, Any]:
        """
        Get summary information about a trace.
        
        Args:
            trace_id: ID of the trace to get info for
            
        Returns:
            Dictionary with trace summary information
        """
        with self.trace_lock:
            spans = self.active_traces.get(trace_id, [])
            
            if not spans:
                return {"error": "Trace not found"}
            
            start_time = min(span.start_time for span in spans)
            end_time = max(span.end_time or time.time() for span in spans)
            
            return {
                "trace_id": trace_id,
                "start_time": start_time,
                "end_time": end_time,
                "duration": end_time - start_time,
                "span_count": len(spans),
                "spans": [span.to_dict() for span in spans]
            }
    
    def get_performance_summary(self) -> Dict[str, Any]:
        """Get a summary of tracing performance."""
        return {
            "span_counts": self.span_counts,
            "active_traces": len(self.get_all_traces()),
            "active_spans": len(self.active_spans)
        }


class TraceDecorator:
    """
    Decorator for easily adding tracing to functions.
    """
    
    def __init__(self, tracer: TracingSystem, span_name: str = None):
        self.tracer = tracer
        self.span_name = span_name
    
    def __call__(self, func):
        def wrapper(*args, **kwargs):
            # Use function name if no explicit span name provided
            span_name = self.span_name or func.__name__
            
            with self.tracer.trace_operation(span_name) as span:
                # Add function parameters as attributes to the span
                span.set_attribute("function_name", func.__name__)
                
                if args:
                    span.set_attribute("args_count", len(args))
                
                if kwargs:
                    span.set_attribute("kwargs_keys", list(kwargs.keys()))
                
                try:
                    result = func(*args, **kwargs)
                    span.set_attribute("result_type", type(result).__name__)
                    return result
                except Exception as e:
                    span.add_event("exception", attributes={
                        "exception_type": type(e).__name__,
                        "message": str(e)
                    })
                    raise
        
        # Preserve original function metadata
        wrapper.__name__ = func.__name__
        wrapper.__doc__ = func.__doc__
        
        return wrapper


# Global tracing system instance
_tracing_system = None


def get_tracing_system(node: Node = None) -> TracingSystem:
    """Get the global tracing system instance."""
    global _tracing_system
    if _tracing_system is None:
        _tracing_system = TracingSystem()
    return _tracing_system


def start_trace(name: str, parent_trace_id: Optional[str] = None) -> str:
    """Start a new trace using the global tracing system."""
    tracer = get_tracing_system()
    return tracer.start_trace(name, parent_trace_id)


def start_span(name: str, trace_id: str, parent_span_id: Optional[str] = None) -> TraceSpan:
    """Start a new span using the global tracing system."""
    tracer = get_tracing_system()
    return tracer.start_span(name, trace_id, parent_span_id)


def trace_operation(name: str, trace_id: Optional[str] = None):
    """Context manager for tracing an operation using the global tracing system."""
    tracer = get_tracing_system()
    return tracer.trace_operation(name, trace_id)


def continue_trace(trace_id: str, span_name: str) -> TraceSpan:
    """Continue an existing trace using the global tracing system."""
    tracer = get_tracing_system()
    return tracer.continue_trace(trace_id, span_name)


def add_attribute_to_current_span(key: str, value: Any):
    """Add an attribute to the current span using the global tracing system."""
    tracer = get_tracing_system()
    tracer.add_attribute_to_current_span(key, value)


def add_event_to_current_span(name: str, attributes: Optional[Dict[str, Any]] = None):
    """Add an event to the current span using the global tracing system."""
    tracer = get_tracing_system()
    tracer.add_event_to_current_span(name, attributes)


def trace_function(span_name: str = None):
    """Decorator to trace a function using the global tracing system."""
    tracer = get_tracing_system()
    return TraceDecorator(tracer, span_name)


if __name__ == "__main__":
    # Example usage
    rclpy.init()
    
    tracing_node = TracingSystem("test_tracing")
    
    # Example 1: Using context manager
    with tracing_node.trace_operation("example_operation") as span:
        span.set_attribute("user_id", "12345")
        span.add_event("starting_work", attributes={"step": 1})
        
        # Simulate some work
        time.sleep(0.1)
        
        span.add_event("completed_work", attributes={"step": 2})
    
    # Example 2: Manual span management
    trace_id = tracing_node.start_trace("manual_trace")
    span1 = tracing_node.start_span("first_step", trace_id)
    span1.set_attribute("step", "one")
    tracing_node.end_span(span1)
    
    span2 = tracing_node.start_span("second_step", trace_id)
    span2.set_attribute("step", "two")
    tracing_node.end_span(span2)
    
    # Get trace info
    trace_info = tracing_node.get_trace_info(trace_id)
    print(f"Trace info: {json.dumps(trace_info, indent=2, default=str)}")
    
    # Example 3: Using decorator
    @trace_function("decorated_function")
    def example_function(x, y):
        time.sleep(0.05)
        return x + y
    
    result = example_function(5, 3)
    print(f"Result: {result}")
    
    tracing_node.destroy_node()
    rclpy.shutdown()