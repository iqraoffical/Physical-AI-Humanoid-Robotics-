# Robot Nervous System

## Overview
The Robot Nervous System implements the communication architecture for a humanoid robot using ROS 2. It serves as the "nervous system" by providing reliable communication, modularity, and fault tolerance for the robotic platform.

## Architecture
The system implements a distributed communication architecture with:
- Publisher/Subscriber nodes for real-time data streaming
- Service nodes for synchronous operations
- Action servers for long-running behaviors
- Python agent bridge for AI integration
- Security framework for protected communications
- Comprehensive observability with logging, metrics, and tracing

## Components

### Core Communication
- `publisher_node`: Publishes sensor data and joint states
- `subscriber_node`: Subscribes to joint commands and processes them
- `service_node`: Handles synchronous operations like configuration
- `action_node`: Manages long-running robot behaviors

### AI Integration
- `python_agent_bridge`: Bridges Python AI agents to ROS controllers

### Security & Observability
- `security_context`: Implements ROS 2 security framework
- `logging_module`: Comprehensive logging infrastructure
- `metrics_collector`: Performance and system metrics
- `tracing_system`: Request tracing across nodes

## Requirements
- ROS 2 Humble Hawksbill
- Python 3.10+
- Ubuntu 22.04 LTS

## Installation
```bash
# Clone the repository
cd ~/ros2_ws/src
git clone <repository_url>

# Build the package
cd ~/ros2_ws
colcon build --packages-select robot_nervous_system
source install/setup.bash
```

## Usage
```bash
# Launch the complete nervous system
ros2 launch robot_nervous_system nervous_system.launch.py

# Run individual components
ros2 run robot_nervous_system publisher_node.py
ros2 run robot_nervous_system subscriber_node.py
ros2 run robot_nervous_system python_agent_bridge.py
```

## Security
The system implements the ROS 2 security framework with:
- Access control policies
- Message encryption
- Certificate-based authentication

To enable security, ensure your security keystore is properly configured and set the appropriate environment variables:
```bash
export ROS_SECURITY_KEYSTORE=~/ros2_ws/keys
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

## Observability
The system provides comprehensive observability through:
- Structured logging with component and context information
- System and application metrics
- Distributed tracing for request flow analysis

## 24+ DOF Humanoid Model
The system includes support for a 24+ degree-of-freedom humanoid model with:
- 6 DOF per leg (hip, knee, ankle)
- 6 DOF per arm (shoulder, elbow, wrist)
- 2 DOF for head/neck
- 2 DOF for torso/waist

## Performance Goals
- <100ms response time for joint commands
- 100Hz joint state publication rate
- Deterministic communication with fault isolation