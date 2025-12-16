# ROS 2 Nervous System API Contract

**Version**: 1.0  
**Feature**: 1-ros2-nervous-system  
**Date**: 2025-12-15

## Overview

This document defines the API contracts for the ROS 2 nervous system of the humanoid robot, including message types, services, and actions. Based on functional requirements from the specification, this ensures deterministic communication and fault isolation, with additional security and observability features as clarified.

## Message Definitions

### JointStateMessage

**Purpose**: Represents the current state of robot joints (from FR-001: publisher/subscriber nodes)

```
std_msgs/Header header
builtin_interfaces/Time header.stamp
string header.frame_id

string[] name
float64[] position
float64[] velocity
float64[] effort
```

**Topic**: `/joint_states`  
**Publisher**: Robot state publisher  
**Subscriber(s)**: Control algorithms, visualization tools, logging systems  
**Security**: Messages subject to validation against security context  
**Observability**: All messages logged for monitoring and debugging

### JointCommandMessage

**Purpose**: Commands for robot joint control (from FR-004: Python agent commands robot joints)

```
builtin_interfaces/Time stamp
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts
```

**Topic**: `/joint_commands`  
**Publisher**: High-level controllers, AI agents  
**Subscriber(s)**: Joint controllers  
**Security**: Commands validated against security context  
**Validation**: Position values must be within joint limits from 24+ DOF URDF model (from clarifications)  
**Error Handling**: Invalid commands trigger error recovery mechanisms (from clarifications)  
**Observability**: All commands logged for monitoring and debugging

### SensorDataMessage

**Purpose**: Contains sensor readings from the robot

```
builtin_interfaces/Time header.stamp
string sensor_type
string sensor_name
float64[] data
string frame_id
```

**Topic**: `/sensor_data`  
**Publisher**: Sensor drivers  
**Subscriber(s)**: Perception algorithms, logging systems  
**Security**: Messages subject to validation against security context  
**Observability**: Data includes metadata for logging and tracing (from clarifications)

## Service Definitions

### LoadURDF

**Purpose**: Load a URDF model into the parameter server (from FR-003: Load URDF humanoid model in simulation)

**Service Type**: `robot_nervous_system/srv/LoadURDF`

**Request**:
```
string robot_description  # URDF content for 24+ DOF humanoid model (from clarifications)
string robot_name         # Name to assign to robot
```

**Response**:
```
bool success              # Whether URDF was loaded successfully
string message            # Description of result
```

**Service Name**: `/load_urdf`  
**Client(s)**: Robot initialization scripts  
**Server**: Description loader node  
**Security**: Access controlled based on security context (from clarifications)  
**Validation**: URDF content must be well-formed XML with valid kinematic chain for 24+ DOF humanoid model (from clarifications)  
**Error Handling**: Invalid URDF triggers error recovery mechanisms (from clarifications)  
**Observability**: All calls logged for monitoring and debugging

### ExecuteAction

**Purpose**: Execute a predefined robot action (from FR-002: Create actions)

**Service Type**: `robot_nervous_system/srv/ExecuteAction`

**Request**:
```
string action_name        # Name of action to execute
dictionary parameters     # Action parameters
```

**Response**:
```
bool success              # Whether action was accepted
string action_id          # Identifier for tracking action
```

**Service Name**: `/execute_action`  
**Client(s)**: High-level task planners  
**Server**: Action execution node  
**Security**: Access controlled based on security context (from clarifications)  
**Validation**: Action parameters validated against security context and joint limits (from clarifications)  
**Error Handling**: Invalid requests trigger error recovery mechanisms (from clarifications)  
**Observability**: All calls logged for monitoring and debugging

## Action Definitions

### MoveRobot

**Purpose**: Move the robot to a target pose (from FR-002: Create actions)

**Action Type**: `robot_nervous_system/action/MoveRobot`

**Goal**:
```
geometry_msgs/Pose target_pose
JointTrajectory trajectory  # Optional trajectory to follow
```

**Feedback**:
```
geometry_msgs/Pose current_pose
float64 distance_remaining
float64 trajectory_progress
```

**Result**:
```
bool success
geometry_msgs/Pose final_pose
int32 error_code
```

**Action Name**: `/move_robot`  
**Client(s)**: Navigation stack  
**Server**: Motion controller  
**Security**: Access controlled based on security context (from clarifications)  
**Validation**: Target pose must be kinematically reachable with 24+ DOF humanoid model (from clarifications)  
**Error Handling**: Invalid poses trigger error recovery mechanisms (from clarifications)  
**Observability**: All actions logged for monitoring and debugging

## Quality of Service (QoS) Profiles

### Joint States Topic
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last
- Depth: 1
- Deadline: 100ms
- **Rationale**: Supports deterministic communication requirement
- **Security**: Messages encrypted based on security context (from clarifications)

### Joint Commands Topic
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last
- Depth: 1
- Deadline: 50ms
- **Rationale**: Supports deterministic communication with low latency
- **Security**: Messages encrypted based on security context (from clarifications)

### Sensor Data Topic
- Reliability: Best Effort (for high-frequency sensors)
- Durability: Volatile
- History: Keep Last
- Depth: 10
- **Rationale**: Optimized for high-frequency data streams
- **Security**: Messages encrypted based on security context (from clarifications)

## Error Handling

### Error Codes

- `0`: Success
- `1`: Invalid parameters (violates joint limits from 24+ DOF URDF model)
- `2`: Hardware error
- `3`: Communication timeout
- `4`: Safety violation (command would cause damage)
- `5`: Unsupported operation
- `6`: Security violation (invalid authentication or authorization)
- **Rationale**: Supports fault isolation by providing specific error responses

## Performance Expectations

- Joint state publication rate: 100 Hz (to support real-time control)
- Command response time: < 50ms (to support responsive control)
- Service response time: < 100ms (to support synchronous operations)
- Action feedback rate: 10 Hz minimum (to support long-running operations)

## Security Framework

- All ROS 2 communications use SROS2 security features
- Access control policies for different node types
- Message encryption for sensitive communication channels
- Certificate-based authentication for nodes in the system
- **Rationale**: Implements ROS 2 security framework with access control and message encryption (from clarifications)

## Observability Features

- Comprehensive logging using ROS 2's built-in logging infrastructure
- Custom metrics collection for performance monitoring
- Tracing capabilities to track requests across nodes
- Dashboards for real-time system monitoring
- **Rationale**: Implements comprehensive logging, metrics, and tracing using ROS 2 standard tools (from clarifications)

## Academic Standards Compliance

All API contracts in this document follow:
- Standard ROS 2 interface definitions
- Quality of Service patterns from official ROS 2 documentation
- Error handling practices from ROS 2 design guidelines
- Security implementation from ROS 2 security guide
- Observability patterns from ROS 2 diagnostic tools

This aligns with the project constitution's principle of "Accuracy through Primary Source Verification" by following standard ROS 2 patterns and official documentation.