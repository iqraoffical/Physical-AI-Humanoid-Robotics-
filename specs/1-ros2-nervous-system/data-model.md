# Data Model: Module 1: The Robotic Nervous System (ROS 2)

**Feature**: 1-ros2-nervous-system
**Date**: 2025-12-15

## Overview

This document describes the core data structures and message types used in the ROS 2 nervous system for the humanoid robot, based on the functional requirements from the specification.

## Message Types

### JointStateMessage
- **Purpose**: Represents the current state of robot joints (from FR-001: publisher/subscriber nodes)
- **Fields**:
  - `timestamp`: Time of measurement (ros time)
  - `joint_names`: Array of strings (names of joints)
  - `position`: Array of float64 (joint positions in radians)
  - `velocity`: Array of float64 (joint velocities in rad/s)
  - `effort`: Array of float64 (joint efforts in N/m)
- **Validation**: Position values must be within joint limits from URDF model (from FR-008: 24+ DOF humanoid model)
- **Security**: Message integrity verified through security context (from clarifications)

### JointCommandMessage
- **Purpose**: Commands for robot joint control (from FR-004: Python agents send joint commands)
- **Fields**:
  - `timestamp`: Time of command (ros time)
  - `joint_names`: Array of strings (names of joints to command)
  - `positions`: Array of float64 (target joint positions in radians)
  - `velocities`: Array of float64 (target joint velocities in rad/s)
  - `efforts`: Array of float64 (target joint efforts in N/m)
- **Validation**: Position values must be within joint limits from URDF model (from FR-008: 24+ DOF humanoid model)
- **Security**: Commands validated against security context (from clarifications)
- **Error Handling**: Invalid commands trigger error recovery mechanisms (from clarifications)

### SensorDataMessage
- **Purpose**: Contains sensor readings from the robot (for bridging to AI agents)
- **Fields**:
  - `timestamp`: Time of sensor reading (ros time)
  - `sensor_type`: String (e.g., "imu", "lidar", "camera")
  - `sensor_name`: String (name of the specific sensor)
  - `data`: Array of float64 (sensor readings)
  - `frame_id`: String (coordinate frame of sensor)
- **Validation**: Data array length must match expected sensor configuration
- **Observability**: Includes metadata for logging and tracing (from clarifications)

### RobotConfigurationMessage
- **Purpose**: Configuration parameters for the robot (from FR-002: services for control)
- **Fields**:
  - `config_id`: String (identifier for configuration set)
  - `parameters`: Dictionary of key-value pairs (configuration parameters)
- **Validation**: Parameters must match expected types for the given config_id
- **Security**: Configuration access controlled through security context (from clarifications)

## Service Types

### LoadURDFService
- **Purpose**: Load a URDF model into the parameter server (from FR-003: load URDF humanoid model)
- **Request**:
  - `robot_description`: String (URDF content for 24+ DOF humanoid model from clarifications)
  - `robot_name`: String (name to assign to robot)
- **Response**:
  - `success`: Boolean (whether URDF was loaded successfully)
  - `message`: String (description of result)
- **Validation**: URDF content must be well-formed XML with valid kinematic chain for 24+ DOF humanoid model
- **Error Handling**: Invalid URDF triggers error recovery mechanisms (from clarifications)

### ExecuteActionService
- **Purpose**: Execute a predefined robot action (from FR-002: create actions)
- **Request**:
  - `action_name`: String (name of action to execute)
  - `parameters`: Dictionary of key-value pairs (action parameters)
- **Response**:
  - `success`: Boolean (whether action was accepted)
  - `action_id`: String (identifier for tracking action)
- **Validation**: Action parameters validated against security context and joint limits (from clarifications)
- **Error Handling**: Invalid requests trigger error recovery mechanisms (from clarifications)

## Action Types

### MoveRobotAction
- **Purpose**: Move the robot to a target pose (from FR-002: create actions)
- **Goal**:
  - `target_pose`: Pose message (desired end position)
  - `trajectory`: Array of JointStateMessage (optional trajectory to follow)
- **Feedback**:
  - `current_pose`: Pose message (current position)
  - `distance_remaining`: Float64 (distance to target)
  - `trajectory_progress`: Float64 (progress along trajectory)
- **Result**:
  - `success`: Boolean (whether goal was achieved)
  - `final_pose`: Pose message (actual final position)
  - `error_code`: Int32 (error code if failure)
- **Validation**: Target pose must be kinematically reachable with 24+ DOF humanoid model (from clarifications)
- **Error Handling**: Invalid poses trigger error recovery mechanisms (from clarifications)

## Robot Description (URDF)

### HumanoidModel
- **Purpose**: Defines the physical properties of the robot (from FR-003 and clarifications for 24+ DOF model)
- **Elements**:
  - Links: Represent rigid parts of the robot (24+ links for full humanoid)
    - Properties: mass, inertia, visual, collision
  - Joints: Connect links with specific kinematics (24+ joints for full humanoid)
    - Properties: type (revolute, prismatic, etc.), limits, axis
  - Transmissions: Define how actuators connect to joints
  - Materials: Visual properties for rendering
- **Validation**: Must form a valid kinematic chain with no unconnected elements, supporting 24+ DOF configuration
- **Joint Limits**: All joints have defined min/max positions within realistic humanoid ranges

### Joint Properties (24+ DOF Model)
- **Revolute Joint**:
  - `min_position`: Float64 (minimum joint angle based on humanoid kinematics)
  - `max_position`: Float64 (maximum joint angle based on humanoid kinematics)
  - `max_velocity`: Float64 (maximum joint velocity)
  - `max_effort`: Float64 (maximum joint effort)
- **Validation**: Limits must be physically realistic for 24+ DOF humanoid design

## Security Context

### SecurityContext
- **Purpose**: Contains access control and encryption settings (from clarifications)
- **Fields**:
  - `access_level`: String (access control level for different components)
  - `encryption_keys`: Dictionary (keys for encrypting messages)
  - `valid_certificates`: Array of strings (valid certificates for authentication)
- **Usage**: Applied to all ROS 2 communications to ensure message integrity

## Observability Data

### ObservabilityData
- **Purpose**: Contains logging, metrics, and tracing information (from clarifications)
- **Fields**:
  - `log_level`: String (severity level of the log)
  - `timestamp`: Time of the event (ros time)
  - `source_node`: String (name of the node generating the data)
  - `message`: String (the log message)
  - `metrics`: Dictionary (key-value pairs of performance metrics)
  - `trace_id`: String (identifier for tracing requests across nodes)

## Data Flow Patterns

### Sensor Data Flow
1. Sensors publish raw data to `/sensor_data` topic
2. Processing nodes subscribe and transform to semantic representations
3. Processed data published to specific topics (e.g., `/joint_states`, `/imu/data`)
- **Requirements**: Real-time delivery with bounded latency
- **Security**: All messages verified against security context
- **Observability**: All communications logged for monitoring and debugging

### Control Command Flow
1. High-level controllers publish commands to `/joint_commands`
2. Controller nodes receive and validate commands against security context and joint limits
3. Commands converted to actuator-specific signals
4. Actuator commands sent to hardware interfaces
- **Requirements**: Deterministic execution with safety validation
- **Error Handling**: Invalid commands trigger error recovery mechanisms (from clarifications)
- **Observability**: All commands logged for monitoring and debugging

## Quality of Service (QoS) Considerations

### Real-time Critical Data
- History: Keep last 1 item
- Reliability: Reliable
- Durability: Volatile
- Lifespan: 100ms
- **Applies to**: Joint states, joint commands
- **Security**: Messages encrypted based on security context

### Configuration Data
- History: Keep all
- Reliability: Reliable
- Durability: Transient local
- Lifespan: Forever
- **Applies to**: Robot configurations
- **Security**: Access controlled based on security context

## Coordinate Systems

### Robot Coordinate Frames
- `base_link`: Origin of the robot (from URDF model)
- `odom`: World-fixed coordinate frame
- Joint-specific frames: Named according to joint
- Sensor frames: Named according to sensor location
- **Requirements**: Consistent transforms between all frames
- **Observability**: Transform data logged for debugging

## Academic Standards Compliance

All data models in this document are based on:
- Official ROS 2 message definitions: https://docs.ros.org/en/humble/Reference/About-ROS-Interfaces.html
- URDF documentation: http://wiki.ros.org/urdf
- Standard ROS 2 practices: https://design.ros2.org/
- ROS 2 security documentation: https://docs.ros.org/en/humble/How-To-Guides/Setting-up-Secure-Communication.html

This aligns with the project constitution's principle of "Accuracy through Primary Source Verification" by citing official ROS 2 documentation and standards.