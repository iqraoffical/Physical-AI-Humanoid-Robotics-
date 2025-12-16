# Digital Twin API Contract

**Version**: 1.0
**Feature**: 2-digital-twin
**Date**: 2025-12-15

## Overview

This document defines the API contracts for the Digital Twin system using Gazebo and Unity for the humanoid robot, including message types for simulation, sensor data, and visualization synchronization. Based on functional requirements from the specification, this ensures accurate physics simulation and proper integration between Gazebo and Unity environments.

## Message Definitions

### DigitalTwinState

**Purpose**: Comprehensive state of the digital twin system (from FR-001: Gazebo robot simulation setup, FR-006: ROS 2 standard logging)

```
std_msgs/Header header
builtin_interfaces/Time header.stamp
string header.frame_id

builtin_interfaces/Time simulation_time

robot_nervous_system/RobotState[] robot_states
robot_nervous_system/EnvironmentalState environment_state
robot_nervous_system/SensorData[] sensor_data
```

**Topic**: `/digital_twin/state`
**Publisher**: Digital twin synchronization node
**Subscriber(s)**: Visualization systems, monitoring tools, logging systems
**Security**: Messages subject to validation against security context
**Observability**: All messages logged for monitoring and debugging

### RobotState

**Purpose**: State of an individual robot in the digital twin (from FR-001: Gazebo robot simulation setup)

```
string robot_name
geometry_msgs/Pose pose
geometry_msgs/Twist twist
sensor_msgs/JointState joint_states
string[] attached_sensors
```

**Topic**: `/digital_twin/robot_states`
**Publisher**: Gazebo simulation
**Subscriber(s)**: Unity visualization, monitoring systems, controllers
**Security**: Messages subject to validation against security context
**Validation**: Robot name and joint states verified against known robot configuration

### SensorData

**Purpose**: Stream of data from simulated sensors with noise characteristics (from FR-003: Realistic sensor simulation, FR-010: Physics simulation validation)

```
builtin_interfaces/Time header.stamp
string sensor_name
string sensor_type
float64[] data
string frame_id
robot_nervous_system/SensorQualityMetrics quality_metrics
```

**Topic**: `/sensors/*/data` (various topics for different sensors)
**Publisher**: Gazebo sensor plugins
**Subscriber(s)**: Perception algorithms, monitoring systems, data loggers
**Security**: Messages subject to validation against security context
**Validation**: Sensor data validated against noise model parameters and expected ranges
**Error Handling**: Invalid measurements trigger error recovery mechanisms

## Service Definitions

### LoadSimulationEnvironment

**Purpose**: Load a specific simulation environment (from FR-001: Gazebo robot simulation setup)

**Service Type**: `digital_twin/srv/LoadSimulationEnvironment`

**Request**:
```
string world_file_path         # Path to SDF world file
string model_file_path         # Path to robot URDF/SDF model
bool enable_physics           # Whether to enable physics simulation
float64 physics_accuracy      # Target physics accuracy setting
```

**Response**:
```
bool success                  # Whether environment was loaded successfully
string message                # Description of result
string simulation_id          # Unique ID for the loaded simulation
```

**Service Name**: `/digital_twin/load_environment`
**Client(s)**: Simulation control systems
**Server**: Gazebo simulation manager
**Security**: Access controlled based on security context
**Validation**: File paths validated to prevent directory traversal attacks
**Error Handling**: Invalid parameters trigger error recovery mechanisms
**Observability**: All calls logged for monitoring and debugging

### ConfigureSensors

**Purpose**: Configure sensor parameters for realistic simulation (from FR-003: Realistic sensor simulation)

**Service Type**: `digital_twin/srv/ConfigureSensors`

**Request**:
```
string robot_name             # Name of the robot to configure
robot_nervous_system/SensorConfig[] sensor_configs  # Configuration for each sensor
```

**Response**:
```
bool success                  # Whether sensors were configured successfully
string message                # Description of result
bool[] sensor_status          # Status of each sensor configuration
```

**Service Name**: `/digital_twin/configure_sensors`
**Client(s)**: Sensor calibration tools
**Server**: Simulation sensor manager
**Security**: Access controlled based on security context
**Validation**: Sensor configurations validated against robot capabilities
**Error Handling**: Invalid configurations trigger error recovery mechanisms
**Observability**: All calls logged for monitoring and debugging

## Action Definitions

### RunSimulationScenario

**Purpose**: Execute a predefined simulation scenario (from FR-001: Gazebo robot simulation setup)

**Action Type**: `digital_twin/action/RunSimulationScenario`

**Goal**:
```
string scenario_name          # Name of the scenario to run
float64 duration              # Duration to run the scenario (0 for indefinite)
bool record_data             # Whether to record sensor and state data
string[] robots_to_control    # List of robots to control in the scenario
```

**Feedback**:
```
float64 elapsed_time          # Time elapsed since scenario started
float64 progress_percentage   # Progress as a percentage
bool scenario_running         # Whether the scenario is still running
```

**Result**:
```
bool success                  # Whether scenario completed successfully
string result_message         # Description of the result
string data_path              # Path to recorded data (if recording was enabled)
int32 error_code              # Error code if failure occurred (0 for success)
```

**Action Name**: `/digital_twin/run_scenario`
**Client(s)**: Validation and testing systems
**Server**: Simulation scenario executor
**Security**: Access controlled based on security context
**Validation**: Scenario parameters validated against system capabilities
**Error Handling**: Invalid scenarios trigger error recovery mechanisms
**Observability**: All actions logged for monitoring and debugging

## Quality of Service (QoS) Profiles

### Robot State Topic
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last
- Depth: 1
- Deadline: 50ms
- **Rationale**: Supports real-time performance (from non-functional requirement)

### Sensor Data Topics
- Reliability: Best Effort (for high-frequency sensors like cameras, Reliable for IMU/LiDAR)
- Durability: Volatile
- History: Keep Last
- Depth: 10 (for cameras) or 1 (for other sensors)
- Deadline: Sensor-specific based on update rate
- **Rationale**: Optimized for high-frequency data streams while ensuring critical sensor data reliability

### Control Command Topics
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last
- Depth: 1
- Deadline: 30ms
- **Rationale**: Supports responsive control with minimal latency

## Error Handling

### Error Codes

- `0`: Success
- `1`: Invalid simulation parameters (e.g., physics accuracy out of range)
- `2`: Sensor configuration error (e.g., invalid noise model parameters)
- `3`: Environment loading failure (e.g., invalid world file)
- `4`: Resource exhaustion (e.g., insufficient GPU memory)
- `5`: Synchronization failure (e.g., Unity/Gazebo desync)
- `6`: Security violation (invalid authentication or authorization)
- **Rationale**: Supports validation of robot behavior under physics (from acceptance criteria) and security requirements

## Performance Expectations

- Unity visualization update rate: 30+ FPS (to meet SC-003: minimum 30 FPS)
- Simulation real-time factor: 1.0x (to meet SC-004: real-time performance)
- Sensor data accuracy: Within 10% of physical sensor characteristics (to meet SC-002)
- Physics accuracy: Mean absolute error under 5% compared to physical tests (to meet SC-009)
- Synchronization accuracy: Maintains consistency between Gazebo and Unity with minimal latency (to meet SC-008)

## Security Framework

- All ROS 2 communications use security context validation
- Access control policies for different simulation components
- Message integrity verification for critical control commands
- Certificate-based authentication for nodes in the system
- **Rationale**: Implements ROS 2 security framework with access control and message validation (from clarifications)

## Integration Points

### Gazebo Simulation
- Physics engine configuration via SDF parameters
- Sensor plugins for realistic data generation
- ROS 2 bridge for communication with external systems
- **Requirements**: Accurate physics simulation with gravity, collisions, and dynamics

### Unity Visualization
- Real-time synchronization with Gazebo simulation state
- High-fidelity rendering of robot and environment
- ROS-TCP-Connector for ROS 2 communication
- **Requirements**: 30+ FPS while maintaining simulation synchronization (from FR-008)

### ROS 2 Integration
- Standard ROS 2 message types for sensor and state data
- Standard control interfaces for robot commands
- Standard coordinate frame conventions (TF)
- **Requirements**: Compatible with ROS 2 ecosystem and perception systems

## Observability Features

- Comprehensive logging using ROS 2's built-in logging infrastructure
- Custom metrics collection for performance monitoring
- Tracing capabilities to track requests across simulation components
- Dashboards for real-time system monitoring
- **Rationale**: Implements ROS 2 standard logging with custom metrics and basic tracing for simulation events (from clarifications)

## Academic Standards Compliance

All API contracts in this document follow:
- Standard ROS 2 interface definitions
- Quality of Service patterns from official ROS 2 documentation
- Error handling practices from ROS 2 design guidelines
- Simulation standards from Gazebo documentation
- Unity integration patterns from official Unity-ROS documentation

This aligns with the project constitution's principle of "Accuracy through Primary Source Verification" by following standard ROS 2 patterns and official documentation.