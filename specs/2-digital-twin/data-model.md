# Data Model: Module 2: Digital Twin (Gazebo & Unity)

**Feature**: 2-digital-twin
**Date**: 2025-12-15

## Overview

This document describes the core data structures and message types used in the digital twin environment for the humanoid robot simulation, based on the functional requirements from the specification.

## Simulation Environment Entities

### SimulationEnvironment
- **Purpose**: Container for physics properties, gravity, collision models and world elements
- **Properties**:
  - `name`: String (name of the simulation environment)
  - `gravity`: Vector3 (gravity vector in m/s², typically [0, 0, -9.81])
  - `physics_engine`: String (name of physics engine: "DART", "ODE", or "Bullet")
  - `time_step`: Float64 (simulation time step in seconds)
  - `real_time_factor`: Float64 (target simulation speed relative to real-time)
  - `collision_detection`: String (collision detection algorithm)
  - `world_elements`: Array of WorldElement objects (static and dynamic objects in the environment)

### WorldElement
- **Purpose**: Represents static or dynamic elements in the simulation environment
- **Properties**:
  - `id`: String (unique identifier for the element)
  - `type`: String (type: "static_object", "obstacle", "movable_object", etc.)
  - `pose`: Pose (position and orientation in the world frame)
  - `geometry`: Geometry object (shape and size of the element)
  - `material_properties`: MaterialProperties object (visual and physical properties)

### MaterialProperties
- **Purpose**: Defines visual and physical material properties for simulation
- **Properties**:
  - `density`: Float64 (material density in kg/m³)
  - `friction`: Float64 (coefficient of friction)
  - `restitution`: Float64 (coefficient of restitution for bounces)
  - `color`: Vector4 (RGBA color values)
  - `texture`: String (path to texture file, if applicable)

## Robot Model Entities

### HumanoidModel
- **Purpose**: Digital representation of the physical robot with kinematic and dynamic properties
- **Properties**:
  - `model_name`: String (name of the robot model)
  - `description_format`: String ("URDF" or "SDF")
  - `base_link`: String (name of the base link)
  - `links`: Array of Link objects (robot links with physical properties)
  - `joints`: Array of Joint objects (robot joints with kinematic properties)
  - `sensors`: Array of Sensor objects (attached sensors)

### Link
- **Purpose**: Represents a rigid body in the robot's kinematic chain
- **Properties**:
  - `name`: String (unique name of the link)
  - `inertial`: Inertial object (mass and inertia properties)
  - `visual`: Visual object (visual representation)
  - `collision`: Collision object (collision shape)
  - `parent_joint`: String (name of the parent joint connecting this link)

### Inertial
- **Purpose**: Physical properties of a robot link
- **Properties**:
  - `mass`: Float64 (mass in kg)
  - `com`: Vector3 (center of mass offset from link origin)
  - `ixx`, `ixy`, `ixz`, `iyy`, `iyz`, `izz`: Float64 (inertia tensor values)

### Joint
- **Purpose**: Represents a connection between two links
- **Properties**:
  - `name`: String (unique name of the joint)
  - `type`: String (type: "revolute", "prismatic", "fixed", etc.)
  - `parent`: String (name of parent link)
  - `child`: String (name of child link)
  - `axis`: Vector3 (axis of rotation or translation)
  - `limits`: JointLimits object (position, velocity, and effort limits)
  - `dynamics`: JointDynamics object (friction and damping coefficients)

### JointLimits
- **Purpose**: Constraints for joint movement
- **Properties**:
  - `lower`: Float64 (minimum position limit)
  - `upper`: Float64 (maximum position limit)
  - `effort`: Float64 (maximum effort limit)
  - `velocity`: Float64 (maximum velocity limit)

### JointDynamics
- **Purpose**: Dynamic properties of a joint
- **Properties**:
  - `damping`: Float64 (damping coefficient)
  - `friction`: Float64 (static friction coefficient)

## Sensor Simulation Entities

### SensorSimulator
- **Purpose**: Component that mimics physical sensor behavior with realistic noise, latency, and accuracy parameters
- **Properties**:
  - `sensor_type`: String ("lidar", "depth_camera", "imu", etc.)
  - `sensor_name`: String (unique name of the sensor)
  - `parent_link`: String (link to which the sensor is attached)
  - `pose`: Pose (pose of the sensor relative to parent link)
  - `noise_model`: NoiseModel object (characteristics of sensor noise)
  - `simulation_parameters`: SensorSimulationParams object (simulation-specific parameters)

### NoiseModel
- **Purpose**: Defines the noise characteristics of a simulated sensor
- **Properties**:
  - `noise_type`: String ("gaussian", "uniform", "custom")
  - `mean`: Float64 (mean value of noise)
  - `std_dev`: Float64 (standard deviation of noise)
  - `bias`: Float64 (systematic bias in measurements)
  - `drift`: Float64 (gradual drift in sensor values over time)

### SensorSimulationParams
- **Purpose**: Simulation-specific parameters for sensor modeling
- **Properties**:
  - `update_rate`: Float64 (sensor update rate in Hz)
  - `latency`: Float64 (sensor data latency in seconds)
  - `range_min`: Float64 (minimum detectable range)
  - `range_max`: Float64 (maximum detectable range)
  - `fov_horizontal`: Float64 (horizontal field of view in radians)
  - `fov_vertical`: Float64 (vertical field of view in radians, for cameras)

### SensorData
- **Purpose**: Stream of data from simulated sensors containing appropriate noise and characteristics
- **Properties**:
  - `sensor_name`: String (name of the sensor that generated the data)
  - `timestamp`: Time (ROS time of data generation)
  - `sensor_type`: String (type of sensor)
  - `data`: Array of Float64 (raw sensor data values)
  - `frame_id`: String (coordinate frame of the sensor data)
  - `quality_metrics`: SensorQualityMetrics object (metrics about data quality)

### SensorQualityMetrics
- **Purpose**: Quality metrics for sensor data
- **Properties**:
  - `signal_to_noise_ratio`: Float64 (SNR of the sensor data)
  - `accuracy`: Float64 (estimated accuracy of measurements)
  - `precision`: Float64 (estimated precision of measurements)
  - `latency_measured`: Float64 (actual measured latency)

## Synchronization Entities

### RealTimeSyncer
- **Purpose**: System that synchronizes simulation state with Unity visualization
- **Properties**:
  - `gazebo_time`: Time (current simulation time in Gazebo)
  - `unity_time`: Time (synchronized time in Unity)
  - `sync_frequency`: Float64 (synchronization frequency in Hz)
  - `max_desync_threshold`: Float64 (maximum allowed time desynchronization in seconds)
  - `sync_enabled`: Boolean (whether synchronization is active)

### SimulationState
- **Purpose**: Current state of the simulation for synchronization
- **Properties**:
  - `robot_poses`: Array of RobotPose objects (current poses of all robots)
  - `sensor_data`: Array of SensorData objects (latest sensor readings)
  - `environment_state`: EnvironmentState object (state of dynamic environment elements)
  - `simulation_time`: Time (current simulation time)

### RobotPose
- **Purpose**: Pose of a robot in the simulation
- **Properties**:
  - `robot_name`: String (name of the robot)
  - `position`: Vector3 (position in world coordinates)
  - `orientation`: Quaternion (orientation in world coordinates)
  - `joint_states`: JointState object (current joint positions and velocities)

### JointState
- **Purpose**: Current state of robot joints
- **Properties**:
  - `joint_names`: Array of String (names of joints)
  - `positions`: Array of Float64 (current joint positions)
  - `velocities`: Array of Float64 (current joint velocities)
  - `efforts`: Array of Float64 (current joint efforts)

### EnvironmentState
- **Purpose**: State of dynamic elements in the environment
- **Properties**:
  - `object_poses`: Array of ObjectPose objects (poses of movable objects)
  - `dynamic_lights`: Array of LightState objects (state of dynamic lights)
  - `environment_conditions`: EnvironmentConditions object (weather, lighting conditions)

### ObjectPose
- **Purpose**: Pose of a dynamic object in the environment
- **Properties**:
  - `object_id`: String (unique identifier of the object)
  - `position`: Vector3 (position in world coordinates)
  - `orientation`: Quaternion (orientation in world coordinates)

### LightState
- **Purpose**: State of a dynamic light source
- **Properties**:
  - `light_id`: String (unique identifier of the light)
  - `position`: Vector3 (position in world coordinates)
  - `intensity`: Float64 (light intensity)
  - `color`: Vector4 (RGBA color values)

### EnvironmentConditions
- **Purpose**: Environmental conditions affecting simulation
- **Properties**:
  - `temperature`: Float64 (ambient temperature in Celsius)
  - `humidity`: Float64 (relative humidity percentage)
  - `atmospheric_pressure`: Float64 (atmospheric pressure in hPa)
  - `gravity_vector`: Vector3 (current gravity vector)

## ROS 2 Message Types

### DigitalTwinState
- **Purpose**: Comprehensive state of the digital twin system for ROS 2 communication
- **Fields**:
  - `header`: std_msgs/Header (timestamp and coordinate frame)
  - `simulation_time`: builtin_interfaces/Time (current simulation time)
  - `robot_states`: Array of RobotState (states of all robots in simulation)
  - `environment_state`: EnvironmentalState (state of the environment)
  - `sensor_data`: Array of SensorData (all current sensor data)

### RobotState
- **Purpose**: State of an individual robot in the digital twin
- **Fields**:
  - `robot_name`: String (name of the robot)
  - `pose`: geometry_msgs/Pose (current pose of the robot)
  - `twist`: geometry_msgs/Twist (current velocity of the robot)
  - `joint_states`: sensor_msgs/JointState (current joint states)
  - `attached_sensors`: Array of String (names of attached sensors)

## Data Flow Patterns

### Simulation to Visualization Pipeline
1. Gazebo publishes robot state information to `/gazebo/robot_states` topic
2. Synchronization node subscribes and processes the state data
3. Unity visualization subscribes to transformed state via ROS-TCP-Connector
4. Unity renders the robot and environment based on the received state

### Sensor Simulation Pipeline
1. Gazebo sensor plugins generate raw sensor data based on physics simulation
2. Noise models are applied to raw data to simulate realistic sensor characteristics
3. Sensor data is published to ROS 2 topics (e.g., `/sensors/lidar/scan`)
4. Perception algorithms subscribe to sensor data for processing

### Control Pipeline
1. High-level controllers publish commands to ROS 2 topics (e.g., `/joint_commands`)
2. Gazebo receives commands through ROS 2 bridge
3. Commands are applied to robot joints in simulation
4. Robot movement is computed based on physics engine

## Quality of Service (QoS) Considerations

### High-Frequency Sensor Data
- Reliability: Best Effort (for high-frequency sensors like cameras)
- Durability: Volatile
- History: Keep Last
- Depth: 1
- Deadline: Appropriate for sensor refresh rate

### Robot State Information
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last
- Depth: 1
- Deadline: 50ms (for real-time performance)

### Control Commands
- Reliability: Reliable
- Durability: Volatile
- History: Keep Last
- Depth: 1
- Deadline: 30ms (for responsive control)

## Coordinate Systems

### Simulation Coordinate Frames
- `world`: Origin of the simulation environment
- `<robot_name>/base_link`: Base frame of the robot
- `<sensor_name>_frame`: Frame of each sensor
- `odom`: Odometry frame for motion tracking
- Joint-specific frames: Named according to joint

## Academic Standards Compliance

All data models in this document are based on:
- ROS 2 message definitions: https://docs.ros.org/en/humble/Reference/About-ROS-Interfaces.html
- Gazebo model format documentation: http://gazebosim.org/
- Standard robotics simulation practices
- Official Unity ROS integration documentation

This aligns with the project constitution's principle of "Accuracy through Primary Source Verification" by citing official documentation and standards.