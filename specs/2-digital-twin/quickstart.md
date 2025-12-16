# Quickstart Guide: Module 2: Digital Twin (Gazebo & Unity)

**Feature**: 2-digital-twin
**Date**: 2025-12-15

## Overview

This guide provides step-by-step instructions to set up and run the digital twin environment for the humanoid robot using Gazebo and Unity. It covers environment setup, simulation configuration, sensor simulation, and visualization integration. This implementation follows the requirements for Ubuntu 22.04 + ROS 2 Humble environment with RTX 4070 Ti or higher GPU as specified in the hardware notes.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Gazebo (Fortress or Garden) installed
- Unity 3D (2022.3 LTS or newer)
- Python 3.10+
- RTX 4070 Ti or higher GPU recommended (from hardware notes: RTX 4070 Ti or higher)
- 32GB+ RAM minimum (from hardware notes: RAM ≥ 32 GB)
- All content follows the academic standards of the project constitution

## Setup Instructions

### 1. Environment Setup

1. Install required ROS 2 packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-*
   sudo apt install ros-humble-ros-gz
   sudo apt install python3-colcon-common-extensions
   ```

2. Install Unity Hub and Unity 3D:
   - Download Unity Hub from Unity's official website
   - Install Unity 2022.3 LTS with the Linux build support module
   - Verify minimum 8GB VRAM for Unity visualization (from clarifications)

3. Install Unity ROS TCP Connector:
   ```bash
   # Clone the ROS-TCP-Connector package
   git clone https://github.com/Unity-Technologies/ROS-TCP-Connector.git
   ```

4. Create simulation workspace:
   ```bash
   mkdir -p ~/simulation_ws/src
   cd ~/simulation_ws
   ```

### 2. Clone and Build Simulation Packages

1. Clone the digital twin packages:
   ```bash
   cd ~/simulation_ws/src
   git clone <simulation_packages_repository>
   ```

2. Build the workspace:
   ```bash
   cd ~/simulation_ws
   colcon build --packages-select digital_twin
   source install/setup.bash
   ```

### 3. Hardware Configuration Check

Verify your system meets the requirements:
```bash
# Check GPU
nvidia-smi

# Check available RAM
free -h

# Check ROS 2 installation
ros2 topic list
```

## Basic Operations

### 1. Launch Digital Twin Environment

```bash
# Terminal 1: Start Gazebo simulation
ros2 launch digital_twin simulation.launch.py

# Terminal 2: Start Unity visualization (if configured)
# Unity scene needs to be loaded and played manually
# Or use the Unity ROS connector to start from command line

# Terminal 3: Launch sensor and control bridge
ros2 launch digital_twin digital_twin_bridge.launch.py
```

### 2. Check Active Simulation Nodes

```bash
ros2 node list
```

You should see nodes like:
- `/gazebo` - Gazebo simulation engine
- `/unity_bridge` - Unity visualization connector
- `/sensor_simulator` - Sensor data generation
- `/simulation_controller` - Simulation control interface

### 3. List Available Topics

```bash
ros2 topic list
```

Look for topics like:
- `/digital_twin/robot_states` - Robot poses in simulation
- `/sensors/lidar/scan` - LiDAR sensor data
- `/sensors/depth_camera/depth/image_raw` - Depth camera data
- `/sensors/imu/data` - IMU sensor data
- `/joint_commands` - Joint command inputs

### 4. Monitor Robot State

```bash
# Listen to robot state messages
ros2 topic echo /digital_twin/robot_states
```

### 5. Monitor Sensor Data

```bash
# Monitor LiDAR data
ros2 topic echo /sensors/lidar/scan

# Monitor IMU data
ros2 topic echo /sensors/imu/data
```

## Running Simulations

### 1. Basic Humanoid Simulation

```bash
# Start the simulation with default humanoid model
ros2 launch digital_twin humanoid_demo.launch.py
```

### 2. Sensor Validation Test

```bash
# Run a test to validate sensor data streams (from FR-003: Realistic sensor simulation)
ros2 run digital_twin test_sensor_accuracy.py
```

### 3. Physics Validation Test

```bash
# Run a test to validate physics behavior (from FR-010: Physics simulation validation)
ros2 run digital_twin test_physics_accuracy.py
```

### 4. Unity Visualization Test

```bash
# In Unity editor:
# 1. Open the DigitalTwin scene
# 2. Press Play to start visualization
# 3. Check the Unity console for connection status
```

## Validation Against Acceptance Criteria

### Criteria: Robot behaves realistically under physics
```bash
# Run physics validation test (from SC-001: Realistic physics behavior)
ros2 run digital_twin validate_physics.py
# Verify mass, inertia, and friction properties match specifications
```

### Criteria: Sensor data streams correctly into ROS 2
```bash
# Monitor sensor topics for appropriate data (from SC-002: Sensor data accuracy)
ros2 topic echo /sensors/lidar/scan --field ranges
ros2 topic echo /sensors/imu/data --field orientation
# Verify data contains appropriate noise characteristics
```

### Additional Validation Tests

#### Unity Synchronization Test
```bash
# Monitor synchronization between Gazebo and Unity (from SC-003: Unity visualization performance)
ros2 run digital_twin test_sync_performance.py
# Verify Unity maintains 30+ FPS while synchronized with Gazebo
```

#### Real-time Performance Test
```bash
# Monitor simulation real-time factor (from SC-004: Real-time performance)
gz stats
# Verify simulation maintains close to 1.0x real-time performance
```

## Observability Features (from clarifications)

### 1. Logging
All nodes write logs to ROS 2's standard logging system:
```bash
# View logs for all nodes
ros2 param list
# Check node-specific logs
ros2 run digital_twin view_logs.py
```

### 2. Metrics
Monitor system performance metrics:
```bash
# View system metrics (from SC-005: ROS 2 standard logging)
ros2 run digital_twin show_metrics.py
```

## Running Tests

1. Execute unit tests for the digital twin:
   ```bash
   cd ~/simulation_ws
   colcon test --packages-select digital_twin
   colcon test-result --all
   ```

2. Verify sensor simulation integrity:
   ```bash
   # Run a sensor simulation test (from FR-003: Realistic sensor simulation)
   ros2 run digital_twin test_sensor_simulation.py
   ```

3. Validate Unity-Gazebo synchronization:
   ```bash
   # Test Unity-Gazebo synchronization (from FR-009: Synchronization protocol)
   ros2 run digital_twin test_sync_protocol.py
   ```

## Unity Visualization Integration

### 1. Synchronize with Gazebo Simulation (from FR-009: Synchronization protocol)

```bash
# In Unity editor:
# 1. Set up ROS-TCP-Connector to communicate with ROS 2
# 2. Configure clock synchronization via /clock topic
# 3. Test connection to ROS 2 topics
```

### 2. Verify Visualization Performance

```bash
# Monitor Unity FPS while connected to Gazebo
# Verify performance meets minimum 30 FPS requirement (from SC-003)
```

## Error Handling and Recovery (from clarifications)

### 1. Unity Disconnection Recovery (from FR-007: Graceful degradation)
```bash
# Test system response when Unity visualization disconnects
# Verify Gazebo continues running normally
python3 examples/test_unity_disconnect.py
```

## Troubleshooting

### Common Issues

1. **Simulation running slowly**
   - Verify GPU acceleration is working
   - Check available RAM (32GB+ recommended)
   - Reduce simulation complexity if needed

2. **Unity/Gazebo synchronization problems** (from FR-009: Synchronization protocol)
   - Check ROS-TCP-Connector settings
   - Verify both systems are on same time source via /clock
   - Adjust synchronization frequency if needed

3. **Sensor data quality issues**
   - Verify noise parameters are configured correctly
   - Check sensor update rates against physical counterparts
   - Validate sensor data ranges and characteristics

4. **Collision detection not working**
   - Verify collision meshes are properly defined
   - Check physics engine parameters
   - Validate joint limits and constraints

### Debug Commands

- View active nodes: `ros2 node list`
- Monitor topics: `ros2 topic list` and `ros2 topic echo <topic_name>`
- Check services: `ros2 service list`
- View system status: `ros2 lifecycle list`
- Gazebo simulation stats: `gz stats`

## Performance Optimization

### Unity Settings (from FR-008: GPU requirements)
- Quality Settings: Use "Very High" or "Ultra" preset with minimum 8GB VRAM
- Real-time Global Illumination: Disable for better performance
- Anti-aliasing: Use FXAA instead of MSAA

### Gazebo Settings
- Reduce physics update rate if performance is insufficient
- Reduce sensor update rates during development
- Use simpler collision meshes for complex models

## Academic Standards Compliance

This implementation follows the project constitution by:
- Using only verified Gazebo, Unity, and ROS 2 documentation and APIs (Accuracy through Primary Source Verification)
- Providing clear, structured explanations (Clarity for Academic Audience)
- Including sufficient detail for reproduction of the simulation environment (Reproducibility)
- Following established robotics simulation practices (Engineering Rigor)
- All technical claims are traceable to ROS 2, Gazebo, and Unity documentation (Traceability of Claims)
- All content is original (Zero-Tolerance Plagiarism Policy)

## References

- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- Gazebo Simulation Guide: http://gazebosim.org/
- Unity ROS Integration: https://github.com/Unity-Technologies/ROS-TCP-Connector
- "Gazebo: A 3D Multi-Robot Simulator" - Koenig & Howard

## Next Steps

1. Explore different simulation environments
2. Test with various humanoid behaviors
3. Integrate perception algorithms with simulated sensors
4. Validate controllers in simulation before physical deployment
5. Test multi-robot scenarios
6. Experiment with different environmental conditions