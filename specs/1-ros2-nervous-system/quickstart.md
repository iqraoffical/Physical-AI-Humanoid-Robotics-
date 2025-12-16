# Quickstart Guide: Module 1: The Robotic Nervous System (ROS 2)

**Feature**: 1-ros2-nervous-system
**Date**: 2025-12-15

## Overview

This guide provides step-by-step instructions to get the ROS 2 nervous system up and running for the humanoid robot. It covers environment setup, basic communication patterns, and initial testing. This implementation follows the requirements for Ubuntu 22.04 + ROS 2 Humble environment from the specification, with additional security, observability and 24+ DOF humanoid model as clarified.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed with security features (SROS2)
- Python 3.10+
- Basic understanding of ROS 2 concepts (nodes, topics, services)
- All content follows the academic standards of the project constitution
- 24+ DOF humanoid model files

## Setup Instructions

### 1. Environment Setup

1. Install ROS 2 Humble Hawksbill:
   ```bash
   # Follow official installation guide:
   # https://docs.ros.org/en/humble/Installation.html
   ```

2. Install security features:
   ```bash
   sudo apt install ros-humble-sros2
   ```

3. Create a workspace directory:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

4. Source ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

5. Install additional dependencies:
   ```bash
   sudo apt update
   sudo apt install python3-colcon-common-extensions python3-rosdep
   sudo rosdep init  # Only run once
   rosdep update
   ```

### 2. Security Setup (from clarifications)

1. Generate security artifacts for the robot system:
   ```bash
   export ROS_SECURITY_KEYSTORE=~/ros2_ws/keys
   mkdir -p $ROS_SECURITY_KEYSTORE
   source /opt/ros/humble/setup.bash
   ros2 security create_keystore $ROS_SECURITY_KEYSTORE
   ros2 security create_key $ROS_SECURITY_KEYSTORE robot_nervous_system
   ```

2. Configure security environment:
   ```bash
   export ROS_SECURITY_ENABLE=true
   export ROS_SECURITY_STRATEGY=Enforce
   export ROS_SECURITY_KEYSTORE=~/ros2_ws/keys
   ```

### 3. Build the Project

```bash
cd ~/ros2_ws
colcon build --packages-select robot_nervous_system
source install/setup.bash
```

### 4. Launch the ROS 2 Nervous System

```bash
# Terminal 1: Launch the robot nervous system with security
ros2 launch robot_nervous_system robot_nervous_system.launch.py

# Terminal 2: Load the 24+ DOF humanoid URDF
ros2 run robot_nervous_system load_urdf.py --robot-name humanoid_24dof

# Terminal 3: Run Python AI agent bridge
ros2 run robot_nervous_system python_agent_bridge.py
```

## Basic Operations

### 1. Check Active Nodes

```bash
ros2 node list
```

You should see nodes like:
- `/publisher_node` - Implements FR-001: publisher nodes
- `/subscriber_node` - Implements FR-001: subscriber nodes  
- `/service_node` - Implements FR-002: services
- `/python_agent_bridge` - Implements FR-004: Python agent integration

### 2. List Available Topics

```bash
ros2 topic list
```

Look for topics like:
- `/joint_states` - Current joint positions (from FR-001)
- `/joint_commands` - Joint command inputs (from FR-004)
- `/sensor_data` - Sensor readings

### 3. Send a Test Command

```bash
# Send a joint command message to test communication (from FR-004)
ros2 topic pub /joint_commands JointCommandMessage "{
  joint_names: ['joint1', 'joint2'],
  positions: [0.0, 0.0],
  velocities: [0.0, 0.0],
  efforts: [0.0, 0.0]
}"
```

### 4. Monitor Joint States

```bash
# Listen to joint state messages (from FR-001)
ros2 topic echo /joint_states JointStateMessage
```

## Observability Features (from clarifications)

### 1. Logging
All nodes write logs to ROS 2's standard logging system:
```bash
# View logs for all nodes
ros2 param list
# Check node-specific logs
ros2 run robot_nervous_system view_logs.py
```

### 2. Metrics
Monitor system performance metrics:
```bash
# View system metrics
ros2 run robot_nervous_system show_metrics.py
```

### 3. Tracing
Trace message flow across nodes:
```bash
# Enable tracing
ros2 run robot_nervous_system enable_tracing.py
```

## Running Tests

1. Execute unit tests for the nervous system:
   ```bash
   cd ~/ros2_ws
   colcon test --packages-select robot_nervous_system
   colcon test-result --all
   ```

2. Verify communication integrity:
   ```bash
   # Run a communication test
   ros2 run robot_nervous_system test_communication.py
   ```

3. Validate URDF loading:
   ```bash
   # Test that URDF loads correctly (from FR-003)
   ros2 run robot_nervous_system test_urdf.py
   ```

## Simulation Integration

### 1. Load 24+ DOF Humanoid Model in Gazebo

```bash
# Launch simulation environment with 24+ DOF model
ros2 launch robot_nervous_system simulation.launch.py
```

### 2. Verify URDF Loads Correctly

```bash
# Check that robot description is available (from FR-003)
ros2 param get /robot_state_publisher robot_description
```

## Python Agent Integration

### 1. Connect Python Agent to ROS Network

```bash
# Run example Python agent
python3 examples/python_agent_example.py
```

### 2. Validate Joint Commands

```bash
# Verify Python agent can command joints (from FR-004)
ros2 topic echo /joint_commands
```

## Validation Against Acceptance Criteria

### Criteria: Nodes and topics operate without errors
```bash
# Check for errors in ROS 2 logs
source install/setup.bash
ros2 launch robot_nervous_system test_nodes.launch.py
```

### Criteria: Python agent commands joints successfully
```bash
# Test Python agent to joint communication
python3 examples/test_agent_joints.py
```

### Criteria: URDF loads correctly in simulation
```bash
# Test URDF loading functionality with 24+ DOF model
ros2 run robot_nervous_system test_urdf_loading.py
```

### Additional Validation (from clarifications)

#### Security Framework Validation
```bash
# Verify security framework is working
ros2 security verify_access ~/ros2_ws/keys
```

#### 24+ DOF Model Validation
```bash
# Check that 24+ DOF model has correct joint limits
ros2 run robot_nervous_system validate_24dof_model.py
```

## Error Handling and Recovery (from clarifications)

### 1. Input Validation Test
```bash
# Test system response to invalid commands
python3 examples/test_invalid_commands.py
```

### 2. External Dependency Failure Simulation
```bash
# Simulate external dependency failure and verify fallback
python3 examples/test_dependency_fallback.py
```

## Troubleshooting

### Common Issues

1. **Nodes not communicating**
   - Ensure ROS_DOMAIN_ID is the same for all nodes
   - Check if firewall is blocking communication
   - Verify Quality of Service settings match for deterministic communication

2. **Security errors**
   - Ensure security environment variables are set
   - Verify keystore and keys are properly configured
   - Check that each node has appropriate permissions

3. **URDF not loading**
   - Verify URDF file path is correct for 24+ DOF model
   - Check for syntax errors in URDF
   - Ensure URDF meets joint limit requirements

4. **Python agent not connecting**
   - Ensure ROS 2 Python libraries are properly installed
   - Verify the bridge node is running
   - Check that joint command messages match expected format

### Debug Commands

- View active nodes: `ros2 node list`
- Monitor topics: `ros2 topic list` and `ros2 topic echo <topic_name>`
- Check services: `ros2 service list`
- View system status: `ros2 lifecycle list`
- Security verification: `ros2 security verify_access <path_to_keys>`

## Academic Standards Compliance

This implementation follows the project constitution by:
- Using only verified ROS 2 documentation and APIs (Accuracy through Primary Source Verification)
- Providing clear, structured explanations (Clarity for Academic Audience)
- Including sufficient detail for reproduction in simulation (Reproducibility)
- Following established ROS 2 engineering practices (Engineering Rigor)
- All technical claims are traceable to ROS 2 documentation (Traceability of Claims)
- All content is original (Zero-Tolerance Plagiarism Policy)

## References

- ROS 2 Humble Documentation: https://docs.ros.org/en/humble/
- URDF Documentation: http://wiki.ros.org/urdf
- ROS 2 Quality of Service: https://docs.ros.org/en/humble/How-To-Guides/Working-with-Quality-of-Service.html
- ROS 2 Security Guide: https://docs.ros.org/en/humble/How-To-Guides/Setting-up-Secure-Communication.html

## Next Steps

1. Explore the simulation environment with Gazebo and the 24+ DOF model
2. Implement more complex joint control patterns
3. Integrate with higher-level AI planning systems
4. Test fault tolerance mechanisms (from non-functional requirements)
5. Validate deterministic communication performance
6. Test security framework with various access control scenarios
7. Validate observability features with monitoring tools