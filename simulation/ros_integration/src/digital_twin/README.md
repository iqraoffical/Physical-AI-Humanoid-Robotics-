# Digital Twin Package

## Overview

This package implements the digital twin environment for the humanoid robot, featuring:
- Gazebo simulation with realistic physics and sensors
- Unity visualization for enhanced representation
- ROS 2 integration for seamless communication

## Features

- Physics-accurate simulation of 24+ DOF humanoid robot
- Realistic sensor simulation (LiDAR, depth camera, IMU)
- Unity visualization synchronized with Gazebo simulation
- ROS 2 integration with standardized message types
- Security and observability features
- 30+ FPS Unity visualization with real-time synchronization

## Architecture

The digital twin consists of several integrated components:

### Gazebo Simulation Environment
- Physics engine with realistic gravity, collisions, and dynamics
- 24+ degree-of-freedom humanoid model with accurate joint limits
- Realistic sensor simulation with configurable noise models
- ROS 2 integration via gazebo_ros_pkgs

### Unity Visualization
- Real-time rendering of robot and environment
- Synchronized with Gazebo simulation state
- Enhanced visualization capabilities for human-robot interaction scenarios
- ROS-TCP-Connector for ROS 2 communication

### ROS 2 Integration Layer
- Standard ROS 2 message types for sensor and state data
- TF2 coordinate transformation
- Real-time clock synchronization
- Security-enhanced communication

## Installation

### Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo Garden/Fortress
- Unity 3D (2022.3 LTS or newer)
- RTX 4070 Ti or higher GPU recommended
- 32GB+ RAM minimum

### Setup

1. Install ROS 2 Humble:
```bash
# Follow official ROS 2 Humble installation guide
sudo apt update && sudo apt install ros-humble-desktop