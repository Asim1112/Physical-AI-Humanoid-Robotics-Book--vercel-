# Module 2: Digital Twin (Gazebo & Unity) - Architecture Diagrams

This document contains text-described diagrams for the key simulation architecture concepts covered in Module 2 of the Physical AI & Humanoid Robotics Book. These diagrams help visualize the simulation integration, physics modeling, and system architecture concepts.

## Directory Structure

```
diagrams/
├── README.md                    # This file
├── simulation_architecture.txt  # Overall simulation architecture
├── gazebo_ros_integration.txt   # Gazebo-ROS integration patterns
├── unity_ros_bridge.txt         # Unity-ROS bridge architecture
├── physics_simulation.txt       # Physics simulation architecture
├── sensor_simulation.txt        # Sensor simulation patterns
└── multi_robot_simulation.txt   # Multi-robot simulation architecture
```

## Diagrams Overview

### 1. Simulation Architecture Overview
- Shows the relationship between simulation environments and ROS 2
- Illustrates data flow between components
- Highlights key integration points

### 2. Gazebo-ROS Integration
- Details the plugin architecture for Gazebo-ROS communication
- Shows sensor and actuator plugin patterns
- Illustrates the ROS 2 Control integration

### 3. Unity-ROS Bridge
- Explains the network-based communication between Unity and ROS
- Shows message serialization and deserialization patterns
- Details visualization and interaction flows

### 4. Physics Simulation Architecture
- Illustrates physics engine integration and configuration
- Shows contact simulation and force application
- Details stability and performance considerations

### 5. Sensor Simulation Patterns
- Shows how different sensor types are simulated
- Illustrates noise modeling and data processing
- Details sensor fusion in simulation

### 6. Multi-Robot Simulation
- Shows coordination between multiple simulated robots
- Illustrates shared simulation environments
- Details communication patterns

Each text-described diagram uses ASCII art and descriptive text to clearly explain the architectural concepts relevant to humanoid robotics simulation.