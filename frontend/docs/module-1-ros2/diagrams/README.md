# Module 1: Robotic Nervous System (ROS 2) - Architecture Diagrams

This document contains text-described diagrams for the key architecture concepts covered in Module 1 of the Physical AI & Humanoid Robotics Book. These diagrams help visualize the communication patterns, system architecture, and robot modeling concepts.

## Directory Structure

```
diagrams/
├── README.md                    # This file
├── ros2_architecture.txt        # ROS 2 architecture overview
├── node_topic_pattern.txt       # Node-topic communication pattern
├── service_pattern.txt          # Service request-response pattern
├── action_pattern.txt           # Action goal-feedback-result pattern
├── urdf_structure.txt           # URDF robot description structure
├── system_integration.txt       # Complete system integration architecture
└── safety_architecture.txt      # Safety system architecture
```

## Diagrams Overview

### 1. ROS 2 Architecture Overview
- Shows the relationship between nodes, topics, services, and actions
- Illustrates the middleware layer and communication infrastructure

### 2. Node-Topic Communication Pattern
- Demonstrates publisher-subscriber pattern
- Shows message flow between components

### 3. Service Communication Pattern
- Illustrates request-response communication
- Shows synchronous communication flow

### 4. Action Communication Pattern
- Demonstrates goal-feedback-result communication
- Shows asynchronous communication with progress tracking

### 5. URDF Robot Structure
- Visualizes the link-joint hierarchy of robot models
- Shows visual, collision, and inertial properties

### 6. System Integration Architecture
- Shows how all components work together in a complete system
- Illustrates the behavior manager coordinating subsystems

### 7. Safety Architecture
- Shows safety layers and protection mechanisms
- Illustrates safety monitoring and emergency procedures

Each text-described diagram uses ASCII art and descriptive text to clearly explain the architectural concepts relevant to humanoid robotics.