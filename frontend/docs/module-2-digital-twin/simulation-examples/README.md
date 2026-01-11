# Module 2: Digital Twin (Gazebo & Unity) - Simulation Examples

This directory contains runnable simulation examples that demonstrate the concepts covered in Module 2 of the Physical AI & Humanoid Robotics Book. Each example is designed to be educational and can be run independently to understand different aspects of simulation for humanoid robotics.

## Directory Structure

```
simulation-examples/
├── README.md                    # This file
├── gazebo_basics/              # Basic Gazebo simulation examples
│   ├── simple_robot.world      # Basic world file
│   ├── robot_spawn.launch.py   # Launch file to spawn robot
│   └── controller_config.yaml  # Controller configuration
├── unity_integration/          # Unity-ROS integration examples
│   ├── unity_scene.unity       # Unity scene file (reference)
│   ├── unity_ros_bridge.py     # Python bridge script
│   └── message_handlers.py     # Message processing scripts
├── physics_validation/         # Physics simulation validation
│   ├── validation_node.py      # Physics validation node
│   ├── contact_analyzer.py     # Contact analysis tools
│   └── fidelity_checker.py     # Simulation fidelity validation
├── sensor_simulation/          # Sensor simulation examples
│   ├── camera_sim.py           # Camera sensor simulation
│   ├── imu_sim.py              # IMU sensor simulation
│   └── lidar_sim.py            # LiDAR sensor simulation
└── integration_examples/       # Complete integration examples
    ├── sim_control_interface.py # Simulation control interface
    ├── ros_gazebo_bridge.py     # ROS-Gazebo bridge example
    └── multi_robot_scenario.py  # Multi-robot simulation example
```

## Running Examples

### Prerequisites

Before running these examples, ensure you have:

1. ROS 2 Humble Hawksbill installed
2. Gazebo Garden or Fortress installed
3. Python 3.8 or higher
4. A ROS 2 workspace set up
5. Unity (if running Unity-specific examples)

### Setup

1. Source your ROS 2 installation:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Navigate to your workspace and source it:
   ```bash
   cd ~/robot_ws
   source install/setup.bash
   ```

### Running Gazebo Examples

To run a basic Gazebo simulation:
```bash
# Launch the example world
ros2 launch gazebo_ros gazebo.launch.py world:=path/to/world_file.sdf

# Or use the example launch file
ros2 launch robot_sim robot_spawn.launch.py
```

### Running Unity Examples

Unity examples require additional setup:
1. Open the Unity project
2. Configure the ROS IP address and port
3. Build and run the Unity application
4. Launch the ROS bridge node:
```bash
ros2 run robot_sim unity_ros_bridge.py
```

## Examples Overview

### Gazebo Basics
- `simple_robot.world`: Basic Gazebo world with humanoid robot
- `robot_spawn.launch.py`: Launch file demonstrating robot spawning
- `controller_config.yaml`: Configuration for ROS 2 Control integration

### Unity Integration
- `unity_ros_bridge.py`: Bidirectional communication between Unity and ROS
- `message_handlers.py`: Processing of Unity-specific messages

### Physics Validation
- `validation_node.py`: Validates physics simulation accuracy
- `contact_analyzer.py`: Analyzes contact forces and stability
- `fidelity_checker.py`: Validates simulation fidelity against real robot

### Sensor Simulation
- `camera_sim.py`: Simulates realistic camera sensors
- `imu_sim.py`: Simulates IMU with realistic noise characteristics
- `lidar_sim.py`: Simulates LiDAR sensors with appropriate parameters

### Integration Examples
- `sim_control_interface.py`: Complete interface between simulation and control
- `ros_gazebo_bridge.py`: Full ROS-Gazebo integration example
- `multi_robot_scenario.py`: Multi-robot simulation coordination

## Educational Goals

These examples are designed to help you understand:

1. **Gazebo Simulation**: Creating and configuring simulation environments
2. **Unity Integration**: Connecting Unity visualization with ROS systems
3. **Physics Validation**: Ensuring simulation accuracy and stability
4. **Sensor Simulation**: Creating realistic sensor models
5. **System Integration**: Combining all components into cohesive systems

Each example includes detailed comments explaining the code and its purpose in the context of humanoid robotics simulation.