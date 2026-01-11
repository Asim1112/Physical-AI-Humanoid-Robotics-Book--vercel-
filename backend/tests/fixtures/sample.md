# Module 1: Robotic Nervous System (ROS 2) - Code Examples

This directory contains runnable Python code examples that demonstrate the concepts covered in Module 1 of the Physical AI & Humanoid Robotics Book. Each example is designed to be educational and can be run independently to understand different aspects of ROS 2.

## Directory Structure

```
code-examples/
├── README.md                    # This file
├── basic_nodes/                 # Basic node examples
│   ├── simple_publisher.py      # Simple topic publisher
│   ├── simple_subscriber.py     # Simple topic subscriber
│   └── publisher_subscriber.py  # Combined publisher-subscriber node
├── advanced_communication/      # Services and actions examples
│   ├── simple_service_server.py # Basic service server
│   ├── simple_service_client.py # Basic service client
│   ├── simple_action_server.py  # Basic action server
│   └── simple_action_client.py  # Basic action client
├── robot_modeling/              # URDF and robot modeling examples
│   ├── simple_robot.urdf        # Simple URDF model
│   ├── validate_urdf.py         # URDF validation example
│   └── kinematics_demo.py       # Basic kinematics example
└── system_integration/          # Complete system examples
    ├── behavior_manager.py      # High-level behavior manager
    ├── launch_example.py        # Launch file example
    └── parameter_demo.py        # Parameter management example
```

## Running Examples

### Prerequisites

Before running these examples, ensure you have:

1. ROS 2 Humble Hawksbill installed
2. Python 3.8 or higher
3. A ROS 2 workspace set up

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

### Running Individual Examples

To run a specific example:
```bash
python3 path/to/example.py
```

For nodes that need to communicate with each other, run them in separate terminals after sourcing ROS 2.

## Examples Overview

### Basic Nodes
- `simple_publisher.py`: Demonstrates basic topic publishing
- `simple_subscriber.py`: Demonstrates basic topic subscription
- `publisher_subscriber.py`: Combines publishing and subscription in one node

### Advanced Communication
- `simple_service_server.py`: Basic service server implementation
- `simple_service_client.py`: Basic service client implementation
- `simple_action_server.py`: Basic action server with feedback
- `simple_action_client.py`: Basic action client with goal tracking

### Robot Modeling
- `simple_robot.urdf`: Basic URDF model of a simple robot
- `validate_urdf.py`: Example of loading and validating URDF models
- `kinematics_demo.py`: Basic kinematics calculations

### System Integration
- `behavior_manager.py`: Complete example of a behavior manager node
- `parameter_demo.py`: Example of parameter management in ROS 2

## Educational Goals

These examples are designed to help you understand:

1. **ROS 2 Fundamentals**: Nodes, topics, publishers, and subscribers
2. **Advanced Communication**: Services for request-response and actions for long-running tasks
3. **Robot Modeling**: URDF for robot description and validation
4. **System Integration**: Combining multiple components into a cohesive system
5. **Best Practices**: Proper error handling, parameter management, and safety considerations

Each example includes detailed comments explaining the code and its purpose in the context of humanoid robotics.