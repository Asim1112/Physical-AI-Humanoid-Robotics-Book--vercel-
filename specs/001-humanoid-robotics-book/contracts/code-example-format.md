# Code Example Format Standards: Physical AI & Humanoid Robotics Book

**Feature**: `001-humanoid-robotics-book` | **Date**: 2025-12-15 | **Input**: spec.md, research.md, data-model.md

## Summary

Standardized format and quality standards for all code examples in the Physical AI & Humanoid Robotics book. Ensures consistency, testability, and educational value across all modules and chapters.

## Code Example Structure

### Basic Format
Each code example follows this structure:

````
```language
# Code example with proper syntax highlighting
# Descriptive comments explaining key concepts
# Expected output or behavior as comments
```
````

### Required Components

#### 1. Language Specification
- **Requirement**: Always specify the programming language for syntax highlighting
- **Examples**: `python`, `bash`, `yaml`, `json`, `xml`, `javascript`
- **Purpose**: Enables proper syntax highlighting and editor support

#### 2. Descriptive Comments
- **Requirement**: Include inline comments explaining complex logic
- **Style**: Clear, concise explanations of what each section does
- **Focus**: Emphasize concepts relevant to Physical AI and humanoid robotics

#### 3. Context Setting
- **Requirement**: Brief explanation of what the code demonstrates
- **Placement**: Before or after the code block
- **Content**: Connection to broader concepts and practical applications

## Code Example Categories

### 1. Conceptual Examples
- **Purpose**: Illustrate fundamental concepts without complex implementation
- **Length**: 5-20 lines of code
- **Focus**: Clarity over completeness
- **Example**:
```python
# Simple ROS 2 publisher example
import rclpy
from std_msgs.msg import String

def create_publisher():
    # Create a publisher for string messages
    publisher = node.create_publisher(String, 'topic_name', 10)
    return publisher
```

### 2. Implementation Examples
- **Purpose**: Show practical implementation of concepts
- **Length**: 20-50 lines of code
- **Focus**: Realistic scenarios with proper error handling
- **Example**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

    def scan_callback(self, msg):
        # Process LiDAR scan data for obstacle detection
        min_distance = min(msg.ranges)
        if min_distance < 1.0:  # 1 meter threshold
            self.get_logger().info('Obstacle detected!')
```

### 3. Integration Examples
- **Purpose**: Demonstrate how multiple concepts work together
- **Length**: 50+ lines of code
- **Focus**: System-level understanding and practical application
- **Example**:
```python
# Integration of perception, planning, and control
class NavigationSystem:
    def __init__(self):
        # Initialize perception, planning, and control components
        self.perception = PerceptionModule()
        self.planner = PathPlanner()
        self.controller = MotionController()

    def navigate_to_goal(self, goal_pose):
        # Integrate all modules for autonomous navigation
        environment_map = self.perception.build_environment_map()
        path = self.planner.plan_path(environment_map, goal_pose)
        self.controller.follow_path(path)
```

## Quality Standards

### 1. Runnability
- **Requirement**: All examples must be testable in simulation environment
- **Validation**: Code should run with minimal setup in Ubuntu 22.04 + ROS 2 Humble
- **Dependencies**: Clearly specify required packages and setup steps
- **Testing**: Include expected output or behavior verification

### 2. Educational Value
- **Requirement**: Each example teaches a specific concept clearly
- **Progression**: Build from simple to complex within each chapter
- **Relevance**: Connect to humanoid robotics applications
- **Clarity**: Prioritize understanding over optimization

### 3. Technical Accuracy
- **Requirement**: All code must follow official ROS 2, NVIDIA Isaac, and other framework best practices
- **Verification**: Reference official documentation for API usage
- **Standards**: Follow PEP 8 for Python, official style guides for other languages
- **Updates**: Align with LTS versions (ROS 2 Humble, etc.)

## Documentation Standards

### 1. Inline Comments
- **Style**: Use `#` for Python comments, appropriate style for other languages
- **Content**: Explain *why* not just *what* the code does
- **Detail**: Sufficient explanation for beginner-to-intermediate audience
- **Brevity**: Concise but comprehensive explanations

### 2. Expected Output
- **Requirement**: Include expected output as comments when applicable
- **Format**: Use `# Expected: ...` or `# Output: ...` pattern
- **Verification**: Show how to verify the code is working correctly
- **Troubleshooting**: Mention common issues and solutions

### 3. Error Handling
- **Requirement**: Include appropriate error handling for educational examples
- **Focus**: Show how to handle common failure scenarios
- **Clarity**: Explain why error handling is important in robotics context
- **Balance**: Educational value over production-level complexity

## Robotics-Specific Considerations

### 1. Safety and Validation
- **Requirement**: Include safety checks where relevant to robotics applications
- **Examples**: Bounds checking, validation of robot states, safety limits
- **Emphasis**: Safety-first approach in all code examples
- **Context**: Explain safety implications in humanoid robotics

### 2. Real-World Constraints
- **Requirement**: Account for real-world limitations in simulation examples
- **Examples**: Sensor noise, computational limits, communication delays
- **Modeling**: Show how to handle imperfect real-world conditions
- **Robustness**: Demonstrate robust implementation patterns

### 3. Hardware Abstraction
- **Requirement**: Write code that works in simulation but indicates hardware integration points
- **Approach**: Use interfaces that can be swapped for real hardware
- **Documentation**: Clearly mark simulation-specific vs. hardware-agnostic code
- **Transition**: Show path from simulation to real hardware (conceptual)

## Format Examples

### Python Example with Full Documentation
```python
#!/usr/bin/env python3
# Example: Simple ROS 2 action client for humanoid robot navigation
# This demonstrates how to send navigation goals to a humanoid robot

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose  # Navigation action for humanoid
import time

class HumanoidNavigator(Node):
    """
    Simple navigation client that sends goals to a humanoid robot's navigation system.
    This example shows how to interact with ROS 2 actions for complex robot behaviors.
    """

    def __init__(self):
        super().__init__('humanoid_navigator')
        # Create action client for navigation
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'  # Action name for humanoid navigation
        )

    def send_goal(self, x, y, theta):
        """
        Send navigation goal to humanoid robot.

        Args:
            x (float): Target X position in map frame
            y (float): Target Y position in map frame
            theta (float): Target orientation in radians
        """
        # Wait for action server to be available
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # Set orientation using quaternion (simplified)
        goal_msg.pose.pose.orientation.z = theta  # Simplified for example

        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Expected: Goal accepted by navigation server
        # The humanoid robot will begin moving toward the specified pose
        return self._send_goal_future

    def feedback_callback(self, feedback_msg):
        """Handle feedback during navigation execution."""
        feedback = feedback_msg.feedback
        # Expected: Periodic feedback about navigation progress
        self.get_logger().info(f'Navigating... Remaining distance: {feedback.distance_remaining:.2f}m')

def main():
    """Main function to demonstrate navigation client usage."""
    rclpy.init()
    navigator = HumanoidNavigator()

    # Send a navigation goal (example coordinates)
    future = navigator.send_goal(1.0, 2.0, 0.0)  # Move to (1,2) with 0 orientation

    # Spin until goal completed
    rclpy.spin_until_future_complete(navigator, future)

    # Expected: Navigation completes successfully or fails with clear error
    print("Navigation goal completed!")

if __name__ == '__main__':
    main()
```

### Configuration Example
```yaml
# Example: ROS 2 launch file for humanoid robot simulation
# This shows how to configure multiple nodes for humanoid robot simulation

launch:
  # Launch Gazebo simulation with humanoid model
  - executable: ros2
    arguments: ["launch", "gazebo_ros", "empty_world.launch.py"]

  # Launch humanoid robot model in simulation
  - executable: ros2
    arguments: ["run", "robot_state_publisher", "robot_state_publisher",
                "--ros-args", "--params-file", "humanoid_params.yaml"]

  # Launch navigation stack
  - executable: ros2
    arguments: ["launch", "nav2_bringup", "navigation_launch.py"]

  # Launch perception stack
  - executable: ros2
    arguments: ["launch", "isaac_ros_apriltag", "apriltag.launch.py"]
```

## Testing and Validation

### 1. Simulation Environment
- **Requirement**: All examples testable in Ubuntu 22.04 + ROS 2 Humble environment
- **Docker**: Provide Docker configuration for consistent testing environment
- **Dependencies**: List all required packages and setup steps
- **Verification**: Include commands to verify example functionality

### 2. Incremental Testing
- **Approach**: Build examples that can be tested incrementally
- **Validation**: Show how to verify each component works individually
- **Integration**: Demonstrate how components work together
- **Debugging**: Include debugging tips and common issues

### 3. Error Scenarios
- **Requirement**: Show how to handle common error conditions
- **Examples**: Invalid inputs, missing dependencies, network issues
- **Recovery**: Demonstrate error recovery patterns
- **Logging**: Include appropriate logging for debugging

## Maintenance and Updates

### 1. Version Alignment
- **Requirement**: Align with LTS versions of frameworks (ROS 2 Humble, etc.)
- **Update Process**: Clear process for updating examples with new versions
- **Backward Compatibility**: Maintain examples for previous versions when possible
- **Change Tracking**: Document changes when updating examples

### 2. Community Contribution
- **Format**: Examples should be easily modifiable by community
- **Standards**: Follow established patterns for consistency
- **Documentation**: Clear documentation for contributing new examples
- **Review Process**: Quality review process for contributed examples

## Compliance with Constitution

### Adherence to Principles
- **Precision & Depth**: Examples provide comprehensive, verifiable code
- **Consistency**: Standardized format across all code examples
- **Source-Awareness**: All API usage references official documentation
- **Modularity**: Examples work independently while connecting to broader concepts
- **Pedagogical Clarity**: Progressive complexity from beginner to advanced
- **Spec-First Execution**: Format follows approved specifications
- **Code Quality**: All examples are runnable and tested

### Quality Gates
- [ ] All examples include proper language specification
- [ ] Examples include descriptive comments explaining key concepts
- [ ] Code follows official style guides and best practices
- [ ] Examples are testable in simulation environment
- [ ] Expected output or behavior is documented
- [ ] Error handling is demonstrated where appropriate
- [ ] Examples connect to humanoid robotics applications
- [ ] Educational value prioritized over optimization