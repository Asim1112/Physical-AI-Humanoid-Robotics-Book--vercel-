#!/usr/bin/env python3
# Example: Parameter management demo for humanoid robot
# Demonstrates ROS 2 parameter system for configuration management

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray
import json


class ParameterDemoNode(Node):
    """
    A node that demonstrates ROS 2 parameter management for robot configuration.
    Shows how to declare, use, and dynamically reconfigure parameters.
    """

    def __init__(self):
        super().__init__('parameter_demo')

        # Declare parameters with different types and defaults
        self.declare_parameter('robot.name', 'simple_humanoid')
        self.declare_parameter('robot.model', 'v1.0')
        self.declare_parameter('control.frequency', 100)
        self.declare_parameter('safety.max_velocity', 2.0)
        self.declare_parameter('safety.max_acceleration', 5.0)
        self.declare_parameter('debug.enabled', False)
        self.declare_parameter('debug.verbosity', 1)
        self.declare_parameter('modules.enabled', ['perception', 'control', 'planning'])

        # PID controller gains as nested parameters
        self.declare_parameter('pid.left_leg.kp', 10.0)
        self.declare_parameter('pid.left_leg.ki', 0.1)
        self.declare_parameter('pid.left_leg.kd', 0.5)
        self.declare_parameter('pid.right_leg.kp', 10.0)
        self.declare_parameter('pid.right_leg.ki', 0.1)
        self.declare_parameter('pid.right_leg.kd', 0.5)

        # Initialize parameters as instance variables
        self.update_parameters()

        # Create a publisher to simulate control commands
        self.control_pub = self.create_publisher(Float64MultiArray, '/joint_group_position_controller/commands', 10)

        # Create timer for periodic operations using parameters
        timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(timer_period, self.control_loop)

        # Add callback for parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Parameter Demo Node initialized for {self.robot_name}')
        self.get_logger().info(f'Control frequency: {self.control_frequency}Hz')

    def update_parameters(self):
        """Update instance variables from parameter values."""
        self.robot_name = self.get_parameter('robot.name').value
        self.robot_model = self.get_parameter('robot.model').value
        self.control_frequency = self.get_parameter('control.frequency').value
        self.max_velocity = self.get_parameter('safety.max_velocity').value
        self.max_acceleration = self.get_parameter('safety.max_acceleration').value
        self.debug_enabled = self.get_parameter('debug.enabled').value
        self.debug_verbosity = self.get_parameter('debug.verbosity').value
        self.enabled_modules = self.get_parameter('modules.enabled').value

        # Get PID parameters
        self.pid_gains = {
            'left_leg': {
                'kp': self.get_parameter('pid.left_leg.kp').value,
                'ki': self.get_parameter('pid.left_leg.ki').value,
                'kd': self.get_parameter('pid.left_leg.kd').value
            },
            'right_leg': {
                'kp': self.get_parameter('pid.right_leg.kp').value,
                'ki': self.get_parameter('pid.right_leg.ki').value,
                'kd': self.get_parameter('pid.right_leg.kd').value
            }
        }

    def parameter_callback(self, params):
        """Callback function for parameter changes."""
        from rclpy.parameter_service import SetParametersResult

        successful = True
        reason = ''
        need_parameter_update = False

        for param in params:
            # Log the parameter change
            self.get_logger().info(f'Parameter {param.name} changed from {self.get_parameter(param.name).value} to {param.value}')

            # Some parameters might require immediate action
            if param.name == 'control.frequency':
                # Update timer period when control frequency changes
                timer_period = 1.0 / param.value
                self.control_timer.timer_period_ns = int(timer_period * 1e9)
                self.get_logger().info(f'Control timer updated to {param.value}Hz')

            # Mark that we need to update our local variables
            need_parameter_update = True

        # Update local parameter values after all changes are processed
        if need_parameter_update:
            self.update_parameters()

        return SetParametersResult(successful=successful, reason=reason)

    def control_loop(self):
        """Main control loop that uses parameter values."""
        if self.debug_enabled and self.debug_verbosity >= 2:
            self.get_logger().debug(f'Control loop running at {self.control_frequency}Hz')

        # Example control logic that uses parameters
        # Generate some control commands based on PID gains
        commands = self.generate_control_commands()

        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands
        self.control_pub.publish(cmd_msg)

    def generate_control_commands(self):
        """Generate control commands using parameterized values."""
        # In a real robot, this would implement actual control logic
        # For this demo, we'll just generate example commands

        # Use PID gains to calculate example commands
        left_cmd = self.pid_gains['left_leg']['kp'] * 0.1  # Simplified calculation
        right_cmd = self.pid_gains['right_leg']['kp'] * 0.1

        # Apply velocity limits
        if abs(left_cmd) > self.max_velocity:
            left_cmd = self.max_velocity if left_cmd > 0 else -self.max_velocity
        if abs(right_cmd) > self.max_velocity:
            right_cmd = self.max_velocity if right_cmd > 0 else -self.max_velocity

        commands = [left_cmd, right_cmd]

        if self.debug_enabled and self.debug_verbosity >= 3:
            self.get_logger().debug(f'Generated commands: {commands}')

        return commands

    def get_system_config(self):
        """Get current system configuration as a dictionary."""
        config = {
            'robot': {
                'name': self.robot_name,
                'model': self.robot_model
            },
            'control': {
                'frequency': self.control_frequency
            },
            'safety': {
                'max_velocity': self.max_velocity,
                'max_acceleration': self.max_acceleration
            },
            'debug': {
                'enabled': self.debug_enabled,
                'verbosity': self.debug_verbosity
            },
            'modules': {
                'enabled': self.enabled_modules
            },
            'pid': self.pid_gains
        }
        return config

    def print_config(self):
        """Print current configuration to console."""
        config = self.get_system_config()
        print("\nCurrent System Configuration:")
        print("=" * 40)
        print(json.dumps(config, indent=2))
        print("=" * 40)


def main(args=None):
    """Main function to run the parameter demo."""
    rclpy.init(args=args)

    node = ParameterDemoNode()

    try:
        # Print initial configuration
        node.print_config()

        # Spin the node
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()