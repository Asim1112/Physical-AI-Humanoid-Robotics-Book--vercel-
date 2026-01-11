#!/usr/bin/env python3
# Example: Behavior manager for humanoid robot system integration
# Demonstrates combining multiple ROS 2 concepts into a complete system

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import time
import threading
from enum import Enum


class RobotBehavior(Enum):
    IDLE = "idle"
    WALKING = "walking"
    MANIPULATING = "manipulating"
    CALIBRATING = "calibrating"
    EMERGENCY_STOP = "emergency_stop"


class BehaviorManager(Node):
    """
    A comprehensive behavior manager that integrates multiple ROS 2 concepts.
    Demonstrates system integration by combining topics, services, actions, and parameters.
    """

    def __init__(self):
        super().__init__('behavior_manager')

        # Declare parameters with defaults
        self.declare_parameter('control_frequency', 50)
        self.declare_parameter('safety.max_tilt_angle', 0.5)
        self.declare_parameter('safety.max_joint_velocity', 5.0)
        self.declare_parameter('debug_mode', False)

        # Get parameter values
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_tilt_angle = self.get_parameter('safety.max_tilt_angle').value
        self.max_joint_velocity = self.get_parameter('safety.max_joint_velocity').value
        self.debug_mode = self.get_parameter('debug_mode').value

        # Initialize state variables
        self.current_behavior = RobotBehavior.IDLE
        self.safety_engaged = False
        self.joint_states = None
        self.imu_data = None

        # Create subscribers for sensor data
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publishers for commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot_status', 10)

        # Create service servers
        self.enable_motors_srv = self.create_service(
            SetBool,
            'enable_robot_motors',
            self.enable_motors_callback
        )

        # Create timer for behavior updates
        timer_period = 1.0 / self.control_frequency
        self.behavior_timer = self.create_timer(timer_period, self.behavior_update_callback)

        # Add parameter callback for dynamic reconfiguration
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Behavior Manager initialized with {self.control_frequency}Hz control rate')

    def parameter_callback(self, params):
        """Handle parameter changes at runtime."""
        from rclpy.parameter import Parameter as rclparam
        from rclpy.parameter_service import SetParametersResult

        successful = True
        reason = ''

        for param in params:
            if param.name == 'control_frequency' and param.type_ == rclparam.Type.INTEGER:
                self.control_frequency = param.value
                # Adjust timer period
                timer_period = 1.0 / self.control_frequency
                self.behavior_timer.timer_period_ns = int(timer_period * 1e9)
                self.get_logger().info(f'Control frequency updated to {self.control_frequency}Hz')
            elif param.name == 'safety.max_tilt_angle':
                self.max_tilt_angle = param.value
                self.get_logger().info(f'Max tilt angle updated to {self.max_tilt_angle}')
            elif param.name == 'debug_mode':
                self.debug_mode = param.value
                self.get_logger().info(f'Debug mode set to {self.debug_mode}')

        return SetParametersResult(successful=successful, reason=reason)

    def joint_state_callback(self, msg):
        """Handle joint state updates and perform safety checks."""
        self.joint_states = msg

        # Perform safety checks on joint data
        for i, (name, pos, vel) in enumerate(zip(msg.name, msg.position, msg.velocity)):
            if abs(vel) > self.max_joint_velocity:
                self.get_logger().warn(f'Joint {name} velocity limit exceeded: {vel} > {self.max_joint_velocity}')
                self.trigger_safety_procedure()

    def imu_callback(self, msg):
        """Handle IMU data updates and check balance."""
        self.imu_data = msg

        # Check if robot is tilting too much
        pitch = self.get_pitch_from_quaternion(msg.orientation)
        if abs(pitch) > self.max_tilt_angle:
            self.get_logger().warn(f'Excessive tilt detected: {pitch:.3f} > {self.max_tilt_angle}')
            self.trigger_safety_procedure()

    def get_pitch_from_quaternion(self, orientation):
        """Extract pitch angle from quaternion."""
        import math
        # Simplified pitch calculation from quaternion
        sinr_cosp = 2 * (orientation.w * orientation.y - orientation.z * orientation.x)
        cosr_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.x * orientation.x)
        return math.atan2(sinr_cosp, cosr_cosp)

    def behavior_update_callback(self):
        """Main behavior update loop."""
        if self.debug_mode:
            self.get_logger().debug(f'Behavior: {self.current_behavior.value}, Safety: {self.safety_engaged}')

        # Check if safety is engaged
        if self.safety_engaged:
            if self.current_behavior != RobotBehavior.EMERGENCY_STOP:
                self.get_logger().warn('Safety engaged, transitioning to emergency stop')
                self.current_behavior = RobotBehavior.EMERGENCY_STOP
                self.execute_emergency_stop()
            return

        # Execute behavior-specific logic
        if self.current_behavior == RobotBehavior.IDLE:
            self.execute_idle_behavior()
        elif self.current_behavior == RobotBehavior.WALKING:
            self.execute_walking_behavior()
        elif self.current_behavior == RobotBehavior.MANIPULATING:
            self.execute_manipulation_behavior()
        elif self.current_behavior == RobotBehavior.CALIBRATING:
            self.execute_calibration_behavior()
        elif self.current_behavior == RobotBehavior.EMERGENCY_STOP:
            self.execute_emergency_stop()

        # Publish status
        status_msg = String()
        status_msg.data = f"Behavior: {self.current_behavior.value}, Safety: {not self.safety_engaged}"
        self.status_pub.publish(status_msg)

    def execute_idle_behavior(self):
        """Execute idle behavior - monitor and wait for commands."""
        # In idle state, just monitor sensors and wait for transitions
        pass

    def execute_walking_behavior(self):
        """Execute walking behavior with safety monitoring."""
        # Send forward velocity command
        cmd = Twist()
        cmd.linear.x = 0.2  # Move forward slowly
        cmd.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(cmd)

        # Check if walking goal is achieved or if we need to stop
        # (simplified condition for example)
        # In a real system, this would check action feedback or other conditions
        if self.should_stop_walking():
            self.current_behavior = RobotBehavior.IDLE

    def execute_manipulation_behavior(self):
        """Execute manipulation behavior."""
        # Placeholder for manipulation logic
        # In a real system, this would control arm joints for manipulation
        pass

    def execute_calibration_behavior(self):
        """Execute calibration behavior."""
        # Placeholder for calibration logic
        # In a real system, this would move joints to calibration positions
        pass

    def execute_emergency_stop(self):
        """Execute emergency stop procedures."""
        # Stop all movement
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)

        # Log emergency stop
        self.get_logger().error('EMERGENCY STOP ACTIVE - ROBOT HALTED')

    def should_stop_walking(self):
        """Check if walking behavior should stop."""
        # Simplified condition - in real system this would check various factors
        return False

    def enable_motors_callback(self, request, response):
        """Handle motor enable/disable requests."""
        if request.data:
            if self.safety_engaged:
                response.success = False
                response.message = "Cannot enable motors: safety engaged"
                self.get_logger().warn("Motor enable request denied: safety engaged")
            else:
                response.success = True
                response.message = "Motors enabled"
                self.get_logger().info("Motors enabled")
        else:
            response.success = True
            response.message = "Motors disabled"
            self.get_logger().info("Motors disabled")
            # Stop robot if motors are disabled
            self.execute_emergency_stop()

        return response

    def trigger_safety_procedure(self):
        """Trigger safety procedures."""
        if not self.safety_engaged:
            self.safety_engaged = True
            self.get_logger().error("SAFETY TRIGGERED - ENGAGING PROTECTIONS")


def main(args=None):
    """Main function to run the behavior manager."""
    rclpy.init(args=args)

    node = BehaviorManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()