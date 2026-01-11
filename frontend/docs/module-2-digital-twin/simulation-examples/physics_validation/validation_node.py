#!/usr/bin/env python3
# Example: Physics validation node for Gazebo simulation

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import numpy as np
import math
import time


class PhysicsValidationNode(Node):
    """
    A node that validates Gazebo physics simulation by comparing
    expected vs actual robot behavior and validating physics properties.
    """

    def __init__(self):
        super().__init__('physics_validation_node')

        # Declare parameters
        self.declare_parameter('validation_frequency', 10.0)
        self.declare_parameter('model_name', 'simple_humanoid')
        self.declare_parameter('validation_threshold', 0.05)
        self.declare_parameter('balance_threshold', 0.3)

        # Get parameter values
        self.validation_frequency = self.get_parameter('validation_frequency').value
        self.model_name = self.get_parameter('model_name').value
        self.validation_threshold = self.get_parameter('validation_threshold').value
        self.balance_threshold = self.get_parameter('balance_threshold').value

        # Gazebo services
        self.get_state_client = self.create_client(
            GetModelState,
            '/gazebo/get_model_state'
        )

        self.set_state_client = self.create_client(
            SetModelState,
            '/gazebo/set_model_state'
        )

        # Publishers
        self.validation_pub = self.create_publisher(
            Float64MultiArray,
            '/physics_validation/results',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/physics_validation/status',
            10
        )

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timers
        self.validation_timer = self.create_timer(
            1.0 / self.validation_frequency,
            self.validation_callback
        )

        # State tracking
        self.joint_states = None
        self.last_validation_time = time.time()
        self.validation_results = {
            'balance_score': 0.0,
            'stability_score': 0.0,
            'energy_conservation': 0.0,
            'kinematic_consistency': 0.0
        }

        # Validation history for averaging
        self.balance_history = []
        self.stability_history = []

        self.get_logger().info(
            f'Physics Validation Node initialized with {self.validation_frequency}Hz frequency'
        )

    def joint_state_callback(self, msg):
        """Handle joint state updates."""
        self.joint_states = msg

    def validation_callback(self):
        """Perform periodic physics validation."""
        current_time = time.time()

        # Get model state from Gazebo
        model_state = self.get_model_state()

        if model_state is None:
            self.get_logger().warn('Could not get model state for validation')
            return

        # Perform various validation checks
        validation_metrics = self.calculate_validation_metrics(model_state, self.joint_states)

        # Update validation results
        self.validation_results.update(validation_metrics)

        # Update history for averaging
        self.balance_history.append(validation_metrics['balance_score'])
        self.stability_history.append(validation_metrics['stability_score'])

        # Keep only recent history
        if len(self.balance_history) > 10:
            self.balance_history.pop(0)
        if len(self.stability_history) > 10:
            self.stability_history.pop(0)

        # Publish validation results
        results_msg = Float64MultiArray()
        results_msg.data = [
            validation_metrics['balance_score'],
            validation_metrics['stability_score'],
            validation_metrics['energy_conservation'],
            validation_metrics['kinematic_consistency'],
            validation_metrics['overall_score']
        ]

        self.validation_pub.publish(results_msg)

        # Publish status
        status_msg = String()
        if validation_metrics['overall_score'] > 0.8:
            status_msg.data = 'VALID'
        elif validation_metrics['overall_score'] > 0.5:
            status_msg.data = 'CAUTION'
        else:
            status_msg.data = 'INVALID'

        self.status_pub.publish(status_msg)

        # Log validation status periodically
        if current_time - self.last_validation_time > 5.0:  # Every 5 seconds
            self.get_logger().info(
                f'Physics Validation - Balance: {validation_metrics["balance_score"]:.3f}, '
                f'Stability: {validation_metrics["stability_score"]:.3f}, '
                f'Overall: {validation_metrics["overall_score"]:.3f}'
            )
            self.last_validation_time = current_time

    def get_model_state(self):
        """Get current model state from Gazebo."""
        if not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Gazebo service not available')
            return None

        request = GetModelState.Request()
        request.model_name = self.model_name
        request.relative_entity_name = 'world'

        future = self.get_state_client.call_async(request)

        # Wait for response (in practice, use proper async handling)
        start_time = time.time()
        while not future.done() and time.time() - start_time < 1.0:
            time.sleep(0.01)

        if future.done():
            try:
                response = future.result()
                if response.success:
                    return response
                else:
                    self.get_logger().error(f'GetModelState failed: {response.status_message}')
                    return None
            except Exception as e:
                self.get_logger().error(f'Error getting model state: {e}')
                return None
        else:
            self.get_logger().error('GetModelState service call timed out')
            return None

    def calculate_validation_metrics(self, model_state, joint_states):
        """Calculate various physics validation metrics."""
        metrics = {}

        # Calculate balance score based on orientation
        balance_score = self.calculate_balance_score(model_state.pose.orientation)
        metrics['balance_score'] = balance_score

        # Calculate stability score based on position and velocity
        stability_score = self.calculate_stability_score(model_state.pose, model_state.twist)
        metrics['stability_score'] = stability_score

        # Calculate energy conservation (simplified)
        energy_conservation = self.calculate_energy_conservation(model_state.twist)
        metrics['energy_conservation'] = energy_conservation

        # Calculate kinematic consistency if joint states available
        if joint_states:
            kinematic_score = self.calculate_kinematic_consistency(model_state.pose, joint_states)
        else:
            kinematic_score = 1.0  # Default to perfect if no joint data
        metrics['kinematic_consistency'] = kinematic_score

        # Calculate overall score
        metrics['overall_score'] = (
            balance_score * 0.3 +
            stability_score * 0.3 +
            energy_conservation * 0.2 +
            kinematic_score * 0.2
        )

        return metrics

    def calculate_balance_score(self, orientation):
        """Calculate balance score from orientation quaternion."""
        # Convert quaternion to roll/pitch/yaw to check balance
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z

        # Calculate pitch (critical for bipedal balance)
        sinr_cosp = 2 * (w * y - z * x)
        cosr_cosp = 1 - 2 * (y * y + x * x)
        pitch = math.atan2(sinr_cosp, cosr_cosp)

        # Calculate roll
        sinp = 2 * (w * x + y * z)
        cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinp, cosp)

        # Balance score: 1.0 for perfect balance (0 tilt), 0.0 for excessive tilt
        tilt_magnitude = math.sqrt(pitch * pitch + roll * roll)
        balance_score = max(0.0, min(1.0, 1.0 - (tilt_magnitude / self.balance_threshold)))

        return balance_score

    def calculate_stability_score(self, pose, twist):
        """Calculate stability score based on position and velocity."""
        # Calculate position magnitude (distance from origin)
        pos_magnitude = math.sqrt(pose.position.x**2 + pose.position.y**2 + pose.position.z**2)

        # Calculate velocity magnitude
        vel_magnitude = math.sqrt(
            twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2 +
            twist.angular.x**2 + twist.angular.y**2 + twist.angular.z**2
        )

        # Stability is inversely related to position and velocity magnitudes
        # Lower values indicate more stable behavior
        stability_score = 1.0 / (1.0 + pos_magnitude * 0.1 + vel_magnitude * 0.5)
        stability_score = min(1.0, stability_score)  # Clamp to 0-1 range

        return stability_score

    def calculate_energy_conservation(self, twist):
        """Calculate energy conservation score."""
        # Calculate kinetic energy from twist
        lin_vel = math.sqrt(twist.linear.x**2 + twist.linear.y**2 + twist.linear.z**2)
        ang_vel = math.sqrt(twist.angular.x**2 + twist.angular.y**2 + twist.angular.z**2)

        # Simplified kinetic energy (assuming mass of 10kg)
        mass = 10.0
        linear_energy = 0.5 * mass * lin_vel * lin_vel
        angular_energy = 0.25 * mass * ang_vel * ang_vel  # Simplified moment of inertia
        total_energy = linear_energy + angular_energy

        # Energy conservation score (simplified - should be relatively constant in absence of external forces)
        # This is a basic check - in reality, you'd track energy changes over time
        energy_score = 1.0 / (1.0 + total_energy * 0.001)  # Higher energy = lower score for this metric
        return min(1.0, energy_score)

    def calculate_kinematic_consistency(self, pose, joint_states):
        """Calculate consistency between joint positions and end-effector pose."""
        # This is a simplified check - in reality, you'd perform forward kinematics
        # For this example, we'll just check if joint positions are reasonable

        if not joint_states.position:
            return 1.0

        # Check for extreme joint positions that might indicate simulation issues
        extreme_count = 0
        for pos in joint_states.position:
            if abs(pos) > 3.14:  # More than 180 degrees
                extreme_count += 1

        # Consistency score based on reasonable joint positions
        consistency_score = max(0.0, 1.0 - (extreme_count / len(joint_states.position)))
        return consistency_score

    def reset_model_state(self, x=0.0, y=0.0, z=1.0):
        """Reset model to initial state."""
        if not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('SetModelState service not available')
            return False

        request = SetModelState.Request()
        request.model_state = ModelState()
        request.model_state.model_name = self.model_name
        request.model_state.pose.position.x = x
        request.model_state.pose.position.y = y
        request.model_state.pose.position.z = z
        request.model_state.pose.orientation.w = 1.0
        request.model_state.pose.orientation.x = 0.0
        request.model_state.pose.orientation.y = 0.0
        request.model_state.pose.orientation.z = 0.0
        request.model_state.twist.linear.x = 0.0
        request.model_state.twist.linear.y = 0.0
        request.model_state.twist.linear.z = 0.0
        request.model_state.twist.angular.x = 0.0
        request.model_state.twist.angular.y = 0.0
        request.model_state.twist.angular.z = 0.0
        request.model_state.reference_frame = 'world'

        future = self.set_state_client.call_async(request)

        start_time = time.time()
        while not future.done() and time.time() - start_time < 1.0:
            time.sleep(0.01)

        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Model {self.model_name} reset to position ({x}, {y}, {z})')
                    return True
                else:
                    self.get_logger().error(f'SetModelState failed: {response.status_message}')
                    return False
            except Exception as e:
                self.get_logger().error(f'Error resetting model state: {e}')
                return False
        else:
            self.get_logger().error('SetModelState service call timed out')
            return False


def main(args=None):
    """Main function to run the physics validation node."""
    rclpy.init(args=args)

    node = PhysicsValidationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Physics validation node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()