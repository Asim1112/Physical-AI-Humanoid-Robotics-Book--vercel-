#!/usr/bin/env python3
# Example: Complete simulation control interface for humanoid robot

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import GetModelState, SetModelState
import time
import threading
import numpy as np
from collections import deque
import json


class SimulationControlInterface(Node):
    """
    A comprehensive interface between simulation and control systems.
    Manages data flow, validation, and coordination between all simulation components.
    """

    def __init__(self):
        super().__init__('simulation_control_interface')

        # Declare parameters
        self.declare_parameter('control_frequency', 100)
        self.declare_parameter('sensor_frequency', 50)
        self.declare_parameter('validation_frequency', 10)
        self.declare_parameter('sim_fidelity_threshold', 0.8)
        self.declare_parameter('use_sim_time', True)

        # Get parameter values
        self.control_frequency = self.get_parameter('control_frequency').value
        self.sensor_frequency = self.get_parameter('sensor_frequency').value
        self.validation_frequency = self.get_parameter('validation_frequency').value
        self.fidelity_threshold = self.get_parameter('sim_fidelity_threshold').value
        self.use_sim_time = self.get_parameter('use_sim_time').value

        # State tracking
        self.joint_states = None
        self.imu_data = None
        self.laser_scan = None
        self.control_commands = None
        self.robot_pose = None

        # History for validation and analysis
        self.state_history = deque(maxlen=1000)
        self.command_history = deque(maxlen=1000)

        # QoS profiles
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers for simulation sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            sensor_qos
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            sensor_qos
        )

        # Publishers for control commands and monitoring
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            control_qos
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            control_qos
        )

        # Publishers for monitoring and validation
        self.sim_status_pub = self.create_publisher(
            String,
            '/simulation/status',
            10
        )

        self.fidelity_pub = self.create_publisher(
            Float64MultiArray,
            '/simulation/fidelity',
            10
        )

        # Action servers for complex behaviors
        self.trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self.execute_trajectory_callback,
            goal_callback=self.trajectory_goal_callback,
            cancel_callback=self.trajectory_cancel_callback
        )

        # Gazebo services
        self.get_state_client = self.create_client(
            GetModelState,
            '/gazebo/get_model_state'
        )

        self.set_state_client = self.create_client(
            SetModelState,
            '/gazebo/set_model_state'
        )

        # Timers for different system components
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_loop)

        validation_period = 1.0 / self.validation_frequency
        self.validation_timer = self.create_timer(validation_period, self.validation_loop)

        # Initialize system state
        self.simulation_running = True
        self.safety_engaged = False
        self.current_behavior = 'idle'
        self.fidelity_score = 1.0

        self.get_logger().info(
            f'Simulation Control Interface initialized with {self.control_frequency}Hz control rate'
        )

    def joint_state_callback(self, msg):
        """Handle joint state updates from simulation."""
        self.joint_states = msg

        # Store in history
        state_entry = {
            'timestamp': time.time(),
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'efforts': list(msg.effort),
            'names': list(msg.name)
        }
        self.state_history.append(state_entry)

        # Perform safety checks
        self.perform_joint_safety_checks(msg)

    def imu_callback(self, msg):
        """Handle IMU data from simulation."""
        self.imu_data = msg

        # Check for balance and safety conditions
        self.perform_balance_safety_check(msg)

    def laser_callback(self, msg):
        """Handle laser scan data from simulation."""
        self.laser_scan = msg

        # Check for obstacles and safety
        self.perform_obstacle_safety_check(msg)

    def perform_joint_safety_checks(self, joint_state):
        """Perform safety checks on joint states."""
        for i, (name, pos, vel, effort) in enumerate(
            zip(joint_state.name, joint_state.position, joint_state.velocity, joint_state.effort)
        ):
            # Check position limits
            if abs(pos) > 3.14:  # More than 180 degrees
                self.get_logger().error(f'Safety: Joint {name} extreme position: {pos}')
                self.trigger_safety_procedure()

            # Check velocity limits
            if abs(vel) > 10.0:  # Example limit
                self.get_logger().warn(f'Safety: Joint {name} high velocity: {vel}')

            # Check effort limits
            if abs(effort) > 100.0:  # Example limit
                self.get_logger().warn(f'Safety: Joint {name} high effort: {effort}')

    def perform_balance_safety_check(self, imu_data):
        """Check robot balance and stability."""
        # Extract orientation from IMU
        w, x, y, z = imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z

        # Calculate pitch and roll
        sinr_cosp = 2 * (w * y - z * x)
        cosr_cosp = 1 - 2 * (y * y + x * x)
        pitch = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * x + y * z)
        cosp = np.sqrt(1 - sinp * sinp)
        roll = np.arctan2(sinp, cosp)

        # Check if robot is tilting too much
        tilt_magnitude = np.sqrt(pitch * pitch + roll * roll)
        if abs(tilt_magnitude) > 1.0:  # More than ~57 degrees
            self.get_logger().error(f'Safety: Excessive tilt detected: {tilt_magnitude:.3f} rad')
            self.trigger_safety_procedure()

    def perform_obstacle_safety_check(self, laser_scan):
        """Check for obstacles in laser scan data."""
        if len(laser_scan.ranges) > 0:
            # Check front-facing range (middle of scan)
            front_range = laser_scan.ranges[len(laser_scan.ranges) // 2]

            if 0 < front_range < 0.3:  # Obstacle within 30cm
                self.get_logger().warn(f'Safety: Obstacle detected at {front_range:.2f}m')

    def trigger_safety_procedure(self):
        """Trigger safety procedures when safety limits are exceeded."""
        if not self.safety_engaged:
            self.safety_engaged = True
            self.get_logger().error('SAFETY LIMIT EXCEEDED - ENGAGING SAFETY PROCEDURES')

            # Stop all movement
            self.stop_robot()

            # Switch to safe behavior
            self.current_behavior = 'safety_stop'

    def stop_robot(self):
        """Send stop commands to robot."""
        # Stop joint movement
        stop_cmd = Float64MultiArray()
        if self.joint_states:
            stop_cmd.data = [0.0] * len(self.joint_states.position)
        else:
            stop_cmd.data = [0.0] * 12  # Default for humanoid with 12 joints
        self.joint_cmd_pub.publish(stop_cmd)

        # Stop velocity
        stop_vel = Twist()
        self.cmd_vel_pub.publish(stop_vel)

    def control_loop(self):
        """Main control loop that processes sensor data and generates commands."""
        if not self.simulation_running or self.safety_engaged:
            return

        # Generate control commands based on current state and behavior
        control_cmd = self.generate_control_commands()

        if control_cmd is not None:
            # Publish control commands
            self.joint_cmd_pub.publish(control_cmd)

            # Store command in history
            command_entry = {
                'timestamp': time.time(),
                'commands': list(control_cmd.data),
                'behavior': self.current_behavior
            }
            self.command_history.append(command_entry)

    def generate_control_commands(self):
        """Generate control commands based on current behavior."""
        if self.joint_states is None:
            return None

        # Example: Different behaviors based on current state
        cmd_msg = Float64MultiArray()

        if self.current_behavior == 'idle':
            # Hold current position
            cmd_msg.data = list(self.joint_states.position)
        elif self.current_behavior == 'calibration':
            # Move to calibration positions
            cmd_msg.data = self.get_calibration_positions()
        elif self.current_behavior == 'balance_test':
            # Perform balance test movements
            cmd_msg.data = self.get_balance_test_positions()
        else:
            # Default behavior - hold position with small oscillations
            current_time = time.time()
            commands = []
            for i, (name, current_pos) in enumerate(zip(self.joint_states.name, self.joint_states.position)):
                if 'hip' in name.lower():
                    offset = 0.05 * np.sin(current_time * 0.5)
                elif 'knee' in name.lower():
                    offset = 0.03 * np.cos(current_time * 0.7)
                elif 'ankle' in name.lower():
                    offset = 0.02 * np.sin(current_time * 0.3)
                else:
                    offset = 0.0
                commands.append(current_pos + offset)
            cmd_msg.data = commands

        return cmd_msg

    def get_calibration_positions(self):
        """Get joint positions for calibration."""
        # Example calibration positions
        if self.joint_states:
            positions = []
            for name in self.joint_states.name:
                if 'hip' in name.lower():
                    positions.append(0.0)  # Neutral position
                elif 'knee' in name.lower():
                    positions.append(0.0)  # Neutral position
                elif 'ankle' in name.lower():
                    positions.append(0.0)  # Neutral position
                elif 'shoulder' in name.lower():
                    positions.append(0.0)  # Neutral position
                elif 'elbow' in name.lower():
                    positions.append(0.0)  # Neutral position
                else:
                    positions.append(0.0)
            return positions
        else:
            return [0.0] * 12  # Default for 12 joints

    def get_balance_test_positions(self):
        """Get joint positions for balance testing."""
        current_time = time.time()
        if self.joint_states:
            positions = []
            for i, name in enumerate(self.joint_states.name):
                if 'hip' in name.lower():
                    # Oscillate hips for balance test
                    positions.append(0.1 * np.sin(current_time * 0.5))
                elif 'knee' in name.lower():
                    # Follow hip movement with phase offset
                    positions.append(0.05 * np.sin(current_time * 0.5 + np.pi/4))
                elif 'ankle' in name.lower():
                    # Ankle adjustment for balance
                    positions.append(0.02 * np.sin(current_time * 0.5 - np.pi/4))
                else:
                    positions.append(self.joint_states.position[i])
            return positions
        else:
            return [0.0] * 12

    def validation_loop(self):
        """Validate simulation performance and fidelity."""
        if not self.joint_states:
            return

        # Calculate various validation metrics
        metrics = self.calculate_validation_metrics()

        # Update fidelity score
        self.fidelity_score = metrics['overall_fidelity']

        # Publish fidelity metrics
        fidelity_msg = Float64MultiArray()
        fidelity_msg.data = [
            metrics['overall_fidelity'],
            metrics['response_time'],
            metrics['control_accuracy'],
            metrics['sensor_consistency'],
            metrics['stability_score']
        ]
        self.fidelity_pub.publish(fidelity_msg)

        # Publish simulation status
        status_msg = String()
        if self.fidelity_score >= self.fidelity_threshold:
            status_msg.data = f'VALID ({self.fidelity_score:.3f})'
        else:
            status_msg.data = f'INVALID ({self.fidelity_score:.3f}) - Below threshold {self.fidelity_threshold}'
        self.sim_status_pub.publish(status_msg)

        # Log status periodically
        current_time = time.time()
        if int(current_time) % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(
                f'Simulation Fidelity: {self.fidelity_score:.3f}, '
                f'Response: {metrics["response_time"]:.3f}s, '
                f'Accuracy: {metrics["control_accuracy"]:.3f}'
            )

    def calculate_validation_metrics(self):
        """Calculate various validation metrics."""
        metrics = {}

        # Calculate response time (simplified)
        if self.state_history and self.command_history:
            recent_state = self.state_history[-1] if self.state_history else None
            recent_cmd = self.command_history[-1] if self.command_history else None

            if recent_state and recent_cmd:
                response_time = abs(recent_state['timestamp'] - recent_cmd['timestamp'])
                metrics['response_time'] = min(response_time, 1.0)  # Cap at 1 second
            else:
                metrics['response_time'] = 0.1  # Default
        else:
            metrics['response_time'] = 0.1

        # Calculate control accuracy (simplified)
        if self.joint_states and self.command_history:
            latest_cmd = self.command_history[-1]['commands']
            current_pos = self.joint_states.position

            if len(latest_cmd) == len(current_pos):
                diffs = [abs(cmd - pos) for cmd, pos in zip(latest_cmd, current_pos)]
                avg_diff = sum(diffs) / len(diffs) if diffs else 0.0
                # Convert to 0-1 scale (1.0 = perfect accuracy)
                metrics['control_accuracy'] = 1.0 / (1.0 + avg_diff * 10)
            else:
                metrics['control_accuracy'] = 0.5
        else:
            metrics['control_accuracy'] = 0.5

        # Calculate sensor consistency (simplified)
        metrics['sensor_consistency'] = 0.9  # Placeholder

        # Calculate stability score (simplified)
        stability_score = 1.0 if not self.safety_engaged else 0.0
        if self.imu_data:
            # Check if IMU indicates stable orientation
            w, x, y, z = self.imu_data.orientation.w, self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z
            pitch = np.arctan2(2*(w*y - z*x), 1 - 2*(y*y + x*x))
            if abs(pitch) < 0.5:  # Stable if pitch < 28 degrees
                stability_score = 1.0
            else:
                stability_score = 0.5 - min(abs(pitch) - 0.5, 0.5)  # Decrease with tilt
        metrics['stability_score'] = max(0.0, min(1.0, stability_score))

        # Overall fidelity score
        metrics['overall_fidelity'] = (
            metrics['control_accuracy'] * 0.4 +
            metrics['stability_score'] * 0.3 +
            metrics['sensor_consistency'] * 0.2 +
            (1.0 - metrics['response_time']) * 0.1
        )

        return metrics

    def execute_trajectory_callback(self, goal_handle):
        """Execute joint trajectory in simulation."""
        self.get_logger().info(f'Executing trajectory with {len(goal_handle.request.trajectory.points)} points')

        feedback_msg = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        # Store original behavior
        original_behavior = self.current_behavior
        self.current_behavior = 'trajectory_execution'

        try:
            for i, point in enumerate(goal_handle.request.trajectory.points):
                # Check if goal was canceled
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    return result

                # Send trajectory point to simulation
                cmd_msg = Float64MultiArray()
                cmd_msg.data = list(point.positions)
                self.joint_cmd_pub.publish(cmd_msg)

                # Wait for the time specified in the trajectory point
                if i < len(goal_handle.request.trajectory.points) - 1:
                    next_point = goal_handle.request.trajectory.points[i + 1]
                    time_to_next = next_point.time_from_start.sec + next_point.time_from_start.nanosec / 1e9
                    # In a real system, you'd wait for this time

                # Update feedback
                feedback_msg.joint_names = goal_handle.request.trajectory.joint_names
                feedback_msg.actual.positions = list(point.positions)
                feedback_msg.desired.positions = list(point.positions)
                feedback_msg.error.positions = [0.0] * len(point.positions)  # Simplified

                progress = float(i + 1) / len(goal_handle.request.trajectory.points)
                feedback_msg.progress = progress

                goal_handle.publish_feedback(feedback_msg)

            # Complete successfully
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info('Trajectory execution completed successfully')

        except Exception as e:
            self.get_logger().error(f'Error executing trajectory: {e}')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL

        finally:
            # Restore original behavior
            self.current_behavior = original_behavior

        return result

    def trajectory_goal_callback(self, goal_request):
        """Accept or reject trajectory goals."""
        # Check if trajectory is valid
        if len(goal_request.trajectory.points) == 0:
            self.get_logger().warn('Received trajectory with no points')
            return GoalResponse.REJECT

        if len(goal_request.trajectory.joint_names) == 0:
            self.get_logger().warn('Received trajectory with no joint names')
            return GoalResponse.REJECT

        # Check if joint names match expected joints
        if self.joint_states:
            expected_joints = set(self.joint_states.name)
            requested_joints = set(goal_request.trajectory.joint_names)

            if not requested_joints.issubset(expected_joints):
                self.get_logger().warn(f'Requested joints {requested_joints} not in expected joints {expected_joints}')
                return GoalResponse.REJECT

        self.get_logger().info(f'Accepting trajectory for joints: {goal_request.trajectory.joint_names}')
        return GoalResponse.ACCEPT

    def trajectory_cancel_callback(self, goal_handle):
        """Accept or reject trajectory cancellation."""
        self.get_logger().info('Received trajectory cancellation request')
        return CancelResponse.ACCEPT

    def get_simulation_status(self):
        """Get current simulation status."""
        status = {
            'running': self.simulation_running,
            'safety_engaged': self.safety_engaged,
            'current_behavior': self.current_behavior,
            'fidelity_score': self.fidelity_score,
            'joint_count': len(self.joint_states.name) if self.joint_states else 0,
            'control_frequency': self.control_frequency,
            'sensor_frequency': self.sensor_frequency
        }
        return status

    def set_behavior(self, behavior_name):
        """Set the current robot behavior."""
        valid_behaviors = ['idle', 'calibration', 'balance_test', 'trajectory_execution', 'safety_stop']

        if behavior_name in valid_behaviors:
            old_behavior = self.current_behavior
            self.current_behavior = behavior_name
            self.get_logger().info(f'Behavior changed from {old_behavior} to {behavior_name}')
            return True
        else:
            self.get_logger().warn(f'Invalid behavior: {behavior_name}')
            return False

    def reset_simulation(self):
        """Reset simulation to initial state."""
        self.get_logger().info('Resetting simulation to initial state')

        # Reset state variables
        self.current_behavior = 'idle'
        self.safety_engaged = False
        self.fidelity_score = 1.0

        # Clear histories
        self.state_history.clear()
        self.command_history.clear()

        # Stop robot
        self.stop_robot()


def main(args=None):
    """Main function to run the simulation control interface."""
    rclpy.init(args=args)

    node = SimulationControlInterface()

    # Use MultiThreadedExecutor to handle multiple callbacks
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Simulation Control Interface started')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Simulation Control Interface interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()