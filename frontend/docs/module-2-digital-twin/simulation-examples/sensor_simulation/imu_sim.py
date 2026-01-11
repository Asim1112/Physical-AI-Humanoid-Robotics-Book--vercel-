#!/usr/bin/env python3
# Example: Realistic IMU sensor simulation for humanoid robot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np
import math
from scipy.spatial.transform import Rotation as R


class IMUSimulator(Node):
    """
    A node that simulates realistic IMU sensor data with proper noise characteristics
    and physical modeling for humanoid robot applications.
    """

    def __init__(self):
        super().__init__('imu_simulator')

        # Declare parameters
        self.declare_parameter('imu_rate', 100)  # Hz
        self.declare_parameter('gyro_noise_density', 0.0001)  # rad/s/sqrt(Hz)
        self.declare_parameter('gyro_random_walk', 0.0001)    # rad/s/sqrt(Hz)
        self.declare_parameter('accel_noise_density', 0.01)   # m/s^2/sqrt(Hz)
        self.declare_parameter('accel_random_walk', 0.01)     # m/s^2/sqrt(Hz)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('use_realistic_model', True)

        # Get parameter values
        self.imu_rate = self.get_parameter('imu_rate').value
        self.gyro_noise_density = self.get_parameter('gyro_noise_density').value
        self.gyro_random_walk = self.get_parameter('gyro_random_walk').value
        self.accel_noise_density = self.get_parameter('accel_noise_density').value
        self.accel_random_walk = self.get_parameter('accel_random_walk').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.use_realistic_model = self.get_parameter('use_realistic_model').value

        # IMU publisher
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Timer for IMU updates
        self.imu_timer = self.create_timer(1.0 / self.imu_rate, self.imu_callback)

        # State variables for realistic IMU simulation
        self.time_prev = self.get_clock().now().nanoseconds / 1e9
        self.gyro_bias = np.array([0.0, 0.0, 0.0])  # Gyro bias (rad/s)
        self.accel_bias = np.array([0.0, 0.0, 0.0])  # Accel bias (m/s^2)

        # Bias random walk state
        self.gyro_bias_rw = np.array([0.0, 0.0, 0.0])
        self.accel_bias_rw = np.array([0.0, 0.0, 0.0])

        # True state (for simulation)
        self.true_orientation = R.from_quat([0, 0, 0, 1])  # Start at identity
        self.true_angular_velocity = np.array([0.0, 0.0, 0.0])  # rad/s
        self.true_linear_acceleration = np.array([0.0, 0.0, 9.81])  # m/s^2 (gravity)

        # Noise accumulators for bias random walk
        self.gyro_bias_drift = np.array([0.0, 0.0, 0.0])
        self.accel_bias_drift = np.array([0.0, 0.0, 0.0])

        self.get_logger().info(
            f'IMU Simulator initialized at {self.imu_rate}Hz with realistic noise model'
        )

    def imu_callback(self):
        """Generate and publish IMU data."""
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds / 1e9) - self.time_prev
        self.time_prev = current_time.nanoseconds / 1e9

        if dt <= 0:
            return  # Skip if time didn't advance

        # Update true state based on some motion model
        # For this example, we'll simulate small oscillations to make it realistic
        t = current_time.nanoseconds / 1e9

        # Simulate small body movements (like breathing, heartbeat for humanoid)
        body_movement = np.array([
            0.01 * math.sin(0.3 * t),      # Small oscillations in X
            0.01 * math.cos(0.4 * t),      # Small oscillations in Y
            0.02 * math.sin(0.2 * t)       # Small oscillations in Z
        ])

        # Add some angular velocity for rotation
        angular_vel_variation = np.array([
            0.005 * math.sin(0.5 * t),
            0.004 * math.cos(0.6 * t),
            0.003 * math.sin(0.7 * t)
        ])

        # Update true angular velocity with variations
        self.true_angular_velocity = body_movement * 0.1 + angular_vel_variation

        # Update orientation based on angular velocity
        # Simple integration: quaternion derivative
        omega_quat = np.array([0, *self.true_angular_velocity]) * 0.5
        quat_vec = np.array([*self.true_orientation.as_quat()])
        dq = np.array([
            -np.dot(omega_quat[1:], quat_vec[1:]),
            omega_quat[0]*quat_vec[1:] + np.cross(omega_quat[1:], quat_vec[1:])
        ])
        quat_new = quat_vec + dq * dt
        self.true_orientation = R.from_quat(quat_new / np.linalg.norm(quat_new))

        # Simulate accelerometer - gravity in body frame plus linear acceleration
        # Rotate gravity vector to body frame
        gravity_body = self.true_orientation.apply([0, 0, -9.81])
        self.true_linear_acceleration = gravity_body + body_movement

        # Generate realistic IMU measurements
        imu_msg = self.generate_realistic_imu_measurement(dt)

        # Set header
        imu_msg.header = Header()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        # Publish IMU message
        self.imu_pub.publish(imu_msg)

    def generate_realistic_imu_measurement(self, dt):
        """Generate IMU measurement with realistic noise and bias."""
        # Generate noise components
        gyro_white_noise = self.generate_gyro_noise(dt)
        accel_white_noise = self.generate_accel_noise(dt)

        # Update bias random walk
        self.update_bias_random_walk(dt)

        # Apply noise and bias to true values
        measured_angular_velocity = self.true_angular_velocity + self.gyro_bias + gyro_white_noise
        measured_linear_acceleration = self.true_linear_acceleration + self.accel_bias + accel_white_noise

        # Create IMU message
        imu_msg = Imu()

        # Set orientation (we'll use the true orientation with some noise for simplicity)
        true_quat = self.true_orientation.as_quat()
        # Add small orientation noise
        orientation_noise = np.random.normal(0, 0.001, 3)  # Small orientation noise
        noisy_quat = R.from_rotvec(orientation_noise).as_quat()

        # Combine true orientation with noise
        combined_rot = R.from_quat(true_quat) * R.from_quat(noisy_quat)
        quat_result = combined_rot.as_quat()

        imu_msg.orientation = Quaternion()
        imu_msg.orientation.x = quat_result[0]
        imu_msg.orientation.y = quat_result[1]
        imu_msg.orientation.z = quat_result[2]
        imu_msg.orientation.w = quat_result[3]

        # Set orientation covariance (diagonal values only for simplicity)
        imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        # Set angular velocity
        imu_msg.angular_velocity = Vector3()
        imu_msg.angular_velocity.x = measured_angular_velocity[0]
        imu_msg.angular_velocity.y = measured_angular_velocity[1]
        imu_msg.angular_velocity.z = measured_angular_velocity[2]

        # Set angular velocity covariance
        imu_msg.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        # Set linear acceleration
        imu_msg.linear_acceleration = Vector3()
        imu_msg.linear_acceleration.x = measured_linear_acceleration[0]
        imu_msg.linear_acceleration.y = measured_linear_acceleration[1]
        imu_msg.linear_acceleration.z = measured_linear_acceleration[2]

        # Set linear acceleration covariance
        imu_msg.linear_acceleration_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]

        return imu_msg

    def generate_gyro_noise(self, dt):
        """Generate gyroscope noise with proper characteristics."""
        # White noise component (proportional to 1/sqrt(dt))
        white_noise = np.random.normal(0, self.gyro_noise_density / math.sqrt(dt), 3)
        return white_noise

    def generate_accel_noise(self, dt):
        """Generate accelerometer noise with proper characteristics."""
        # White noise component (proportional to 1/sqrt(dt))
        white_noise = np.random.normal(0, self.accel_noise_density / math.sqrt(dt), 3)
        return white_noise

    def update_bias_random_walk(self, dt):
        """Update bias random walk components."""
        # Update gyro bias random walk
        self.gyro_bias_rw += np.random.normal(0, self.gyro_random_walk * math.sqrt(dt), 3)
        self.gyro_bias = self.gyro_bias_rw  # For simplicity, bias equals random walk

        # Update accel bias random walk
        self.accel_bias_rw += np.random.normal(0, self.accel_random_walk * math.sqrt(dt), 3)
        self.accel_bias = self.accel_bias_rw  # For simplicity, bias equals random walk

    def get_imu_status(self):
        """Get current IMU status and parameters."""
        status = {
            'rate': self.imu_rate,
            'gyro_noise_density': self.gyro_noise_density,
            'accel_noise_density': self.accel_noise_density,
            'gyro_bias': self.gyro_bias.tolist(),
            'accel_bias': self.accel_bias.tolist(),
            'frame_id': self.imu_frame_id
        }
        return status


class IMUCalibrationSimulator(Node):
    """
    A node that simulates IMU calibration procedures and validation.
    """

    def __init__(self):
        super().__init__('imu_calibration_simulator')

        # Parameters for calibration simulation
        self.declare_parameter('calibration_samples', 100)
        self.declare_parameter('stationary_threshold', 0.1)

        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.stationary_threshold = self.get_parameter('stationary_threshold').value

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Calibration result publisher
        self.calibration_pub = self.create_publisher(
            Imu,
            '/imu/data_calibrated',
            10
        )

        # State for calibration
        self.imu_samples = []
        self.is_calibrating = False
        self.calibration_offset = np.array([0.0, 0.0, 0.0])  # Accelerometer offset (gravity should be 9.81)

        self.get_logger().info('IMU Calibration Simulator initialized')

    def imu_callback(self, msg):
        """Process IMU data for calibration."""
        # Convert IMU message to numpy array for processing
        accel_data = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        angular_vel_data = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Check if robot is stationary (low angular velocity)
        is_stationary = np.linalg.norm(angular_vel_data) < self.stationary_threshold

        if is_stationary and len(self.imu_samples) < self.calibration_samples:
            # Collect stationary samples for calibration
            self.imu_samples.append(accel_data)
            self.get_logger().info(f'Collected {len(self.imu_samples)}/{self.calibration_samples} calibration samples')

            if len(self.imu_samples) == self.calibration_samples:
                # Perform calibration when we have enough samples
                self.perform_calibration()
        elif len(self.imu_samples) >= self.calibration_samples:
            # Publish calibrated data
            calibrated_msg = self.apply_calibration(msg)
            self.calibration_pub.publish(calibrated_msg)

    def perform_calibration(self):
        """Perform IMU calibration based on collected samples."""
        if not self.imu_samples:
            self.get_logger().warn('No samples collected for calibration')
            return

        # Calculate average acceleration during stationary period
        avg_accel = np.mean(self.imu_samples, axis=0)

        # The expected acceleration during stationary period should be gravity (0, 0, 9.81)
        expected_gravity = np.array([0.0, 0.0, 9.81])

        # Calculate offset (bias) to be removed
        self.calibration_offset = avg_accel - expected_gravity

        self.get_logger().info(f'IMU calibration completed. Offset: {self.calibration_offset}')
        self.get_logger().info(f'Average measured: {avg_accel}, Expected gravity: {expected_gravity}')

    def apply_calibration(self, original_msg):
        """Apply calibration to IMU message."""
        calibrated_msg = Imu()
        calibrated_msg.header = original_msg.header

        # Apply accelerometer calibration
        original_accel = np.array([
            original_msg.linear_acceleration.x,
            original_msg.linear_acceleration.y,
            original_msg.linear_acceleration.z
        ])

        calibrated_accel = original_accel - self.calibration_offset

        calibrated_msg.linear_acceleration = Vector3()
        calibrated_msg.linear_acceleration.x = calibrated_accel[0]
        calibrated_msg.linear_acceleration.y = calibrated_accel[1]
        calibrated_msg.linear_acceleration.z = calibrated_accel[2]

        # Copy angular velocity (usually doesn't need calibration)
        calibrated_msg.angular_velocity = original_msg.angular_velocity

        # Copy orientation (if available)
        calibrated_msg.orientation = original_msg.orientation

        # Copy covariances
        calibrated_msg.orientation_covariance = original_msg.orientation_covariance
        calibrated_msg.angular_velocity_covariance = original_msg.angular_velocity_covariance
        calibrated_msg.linear_acceleration_covariance = original_msg.linear_acceleration_covariance

        return calibrated_msg


def main(args=None):
    """Main function to run the IMU simulation."""
    rclpy.init(args=args)

    # Create both nodes
    imu_sim = IMUSimulator()
    imu_cal = IMUCalibrationSimulator()

    # Use MultiThreadedExecutor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(imu_sim)
    executor.add_node(imu_cal)

    try:
        imu_sim.get_logger().info('Starting IMU simulation nodes...')
        executor.spin()
    except KeyboardInterrupt:
        imu_sim.get_logger().info('IMU simulation interrupted by user')
    finally:
        imu_sim.destroy_node()
        imu_cal.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()