#!/usr/bin/env python3
# Example: Combined publisher-subscriber node for humanoid robot
# Demonstrates both publishing and subscribing in a single node

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, Bool
import math
import time


class CombinedNode(Node):
    """
    A node that demonstrates both publishing and subscribing.
    This simulates a humanoid robot state monitoring and control node.
    """

    def __init__(self):
        super().__init__('combined_publisher_subscriber')

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
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

        # Timers for different functions
        self.publish_timer = self.create_timer(0.05, self.publish_joint_states)  # 20Hz
        self.control_timer = self.create_timer(0.1, self.run_control_logic)      # 10Hz

        # State variables
        self.time_offset = time.time()
        self.latest_imu = None
        self.safety_engaged = False

        self.get_logger().info('Combined Publisher-Subscriber node started')

    def publish_joint_states(self):
        """Publish simulated joint state data."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Define joint names for a humanoid robot
        msg.name = [
            'left_hip', 'left_knee', 'left_ankle',
            'right_hip', 'right_knee', 'right_ankle'
        ]

        # Calculate positions with oscillating pattern
        t = time.time() - self.time_offset
        positions = [
            0.1 * math.sin(t * 0.5),   # left_hip
            0.05 * math.sin(t * 0.7),  # left_knee
            0.08 * math.sin(t * 0.3),  # left_ankle
            0.1 * math.sin(t * 0.5),   # right_hip
            0.05 * math.sin(t * 0.7),  # right_knee
            0.08 * math.sin(t * 0.3)   # right_ankle
        ]

        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)

        self.joint_pub.publish(msg)

    def joint_state_callback(self, msg):
        """Process incoming joint state messages."""
        self.get_logger().debug(f'Received {len(msg.name)} joint states')

        # Check for safety conditions
        for name, pos in zip(msg.name, msg.position):
            if abs(pos) > 1.5:  # Extreme position check
                self.get_logger().warn(f'Safety: Joint {name} extreme position: {pos:.3f}')
                self.safety_engaged = True

    def imu_callback(self, msg):
        """Process incoming IMU data."""
        self.latest_imu = msg

        # Extract pitch from orientation (simplified)
        import math
        w, x, y, z = msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        sinr_cosp = 2 * (w * y - z * x)
        cosr_cosp = 1 - 2 * (y * y + x * x)
        pitch = math.atan2(sinr_cosp, cosr_cosp)

        # Check if robot is tilting too much
        if abs(pitch) > 0.5:  # More than ~28 degrees
            self.get_logger().warn(f'Safety: Excessive tilt detected: {pitch:.3f} rad')
            self.safety_engaged = True

    def run_control_logic(self):
        """Run control logic based on sensor data."""
        if self.safety_engaged:
            self.get_logger().warn('Safety engaged - stopping all movement')
            self.stop_robot()
            return

        # Simple control logic - move forward if stable
        cmd = Twist()
        cmd.linear.x = 0.1  # Move forward slowly
        cmd.angular.z = 0.0  # No rotation

        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Send stop command to robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    """Main function to run the combined node."""
    rclpy.init(args=args)

    node = CombinedNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()