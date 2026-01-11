#!/usr/bin/env python3
# Example: Simple ROS 2 publisher for humanoid robot joint commands
# Demonstrates basic publisher creation and message publishing

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time


class SimpleJointPublisher(Node):
    """
    A simple publisher node that demonstrates basic ROS 2 publishing concepts.
    This node publishes simulated joint state data for a humanoid robot.
    """

    def __init__(self):
        super().__init__('simple_joint_publisher')

        # Create a publisher for joint states
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Create a timer to publish data at 10Hz
        self.timer = self.create_timer(0.1, self.publish_joint_data)

        # Initialize time for oscillating pattern
        self.start_time = time.time()

        self.get_logger().info('Simple Joint Publisher node started')

    def publish_joint_data(self):
        """Publish simulated joint state data."""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Define joint names for a simple humanoid
        msg.name = ['left_hip', 'left_knee', 'right_hip', 'right_knee']

        # Calculate positions with oscillating pattern
        t = time.time() - self.start_time
        positions = [
            0.1 * math.sin(t * 0.5),   # left_hip
            0.05 * math.sin(t * 0.7),  # left_knee
            0.1 * math.sin(t * 0.5),   # right_hip
            0.05 * math.sin(t * 0.7)   # right_knee
        ]

        msg.position = positions
        msg.velocity = [0.0] * len(positions)
        msg.effort = [0.0] * len(positions)

        self.publisher.publish(msg)
        self.get_logger().debug(f'Published joint positions: {positions}')


def main(args=None):
    """Main function to run the publisher node."""
    rclpy.init(args=args)

    publisher = SimpleJointPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()