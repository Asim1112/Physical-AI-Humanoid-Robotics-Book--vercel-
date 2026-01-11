#!/usr/bin/env python3
# Example: Simple ROS 2 subscriber for humanoid robot joint states
# Demonstrates basic subscriber creation and message handling

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SimpleJointSubscriber(Node):
    """
    A simple subscriber node that demonstrates basic ROS 2 subscription concepts.
    This node subscribes to joint state data from a humanoid robot.
    """

    def __init__(self):
        super().__init__('simple_joint_subscriber')

        # Create a subscriber for joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Store the latest joint state for reference
        self.latest_joint_state = None

        self.get_logger().info('Simple Joint Subscriber node started')

    def joint_state_callback(self, msg):
        """Process incoming joint state messages."""
        self.latest_joint_state = msg

        # Log basic information about the received joint states
        self.get_logger().info(
            f'Received {len(msg.name)} joints: {list(zip(msg.name, msg.position))}'
        )

        # Calculate and log average position
        if msg.position:
            avg_pos = sum(abs(pos) for pos in msg.position) / len(msg.position)
            self.get_logger().info(f'Average absolute position: {avg_pos:.3f}')

        # Check for any extreme positions (potential safety issue)
        for name, pos in zip(msg.name, msg.position):
            if abs(pos) > 1.5:  # Check if position is too extreme
                self.get_logger().warn(f'Joint {name} has extreme position: {pos:.3f}')


def main(args=None):
    """Main function to run the subscriber node."""
    rclpy.init(args=args)

    subscriber = SimpleJointSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()