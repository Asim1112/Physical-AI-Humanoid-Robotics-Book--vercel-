#!/usr/bin/env python3
# Example: Simple ROS 2 action client for humanoid robot walking
# Demonstrates action client implementation with feedback handling

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
import time


class SimpleActionClient(Node):
    """
    A simple action client that sends walk commands.
    Demonstrates action client implementation with feedback handling.
    Note: This example uses a hypothetical action type - in practice you would
    use an action defined in a custom package.
    """

    def __init__(self):
        super().__init__('simple_action_client')

        # Note: In a real implementation, you would create an action client like this:
        # self._action_client = ActionClient(self, WalkToPose, 'walk_to_pose')
        # For this example, we'll simulate the action client behavior

        self.get_logger().info('Simple Action Client initialized (simulated)')

    def send_walk_goal(self, x, y, step_size=0.1, max_steps=100):
        """
        Simulated function to send a walk goal.
        In a real implementation, this would use the action client to send goals.
        """
        self.get_logger().info(f'Sending goal to walk to ({x}, {y})')

        # Simulate waiting for the action server
        self.get_logger().info('Waiting for action server...')
        time.sleep(0.5)  # Simulate connection time

        # Simulate sending the goal and receiving feedback
        self.get_logger().info('Goal sent, waiting for result...')

        # Simulate feedback during execution
        for step in range(0, max_steps, max(1, max_steps // 10)):  # Report 10 feedback updates
            progress = (step / max_steps) * 100
            self.get_logger().info(f'Feedback: Walking... {progress:.1f}% complete')
            time.sleep(0.3)  # Simulate time between feedback

        # Simulate final result
        success = True  # Simulate success
        message = f'Successfully walked to target ({x}, {y})'
        result = {
            'success': success,
            'message': message,
            'steps_taken': int(max(abs(x), abs(y)) / step_size) + 1,
            'final_pose': {'x': x, 'y': y}
        }

        self.get_logger().info(f'Action completed: {result["message"]}')
        return result


def main(args=None):
    """Main function to run the action client."""
    rclpy.init(args=args)

    client = SimpleActionClient()

    # Send a goal to walk to position (2.0, 1.0)
    result = client.send_walk_goal(2.0, 1.0, step_size=0.2, max_steps=50)

    if result and result['success']:
        print(f"Successfully completed action! Result: {result['message']}")
        print(f"Final position: ({result['final_pose']['x']}, {result['final_pose']['y']})")
    else:
        print("Action failed")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()