#!/usr/bin/env python3
# Example: Simple ROS 2 action server for humanoid robot walking
# Demonstrates action server implementation with feedback and result

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
import math
from geometry_msgs.msg import Pose, Point
from my_robot_msgs.action import WalkToPose  # This would be a custom action


class SimpleActionServer(Node):
    """
    A simple action server that handles walking to a target pose.
    Demonstrates action server implementation with feedback and cancellation.
    Note: This example uses a hypothetical action type - in practice you would
    define your own action files in a custom package.
    """

    def __init__(self):
        super().__init__('simple_action_server')

        # Use a reentrant callback group to handle multiple callbacks
        callback_group = ReentrantCallbackGroup()

        # Create the action server
        # Note: In a real implementation, you would need to define the WalkToPose action
        # For this example, we'll simulate the action behavior
        self.get_logger().info('Simple Action Server initialized (simulated)')

    def execute_walk_goal(self, target_pose, step_size=0.1, max_steps=100):
        """
        Simulated function to execute a walk goal.
        In a real implementation, this would be the execute_callback function
        for the action server.
        """
        self.get_logger().info(f'Executing walk to pose: ({target_pose.position.x}, {target_pose.position.y})')

        # Simulate walking progress
        current_x, current_y = 0.0, 0.0  # Starting position
        target_x, target_y = target_pose.position.x, target_pose.position.y

        # Calculate distance to target
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        steps_needed = int(distance / step_size) + 1
        total_steps = min(steps_needed, max_steps)

        # Simulate walking progress
        for step in range(total_steps):
            # Simulate taking a step
            progress = float(step) / float(total_steps)
            current_x += (target_x - current_x) * (1.0 / (total_steps - step))
            current_y += (target_y - current_y) * (1.0 / (total_steps - step))

            # Simulate feedback (in a real action server, this would use goal_handle.publish_feedback)
            feedback_msg = f'Walking... Step {step+1}/{total_steps} ({progress*100:.1f}%)'
            self.get_logger().info(feedback_msg)

            # Simulate time for each step
            time.sleep(0.2)

        # Check if we reached the target
        final_distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        success = final_distance <= step_size
        message = f'Successfully walked to target in {total_steps} steps' if success else f'Failed to reach target after {total_steps} steps'

        result = {
            'success': success,
            'message': message,
            'steps_taken': total_steps,
            'final_pose': {'x': current_x, 'y': current_y}
        }

        self.get_logger().info(f'Action completed: {result["message"]}')
        return result


def main(args=None):
    """Main function to run the action server."""
    rclpy.init(args=args)

    node = SimpleActionServer()

    # Simulate executing a goal
    target_pose = Pose()
    target_pose.position.x = 2.0
    target_pose.position.y = 1.0

    result = node.execute_walk_goal(target_pose, step_size=0.2, max_steps=50)
    print(f"Action result: {result}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()