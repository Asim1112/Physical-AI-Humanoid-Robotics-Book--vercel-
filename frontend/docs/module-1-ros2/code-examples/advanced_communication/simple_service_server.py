#!/usr/bin/env python3
# Example: Simple ROS 2 service server for humanoid robot control
# Demonstrates service server implementation

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
import time


class SimpleServiceServer(Node):
    """
    A simple service server that handles requests to enable/disable robot functions.
    Demonstrates service server implementation for humanoid robot control.
    """

    def __init__(self):
        super().__init__('simple_service_server')

        # Create a service server
        self.srv = self.create_service(
            SetBool,
            'enable_robot_motors',
            self.enable_motors_callback
        )

        # Simulate robot state
        self.motors_enabled = False

        # Create a publisher for joint states to simulate motor effects
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.get_logger().info('Simple Service Server node started')

    def enable_motors_callback(self, request, response):
        """Handle incoming service requests to enable/disable motors."""
        self.get_logger().info(f'Received request to enable motors: {request.data}')

        try:
            # Simulate the actual hardware operation
            # In a real robot, this would send commands to the motor controllers
            self.motors_enabled = request.data

            # Simulate processing time
            time.sleep(0.1)

            # Set response
            response.success = True
            response.message = f'Motors successfully {"enabled" if request.data else "disabled"}'

            self.get_logger().info(f'Service completed: {response.message}')

            # Publish a joint state to show the effect
            if request.data:
                self.publish_demo_joint_state()

        except Exception as e:
            response.success = False
            response.message = f'Failed to control motors: {str(e)}'
            self.get_logger().error(f'Service failed: {str(e)}')

        return response

    def publish_demo_joint_state(self):
        """Publish a demo joint state to show motors are active."""
        msg = JointState()
        msg.name = ['demo_joint']
        msg.position = [0.1]  # Small movement to show motors are active
        msg.velocity = [0.0]
        msg.effort = [0.0]

        self.joint_pub.publish(msg)


def main(args=None):
    """Main function to run the service server."""
    rclpy.init(args=args)

    server = SimpleServiceServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()