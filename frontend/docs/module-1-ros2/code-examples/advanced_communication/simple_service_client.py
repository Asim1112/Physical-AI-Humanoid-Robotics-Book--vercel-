#!/usr/bin/env python3
# Example: Simple ROS 2 service client for humanoid robot control
# Demonstrates service client implementation

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import sys


class SimpleServiceClient(Node):
    """
    A simple service client that calls the motor enable service.
    Demonstrates service client implementation for humanoid robot control.
    """

    def __init__(self):
        super().__init__('simple_service_client')

        # Create a service client
        self.cli = self.create_client(SetBool, 'enable_robot_motors')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service client initialized')

    def send_request(self, enable_motors):
        """Send a request to the service."""
        request = SetBool.Request()
        request.data = enable_motors

        self.get_logger().info(f'Sending request to enable motors: {enable_motors}')

        # Call the service asynchronously
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(
                f'Service response - Success: {response.success}, Message: {response.message}'
            )
            return response
        else:
            self.get_logger().error('Service call failed')
            return None


def main(args=None):
    """Main function to run the service client."""
    rclpy.init(args=args)

    client = SimpleServiceClient()

    # Get command line argument for motor enable/disable
    if len(sys.argv) < 2:
        print("Usage: python3 simple_service_client.py <true|false>")
        print("Example: python3 simple_service_client.py true")
        return

    enable_motors = sys.argv[1].lower() == 'true'

    response = client.send_request(enable_motors)

    if response and response.success:
        print(f"Successfully set motors to {enable_motors}")
    else:
        print(f"Failed to set motors to {enable_motors}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()