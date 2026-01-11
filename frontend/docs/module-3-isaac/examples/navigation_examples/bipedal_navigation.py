#!/usr/bin/env python3
# Example: Bipedal navigation for humanoid robot

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import numpy as np
import math
from scipy.spatial import distance
import threading
import time
from collections import deque

class BipedalNavigationNode(Node):
    """
    A navigation system for bipedal humanoid robots.
    Implements path planning, footstep planning, and balance-aware navigation.
    """

    def __init__(self):
        super().__init__('bipedal_navigation_node')

        # Declare parameters
        self.declare_parameter('planner_frequency', 10.0)  # Hz
        self.declare_parameter('controller_frequency', 50.0)  # Hz
        self.declare_parameter('step_length', 0.3)  # meters
        self.declare_parameter('step_width', 0.25)  # meters
        self.declare_parameter('max_linear_speed', 0.3)  # m/s
        self.declare_parameter('max_angular_speed', 0.5)  # rad/s
        self.declare_parameter('foot_separation', 0.2)  # meters
        self.declare_parameter('com_height', 0.8)  # meters
        self.declare_parameter('balance_threshold', 0.1)  # meters

        # Get parameters
        self.planner_frequency = self.get_parameter('planner_frequency').value
        self.controller_frequency = self.get_parameter('controller_frequency').value
        self.step_length = self.get_parameter('step_length').value
        self.step_width = self.get_parameter('step_width').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.foot_separation = self.get_parameter('foot_separation').value
        self.com_height = self.get_parameter('com_height').value
        self.balance_threshold = self.get_parameter('balance_threshold').value

        # Robot state
        self.current_pose = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.current_velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vtheta
        self.odom_data = None
        self.imu_data = None
        self.laser_data = None

        # Navigation state
        self.goal_pose = None
        self.global_path = []
        self.local_path = []
        self.footstep_plan = []
        self.current_footstep_index = 0
        self.navigation_active = False

        # Balance state
        self.left_foot_pose = np.array([0.0, self.foot_separation/2, 0.0])
        self.right_foot_pose = np.array([0.0, -self.foot_separation/2, 0.0])
        self.support_foot = 'left'  # Which foot is currently supporting
        self.com_position = np.array([0.0, 0.0, self.com_height])  # Center of mass

        # Obstacle avoidance
        self.obstacles = []
        self.collision_distance = 0.5  # meters

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.footstep_pub = self.create_publisher(MarkerArray, '/footsteps', 10)
        self.com_pub = self.create_publisher(Marker, '/center_of_mass', 10)
        self.support_polygon_pub = self.create_publisher(Marker, '/support_polygon', 10)
        self.navigation_status_pub = self.create_publisher(String, '/navigation/status', 10)

        # Timers
        self.planner_timer = self.create_timer(
            1.0 / self.planner_frequency, self.planning_step)
        self.controller_timer = self.create_timer(
            1.0 / self.controller_frequency, self.control_step)

        # Path planning components
        self.path_planner = PathPlanner()
        self.footstep_planner = FootstepPlanner(
            step_length=self.step_length,
            step_width=self.step_width
        )

        # Performance tracking
        self.navigation_stats = {
            'distance_traveled': 0.0,
            'time_elapsed': 0.0,
            'steps_taken': 0,
            'balance_maintained': True
        }

        self.get_logger().info(
            f'Bipedal Navigation initialized with step length: {self.step_length}m, '
            f'step width: {self.step_width}m'
        )

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.odom_data = msg
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y

        # Extract orientation (simplified - would need full quaternion conversion)
        orientation = msg.pose.pose.orientation
        # Convert quaternion to yaw (simplified)
        self.current_pose[2] = math.atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )

        # Update velocity
        self.current_velocity[0] = msg.twist.twist.linear.x
        self.current_velocity[1] = msg.twist.twist.linear.y
        self.current_velocity[2] = msg.twist.twist.angular.z

    def imu_callback(self, msg):
        """Update IMU data for balance control."""
        self.imu_data = msg

        # Update center of mass based on IMU data
        # This is a simplified representation
        self.com_position[2] = self.com_height  # Maintain height

    def laser_callback(self, msg):
        """Process laser scan for obstacle detection."""
        self.laser_data = msg

        # Detect obstacles from laser scan
        self.obstacles = []
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, range_val in enumerate(msg.ranges):
            if 0 < range_val < msg.range_max:
                angle = angle_min + i * angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                self.obstacles.append((x, y, range_val))

    def goal_callback(self, msg):
        """Receive navigation goal."""
        self.goal_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            0.0  # We'll calculate orientation later
        ])

        self.navigation_active = True
        self.get_logger().info(f'Navigation goal received: {self.goal_pose[:2]}')

    def planning_step(self):
        """Execute navigation planning step."""
        if not self.navigation_active or self.goal_pose is None:
            return

        # Plan global path
        if self.odom_data:
            start_pose = self.current_pose[:2]  # x, y
            goal_pose = self.goal_pose[:2]      # x, y

            # Plan path using A* or similar algorithm
            self.global_path = self.path_planner.plan_path(start_pose, goal_pose, self.obstacles)

            # Generate local path with obstacle avoidance
            self.local_path = self.path_planner.plan_local_path(
                self.global_path, self.obstacles
            )

            # Plan footsteps based on path
            self.footstep_plan = self.footstep_planner.plan_footsteps(
                self.local_path, self.left_foot_pose, self.right_foot_pose
            )

            # Publish path
            self.publish_path()

    def control_step(self):
        """Execute navigation control step."""
        if not self.navigation_active or len(self.footstep_plan) == 0:
            # Stop robot if no plan
            self.stop_robot()
            return

        # Check if we need to execute next footstep
        if self.current_footstep_index < len(self.footstep_plan):
            next_footstep = self.footstep_plan[self.current_footstep_index]

            # Check balance before executing step
            if self.is_balance_safe(next_footstep):
                # Execute the footstep
                self.execute_footstep(next_footstep)
                self.current_footstep_index += 1

                # Update support polygon
                self.update_support_polygon(next_footstep)

                # Update navigation stats
                self.navigation_stats['steps_taken'] += 1
            else:
                # Stop and replan if balance is not safe
                self.get_logger().warn('Balance not safe, stopping navigation')
                self.stop_robot()
                self.navigation_active = False

        # Check if we've reached the goal
        if self.current_footstep_index >= len(self.footstep_plan):
            self.reached_goal()

        # Publish visualization
        self.publish_visualization()

    def is_balance_safe(self, next_footstep):
        """Check if executing next footstep is balance-safe."""
        # Calculate center of mass position relative to support polygon
        com_x, com_y = self.com_position[0], self.com_position[1]

        # Determine support polygon based on current support foot
        if self.support_foot == 'left':
            support_x = self.left_foot_pose[0]
            support_y = self.left_foot_pose[1]
        else:
            support_x = self.right_foot_pose[0]
            support_y = self.right_foot_pose[1]

        # Calculate distance from CoM to support point
        com_to_support_dist = math.sqrt((com_x - support_x)**2 + (com_y - support_y)**2)

        # Check if CoM is within balance threshold of support polygon
        return com_to_support_dist <= self.balance_threshold

    def execute_footstep(self, footstep):
        """Execute a single footstep."""
        foot_type = footstep['foot']
        target_pos = footstep['position']

        # Move the appropriate foot to target position
        if foot_type == 'left':
            self.left_foot_pose[:2] = target_pos[:2]
            self.support_foot = 'right'  # Right foot becomes support
        else:  # right foot
            self.right_foot_pose[:2] = target_pos[:2]
            self.support_foot = 'left'   # Left foot becomes support

        # Update robot pose (simplified - in reality this would be more complex)
        self.current_pose[0] = (self.left_foot_pose[0] + self.right_foot_pose[0]) / 2
        self.current_pose[1] = (self.left_foot_pose[1] + self.right_foot_pose[1]) / 2

        # Calculate robot orientation based on foot positions
        dx = self.right_foot_pose[0] - self.left_foot_pose[0]
        dy = self.right_foot_pose[1] - self.left_foot_pose[1]
        self.current_pose[2] = math.atan2(dy, dx)

        # Publish velocity command to move toward target
        cmd_vel = Twist()
        cmd_vel.linear.x = min(self.max_linear_speed, 0.1)  # Move forward slowly
        cmd_vel.angular.z = 0.0  # Simplified for this example
        self.cmd_vel_pub.publish(cmd_vel)

    def update_support_polygon(self, footstep):
        """Update the support polygon after a footstep."""
        # In a real implementation, this would update the convex hull
        # of all supporting contact points
        pass

    def reached_goal(self):
        """Handle goal reached condition."""
        self.navigation_active = False
        self.stop_robot()

        # Calculate distance to goal
        current_pos = self.current_pose[:2]
        goal_pos = self.goal_pose[:2]
        dist_to_goal = np.linalg.norm(current_pos - goal_pos)

        if dist_to_goal < 0.3:  # Within 30cm of goal
            self.get_logger().info(f'Goal reached! Distance: {dist_to_goal:.2f}m')
            status_msg = String()
            status_msg.data = f'GOAL_REACHED distance={dist_to_goal:.2f}m'
            self.navigation_status_pub.publish(status_msg)
        else:
            self.get_logger().info(f'Navigation completed but goal not reached. Distance: {dist_to_goal:.2f}m')
            status_msg = String()
            status_msg.data = f'NAVIGATION_COMPLETED distance_to_goal={dist_to_goal:.2f}m'
            self.navigation_status_pub.publish(status_msg)

    def stop_robot(self):
        """Stop the robot by sending zero velocity commands."""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def publish_path(self):
        """Publish the planned path."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for point in self.global_path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_visualization(self):
        """Publish visualization markers."""
        # Publish footsteps
        self.publish_footsteps()

        # Publish center of mass
        self.publish_center_of_mass()

        # Publish support polygon
        self.publish_support_polygon()

    def publish_footsteps(self):
        """Publish footsteps as visualization markers."""
        marker_array = MarkerArray()

        for i, footstep in enumerate(self.footstep_plan):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'footsteps'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            pos = footstep['position']
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = 0.05  # Height above ground

            # Orientation based on foot type
            marker.pose.orientation.w = 1.0

            # Scale (foot size)
            marker.scale.x = 0.15  # Length
            marker.scale.y = 0.08  # Width
            marker.scale.z = 0.02  # Height

            # Color based on foot type
            if footstep['foot'] == 'left':
                marker.color.r = 1.0  # Red for left foot
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0  # Blue for right foot
                marker.color.g = 0.0
                marker.color.b = 1.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.footstep_pub.publish(marker_array)

    def publish_center_of_mass(self):
        """Publish center of mass as visualization marker."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'center_of_mass'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.com_position[0]
        marker.pose.position.y = self.com_position[1]
        marker.pose.position.z = self.com_position[2]

        marker.pose.orientation.w = 1.0

        # Scale
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Color (yellow for CoM)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.com_pub.publish(marker)

    def publish_support_polygon(self):
        """Publish support polygon as visualization marker."""
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'support_polygon'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Create polygon from foot positions (simplified as line between feet)
        p1 = Point()
        p1.x = self.left_foot_pose[0]
        p1.y = self.left_foot_pose[1]
        p1.z = 0.05

        p2 = Point()
        p2.x = self.right_foot_pose[0]
        p2.y = self.right_foot_pose[1]
        p2.z = 0.05

        # Close the polygon (simplified)
        p3 = Point()
        p3.x = self.left_foot_pose[0]
        p3.y = self.left_foot_pose[1]
        p3.z = 0.05

        marker.points = [p1, p2, p3]

        # Scale
        marker.scale.x = 0.02  # Line width

        # Color (green for support polygon)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.support_polygon_pub.publish(marker)

    def get_navigation_status(self):
        """Get current navigation status."""
        status = {
            'active': self.navigation_active,
            'goal_set': self.goal_pose is not None,
            'steps_remaining': len(self.footstep_plan) - self.current_footstep_index,
            'balance_safe': True,  # Would check actual balance
            'obstacles_detected': len(self.obstacles) > 0
        }
        return status


class PathPlanner:
    """Simple path planner for bipedal navigation."""

    def __init__(self):
        self.grid_resolution = 0.1  # meters
        self.inflation_radius = 0.3  # meters

    def plan_path(self, start, goal, obstacles):
        """Plan path from start to goal avoiding obstacles."""
        # Simplified path planning - in practice, use A*, RRT*, or similar
        path = []

        # Calculate straight-line path
        steps = max(int(np.linalg.norm(np.array(goal) - np.array(start)) / 0.3), 5)

        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (goal[0] - start[0])
            y = start[1] + t * (goal[1] - start[1])
            path.append([x, y])

        return path

    def plan_local_path(self, global_path, obstacles):
        """Plan local path with obstacle avoidance."""
        if not global_path:
            return []

        local_path = []
        for point in global_path:
            # Check for obstacles near this point
            safe_point = self.avoid_obstacles_at_point(point, obstacles)
            local_path.append(safe_point)

        return local_path

    def avoid_obstacles_at_point(self, point, obstacles):
        """Modify point to avoid nearby obstacles."""
        safe_point = [point[0], point[1]]

        for obs_x, obs_y, obs_dist in obstacles:
            if obs_dist < self.inflation_radius:
                # Calculate vector from obstacle to point
                dx = safe_point[0] - obs_x
                dy = safe_point[1] - obs_y
                dist = math.sqrt(dx*dx + dy*dy)

                if dist < self.inflation_radius and dist > 0:
                    # Move away from obstacle
                    dx_norm = dx / dist
                    dy_norm = dy / dist
                    safe_dist = self.inflation_radius + 0.1
                    safe_point[0] = obs_x + dx_norm * safe_dist
                    safe_point[1] = obs_y + dy_norm * safe_dist

        return safe_point


class FootstepPlanner:
    """Plan footsteps for bipedal locomotion."""

    def __init__(self, step_length=0.3, step_width=0.25):
        self.step_length = step_length
        self.step_width = step_width

    def plan_footsteps(self, path, left_foot_start, right_foot_start):
        """Plan sequence of footsteps along path."""
        footsteps = []

        if len(path) < 2:
            return footsteps

        # Start with initial foot positions
        left_pos = left_foot_start.copy()
        right_pos = right_foot_start.copy()

        # Determine which foot to move first
        support_foot = 'left'  # Start with left foot as support

        for i in range(len(path) - 1):
            # Calculate direction to next waypoint
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            dist = math.sqrt(dx*dx + dy*dy)

            if dist > self.step_length / 2:  # If significant movement
                # Plan a step with the swing foot
                if support_foot == 'left':
                    # Move right foot
                    right_pos[0] = path[i][0] + dx * 0.5
                    right_pos[1] = path[i][1] + dy * 0.5 + (self.step_width if i % 2 == 0 else -self.step_width)
                    footsteps.append({
                        'foot': 'right',
                        'position': right_pos.copy(),
                        'step_num': len(footsteps)
                    })
                    support_foot = 'right'
                else:
                    # Move left foot
                    left_pos[0] = path[i][0] + dx * 0.5
                    left_pos[1] = path[i][1] + dy * 0.5 + (-self.step_width if i % 2 == 0 else self.step_width)
                    footsteps.append({
                        'foot': 'left',
                        'position': left_pos.copy(),
                        'step_num': len(footsteps)
                    })
                    support_foot = 'left'

        return footsteps


def main(args=None):
    """Main function to run the Bipedal Navigation node."""
    rclpy.init(args=args)

    bipedal_nav = BipedalNavigationNode()

    try:
        bipedal_nav.get_logger().info('Starting Bipedal Navigation node...')
        rclpy.spin(bipedal_nav)
    except KeyboardInterrupt:
        bipedal_nav.get_logger().info('Bipedal Navigation node interrupted by user')
    finally:
        bipedal_nav.stop_robot()
        bipedal_nav.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()