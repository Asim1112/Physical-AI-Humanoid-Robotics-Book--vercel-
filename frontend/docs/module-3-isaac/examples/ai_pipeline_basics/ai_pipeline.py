#!/usr/bin/env python3
# Example: Basic AI pipeline for humanoid robot perception and control

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan, JointState
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
import numpy as np
import cv2
from cv_bridge import CvBridge
import threading
import queue
from collections import deque
import time

class HumanoidAIPipeline(Node):
    """
    A basic AI pipeline for humanoid robot perception and control.
    Demonstrates the integration of perception, planning, and control systems.
    """

    def __init__(self):
        super().__init__('humanoid_ai_pipeline')

        # Declare parameters
        self.declare_parameter('perception_frequency', 30)  # Hz
        self.declare_parameter('planning_frequency', 10)    # Hz
        self.declare_parameter('control_frequency', 100)    # Hz
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')

        # Get parameters
        self.perception_frequency = self.get_parameter('perception_frequency').value
        self.planning_frequency = self.get_parameter('planning_frequency').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.map_frame = self.get_parameter('map_frame').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Data queues for multi-threading
        self.camera_queue = queue.Queue(maxsize=5)
        self.imu_queue = queue.Queue(maxsize=10)
        self.lidar_queue = queue.Queue(maxsize=10)

        # State tracking
        self.joint_states = None
        self.imu_data = None
        self.camera_data = None
        self.odometry_data = None

        # Perception results
        self.detected_objects = []
        self.map_features = []
        self.localization_pose = None

        # Planning results
        self.global_plan = []
        self.local_plan = []
        self.footstep_plan = []

        # Control commands
        self.desired_velocities = []
        self.balance_commands = []

        # Subscribers
        self.camera_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.camera_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.perception_pub = self.create_publisher(
            MarkerArray, '/perception_markers', 10)
        self.plan_pub = self.create_publisher(
            MarkerArray, '/planned_path', 10)
        self.status_pub = self.create_publisher(
            String, '/ai_pipeline/status', 10)

        # Timers for different pipeline components
        self.perception_timer = self.create_timer(
            1.0 / self.perception_frequency, self.perception_step)
        self.planning_timer = self.create_timer(
            1.0 / self.planning_frequency, self.planning_step)
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_step)

        # Performance tracking
        self.pipeline_stats = {
            'perception_time': deque(maxlen=100),
            'planning_time': deque(maxlen=100),
            'control_time': deque(maxlen=100),
            'total_cycles': 0
        }

        self.get_logger().info(
            f'Humanoid AI Pipeline initialized with frequencies: '
            f'Perception={self.perception_frequency}Hz, '
            f'Planning={self.planning_frequency}Hz, '
            f'Control={self.control_frequency}Hz'
        )

    def camera_callback(self, msg):
        """Process camera data for perception."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_data = cv_image
            # Add to processing queue
            if not self.camera_queue.full():
                self.camera_queue.put((msg.header.stamp, cv_image))
        except Exception as e:
            self.get_logger().error(f'Camera callback error: {e}')

    def imu_callback(self, msg):
        """Process IMU data for balance and localization."""
        self.imu_data = msg
        if not self.imu_queue.full():
            self.imu_queue.put((msg.header.stamp, msg))

    def lidar_callback(self, msg):
        """Process LiDAR data for obstacle detection."""
        if not self.lidar_queue.full():
            self.lidar_queue.put((msg.header.stamp, msg))

    def joint_callback(self, msg):
        """Update joint state information."""
        self.joint_states = msg

    def odom_callback(self, msg):
        """Update odometry information."""
        self.odometry_data = msg

    def perception_step(self):
        """Execute perception pipeline step."""
        start_time = time.time()

        # Process camera data
        if not self.camera_queue.empty():
            try:
                timestamp, image = self.camera_queue.get_nowait()
                self.detected_objects = self.process_camera_data(image)
            except queue.Empty:
                pass

        # Process LiDAR data
        if not self.lidar_queue.empty():
            try:
                timestamp, scan = self.lidar_queue.get_nowait()
                self.map_features = self.process_lidar_data(scan)
            except queue.Empty:
                pass

        # Process IMU data for localization
        if self.imu_data:
            self.localization_pose = self.process_imu_data(self.imu_data)

        # Performance tracking
        perception_time = time.time() - start_time
        self.pipeline_stats['perception_time'].append(perception_time)

        # Log performance periodically
        if self.pipeline_stats['total_cycles'] % 100 == 0:
            avg_time = np.mean(self.pipeline_stats['perception_time'])
            self.get_logger().info(f'Perception avg time: {avg_time*1000:.2f}ms')

    def planning_step(self):
        """Execute planning pipeline step."""
        start_time = time.time()

        # Global path planning (if goal is set)
        if hasattr(self, 'target_pose') and self.localization_pose:
            self.global_plan = self.plan_global_path(
                self.localization_pose, self.target_pose
            )

        # Local path planning and obstacle avoidance
        if self.global_plan and self.map_features:
            self.local_plan = self.plan_local_path(
                self.global_plan, self.map_features
            )

        # Footstep planning for bipedal navigation
        if self.local_plan and self.joint_states:
            self.footstep_plan = self.plan_footsteps(
                self.local_plan, self.joint_states
            )

        # Performance tracking
        planning_time = time.time() - start_time
        self.pipeline_stats['planning_time'].append(planning_time)

        # Log performance periodically
        if self.pipeline_stats['total_cycles'] % 100 == 0:
            avg_time = np.mean(self.pipeline_stats['planning_time'])
            self.get_logger().info(f'Planning avg time: {avg_time*1000:.2f}ms')

    def control_step(self):
        """Execute control pipeline step."""
        start_time = time.time()

        # Generate control commands based on plans
        if self.footstep_plan and self.joint_states:
            control_commands = self.generate_control_commands(
                self.footstep_plan, self.joint_states
            )

            # Publish joint commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = control_commands
            self.joint_cmd_pub.publish(cmd_msg)

        # Balance control
        if self.joint_states and self.imu_data:
            balance_commands = self.balance_control(
                self.joint_states, self.imu_data
            )

            # Apply balance corrections
            self.apply_balance_corrections(balance_commands)

        # Performance tracking
        control_time = time.time() - start_time
        self.pipeline_stats['control_time'].append(control_time)

        # Log performance periodically
        if self.pipeline_stats['total_cycles'] % 100 == 0:
            avg_time = np.mean(self.pipeline_stats['control_time'])
            self.get_logger().info(f'Control avg time: {avg_time*1000:.2f}ms')

        # Update cycle counter
        self.pipeline_stats['total_cycles'] += 1

    def process_camera_data(self, image):
        """Process camera data for object detection and feature extraction."""
        # Convert to grayscale for feature detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect features (using ORB as example)
        orb = cv2.ORB_create(nfeatures=500)
        keypoints, descriptors = orb.detectAndCompute(gray, None)

        # Simple object detection (example: detect colored objects)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Detect red objects
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter small objects
                x, y, w, h = cv2.boundingRect(contour)
                detected_objects.append({
                    'type': 'red_object',
                    'bbox': (x, y, w, h),
                    'center': (x + w//2, y + h//2)
                })

        return detected_objects

    def process_lidar_data(self, scan):
        """Process LiDAR data for obstacle detection and mapping."""
        # Convert scan to points
        points = []
        for i, range_val in enumerate(scan.ranges):
            if 0 < range_val < scan.range_max:
                angle = scan.angle_min + i * scan.angle_increment
                x = range_val * np.cos(angle)
                y = range_val * np.sin(angle)
                points.append((x, y))

        # Detect obstacles
        obstacles = []
        for i, (x, y) in enumerate(points):
            distance = np.sqrt(x**2 + y**2)
            if distance < 1.0:  # Within 1 meter
                obstacles.append((x, y, distance))

        return obstacles

    def process_imu_data(self, imu_msg):
        """Process IMU data for orientation and balance."""
        # Extract orientation
        orientation = {
            'x': imu_msg.orientation.x,
            'y': imu_msg.orientation.y,
            'z': imu_msg.orientation.z,
            'w': imu_msg.orientation.w
        }

        # Extract angular velocity
        angular_vel = {
            'x': imu_msg.angular_velocity.x,
            'y': imu_msg.angular_velocity.y,
            'z': imu_msg.angular_velocity.z
        }

        # Calculate roll/pitch/yaw for balance
        w, x, y, z = orientation['w'], orientation['x'], orientation['y'], orientation['z']

        # Convert quaternion to euler angles
        import math
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Return pose with balance information
        pose = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},  # Simplified
            'orientation': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
            'balance': {'roll': roll, 'pitch': pitch}
        }

        return pose

    def plan_global_path(self, start_pose, goal_pose):
        """Plan global path from start to goal."""
        # Simplified path planning (in practice, use A*, RRT*, etc.)
        path = []

        # Calculate straight-line path
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            x = start_pose['position']['x'] + t * (goal_pose['x'] - start_pose['position']['x'])
            y = start_pose['position']['y'] + t * (goal_pose['y'] - start_pose['position']['y'])
            path.append({'x': x, 'y': y})

        return path

    def plan_local_path(self, global_path, obstacles):
        """Plan local path avoiding obstacles."""
        # Simplified local planning with obstacle avoidance
        local_path = []

        if not global_path:
            return local_path

        # Check each point in global path for obstacles
        for point in global_path:
            safe_point = self.avoid_obstacles_at_point(point, obstacles)
            local_path.append(safe_point)

        return local_path

    def avoid_obstacles_at_point(self, point, obstacles):
        """Modify point to avoid obstacles."""
        # Simple obstacle avoidance
        safe_point = {'x': point['x'], 'y': point['y']}

        for obs_x, obs_y, distance in obstacles:
            if distance < 0.5:  # Within safety margin
                # Move away from obstacle
                dx = safe_point['x'] - obs_x
                dy = safe_point['y'] - obs_y
                dist = np.sqrt(dx*dx + dy*dy)

                if dist < 0.5:
                    # Normalize and scale to safety distance
                    if dist > 0:
                        dx_norm = dx / dist
                        dy_norm = dy / dist
                        safe_point['x'] = obs_x + dx_norm * 0.6
                        safe_point['y'] = obs_y + dy_norm * 0.6

        return safe_point

    def plan_footsteps(self, path, joint_states):
        """Plan footsteps for bipedal navigation."""
        footsteps = []

        if not path or not joint_states:
            return footsteps

        # Simplified footstep planning
        step_length = 0.3  # meters
        step_width = 0.25  # meters (distance between feet)

        current_pos = {'x': 0.0, 'y': 0.0}
        support_foot = 'left'  # Start with left foot support

        for i, waypoint in enumerate(path):
            # Calculate direction to next waypoint
            dx = waypoint['x'] - current_pos['x']
            dy = waypoint['y'] - current_pos['y']
            dist = np.sqrt(dx*dx + dy*dy)

            if dist > step_length:
                # Plan a step
                step_dir = np.array([dx, dy]) / dist if dist > 0 else np.array([1, 0])

                if support_foot == 'left':
                    # Move right foot
                    right_foot_pos = np.array([current_pos['x'], current_pos['y']])
                    right_foot_pos += step_dir * step_length
                    right_foot_pos[1] += step_width / 2  # Lateral offset
                    footsteps.append({'foot': 'right', 'position': right_foot_pos, 'step_num': i})
                    support_foot = 'right'
                else:
                    # Move left foot
                    left_foot_pos = np.array([current_pos['x'], current_pos['y']])
                    left_foot_pos += step_dir * step_length
                    left_foot_pos[1] -= step_width / 2  # Lateral offset
                    footsteps.append({'foot': 'left', 'position': left_foot_pos, 'step_num': i})
                    support_foot = 'left'

                current_pos = {'x': waypoint['x'], 'y': waypoint['y']}

        return footsteps

    def generate_control_commands(self, footsteps, joint_states):
        """Generate joint control commands from footsteps."""
        if not footsteps or not joint_states:
            # Return current joint positions if no footsteps
            return list(joint_states.position) if joint_states else [0.0] * 12

        # Simplified control command generation
        # In practice, this would involve inverse kinematics, trajectory generation, etc.

        # For now, return a simple pattern that moves the robot forward
        commands = []
        for i, joint_name in enumerate(joint_states.name):
            if 'hip' in joint_name:
                # Hip joints - move forward/backward
                commands.append(0.0)  # Neutral position
            elif 'knee' in joint_name:
                # Knee joints - maintain standing position
                commands.append(-0.5)  # Bent position for standing
            elif 'ankle' in joint_name:
                # Ankle joints - maintain balance
                commands.append(0.0)  # Neutral position
            else:
                # Other joints - maintain default
                commands.append(0.0)

        return commands

    def balance_control(self, joint_states, imu_data):
        """Generate balance control commands based on IMU data."""
        # Extract balance information from IMU
        orientation = imu_data.orientation
        angular_vel = imu_data.angular_velocity

        # Convert quaternion to roll/pitch
        w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        # Simple PD controller for balance
        roll_error = -roll  # Negative because we want to counteract tilt
        pitch_error = -pitch

        # Balance control gains
        roll_gain = 10.0
        pitch_gain = 10.0
        roll_deriv_gain = 1.0
        pitch_deriv_gain = 1.0

        # Calculate balance corrections
        roll_correction = roll_gain * roll_error + roll_deriv_gain * angular_vel.x
        pitch_correction = pitch_gain * pitch_error + pitch_deriv_gain * angular_vel.y

        balance_commands = {
            'roll_correction': roll_correction,
            'pitch_correction': pitch_correction,
            'angular_vel_x': angular_vel.x,
            'angular_vel_y': angular_vel.y
        }

        return balance_commands

    def apply_balance_corrections(self, balance_commands):
        """Apply balance corrections to joint commands."""
        # In a real implementation, this would modify joint commands
        # to maintain balance based on the corrections
        pass

    def set_target_pose(self, x, y, z=0.0):
        """Set navigation target pose."""
        self.target_pose = {'x': x, 'y': y, 'z': z}

    def get_pipeline_status(self):
        """Get current status of the AI pipeline."""
        status = {
            'perception_rate': self.perception_frequency,
            'planning_rate': self.planning_frequency,
            'control_rate': self.control_frequency,
            'objects_detected': len(self.detected_objects),
            'obstacles_found': len(self.map_features),
            'path_length': len(self.global_plan),
            'footsteps_planned': len(self.footstep_plan)
        }
        return status


def main(args=None):
    """Main function to run the humanoid AI pipeline."""
    rclpy.init(args=args)

    ai_pipeline = HumanoidAIPipeline()

    # Example: Set a target for navigation
    ai_pipeline.set_target_pose(2.0, 2.0)

    try:
        ai_pipeline.get_logger().info('Starting Humanoid AI Pipeline...')
        rclpy.spin(ai_pipeline)
    except KeyboardInterrupt:
        ai_pipeline.get_logger().info('AI Pipeline interrupted by user')
    finally:
        ai_pipeline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()