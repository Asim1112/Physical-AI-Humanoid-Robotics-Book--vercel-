#!/usr/bin/env python3
# Example: Visual SLAM implementation for humanoid robot

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading
import queue
from collections import deque
import time

class VisualSLAMNode(Node):
    """
    A visual SLAM implementation for humanoid robots.
    Demonstrates feature detection, tracking, mapping, and localization.
    """

    def __init__(self):
        super().__init__('visual_slam_node')

        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/rgb/image_raw')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('feature_detector', 'ORB')
        self.declare_parameter('max_features', 1000)
        self.declare_parameter('matching_threshold', 0.7)
        self.declare_parameter('map_size', 100.0)  # meters
        self.declare_parameter('publish_rate', 10.0)  # Hz

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.feature_detector_type = self.get_parameter('feature_detector').value
        self.max_features = self.get_parameter('max_features').value
        self.matching_threshold = self.get_parameter('matching_threshold').value
        self.map_size = self.get_parameter('map_size').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Feature detector and matcher
        if self.feature_detector_type == 'ORB':
            self.feature_detector = cv2.ORB_create(nfeatures=self.max_features)
        elif self.feature_detector_type == 'SIFT':
            self.feature_detector = cv2.SIFT_create(nfeatures=self.max_features)
        else:
            self.feature_detector = cv2.ORB_create(nfeatures=self.max_features)

        self.descriptor_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # Data queues
        self.image_queue = queue.Queue(maxsize=10)
        self.imu_queue = queue.Queue(maxsize=10)

        # SLAM state
        self.current_frame = None
        self.current_features = None
        self.current_descriptors = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.keyframes = deque(maxlen=100)  # Keep last 100 keyframes
        self.map_points = {}  # 3D map points
        self.feature_tracks = {}  # Track features across frames

        # Camera parameters (will be set from camera info)
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.camera_info_received = False

        # IMU integration
        self.imu_data = None
        self.orientation_filter = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/rgb/camera_info', self.camera_info_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/odometry', 10)
        self.map_pub = self.create_publisher(MarkerArray, '/visual_slam/map', 10)
        self.keyframe_pub = self.create_publisher(MarkerArray, '/visual_slam/keyframes', 10)
        self.tracking_pub = self.create_publisher(MarkerArray, '/visual_slam/tracking', 10)

        # Timer for publishing results
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_results)

        # Performance tracking
        self.processing_times = deque(maxlen=100)
        self.tracking_rate = 0.0

        self.get_logger().info(
            f'Visual SLAM node initialized with {self.feature_detector_type} detector'
        )

    def camera_info_callback(self, msg):
        """Receive camera calibration parameters."""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.distortion_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('Camera calibration parameters received')

    def image_callback(self, msg):
        """Process incoming camera images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Add to processing queue
            if not self.image_queue.full():
                self.image_queue.put((timestamp, cv_image))
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    def imu_callback(self, msg):
        """Process IMU data for sensor fusion."""
        self.imu_data = msg

    def process_slam_step(self):
        """Execute one step of the SLAM algorithm."""
        if self.image_queue.empty() or not self.camera_info_received:
            return

        try:
            timestamp, image = self.image_queue.get_nowait()
        except queue.Empty:
            return

        start_time = time.time()

        # Extract features from current image
        keypoints, descriptors = self.extract_features(image)

        if descriptors is None or len(keypoints) < 50:
            # Not enough features to proceed
            return

        # Initialize if this is the first frame
        if self.current_descriptors is None:
            self.current_frame = image
            self.current_features = keypoints
            self.current_descriptors = descriptors
            self.initialize_first_frame()
            return

        # Match features with previous frame
        matches = self.match_features(self.current_descriptors, descriptors)

        if len(matches) < 20:
            # Not enough matches to proceed
            self.current_frame = image
            self.current_features = keypoints
            self.current_descriptors = descriptors
            return

        # Estimate motion using matched features
        motion_estimate = self.estimate_motion(matches, keypoints)

        if motion_estimate is not None:
            # Update pose
            self.current_pose = self.current_pose @ motion_estimate

            # Add keyframe if motion is significant
            if self.should_add_keyframe():
                self.add_keyframe(timestamp, keypoints, descriptors, self.current_pose.copy())

            # Update map with new observations
            self.update_map(matches, keypoints, image)

        # Update current state
        self.current_frame = image
        self.current_features = keypoints
        self.current_descriptors = descriptors

        # Process time tracking
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)

    def extract_features(self, image):
        """Extract features from image using selected detector."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        keypoints, descriptors = self.feature_detector.detectAndCompute(gray, None)
        return keypoints, descriptors

    def match_features(self, desc1, desc2):
        """Match features between two descriptor sets."""
        if desc1 is None or desc2 is None:
            return []

        # Use FLANN matcher for better performance
        matches = self.descriptor_matcher.knnMatch(desc1, desc2, k=2)

        # Apply Lowe's ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < self.matching_threshold * n.distance:
                    good_matches.append(m)

        return good_matches

    def estimate_motion(self, matches, new_keypoints):
        """Estimate camera motion from feature matches."""
        if len(matches) < 10:
            return None

        # Get matched points
        prev_pts = []
        curr_pts = []

        for match in matches:
            prev_idx = match.queryIdx
            curr_idx = match.trainIdx

            prev_keypoint = self.current_features[prev_idx]
            curr_keypoint = new_keypoints[curr_idx]

            prev_pts.append([prev_keypoint.pt[0], prev_keypoint.pt[1]])
            curr_pts.append([curr_keypoint.pt[0], curr_keypoint.pt[1]])

        prev_pts = np.float32(prev_pts).reshape(-1, 1, 2)
        curr_pts = np.float32(curr_pts).reshape(-1, 1, 2)

        # Estimate essential matrix
        E, mask = cv2.findEssentialMat(
            curr_pts, prev_pts,
            self.camera_matrix,
            method=cv2.RANSAC,
            threshold=1.0,
            prob=0.999
        )

        if E is None or E.size == 0:
            return None

        # Decompose essential matrix to get rotation and translation
        _, R, t, mask = cv2.recoverPose(E, curr_pts, prev_pts, self.camera_matrix)

        # Create transformation matrix
        transform = np.eye(4)
        transform[:3, :3] = R
        transform[:3, 3] = t.flatten()

        return transform

    def initialize_first_frame(self):
        """Initialize SLAM with the first frame."""
        # Set initial pose to identity
        self.current_pose = np.eye(4)

        # Add first keyframe
        self.add_keyframe(0.0, self.current_features, self.current_descriptors, self.current_pose)

    def should_add_keyframe(self):
        """Determine if we should add a new keyframe."""
        if len(self.keyframes) == 0:
            return True

        # Check if enough time has passed or significant motion occurred
        # For simplicity, add keyframe every 10th frame or if translation > 0.1m
        return len(self.keyframes) % 10 == 0

    def add_keyframe(self, timestamp, features, descriptors, pose):
        """Add a new keyframe to the map."""
        keyframe = {
            'timestamp': timestamp,
            'features': features,
            'descriptors': descriptors,
            'pose': pose.copy(),
            'id': len(self.keyframes)
        }
        self.keyframes.append(keyframe)

    def update_map(self, matches, new_keypoints, image):
        """Update the map with new observations."""
        for match in matches:
            prev_idx = match.queryIdx
            curr_idx = match.trainIdx

            # This is a simplified map update
            # In a real implementation, you would triangulate 3D points
            curr_keypoint = new_keypoints[curr_idx]

            # Add to map points (simplified - would need triangulation in practice)
            point_2d = curr_keypoint.pt
            point_3d = self.triangulate_point(prev_idx, curr_idx, point_2d)

            if point_3d is not None:
                point_id = f"point_{len(self.map_points)}"
                self.map_points[point_id] = {
                    'coordinates': point_3d,
                    'observations': [len(self.keyframes) - 1]  # Current keyframe
                }

    def triangulate_point(self, prev_idx, curr_idx, curr_point_2d):
        """Triangulate 3D point from stereo observations."""
        # Simplified triangulation - in practice, you'd need stereo or multiple views
        # For now, return a placeholder
        return np.array([curr_point_2d[0] * 0.01, curr_point_2d[1] * 0.01, 1.0])

    def publish_results(self):
        """Publish SLAM results as ROS messages."""
        # Process SLAM step
        self.process_slam_step()

        # Publish odometry
        self.publish_odometry()

        # Publish map
        self.publish_map()

        # Publish keyframes
        self.publish_keyframes()

        # Publish tracking visualization
        self.publish_tracking()

        # Log performance periodically
        if len(self.processing_times) > 0:
            avg_time = np.mean(self.processing_times)
            fps = 1.0 / avg_time if avg_time > 0 else 0
            self.get_logger().info_throttle(
                5.0,  # Log every 5 seconds
                f'SLAM Performance: {fps:.1f} FPS, '
                f'Avg processing: {avg_time*1000:.1f}ms, '
                f'Map points: {len(self.map_points)}, '
                f'Keyframes: {len(self.keyframes)}'
            )

    def publish_odometry(self):
        """Publish odometry information."""
        if len(self.keyframes) == 0:
            return

        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'camera_link'

        # Extract position and orientation from current pose
        position = self.current_pose[:3, 3]
        rotation_matrix = self.current_pose[:3, :3]

        # Convert rotation matrix to quaternion
        r = R.from_matrix(rotation_matrix)
        quat = r.as_quat()  # [x, y, z, w]

        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Publish odometry
        self.odom_pub.publish(odom_msg)

    def publish_map(self):
        """Publish 3D map points."""
        marker_array = MarkerArray()

        for i, (point_id, point_data) in enumerate(self.map_points.items()):
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'map_points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position
            pos = point_data['coordinates']
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]

            # Orientation (identity for points)
            marker.pose.orientation.w = 1.0

            # Scale
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Color (blue for map points)
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.map_pub.publish(marker_array)

    def publish_keyframes(self):
        """Publish keyframe poses."""
        marker_array = MarkerArray()

        for i, keyframe in enumerate(self.keyframes):
            # Keyframe position marker
            marker = Marker()
            marker.header = Header()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = 'map'
            marker.ns = 'keyframes'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            pose = keyframe['pose']
            marker.pose.position.x = pose[0, 3]
            marker.pose.position.y = pose[1, 3]
            marker.pose.position.z = pose[2, 3]

            # Convert rotation matrix to quaternion
            r = R.from_matrix(pose[:3, :3])
            quat = r.as_quat()
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]

            # Scale
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1

            # Color (green for keyframes)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.keyframe_pub.publish(marker_array)

    def publish_tracking(self):
        """Publish feature tracking visualization."""
        marker_array = MarkerArray()

        # For simplicity, visualize some tracked features
        if self.current_features and len(self.current_features) > 0:
            for i, feature in enumerate(self.current_features[:20]):  # Limit to 20 features
                marker = Marker()
                marker.header = Header()
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.header.frame_id = 'camera_link'  # Relative to camera
                marker.ns = 'tracked_features'
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                # Position (project feature to 3D assuming depth)
                marker.pose.position.x = feature.pt[0] * 0.001  # Scale down
                marker.pose.position.y = feature.pt[1] * 0.001
                marker.pose.position.z = 1.0  # Assume depth of 1m

                marker.pose.orientation.w = 1.0

                # Scale
                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02

                # Color (red for tracked features)
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8

                marker_array.markers.append(marker)

        self.tracking_pub.publish(marker_array)


def main(args=None):
    """Main function to run the Visual SLAM node."""
    rclpy.init(args=args)

    visual_slam = VisualSLAMNode()

    try:
        visual_slam.get_logger().info('Starting Visual SLAM node...')
        rclpy.spin(visual_slam)
    except KeyboardInterrupt:
        visual_slam.get_logger().info('Visual SLAM node interrupted by user')
    finally:
        visual_slam.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()