#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseArray, Pose
from std_msgs.msg import Header
import pyrealsense2 as rs
import numpy as np
import cv2
import sys


class RedObjectTrackerNode(Node):
    def __init__(self):
        super().__init__('red_object_tracker_node')

        # Declare ROS 2 parameters for runtime tuning
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('min_contour_area', 750)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')

        # Fetch parameter values
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.min_area = self.get_parameter('min_contour_area').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Publishers
        # 1. Publishes primary/closest object coordinate
        self.single_target_pub = self.create_publisher(PointStamped, '/tracked_object/point_3d', 10)
        # 2. Publishes all detected red objects as a list of poses
        self.multi_target_pub = self.create_publisher(PoseArray, '/tracked_object/all_points_3d', 10)

        # Initialize RealSense Pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        try:
            self.pipeline.start(self.config)
            self.align = rs.align(rs.stream.color)
            self.get_logger().info("RealSense D415 stream started and aligned to color.")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense camera: {e}")
            raise e

        # Timer loop for processing frames
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.process_frame_callback)

    def process_frame_callback(self):
        frames = self.pipeline.poll_for_frames()
        if not frames:
            return

        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

        # HSV Thresholding for Red Color
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 80])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 80])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = self.frame_id

        first_valid_point = None

        for contour in contours:
            if cv2.contourArea(contour) > self.min_area:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2

                # Verify depth bounds
                if center_x >= depth_frame.get_width() or center_y >= depth_frame.get_height():
                    continue

                depth_m = depth_frame.get_distance(center_x, center_y)
                if depth_m <= 0:
                    continue  # Invalid depth data pixel

                # Convert 2D pixel + depth to 3D spatial coordinate (X, Y, Z in meters)
                point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], depth_m)

                # Append to PoseArray
                pose = Pose()
                pose.position.x = float(point_3d[0])
                pose.position.y = float(point_3d[1])
                pose.position.z = float(point_3d[2])
                pose_array_msg.poses.append(pose)

                if first_valid_point is None:
                    first_valid_point = point_3d

        # Publish multi-target positions
        if len(pose_array_msg.poses) > 0:
            self.multi_target_pub.publish(pose_array_msg)

            # Publish single primary target (first detected object)
            point_msg = PointStamped()
            point_msg.header = pose_array_msg.header
            point_msg.point.x = float(first_valid_point[0])
            point_msg.point.y = float(first_valid_point[1])
            point_msg.point.z = float(first_valid_point[2])
            self.single_target_pub.publish(point_msg)
            self.get_logger().info(f"x={first_valid_point[0]:.4f}, y={first_valid_point[1]:.4f}, z={first_valid_point[2]:.4f}")

    def destroy_node(self):
        if self.pipeline:
            self.pipeline.stop()
            self.get_logger().info("RealSense camera pipeline stopped.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RedObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()
