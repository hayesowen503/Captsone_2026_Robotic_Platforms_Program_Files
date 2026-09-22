import pyrealsense2 as rs
import numpy as np
import cv2


class RealSenseCamera:
    """RealSense camera handler class"""

    def __init__(self, width=640, height=480, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = None
        self.config = None
        self.align = None
        self.is_streaming = False

    def initialize(self):
        """Initialize the camera"""
        try:
            self.pipeline = rs.pipeline()
            self.config = rs.config()

            self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
            self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

            # Start streaming
            self.pipeline.start(self.config)

            # Create an align object to map depth frames to color frames
            align_to = rs.stream.color
            self.align = rs.align(align_to)

            self.is_streaming = True

            print("Camera initialized successfully (Aligned Mode)")
            return True

        except Exception as e:
            print(f"Camera initialization failed: {e}")
            return False

    def get_frames(self):
        """Get aligned frames and intrinsics from camera"""
        if not self.is_streaming:
            return None, None, None, None

        try:
            frames = self.pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if depth_frame and color_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                # Retrieve camera intrinsics required for 3D calculation
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

                # We now return the raw depth_frame and intrinsics too
                return depth_image, color_image, depth_frame, depth_intrin
            else:
                return None, None, None, None

        except Exception as e:
            print(f"Error getting frames: {e}")
            return None, None, None, None

    def stop(self):
        """Stop camera streaming"""
        if self.pipeline and self.is_streaming:
            self.pipeline.stop()
            self.is_streaming = False
            print("Camera stopped")

    """
    def get_camera_info(self):
        #Get camera information
        if not self.is_streaming:
            return None
        try:
            device = self.pipeline.get_active_profile().get_device()
            return {
                'name': device.get_info(rs.camera_info.name),
                'serial': device.get_info(rs.camera_info.serial_number),
                'firmware': device.get_info(rs.camera_info.firmware_version)
            }
        except Exception as e:
            print(f"Error getting camera info: {e}")
            return None
    """