import cv2
import numpy as np
from typing import Tuple, List


class MeasurementUI:
    """User interface for distance measurement"""

    def __init__(self, window_name="RealSense Distance Measurement"):
        self.window_name = window_name
        self.crosshair_size = 20
        self.crosshair_thickness = 2
        self.text_color = (255, 255, 255)  # White
        self.text_bg_color = (0, 0, 0)  # Black

    def draw_crosshair(self, image: np.ndarray, x: int, y: int, is_tracking: bool) -> np.ndarray:
        # OpenCV uses BGR formatting: Green if tracking, Red if resting in center
        color = (0, 255, 0) if is_tracking else (0, 0, 255)

        cv2.line(image, (x - self.crosshair_size, y), (x + self.crosshair_size, y), color, self.crosshair_thickness)
        cv2.line(image, (x, y - self.crosshair_size), (x, y + self.crosshair_size), color, self.crosshair_thickness)
        return image

    def draw_text_with_background(self, image: np.ndarray, text: str, position: Tuple[int, int]) -> np.ndarray:
        x, y = position
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5  # Slightly smaller to accommodate multiple objects
        thickness = 1
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

        # Prevent text from drawing outside the window frame
        x = max(0, min(x, image.shape[1] - text_width))
        y = max(text_height + baseline, min(y, image.shape[0]))

        cv2.rectangle(image, (x, y - text_height - baseline), (x + text_width, y + baseline), self.text_bg_color, -1)
        cv2.putText(image, text, (x, y), font, font_scale, self.text_color, thickness)
        return image

    def draw_measurement_info(self, image: np.ndarray, measurement_info: dict, x: int, y: int,
                              obj_id: int, is_tracking: bool) -> np.ndarray:
        """Draw 3D measurement info dynamically near the crosshair"""
        # Offset the text slightly above and to the right of the crosshair
        text_x_offset = x + 15
        text_y_offset = y - 25

        if not measurement_info['valid']:
            text = f"Obj {obj_id}: {measurement_info['message']}" if is_tracking else f"{measurement_info['message']}"
            self.draw_text_with_background(image, text, (text_x_offset, text_y_offset))
            return image

        spatial_text = f"Obj {obj_id}: {measurement_info['message']}" if is_tracking else f"{measurement_info['message']}"
        coord_text = f"Pixel: ({x}, {y})"

        self.draw_text_with_background(image, spatial_text, (text_x_offset, text_y_offset))
        self.draw_text_with_background(image, coord_text, (text_x_offset, text_y_offset + 25))

        return image

    def create_depth_colormap(self, depth_image: np.ndarray) -> np.ndarray:
        depth_normalized = cv2.convertScaleAbs(depth_image, alpha=0.03)
        return cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

    def display_images(self, color_image: np.ndarray, depth_image: np.ndarray, targets_data: List[dict]) -> int:
        depth_colormap = self.create_depth_colormap(depth_image)

        color_render = color_image.copy()
        depth_render = depth_colormap.copy()

        # Loop through all targets and draw their specific data
        for i, target in enumerate(targets_data):
            x, y = target['x'], target['y']
            is_tracking = target['is_tracking']
            info = target['info']

            color_render = self.draw_crosshair(color_render, x, y, is_tracking)
            depth_render = self.draw_crosshair(depth_render, x, y, is_tracking)

            color_render = self.draw_measurement_info(color_render, info, x, y, i + 1, is_tracking)
            depth_render = self.draw_measurement_info(depth_render, info, x, y, i + 1, is_tracking)

        cv2.imshow('Color Stream', color_render)
        cv2.imshow('Depth Stream', depth_render)

        # Return the keypress integer directly
        return cv2.waitKey(1) & 0xFF