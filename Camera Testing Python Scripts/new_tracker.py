import cv2
import numpy as np
from typing import Tuple, List

class ObjectTracker:
    """Tracks multiple specific colored objects and provides their coordinates."""

    def __init__(self):
        pass

    def process_frame(self, frame: np.ndarray) -> Tuple[List[Tuple[int, int]], bool, np.ndarray]:
        """Finds multiple red objects and returns their coordinates."""
        height, width, _ = frame.shape
        center_x = width // 2
        center_y = height // 2

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # red color ranges (HSV)
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([165, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # find red objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        targets = []

        if contours:
            for contour in contours:
                if cv2.contourArea(contour) > 1000:
                    x, y, w, h = cv2.boundingRect(contour)
                    target_x = x + w // 2
                    target_y = y + h // 2
                    targets.append((target_x, target_y))

        # If we found objects, return the list and True (tracking active)
        if len(targets) > 0:
            return targets, True, frame

        # Default fallback: return center point and False (tracking inactive)
        return [(center_x, center_y)], False, frame