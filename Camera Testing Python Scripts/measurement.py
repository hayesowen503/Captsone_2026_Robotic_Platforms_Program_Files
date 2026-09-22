import pyrealsense2 as rs
#import numpy as np
#import csv
#import datetime
#from typing import Tuple, Optional


class DistanceMeasurer:
    """Distance measurement and data handling"""

    def __init__(self, data_file='data/measurements.csv'):
        self.data_file = data_file
        self.measurements = []

    def mm_to_inches(self, mm: float) -> float:
        """Convert millimeters to inches"""
        return mm / 25.4

    def get_measurement_info(self, x: int, y: int, depth_frame, depth_intrin) -> dict:
        """Get comprehensive measurement info including 3D coordinates"""
        try:
            # Check bounds against the frame
            if x < 0 or x >= depth_frame.get_width() or y < 0 or y >= depth_frame.get_height():
                return {'valid': False, 'message': 'Out of bounds', 'point_3d': None}

            # get_distance() retrieves the exact depth in meters
            depth_m = depth_frame.get_distance(x, y)

            if depth_m == 0:
                return {'valid': False, 'message': 'No valid depth data', 'point_3d': None}

            # --- THE MAGIC HAPPENS HERE ---
            # Calculate the 3D point (X, Y, Z in meters)
            point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [x, y], depth_m)
            x_m, y_m, z_m = point_3d

            distance_mm = z_m * 1000
            distance_inches = self.mm_to_inches(distance_mm)

            return {
                'valid': True,
                # RealSense orientation: Z is forward, X is right, Y is down
                'message': f"X:{x_m:.2f}m, Y:{y_m:.2f}m, Z:{z_m:.2f}m",
                'distance_mm': distance_mm,
                'distance_inches': distance_inches,
                'point_3d': point_3d
            }

        except Exception as e:
            print(f"Error measuring distance: {e}")
            return {'valid': False, 'message': 'Measurement Error', 'point_3d': None}

    """
    def save_measurement(self, x_pixel: int, y_pixel: int, measurement_info: dict, timestamp: str = None):
        #Save 3D measurement to CSV file
        if timestamp is None:
            timestamp = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        x_m, y_m, z_m = measurement_info['point_3d']

        measurement = {
            'timestamp': timestamp,
            'x_pixel': x_pixel,
            'y_pixel': y_pixel,
            'x_meters': round(x_m, 4),
            'y_meters': round(y_m, 4),
            'z_meters': round(z_m, 4),
            'distance_inches': round(measurement_info['distance_inches'], 2)
        }

        self.measurements.append(measurement)

        # Save to CSV
        try:
            with open(self.data_file, 'a', newline='') as csvfile:
                fieldnames = ['timestamp', 'x_pixel', 'y_pixel', 'x_meters', 'y_meters', 'z_meters', 'distance_inches']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                # Write header if file is empty
                if csvfile.tell() == 0:
                    writer.writeheader()

                writer.writerow(measurement)

            print(f"Measurement saved: {measurement_info['message']}")

        except Exception as e:
            print(f"Error saving measurement: {e}")

    def load_measurements(self) -> list:
        # Same as your original code
        try:
            with open(self.data_file, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                return list(reader)
        except FileNotFoundError:
            print(f"No existing measurements file found")
            return []
        except Exception as e:
            print(f"Error loading measurements: {e}")
            return []
    """