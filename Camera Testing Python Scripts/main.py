import cv2
import os
from camera_handler import RealSenseCamera
from measurement import DistanceMeasurer
from ui import MeasurementUI
from new_tracker import ObjectTracker


def main():
    os.makedirs('data', exist_ok=True)

    camera = RealSenseCamera()
    measurer = DistanceMeasurer()
    ui = MeasurementUI()
    tracker = ObjectTracker()

    if not camera.initialize():
        print("Failed to initialize camera. Exiting.")
        return

    """
    camera_info = camera.get_camera_info()
    if camera_info:
        print(f"Camera: {camera_info['name']}")
        print(f"Serial: {camera_info['serial']}")
        print(f"Firmware: {camera_info['firmware']}")
    """

    cv2.namedWindow('Color Stream', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Depth Stream', cv2.WINDOW_AUTOSIZE)

    print("\nAuto-Tracking 3D Spatial Measurement Tool Started!")
    print("Show red objects to track them, press 'q' to quit")

    try:
        while True:
            depth_image, color_image, depth_frame, depth_intrin = camera.get_frames()

            if depth_image is None or color_image is None:
                continue

                # Process the color frame to find all targets
            targets, is_tracking, tracked_color_image = tracker.process_frame(color_image)

            targets_data = []

            # Retrieve spatial math for each object found
            for (crosshair_x, crosshair_y) in targets:
                measurement_info = measurer.get_measurement_info(
                    crosshair_x, crosshair_y, depth_frame, depth_intrin
                )

                targets_data.append({
                    'x': crosshair_x,
                    'y': crosshair_y,
                    'info': measurement_info,
                    'is_tracking': is_tracking
                })

            # Pass the aggregated target list to the UI
            key = ui.display_images(tracked_color_image, depth_image, targets_data)

            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nApplication interrupted by user")

    except Exception as e:
        print(f"Unexpected error: {e}")

    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Application closed successfully")


if __name__ == "__main__":
    main()