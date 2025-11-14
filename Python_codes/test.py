import cv2
import time
import numpy as np
from picamera2 import Picamera2

from sign_detect import SignDetector
from lane_detect import LaneDetector
from drive_control import DriveControl
from motors import MotorConfig


def main():

    # --- 1. Initialize sign detector ---
    sign_detector = SignDetector(
        debug=True,
        display=True,
        base_path="sign_database",
        preload=True
    )

    # --- 2. Initialize lane detector ---
    lane_detector = LaneDetector(
        frame_width=640,
        frame_height=480,
        debug=True,
        display=True,
    )

    # --- 3. Initialize motor config + drive control ---
    motor_config = MotorConfig(
        pwm_freq=5000,
        max_speed=40.0
    )

    drive_controller = DriveControl(
        motor_config,
        DEBUG=True
    )

    # --- 4. Initialize Raspberry Pi Camera ---
    print("[INFO] Initializing Raspberry Pi camera...")

    picam2 = Picamera2()

    config = picam2.create_video_configuration(
        main={"size": (1280, 720), "format": "RGB888"}  # szybsza, lekka rozdzielczość
    )
    picam2.configure(config)
    picam2.start()

    time.sleep(1)
    print("[INFO] Camera started. Press 'q' to quit.")

    try:
        while True:
            # --- 5. Grab frame from Pi camera ---
            frame = picam2.capture_array()

            # Resize to lane detector input resolution
            frame = cv2.resize(frame, (640, 480))

            # --- 6. Run detectors ---
            sign_result = sign_detector.detect(frame)
            lane_result = lane_detector.process_frame(frame)

            # --- 7. Drive control ---
            drive_controller.control_step(lane_result)

            # --- 8. Log detected signs ---
            detections = sign_result["detections"]
            if sign_result["detected"]:
                print(f"[INFO] Detected {len(detections)} sign(s):")
                for d in detections:
                    print(f"   - {d['name']} ({d['score']:.1f}%) [{d['color']} {d['shape']}]")

            # --- 9. Display output frame from sign detector ---
            cv2.imshow("Detected Output", sign_result["output_frame"])

            # --- 10. Quit on 'q' ---
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("[INFO] Exiting program.")
                break

    finally:
        # --- 11. Cleanup ---
        picam2.stop()
        cv2.destroyAllWindows()
        # Dodatkowo możesz dodać GPIO cleanup w MotorConfig jeśli dodasz tam taką metodę.
        print("[INFO] Program ended cleanly.")


if __name__ == "__main__":
    main()
