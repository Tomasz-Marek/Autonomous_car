import cv2

from sign_detect import SignDetector
from lane_detect import LaneDetector
from drive_control import DriveControl
from motors import MotorConfig


def main():
    # 1. Initialize sign detector
    sign_detector = SignDetector(
        debug=True,          # console logs
        display=True,        # sign detection debug windows
        base_path="sign_database",
        preload=True         # preload all sign groups
    )

    # 2. Initialize lane detector
    lane_detector = LaneDetector(
        frame_width=640,
        frame_height=480,
        debug=True,
        display=True,
        # ipm_trapezoid_init left as default
    )

    # 3. Initialize motor config and drive controller
    motor_config = MotorConfig(
        pwm_freq=5000,
        max_speed=40.0
    )

    drive_controller = DriveControl(
        motor_config,
        DEBUG=True
    )

    # 4. Initialize camera (0 = default webcam / USB camera)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Camera is not available.")
        return

    print("[INFO] Camera started. Press 'q' to quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[WARN] Failed to grab frame.")
                break

            # Optionally resize frame to match lane detector resolution
            frame = cv2.resize(frame, (640, 480))

            # 5. Run detectors
            sign_result = sign_detector.detect(frame)
            lane_result = lane_detector.process_frame(frame)

            # 6. High-level drive control based on lane detection
            drive_controller.control_step(lane_result)

            # 7. Print detected signs to console (if any)
            detections = sign_result["detections"]
            if sign_result["detected"]:
                print(f"[INFO] Detected {len(detections)} sign(s):")
                for d in detections:
                    print(f"   - {d['name']} ({d['score']:.1f}%) [{d['color']} {d['shape']}]")

            # 8. Show sign detector output frame (if display=True, it may already show its own windows)
            cv2.imshow("Detected Output", sign_result["output_frame"])

            # 9. Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[INFO] Test finished by user.")
                break

    finally:
        # 10. Cleanup
        cap.release()
        cv2.destroyAllWindows()
        # (opcjonalnie możesz tu dodać stop silników / GPIO.cleanup(),
        #  jeśli zaimplementujesz takie metody w MotorConfig / DriveControl)


if __name__ == "__main__":
    main()
