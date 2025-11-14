import time
import numpy as np
import cv2
from picamera2 import Picamera2

from Lane_detect import LaneDetector
from Drive_control import DriveControl
from Motors import MotorConfig


# ----------------- Calibration loading ----------------- #

def load_fisheye_calibration(path="camera_fisheye_calib.npz"):
    """
    Load fisheye calibration from .npz file.

    Returns:
        K   : 3x3 camera matrix
        D   : 4x1 fisheye distortion coefficients
        DIM : (width, height)
    """
    data = np.load(path)
    K = data["K"]
    D = data["D"]
    DIM = tuple(data["DIM"])  # (width, height)
    print("[INFO] Loaded calibration from", path)
    print("[INFO] DIM =", DIM)
    return K, D, DIM


# ----------------- Camera initialization ----------------- #

def init_picamera(dim):
    """
    Initialize Picamera2 with given frame size.

    Args:
        dim: (width, height)

    Returns:
        picam2: configured and started Picamera2 instance
    """
    width, height = dim
    picam2 = Picamera2()

    config = picam2.create_video_configuration(
        main={"size": (width, height), "format": "RGB888"}
    )
    picam2.configure(config)

    picam2.start()
    time.sleep(1.0)

    print("[INFO] Picamera2 started with size", dim)
    return picam2


# ----------------- Undistortion maps ----------------- #

def create_undistort_maps(K, D, dim, balance=0.0):
    """
    Create undistort rectify maps for fisheye lens.

    Args:
        K      : 3x3 camera matrix
        D      : 4x1 distortion coefficients
        dim    : (width, height)
        balance: 0.0..1.0, trade-off between FOV and cropping

    Returns:
        map1, map2: maps for cv2.remap
    """
    width, height = dim
    DIM = (width, height)
    R = np.eye(3)

    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
        K, D, DIM, R, balance=balance
    )

    map1, map2 = cv2.fisheye.initUndistortRectifyMap(
        K, D, R, new_K, DIM, cv2.CV_16SC2
    )

    print("[INFO] Undistort maps created (balance =", balance, ")")
    return map1, map2


# ----------------- Modules initialization ----------------- #

def init_modules(frame_width, frame_height):
    """
    Initialize LaneDetector, MotorConfig and DriveControl.

    Returns:
        lane_detector, drive_controller
    """
    lane_detector = LaneDetector(
        frame_width=frame_width,
        frame_height=frame_height,
        debug=True,
        display=True,
        ipm_trapezoid_init=(140, 240, 116, 240),  # adjust if needed
    )

    motor_config = MotorConfig(
        pwm_freq=5000,
        max_speed=40.0,
    )

    drive_controller = DriveControl(
        motor_config=motor_config,
        DEBUG=True,
    )

    print("[INFO] Modules initialized (LaneDetector, DriveControl)")
    return lane_detector, drive_controller


# ----------------- Main processing loop ----------------- #

def run_main_loop(picam2, map1, map2, lane_detector, drive_controller, show_debug=True):
    """
    Main loop: capture frame, undistort, run lane detection and drive control.
    """
    print("[INFO] Main loop started. Press 'q' to quit.")

    while True:
        # 1) Capture RGB frame from Picamera2
        frame_rgb = picam2.capture_array()

        # 2) Convert to BGR for OpenCV
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # 3) Undistort using precomputed maps
        undistorted = cv2.remap(
            frame_bgr,
            map1,
            map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )

        # 4) Lane detection on undistorted image
        lane_result = lane_detector.process_frame(undistorted)

        # 5) Drive control based on lane detection result
        drive_controller.control_step(lane_result)

        # 6) Optional debug windows
        if show_debug:
            cv2.imshow("Original (BGR)", frame_bgr)
            cv2.imshow("Undistorted", undistorted)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            print("[INFO] Quit requested.")
            break

    print("[INFO] Exiting main loop.")


# ----------------- Entry point ----------------- #

def main():
    # 1) Load calibration
    K, D, DIM = load_fisheye_calibration("camera_fisheye_calib.npz")

    # 2) Init camera
    picam2 = init_picamera(DIM)

    # 3) Prepare undistortion maps
    map1, map2 = create_undistort_maps(K, D, DIM, balance=0.0)

    # 4) Init modules (lane detection + drive)
    width, height = DIM
    lane_detector, drive_controller = init_modules(width, height)

    # 5) Run main loop
    try:
        run_main_loop(picam2, map1, map2, lane_detector, drive_controller, show_debug=True)
    finally:
        # Make sure camera and windows are properly closed
        picam2.stop()
        cv2.destroyAllWindows()
        print("[INFO] Clean shutdown.")


if __name__ == "__main__":
    main()
