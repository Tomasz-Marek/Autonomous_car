from picamera2 import Picamera2
import cv2
import time
import os


def main():
    # Number of internal corners (for a 10x7 squares board -> 9x6 corners)
    pattern_size = (9, 6)

    # Folder where calibration images will be stored
    save_dir = "calib_images"
    os.makedirs(save_dir, exist_ok=True)

    img_counter = 0

    # Initialize Raspberry Pi camera
    picam2 = Picamera2()

    frame_width = 640
    frame_height = 480

    config = picam2.create_video_configuration(
        main={"size": (frame_width, frame_height), "format": "RGB888"}
    )
    picam2.configure(config)

    picam2.start()
    time.sleep(1.0)

    print("[INFO] Camera started.")
    print("[INFO] Press 's' to save frame.")
    print("[INFO] Press 'q' to quit.")

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    while True:
        frame_bgr = picam2.capture_array()
        #frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags=(
                cv2.CALIB_CB_ADAPTIVE_THRESH
                + cv2.CALIB_CB_NORMALIZE_IMAGE
                + cv2.CALIB_CB_FAST_CHECK
            ),
        )

        vis = frame_bgr.copy()

        if found:
            corners_refined = cv2.cornerSubPix(
                gray,
                corners,
                winSize=(5, 5),
                zeroZone=(-1, -1),
                criteria=criteria,
            )
            cv2.drawChessboardCorners(vis, pattern_size, corners_refined, found)
            label = "FOUND"
            color = (0, 255, 0)
        else:
            label = "NOT FOUND"
            color = (0, 0, 255)

        cv2.putText(
            vis,
            label,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            2,
        )

        cv2.imshow("Chessboard test (Picamera2)", vis)

        key = cv2.waitKey(1) & 0xFF

        # Quit
        if key == ord("q"):
            print("[INFO] Quit requested.")
            break

        # Save frame
        if key == ord("s"):
            filename = os.path.join(save_dir, f"calib_{img_counter:03d}.png")
            cv2.imwrite(filename, frame_bgr)
            img_counter += 1
            print(f"[INFO] Saved: {filename}")

    picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
