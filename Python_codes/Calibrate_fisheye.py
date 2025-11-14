import cv2
import numpy as np
import glob
import os


def main():
    # Internal corners of the chessboard (for 10x7 squares -> 9x6 corners)
    pattern_size = (9, 6)

    # Square size in meters (for 20 mm squares -> 0.02)
    square_size = 0.02

    # Folder with calibration images
    images_glob = os.path.join("calib_images", "calib_*.png")
    images = glob.glob(images_glob)

    if len(images) == 0:
        print("[ERROR] No images found in", images_glob)
        return

    print(f"[INFO] Found {len(images)} calibration images")

    # Prepare object points for one view
    # For fisheye calibrate, shape should be (1, N, 3)
    objp = np.zeros((1, pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []  # list of object points
    imgpoints = []  # list of image points

    img_shape = None

    # Criteria for corner refinement
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            print(f"[WARN] Could not read {fname}, skipping")
            continue

        if img_shape is None:
            img_shape = img.shape[:2][::-1]  # (width, height)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        found, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags=(
                cv2.CALIB_CB_ADAPTIVE_THRESH
                + cv2.CALIB_CB_NORMALIZE_IMAGE
                + cv2.CALIB_CB_FAST_CHECK
            ),
        )

        if not found:
            print(f"[WARN] Chessboard NOT found in {fname}, skipping")
            continue

        corners_sub = cv2.cornerSubPix(
            gray,
            corners,
            winSize=(5, 5),
            zeroZone=(-1, -1),
            criteria=criteria,
        )

        # For fisheye: corners must be shape (1, N, 2)
        objpoints.append(objp)
        imgpoints.append(corners_sub.reshape(1, -1, 2))

        print(f"[INFO] Used {fname} for calibration")

    if len(objpoints) < 3:
        print("[ERROR] Not enough valid calibration images (need at least 3).")
        return

    print(f"[INFO] Using {len(objpoints)} valid images for calibration")

    # Fisheye calibration
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = []
    tvecs = []

    rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
        objectPoints=objpoints,
        imagePoints=imgpoints,
        image_size=img_shape,
        K=K,
        D=D,
        rvecs=rvecs,
        tvecs=tvecs,
        flags=(
            cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
            + cv2.fisheye.CALIB_CHECK_COND
            + cv2.fisheye.CALIB_FIX_SKEW
        ),
        criteria=(
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            100,
            1e-6,
        ),
    )

    print("[INFO] Fisheye RMS error:", rms)
    print("[INFO] Camera matrix K:")
    print(K)
    print("[INFO] Distortion coefficients D:")
    print(D.ravel())

    # Save calibration to file
    np.savez(
        "camera_fisheye_calib.npz",
        K=K,
        D=D,
        DIM=np.array(img_shape, dtype=np.int32),
    )
    print("[INFO] Calibration saved to camera_fisheye_calib.npz")


if __name__ == "__main__":
    main()
