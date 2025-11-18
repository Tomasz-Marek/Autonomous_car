import math
import cv2
import numpy as np
from collections import deque

from Motors import MotorConfig


class DriveControl:
    """
    DriveControl

    High-level drive controller that:
      - Receives lane detection results from LaneDetector (result dict).
      - Computes steering and wheel speeds for lane following using a simple P-like controller.
      - Maintains a short history of lane detection quality to decide when to enter/exit fallback mode.
      - Provides a fallback behavior when lane detection becomes unreliable.
      - Stops the vehicle gracefully in clearly unsafe situations.
      - Exposes human-readable status messages and a debug banner window for diagnostics.
    """

    # ------------- basic callback for trackbars -------------
    @staticmethod
    def _nothing(a):
        """Dummy callback for OpenCV trackbars."""
        pass

    def __init__(self, motor_config: MotorConfig, DEBUG: bool = False):
        """
        Initialize drive control module.

        Args:
            motor_config: MotorConfig instance used to control motor PWM pins.
            DEBUG: If True, creates OpenCV trackbars and debug banner window.
        """
        self.DEBUG = DEBUG
        self.manual_stop = False

        # Human-readable status for diagnostics / banner
        self.status_mode = "idle"        # e.g.: "lane_following - run", "fallback - stopped"
        self.status_msg = ""             # detailed status message
        self.status_stopped = False      # True if controller intentionally stopped the car

        # Motor configuration / setup
        self.motor = motor_config
        self.motor.setup()

        # History of recent lane detection results (for fallback decision)
        self.history_length = 20
        self.history = deque(maxlen=self.history_length)  # each element: dict with lane_ok, left_lane_ok, ...

        # Fallback state flags
        self.in_fallback = False
        self.last_good_state = None  # reserved for future, not actively used yet

        # Base speed and controller gains
        self.BASE_SPEED = 30          # nominal forward speed (in percent of MAX_SPEED in MotorConfig)
        self.MAX_STEERING = 46        # maximum steering correction (also in "speed units")
        self.K_lateral = 1.3          # gain for lateral error
        self.K_head = 1.6             # gain for heading/angle error

        # Hysteresis thresholds for entering / exiting fallback based on lane_ok history
        self.Min_good_state_count = int(self.history_length * 0.5)   # below this → enter fallback
        self.Max_good_state_count = int(self.history_length * 0.75)  # above this → leave fallback

        # Lookahead on normalized lane curve: 0.0 = bottom (near vehicle), 1.0 = top (far)
        self.y_L = 0.3  # Lookahead position on normalized lane curve (0.0 = bottom, 1.0 = top)

        # Initialize OpenCV trackbars (debug only)
        if self.DEBUG:
            self._initialize_values_trackbars()

    # ====================== STATUS BANNER ======================
    def show_status_banner(self):
        """
        Show a separate OpenCV window with a status banner.

        The banner color indicates:
          - Red    -> vehicle is stopped by controller.
          - Orange -> fallback mode is active and vehicle is moving.
          - Green  -> normal lane following mode.
        """
        if not self.DEBUG:
            return

        width, height = 700, 100
        banner = np.zeros((height, width, 3), dtype=np.uint8)

        # Background color based on state
        mode_str = (self.status_mode or "").lower()
        if self.status_stopped:
            color = (0, 0, 255)        # red
        elif "fallback" in mode_str:
            color = (0, 165, 255)      # orange
        else:
            color = (0, 128, 0)        # green

        cv2.rectangle(banner, (0, 0), (width, height), color, -1)

        # Line 1: MODE
        mode_text = f"MODE: {self.status_mode or 'UNKNOWN'}"
        cv2.putText(
            banner,
            mode_text,
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )

        # Line 2: MESSAGE
        msg_text = self.status_msg or "NO STATUS MESSAGE"
        cv2.putText(
            banner,
            msg_text,
            (10, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        cv2.imshow("Drive Status", banner)

    # ====================== TRACKBARS (GAINS & LOOKAHEAD) ======================
    def _initialize_values_trackbars(self):
        """
        Create OpenCV trackbars to tune:
          - MAX_STEERING
          - K_lateral
          - K_head
          - y_L

        These are only available in DEBUG mode.
        """
        cv2.namedWindow("Values Setup")
        cv2.resizeWindow("Values Setup", 400, 200)

        # Max steering: 0..50
        cv2.createTrackbar(
            "MaxSteering", "Values Setup",
            int(self.MAX_STEERING), 50,
            self._nothing
        )

        # K_lateral in range 0.00..3.00, stored as x100
        cv2.createTrackbar(
            "K_lat x100", "Values Setup",
            int(self.K_lateral * 100), 300,
            self._nothing
        )

        # K_head in range 0.00..2.00, stored as x100
        cv2.createTrackbar(
            "K_head x100", "Values Setup",
            int(self.K_head * 100), 200,
            self._nothing
        )

        # y_L in range 0.30..0.90 (x100)
        cv2.createTrackbar(
            "y_L x100", "Values Setup",
            int(self.y_L * 100), 90,
            self._nothing
        )

    def _update_values_from_trackbars(self):
        """
        Read current values from trackbars and update:
          - MAX_STEERING
          - K_lateral
          - K_head
          - y_L
        """
        if not self.DEBUG:
            return

        self.MAX_STEERING = cv2.getTrackbarPos("MaxSteering", "Values Setup")

        k_lat_x100 = cv2.getTrackbarPos("K_lat x100", "Values Setup")
        self.K_lateral = k_lat_x100 / 100.0

        k_head_x100 = cv2.getTrackbarPos("K_head x100", "Values Setup")
        self.K_head = k_head_x100 / 100.0

        y_L_x100 = cv2.getTrackbarPos("y_L x100", "Values Setup")
        # clamp from below to avoid degenerate lookahead
        self.y_L = max(0.3, y_L_x100 / 100.0)

    # ====================== HISTORY & FALLBACK LOGIC ======================
    def update_history(self, lane_result: dict):
        """
        Append current lane detection result to history and update last_good_state.

        Args:
            lane_result: dict returned by LaneDetector.process_frame().
        """
        if lane_result is None:
            # Safety: no lane result -> stop motors
            self.motor.set_speeds(0, 0)
            return

        history_result = {
            "lane_ok": lane_result.get("lane_ok"),
            "left_lane_ok": lane_result.get("left_lane_ok"),
            "right_lane_ok": lane_result.get("right_lane_ok"),
            "crossroad_type": lane_result.get("crossroad_type"),
            "poly": lane_result.get("poly"),
        }
        self.history.append(history_result)

        # Keep last good state (can be used in more advanced fallback strategies)
        if history_result["lane_ok"]:
            self.last_good_state = history_result

    def update_fallback(self):
        """
        Decide whether to enter or exit fallback mode based on lane_ok history.

        Uses hysteresis:
          - if good detections < Min_good_state_count -> enter fallback.
          - if good detections > Max_good_state_count -> exit fallback.
        """
        if len(self.history) < self.history_length:
            # Not enough data yet to make a reliable decision
            return

        good_state_count = sum(1 for state in self.history if state["lane_ok"])
        if good_state_count < self.Min_good_state_count:
            self.in_fallback = True
        elif good_state_count > self.Max_good_state_count:
            self.in_fallback = False

    # ====================== CROSSROAD HANDLING (PLACEHOLDER) ======================
    def crossroad_step(self, lane_result: dict):
        """
        Handle driving behavior when a crossroad is detected.

        Currently: simple, safe behavior -> stop the vehicle and show a message.
        Later this can be extended with sign-based decision (turn left/right/straight).
        """
        self.motor.set_speeds(0, 0)
        self.status_mode = "crossroad - stopped"
        self.status_stopped = True
        cross_type = lane_result.get("crossroad_type", "unknown")
        self.status_msg = f"STOP: CROSSROAD DETECTED ({cross_type}), BEHAVIOR NOT IMPLEMENTED YET"

    # ====================== FALLBACK MODE ======================
    def fallback_step(self, lane_result: dict):
        """
        Fallback control step used when lane detection becomes unreliable.

        Logic:
          - Reduce base speed.
          - If only one side is likely visible -> gently steer towards it.
          - If both sides have very low confidence -> stop the vehicle.
        """
        base_speed = self.BASE_SPEED * 0.6
        turn_amount = self.MAX_STEERING * 0.5  # 50% of maximum steering

        left_ok = bool(lane_result.get("left_lane_ok"))
        right_ok = bool(lane_result.get("right_lane_ok"))
        left_conf = lane_result.get("left_confidence", 0.0)
        right_conf = lane_result.get("right_confidence", 0.0)

        # Default: slow straight drive
        left_speed = base_speed
        right_speed = base_speed

        # Case 1: we probably see only the right lane -> steer towards the right side
        if (not left_ok and right_ok) or (left_conf < 0.3 and right_conf > 0.6):
            # Left wheel faster, right wheel slower -> turn right (depending on kinematics)
            left_speed = base_speed + turn_amount
            right_speed = base_speed - turn_amount

        # Case 2: we probably see only the left lane -> steer towards the left side
        elif (left_ok and not right_ok) or (left_conf > 0.6 and right_conf < 0.3):
            left_speed = base_speed - turn_amount
            right_speed = base_speed + turn_amount

        # Case 3: both sides are weak -> stop completely
        elif left_conf < 0.3 and right_conf < 0.3:
            left_speed = 0
            right_speed = 0
            self.status_mode = "fallback - stopped"
            self.status_stopped = True
            self.status_msg = "STOP: BOTH LINES HAVE BEEN LOST (LOW CONFIDENCE)"

        self.motor.set_speeds(left_speed, right_speed)

        if self.DEBUG:
            print(
                f"[FALLBACK] l_ok={left_ok}, r_ok={right_ok}, "
                f"l_conf={left_conf:.2f}, r_conf={right_conf:.2f}, "
                f"ls={left_speed:.1f}, rs={right_speed:.1f}"
            )

        # Update status mode / message based on movement
        if left_speed == 0 and right_speed == 0:
            self.status_mode = "fallback - stopped"
            self.status_stopped = True
            # Do not overwrite more specific message if already set
            self.status_msg = self.status_msg or "STOP: FALLBACK MODE HAS STOPPED THE CAR"
        else:
            self.status_mode = "fallback - run"
            self.status_stopped = False
            self.status_msg = "FALLBACK: SEARCHING FOR LANE"

    # ====================== NORMAL LANE FOLLOWING ======================
    def lane_following_step(self, lane_result: dict):
        """
        Main lane-following control step.

        Uses the polynomial fit (poly) from LaneDetector:
          x(y) = a*y^2 + b*y + c in normalized coordinates:
            - y in [0,1] from top to bottom of the warped image,
            - x in [-1,1] left/right offset from image center.

        Controller:
          - e_lat  = lateral error at lookahead y_L.
          - e_head = heading error derived from polynomial derivative.
          - steering = -K_lateral * e_lat - K_head * e_head
        """
        poly = lane_result.get("poly")
        if poly is None or len(poly) < 3:
            # No valid polynomial -> stop and signal error
            self.motor.set_speeds(0, 0)
            self.status_mode = "lane following - stopped"
            self.status_stopped = True
            self.status_msg = "STOP: NO POLYNOMIAL DATA FROM LANE DETECTOR"
            return

        a, b, c = poly

        # Lateral error at lookahead y_L
        x_L = a * self.y_L**2 + b * self.y_L + c
        e_lat = x_L

        # Heading error from derivative dx/dy
        dx_dy = 2 * a * self.y_L + b
        e_head = math.atan(dx_dy)

        # P-like controller on lateral and heading errors
        steering = -self.K_lateral * e_lat - self.K_head * e_head

        # Saturate steering to allowed range
        if steering > self.MAX_STEERING:
            steering = self.MAX_STEERING
        elif steering < -self.MAX_STEERING:
            steering = -self.MAX_STEERING

        base_speed = self.BASE_SPEED
        left_speed = base_speed + steering
        right_speed = base_speed - steering

        self.motor.set_speeds(left_speed, right_speed)

        if self.DEBUG:
            print(f"[LANE] e_lat={e_lat:.3f}, e_head={e_head:.3f}, steering={steering:.2f}")

        # Update status
        if left_speed == 0 and right_speed == 0:
            self.status_mode = "lane following - stopped"
            self.status_stopped = True
            self.status_msg = self.status_msg or "STOP: LANE FOLLOWING HAS STOPPED THE CAR"
        else:
            self.status_mode = "lane following - run"
            self.status_stopped = False
            self.status_msg = "RUNNING"

    # ====================== TOP-LEVEL CONTROL ENTRY ======================
    def control_step(self, lane_result: dict):
        """
        Main entry point called once per frame.

        High-level logic:
          1. If there is no lane_result -> stop vehicle and set error status.
          2. Optionally update controller parameters from trackbars (DEBUG only).
          3. Update detection history.
          4. If a crossroad is detected -> call crossroad_step() and stop.
          5. Update fallback state (enter/exit fallback mode).
          6. If fallback is active OR lane_ok is False -> fallback_step(),
             otherwise -> lane_following_step().
          7. Show status banner (DEBUG only).
        """
        # Manual emergency stop
        if self.manual_stop:
            self.motor.set_speeds(0, 0)
            self.status_mode = "manual stop"
            self.status_msg = "STOP: MANUAL S KEY PRESSED"
            self.status_stopped = True
            if self.DEBUG:
                self.show_status_banner()
            return
        # 1. No result from detector -> emergency stop
        if lane_result is None:
            self.motor.set_speeds(0, 0)
            self.status_mode = "control step - stopped"
            self.status_stopped = True
            self.status_msg = "STOP: NO DATA FROM LANE DETECTOR"
            if self.DEBUG:
                self.show_status_banner()
            return

        # 2. Update tunable values from trackbars
        if self.DEBUG:
            self._update_values_from_trackbars()

        # 3. Update detection history
        self.update_history(lane_result)

        # 4. Check crossroad type
        crossroad_type = lane_result.get("crossroad_type", "not_detected")
        if crossroad_type != "not_detected":
            #self.crossroad_step(lane_result)
            self.lane_following_step(lane_result)
            if self.DEBUG:
                self.show_status_banner()
            return

        # 5. Update fallback state based on history
        self.update_fallback()

        # 6. Choose control mode: fallback vs normal lane following
        if self.in_fallback or not lane_result.get("lane_ok", False):
            #self.fallback_step(lane_result)
            self.lane_following_step(lane_result)
        else:
            self.lane_following_step(lane_result)

        # 7. Show status banner in debug mode
        if self.DEBUG:
            self.show_status_banner()
