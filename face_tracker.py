"""
Face Tracker for Dual-Eye Robot
===============================
Tracks a face with a webcam and moves both eyes to follow it, with
random blinks.

Eyelid poses (right-eye values, pre-mirror):
  OPEN   = (upper=65, lower=110)   ← matches eye_system.py's blink "open"
  CLOSED = (upper=90, lower=90)    ← matches eye_system.py's blink "closed"

The left eye is produced by the controller's mirror config + gains.

Requires:
  pip install opencv-python

Usage:
  python face_tracker.py --port COM3
  python face_tracker.py --port /dev/ttyUSB0 --no-preview
"""

import argparse
import random
import signal
import sys
import time

import cv2
import numpy as np

from eye_system import EyeController, EYE_SIZE, NUM_SERVOS


# =============================================================================
# DEFAULT EYELID POSES  (right-eye values, pre-mirror)
# -----------------------------------------------------------------------------
# Matches eye_system.py:
#   _pattern_blink OPEN   -> upper=65, lower=110
#   _pattern_blink CLOSED -> upper=90, lower=90
# Change these to match your own calibration if you prefer.
# =============================================================================

DEFAULT_UPPER_OPEN   = 65.0   # right upper lid: fully open
DEFAULT_LOWER_OPEN   = 110.0  # right lower lid: fully open
DEFAULT_UPPER_CLOSED = 90.0   # right upper lid: fully closed
DEFAULT_LOWER_CLOSED = 90.0   # right lower lid: fully closed


# =============================================================================
# BLINK TIMING  (milliseconds)
# =============================================================================

BLINK_CLOSE_MS = 70   # how fast lids shut
BLINK_HOLD_MS  = 50   # how long fully closed
BLINK_OPEN_MS  = 90   # how fast lids open

# Blink state machine
BLINK_NONE    = 0
BLINK_CLOSING = 1
BLINK_HELD    = 2
BLINK_OPENING = 3

CX_GAIN = 2.0  # how aggressively to track horizontal face movement
CY_GAIN = -2.0  # how aggressively to track vertical face movement


# Haar cascade shipped with OpenCV
CASCADE_PATH = cv2.data.haarcascades + "haarcascade_frontalface_default.xml"


# =============================================================================
# TRACKER
# =============================================================================

class FaceTracker:
    def __init__(self, controller: EyeController, args):
        self.ctrl = controller
        self.args = args

        self.cascade = cv2.CascadeClassifier(CASCADE_PATH)
        if self.cascade.empty():
            raise RuntimeError(f"Could not load Haar cascade at {CASCADE_PATH}")

        self.cap = cv2.VideoCapture(args.camera)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {args.camera}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Neutrals from calibration JSON (used for pan / tilt pivot and brows)
        self.neutral = list(controller.initial_positions)

        # Smoothed angular offsets from neutral (logical degrees)
        self.pan   = 0.0
        self.tilt  = 0.0
        self.pan_t  = 0.0
        self.tilt_t = 0.0

        # Face state
        self.has_face  = False
        self.last_seen = 0.0

        # Blink
        self.blink_state = BLINK_NONE
        self.blink_t0    = 0.0
        self.blink_next  = time.time() + self._next_blink_interval()

        # Diagnostics
        self._fps_t0    = time.time()
        self._fps_count = 0
        self._fps       = 0.0

    # -------------------------------------------------------------------------

    def step(self):
        ok, frame = self.cap.read()
        if not ok:
            return None

        h, w = frame.shape[:2]

        # ---- Detect ----
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        faces = self.cascade.detectMultiScale(
            gray, scaleFactor=1.15, minNeighbors=5, minSize=(80, 80),
        )

        cx_norm, cy_norm = 0.0, 0.0
        if len(faces):
            x, y, fw, fh = max(faces, key=lambda r: r[2] * r[3])
            cx = x + fw / 2.0
            cy = y + fh / 2.0
            cx_norm = max(-1.0, min(1.0, (cx - w / 2.0) / (w / 2.0))) * CX_GAIN
            cy_norm = max(-1.0, min(1.0, (cy - h / 2.0) / (h / 2.0))) * CY_GAIN
            self.last_seen = time.time()
            self.has_face = True

            if not self.args.no_preview:
                cv2.rectangle(frame, (x, y), (x + fw, y + fh), (0, 255, 0), 2)
                cv2.circle(frame, (int(cx), int(cy)), 4, (0, 255, 0), -1)
        else:
            if time.time() - self.last_seen > self.args.lost_grace:
                self.has_face = False

        # ---- Targets ----
        if self.has_face:
            self.pan_t  = cx_norm * self.args.max_pan
            self.tilt_t = cy_norm * self.args.max_tilt
        else:
            self.pan_t  = 0.0
            self.tilt_t = 0.0

        # ---- Smoothing ----
        a = self.args.smoothing
        self.pan  += a * (self.pan_t  - self.pan)
        self.tilt += a * (self.tilt_t - self.tilt)

        # ---- Blink ----
        self._update_blink()

        # ---- Send ----
        angles = self._compute_angles()
        self.ctrl.send_batch_command(angles)

        # ---- Diagnostics ----
        self._fps_count += 1
        now = time.time()
        if now - self._fps_t0 >= 1.0:
            self._fps = self._fps_count / (now - self._fps_t0)
            self._fps_count = 0
            self._fps_t0 = now

        if not self.args.no_preview:
            status = "FACE" if self.has_face else "no face"
            blink_pct = int(self._blink_amount() * 100)
            cv2.putText(frame, f"FPS {self._fps:4.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, f"pan {self.pan:+5.1f}  tilt {self.tilt:+5.1f}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"{status}  blink {blink_pct}%", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return frame

    # -------------------------------------------------------------------------

    def _next_blink_interval(self) -> float:
        return random.uniform(self.args.blink_min, self.args.blink_max)

    def _update_blink(self):
        now = time.time()
        if self.blink_state == BLINK_NONE:
            if now >= self.blink_next:
                self.blink_state = BLINK_CLOSING
                self.blink_t0 = now
        elif self.blink_state == BLINK_CLOSING:
            if now - self.blink_t0 >= BLINK_CLOSE_MS / 1000.0:
                self.blink_state = BLINK_HELD
                self.blink_t0 = now
        elif self.blink_state == BLINK_HELD:
            if now - self.blink_t0 >= BLINK_HOLD_MS / 1000.0:
                self.blink_state = BLINK_OPENING
                self.blink_t0 = now
        elif self.blink_state == BLINK_OPENING:
            if now - self.blink_t0 >= BLINK_OPEN_MS / 1000.0:
                self.blink_state = BLINK_NONE
                self.blink_next = now + self._next_blink_interval()

    def _blink_amount(self) -> float:
        """0.0 = fully open, 1.0 = fully closed."""
        now = time.time()
        if self.blink_state == BLINK_NONE:
            return 0.0
        if self.blink_state == BLINK_CLOSING:
            return min(1.0, (now - self.blink_t0) / (BLINK_CLOSE_MS / 1000.0))
        if self.blink_state == BLINK_HELD:
            return 1.0
        if self.blink_state == BLINK_OPENING:
            return max(0.0, 1.0 - (now - self.blink_t0) / (BLINK_OPEN_MS / 1000.0))
        return 0.0

    # -------------------------------------------------------------------------

    def _compute_angles(self):
        n = self.neutral

        # ---- Pan / tilt (tracking) ----
        r_pan  = n[0] + self.pan
        r_tilt = n[1] + self.tilt

        # ---- Eyelids: interpolate OPEN <-> CLOSED ----
        bl = self._blink_amount()
        u_open   = float(self.args.upper_open)
        l_open   = float(self.args.lower_open)
        u_closed = float(self.args.upper_closed)
        l_closed = float(self.args.lower_closed)

        r_uLid = u_open   + (u_closed - u_open)   * bl
        r_lLid = l_open   + (l_closed - l_open)   * bl

        # ---- Brows / neutral ----
        r_bIn  = n[4]
        r_bOut = n[5]

        right6 = [
            max(0, min(180, int(round(r_pan)))),
            max(0, min(180, int(round(r_tilt)))),
            max(0, min(180, int(round(r_uLid)))),
            max(0, min(180, int(round(r_lLid)))),
            max(0, min(180, int(round(r_bIn)))),
            max(0, min(180, int(round(r_bOut)))),
        ]

        left6 = self.ctrl.mirror_right_to_left(
            right6,
            enabled_axes=self.ctrl.mirror_axes,
            gains=self.ctrl.mirror_gains,
            right_neutrals=n[:EYE_SIZE],
            left_neutrals=n[EYE_SIZE:],
        )

        return right6 + left6

    # -------------------------------------------------------------------------

    def close(self):
        try:
            self.cap.release()
        except Exception:
            pass


# =============================================================================
# MAIN
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Webcam face follower for dual-eye robot",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", required=True,
                        help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--camera", type=int, default=0,
                        help="Camera index")
    parser.add_argument("--max-pan", type=float, default=30.0,
                        help="Max pan offset from neutral (deg)")
    parser.add_argument("--max-tilt", type=float, default=20.0,
                        help="Max tilt offset from neutral (deg)")
    parser.add_argument("--smoothing", type=float, default=0.15,
                        help="EMA factor [0-1], higher = snappier tracking")
    parser.add_argument("--blink-min", type=float, default=1.0,
                        help="Min seconds between blinks")
    parser.add_argument("--blink-max", type=float, default=3.0,
                        help="Max seconds between blinks")

    # Eyelid poses (right-eye values) — override via CLI if needed
    parser.add_argument("--upper-open",   type=float, default=DEFAULT_UPPER_OPEN,
                        help="Right upper lid angle when fully OPEN")
    parser.add_argument("--lower-open",   type=float, default=DEFAULT_LOWER_OPEN,
                        help="Right lower lid angle when fully OPEN")
    parser.add_argument("--upper-closed", type=float, default=DEFAULT_UPPER_CLOSED,
                        help="Right upper lid angle when fully CLOSED")
    parser.add_argument("--lower-closed", type=float, default=DEFAULT_LOWER_CLOSED,
                        help="Right lower lid angle when fully CLOSED")

    parser.add_argument("--lost-grace", type=float, default=1.0,
                        help="Seconds before returning to neutral after face lost")
    parser.add_argument("--no-preview", action="store_true",
                        help="Run headless (no camera window)")
    args = parser.parse_args()

    if args.blink_min > args.blink_max:
        args.blink_min, args.blink_max = args.blink_max, args.blink_min

    # ---- Connect ----
    ctrl = EyeController()
    ok, msg = ctrl.connect(args.port)
    print(msg)
    if not ok:
        return 1

    # ---- Init ----
    try:
        tracker = FaceTracker(ctrl, args)
    except Exception as e:
        print(f"❌ Tracker init failed: {e}")
        ctrl.disconnect()
        return 1

    # ---- Graceful shutdown ----
    running = True
    def _on_sigint(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _on_sigint)

    print("▶️  Tracking… press 'q' in preview or Ctrl+C to quit.")
    print(f"   Eyelids  OPEN = ({args.upper_open:.0f}, {args.lower_open:.0f})   "
          f"CLOSED = ({args.upper_closed:.0f}, {args.lower_closed:.0f})")
    try:
        while running:
            frame = tracker.step()
            if frame is None:
                time.sleep(0.05)
                continue

            if not args.no_preview:
                cv2.imshow("Face Tracker", frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    break
            else:
                time.sleep(1.0 / 30.0)

    finally:
        print("\n⏹  Shutting down…")
        tracker.close()
        if not args.no_preview:
            cv2.destroyAllWindows()
        try:
            ctrl.go_to_rest()
        except Exception:
            pass
        ctrl.disconnect()
        print("✅ Stopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())