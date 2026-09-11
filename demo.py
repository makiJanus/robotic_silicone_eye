"""
Wander Demo for Dual-Eye Robot
==============================
Autonomous "alive" behavior:
  - Wandering gaze (saccades, holds, micro-jitter)
  - Random blinks with per-expression frequency
  - Cycling expressions with smooth ease-in transitions

All three run at the same time. The output is composed into a single
12-servo batch command each tick.

Reuses:
  - eye_system.EyeController         (serial + calibration + mirror + gains)
  - eye_expressions.json             (expression library)
  - eye_calibration.json             (neutrals, offsets, mirror config)

Usage:
  python demo.py --port COM3
  python demo.py --port /dev/ttyUSB0 --max-pan 35 --expression-interval 6
"""

import argparse
import random
import signal
import sys
import time
from typing import List, Tuple

from eye_system import EyeController, EYE_SIZE


# =============================================================================
# TUNABLES
# =============================================================================

# ---- Eyelid poses (right-eye values, matching eye_system.py) ----
UPPER_CLOSED = 90.0   # upper lid fully closed
LOWER_CLOSED = 90.0   # lower lid fully closed

# Default expressions in eye_system.py are expressed relative to a
# neutral eyelid reference of 90. The expression's eyelid contribution
# is measured as a delta from this and then weighted.
EXPR_EYELID_REF    = 90.0
EXPR_EYELID_WEIGHT = 0.7

# ---- Wander timing (seconds) ----
WANDER_IDLE_MIN    = 1.2
WANDER_IDLE_MAX    = 3.5
WANDER_SACCADE_MIN = 0.08
WANDER_SACCADE_MAX = 0.16

# ---- Micro-saccade jitter ----
JITTER_PAN     = 1.5     # degrees
JITTER_TILT    = 1.0
JITTER_INT_MIN = 0.15    # seconds between jitter updates
JITTER_INT_MAX = 0.40

# ---- Blink timing (milliseconds) ----
BLINK_CLOSE_MS = 70
BLINK_HOLD_MS  = 50
BLINK_OPEN_MS  = 90

DOUBLE_BLINK_CHANCE    = 0.15
DOUBLE_BLINK_DELAY_MIN = 0.10    # seconds
DOUBLE_BLINK_DELAY_MAX = 0.22

# Per-expression blink frequency multiplier (higher = blinks more often)
BLINK_FREQ_MULT = {
    "Sleepy":     2.0,
    "Sad":        1.2,
    "Neutral":    1.0,
    "Happy":      1.0,
    "Angry":      0.9,
    "Suspicious": 0.8,
    "Surprised":  0.6,
}

# ---- Expression timing ----
EXPR_TRANSITION = 0.5    # seconds for the ease-in blend between expressions


# ---- Blink state machine states ----
BLINK_NONE        = 0
BLINK_CLOSING     = 1
BLINK_HELD        = 2
BLINK_OPENING     = 3
BLINK_WAIT_DOUBLE = 4


# =============================================================================
# WANDER ENGINE
# =============================================================================

class WanderEngine:
    """
    Saccade → hold → saccade with micro-jitter during holds.
    Output: (pan_offset, tilt_offset) in degrees from neutral.
    """

    def __init__(self, max_pan: float, max_tilt: float):
        self.max_pan  = max_pan
        self.max_tilt = max_tilt

        self.pan  = 0.0
        self.tilt = 0.0

        self._target_pan  = 0.0
        self._target_tilt = 0.0
        self._from_pan    = 0.0
        self._from_tilt   = 0.0

        self._saccade_active = False
        self._saccade_start  = 0.0
        self._saccade_end    = 0.0
        self._next_saccade   = time.time() + random.uniform(
            WANDER_IDLE_MIN, WANDER_IDLE_MAX
        )

        self._jitter_pan   = 0.0
        self._jitter_tilt  = 0.0
        self._next_jitter  = time.time()

    def update(self) -> Tuple[float, float]:
        now = time.time()

        # --- Schedule a new saccade when idle ---
        if not self._saccade_active and now >= self._next_saccade:
            self._from_pan  = self.pan
            self._from_tilt = self.tilt
            self._target_pan  = random.uniform(-self.max_pan, self.max_pan)
            self._target_tilt = random.uniform(-self.max_tilt, self.max_tilt)
            self._saccade_start = now
            self._saccade_end   = now + random.uniform(
                WANDER_SACCADE_MIN, WANDER_SACCADE_MAX
            )
            self._saccade_active = True

        # --- Advance saccade ---
        if self._saccade_active:
            if now >= self._saccade_end:
                self.pan  = self._target_pan
                self.tilt = self._target_tilt
                self._saccade_active = False
                self._next_saccade = now + random.uniform(
                    WANDER_IDLE_MIN, WANDER_IDLE_MAX
                )
            else:
                t = (now - self._saccade_start) / (
                    self._saccade_end - self._saccade_start
                )
                e = 1 - (1 - t) ** 3   # ease-out for saccades
                self.pan  = self._from_pan  + (self._target_pan  - self._from_pan)  * e
                self.tilt = self._from_tilt + (self._target_tilt - self._from_tilt) * e

        # --- Micro-jitter during holds ---
        if not self._saccade_active and now >= self._next_jitter:
            self._jitter_pan  = random.uniform(-JITTER_PAN,  JITTER_PAN)
            self._jitter_tilt = random.uniform(-JITTER_TILT, JITTER_TILT)
            self._next_jitter = now + random.uniform(JITTER_INT_MIN, JITTER_INT_MAX)

        return self.pan + self._jitter_pan, self.tilt + self._jitter_tilt


# =============================================================================
# EXPRESSION ENGINE
# =============================================================================

class ExpressionEngine:
    """
    Random expression picker with ease-in transitions.
    Output: (right_eye_6, current_name).
    """

    def __init__(self, expressions: dict, neutral: List[int],
                 interval: float, transition: float):
        self.expressions = expressions
        self.neutral     = neutral
        self.interval    = interval
        self.transition  = transition

        # Start on Neutral if available, else first entry, else fallback
        if "Neutral" in expressions:
            self.current_name = "Neutral"
        elif expressions:
            self.current_name = next(iter(expressions))
        else:
            self.current_name = "Neutral"

        self.current_pose = list(self._get_pose(self.current_name))
        self._from_pose   = list(self.current_pose)
        self._to_pose     = list(self.current_pose)
        self._trans_start = 0.0
        self._trans_active = False
        self._next_change = time.time() + self._random_hold()

    # -------------------------------------------------------------

    def _random_hold(self) -> float:
        # ±30% jitter around the target interval
        return random.uniform(self.interval * 0.7, self.interval * 1.3)

    def _get_pose(self, name: str) -> List[int]:
        if name not in self.expressions:
            return [self.neutral[i] for i in range(EYE_SIZE)]
        sa = self.expressions[name]["servo_angles"]
        return [int(sa.get(str(i), self.neutral[i])) for i in range(EYE_SIZE)]

    def update(self) -> Tuple[List[int], str]:
        now = time.time()

        # --- Trigger a new expression change ---
        if not self._trans_active and now >= self._next_change:
            choices = [n for n in self.expressions.keys() if n != self.current_name]
            if choices:
                self.current_name = random.choice(choices)
                self._from_pose = list(self.current_pose)
                self._to_pose   = self._get_pose(self.current_name)
                self._trans_start = now
                self._trans_active = True

        # --- Advance transition ---
        if self._trans_active:
            t = (now - self._trans_start) / self.transition
            if t >= 1.0:
                self.current_pose = list(self._to_pose)
                self._trans_active = False
                self._next_change = now + self._random_hold()
            else:
                e = t * t   # ease-in
                self.current_pose = [
                    int(round(self._from_pose[i]
                              + (self._to_pose[i] - self._from_pose[i]) * e))
                    for i in range(EYE_SIZE)
                ]

        return list(self.current_pose), self.current_name


# =============================================================================
# BLINK ENGINE
# =============================================================================

class BlinkEngine:
    """
    Random blink scheduler with optional double-blinks and per-expression
    frequency scaling. Output: blink amount in [0.0, 1.0].
    """

    def __init__(self, blink_min: float, blink_max: float):
        self.blink_min = blink_min
        self.blink_max = blink_max

        # These MUST be set before the first _random_interval() call
        self._freq_mult      = 1.0
        self._pending_double = False
        self._double_delay   = 0.0

        self.state = BLINK_NONE
        self.t0    = 0.0
        self._next_blink = time.time() + self._random_interval()

        self._freq_mult     = 1.0
        self._pending_double = False
        self._double_delay  = 0.0

    # -------------------------------------------------------------

    def set_frequency_mult(self, m: float):
        self._freq_mult = max(0.1, float(m))

    def _random_interval(self) -> float:
        base = random.uniform(self.blink_min, self.blink_max)
        return base / max(0.1, self._freq_mult)

    def update(self) -> float:
        now = time.time()

        if self.state == BLINK_NONE:
            if now >= self._next_blink:
                self.state = BLINK_CLOSING
                self.t0 = now
                self._pending_double = random.random() < DOUBLE_BLINK_CHANCE

        elif self.state == BLINK_CLOSING:
            if now - self.t0 >= BLINK_CLOSE_MS / 1000.0:
                self.state = BLINK_HELD
                self.t0 = now

        elif self.state == BLINK_HELD:
            if now - self.t0 >= BLINK_HOLD_MS / 1000.0:
                self.state = BLINK_OPENING
                self.t0 = now

        elif self.state == BLINK_OPENING:
            if now - self.t0 >= BLINK_OPEN_MS / 1000.0:
                if self._pending_double:
                    self._pending_double = False
                    self.state = BLINK_WAIT_DOUBLE
                    self.t0 = now
                    self._double_delay = random.uniform(
                        DOUBLE_BLINK_DELAY_MIN, DOUBLE_BLINK_DELAY_MAX
                    )
                else:
                    self.state = BLINK_NONE
                    self._next_blink = now + self._random_interval()

        elif self.state == BLINK_WAIT_DOUBLE:
            if now - self.t0 >= self._double_delay:
                self.state = BLINK_CLOSING
                self.t0 = now
                self._pending_double = False

        return self._amount()

    def _amount(self) -> float:
        now = time.time()
        if self.state in (BLINK_NONE, BLINK_WAIT_DOUBLE):
            return 0.0
        if self.state == BLINK_CLOSING:
            return min(1.0, (now - self.t0) / (BLINK_CLOSE_MS / 1000.0))
        if self.state == BLINK_HELD:
            return 1.0
        if self.state == BLINK_OPENING:
            return max(0.0, 1.0 - (now - self.t0) / (BLINK_OPEN_MS / 1000.0))
        return 0.0


# =============================================================================
# DEMO RUNNER
# =============================================================================

class DemoRunner:
    """Owns the three engines and composes their output every tick."""

    def __init__(self, ctrl: EyeController, args):
        self.ctrl = ctrl
        self.args = args
        self.neutral = list(ctrl.initial_positions)

        self.wander = WanderEngine(args.max_pan, args.max_tilt)
        self.expression = ExpressionEngine(
            ctrl.expressions, self.neutral,
            interval=args.expression_interval,
            transition=EXPR_TRANSITION,
        )
        self.blink = BlinkEngine(args.blink_min, args.blink_max)

    # -------------------------------------------------------------

    def tick(self):
        # --- Advance each engine ---
        w_pan, w_tilt = self.wander.update()
        expr_pose, expr_name = self.expression.update()

        # Blink frequency depends on the current expression
        self.blink.set_frequency_mult(
            BLINK_FREQ_MULT.get(expr_name, 1.0)
        )
        blink = self.blink.update()

        n = self.neutral

        # --- Pan / tilt: wander + weighted expression bias ---
        expr_pan_bias  = (expr_pose[0] - n[0]) * self.args.bias_weight
        expr_tilt_bias = (expr_pose[1] - n[1]) * self.args.bias_weight

        r_pan  = n[0] + w_pan  + expr_pan_bias
        r_tilt = n[1] + w_tilt + expr_tilt_bias

        # --- Eyelids: calibration + weighted expression delta ---
        # Expression eyelid values are interpreted as deltas from 90.
        expr_uLid_delta = (expr_pose[2] - EXPR_EYELID_REF) * EXPR_EYELID_WEIGHT
        expr_lLid_delta = (expr_pose[3] - EXPR_EYELID_REF) * EXPR_EYELID_WEIGHT

        base_uLid = n[2] + expr_uLid_delta
        base_lLid = n[3] + expr_lLid_delta

        # Blink only ever ADDS closure, never opens:
        #   Upper lid can move toward UPPER_CLOSED but not past it.
        #   Lower lid can move toward LOWER_CLOSED but not past it.
        r_uLid = base_uLid + max(0.0, UPPER_CLOSED - base_uLid) * blink
        r_lLid = base_lLid + min(0.0, LOWER_CLOSED - base_lLid) * blink

        # --- Brows: expression drives them fully ---
        r_bIn  = expr_pose[4]
        r_bOut = expr_pose[5]

        # --- Clamp to valid servo range ---
        right6 = [
            max(0, min(180, int(round(r_pan)))),
            max(0, min(180, int(round(r_tilt)))),
            max(0, min(180, int(round(r_uLid)))),
            max(0, min(180, int(round(r_lLid)))),
            max(0, min(180, int(round(r_bIn)))),
            max(0, min(180, int(round(r_bOut)))),
        ]

        # --- Left eye via mirror config + gains + neutrals ---
        left6 = self.ctrl.mirror_right_to_left(
            right6,
            enabled_axes=self.ctrl.mirror_axes,
            gains=self.ctrl.mirror_gains,
            right_neutrals=n[:EYE_SIZE],
            left_neutrals=n[EYE_SIZE:],
        )

        self.ctrl.send_batch_command(right6 + left6)


# =============================================================================
# MAIN
# =============================================================================

def main():
    p = argparse.ArgumentParser(
        description="Wander demo for dual-eye robot",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--port", required=True,
                   help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    p.add_argument("--max-pan", type=float, default=25.0,
                   help="Max pan offset from neutral (degrees)")
    p.add_argument("--max-tilt", type=float, default=18.0,
                   help="Max tilt offset from neutral (degrees)")
    p.add_argument("--blink-min", type=float, default=1.5,
                   help="Min seconds between blinks")
    p.add_argument("--blink-max", type=float, default=4.0,
                   help="Max seconds between blinks")
    p.add_argument("--expression-interval", type=float, default=8.0,
                   help="Average seconds between expression changes")
    p.add_argument("--bias-weight", type=float, default=0.3,
                   help="How strongly expression pan/tilt biases wandering (0-1)")
    p.add_argument("--rate", type=float, default=30.0,
                   help="Update rate in Hz")
    args = p.parse_args()

    if args.blink_min > args.blink_max:
        args.blink_min, args.blink_max = args.blink_max, args.blink_min

    # ---- Connect ----
    ctrl = EyeController()
    ok, msg = ctrl.connect(args.port)
    print(msg)
    if not ok:
        return 1

    # ---- Init engines ----
    runner = DemoRunner(ctrl, args)

    # ---- Graceful shutdown ----
    running = True
    def _on_sigint(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _on_sigint)

    dt = 1.0 / max(1.0, args.rate)

    print(f"▶️  Wandering at {args.rate:.0f} Hz — Ctrl+C to stop.")
    print(f"   pan ±{args.max_pan}°   tilt ±{args.max_tilt}°   "
          f"blink {args.blink_min}-{args.blink_max}s   "
          f"expression ~{args.expression_interval:.0f}s   "
          f"bias {args.bias_weight:.2f}")
    print(f"   expressions loaded: {len(ctrl.expressions)}")

    try:
        while running:
            t0 = time.time()
            runner.tick()
            elapsed = time.time() - t0
            sleep = dt - elapsed
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n⏹  Shutting down…")
        try:
            ctrl.go_to_rest()
        except Exception:
            pass
        ctrl.disconnect()
        print("✅ Stopped.")

    return 0


if __name__ == "__main__":
    sys.exit(main())