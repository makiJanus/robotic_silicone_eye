"""
Unified Dual-Eye Control System
===============================
Controls BOTH left and right robot eyes.

  Right eye -> PCA9685 channels 0-5
  Left  eye -> PCA9685 channels 8-13

Per-eye servo layout (identical order for both sides):
  0: Eye Pan        (horizontal)
  1: Eye Tilt       (vertical)
  2: Upper Eyelid
  3: Lower Eyelid
  4: Eyebrow Inner
  5: Eyebrow Outer

Mirroring (per-axis, user-configurable):
  For each axis i:
      delta = right_angle - right_neutral
      if axis_is_flipped:  delta = -delta
      delta *= gain[i]                       # tune for mechanical gain mismatch
      left_angle = left_neutral + delta      # clamped to [0, 180]

  - enabled_axes[i] : True = flip (180 - v), False = same direction
  - gains[i]        : 1.0 = same sweep size, <1 = left moves less, >1 = more
  - neutrals are taken from the Calibration tab (initial_positions)

Tested with Gradio 4.44.1 + huggingface_hub 0.23-0.26.
"""

import gradio as gr
import serial
import serial.tools.list_ports
import time
import json
import os
import threading
import numpy as np
from typing import List, Optional, Tuple


# =============================================================================
# CONSTANTS
# =============================================================================

NUM_SERVOS = 12
EYE_SIZE = 6

SERVO_NAMES = [
    "R Eye Pan", "R Eye Tilt", "R Upper Eyelid",
    "R Lower Eyelid", "R Eyebrow Inner", "R Eyebrow Outer",
    "L Eye Pan", "L Eye Tilt", "L Upper Eyelid",
    "L Lower Eyelid", "L Eyebrow Inner", "L Eyebrow Outer",
]

# Per-axis mirror defaults (empirically derived from your setup)
#               pan    tilt   uLid   lLid   bIn    bOut
DEFAULT_MIRROR_AXES = [False, True, True, True, True, True]

# Per-axis gain defaults. 1.0 = both mechanisms move the same amount.
DEFAULT_MIRROR_GAINS = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

MIRROR_LABELS = ["Pan", "Tilt", "Upper Lid", "Lower Lid", "Brow Inner", "Brow Outer"]

CUSTOM_CSS = """
.gradio-container { max-width: 100% !important; }
.slider-wrap { touch-action: none !important; user-select: none !important; }
input[type="range"] { width: 100% !important; height: 28px !important; }
input[type="range"]::-webkit-slider-thumb {
    width: 26px !important; height: 26px !important;
    background: #6c5ce7 !important; border-radius: 50% !important;
}
"""


def _clamp(v, lo=0, hi=180) -> int:
    return max(lo, min(hi, int(round(v))))


# =============================================================================
# CONTROLLER
# =============================================================================

class EyeController:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        self.port = None

        self.angles = [90] * NUM_SERVOS
        self.offsets = [0] * NUM_SERVOS
        self.initial_positions = [90] * NUM_SERVOS

        self.mirror_axes = list(DEFAULT_MIRROR_AXES)
        self.mirror_gains = list(DEFAULT_MIRROR_GAINS)

        self.calibration_file = "eye_calibration.json"
        self.expressions_file = "eye_expressions.json"
        self.patterns_file = "eye_patterns.json"

        self.expressions = {}
        self.patterns = {}

        self._movement_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._movement_active = False

        self.load_calibration()
        self.load_expressions()
        self.load_patterns()

    # -------------------------------------------------------------------------
    # HARDWARE
    # -------------------------------------------------------------------------

    def get_ports(self) -> List[str]:
        try:
            return [p.device for p in serial.tools.list_ports.comports()]
        except Exception:
            return []

    def connect(self, port: str) -> Tuple[bool, str]:
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                time.sleep(0.5)

            self.ser = serial.Serial(port, 115200, timeout=2)
            time.sleep(3)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.ser.write(b"STATUS\n")
            self.ser.flush()
            time.sleep(0.5)

            if self.ser.in_waiting:
                response = self.ser.readline().decode().strip()
                if response == "OK":
                    self.connected = True
                    self.port = port
                    self.go_to_rest()
                    return True, f"✅ Connected to {port}"

            self.ser.close()
            self.connected = False
            return False, "❌ Arduino not responding"

        except Exception as e:
            self.connected = False
            return False, f"❌ Error: {e}"

    def disconnect(self):
        if self._movement_active:
            self.stop_movement()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        self.port = None

    def send_command(self, command: str) -> bool:
        if not self.connected or not self.ser:
            return False
        try:
            self.ser.write(f"{command}\n".encode())
            self.ser.flush()
            time.sleep(0.02)
            return True
        except Exception:
            return False

    def send_batch_command(self, logical_angles: List[int]) -> bool:
        if not self.connected or not self.ser:
            return False
        try:
            actuals = []
            for i, a in enumerate(logical_angles):
                a_act = _clamp(int(a) + self.offsets[i])
                actuals.append(a_act)
                self.angles[i] = int(a)
            cmd = "P," + ",".join(str(v) for v in actuals)
            self.ser.write(f"{cmd}\n".encode())
            self.ser.flush()
            return True
        except Exception:
            return False

    def move_servo(self, servo_id: int, logical_angle: int) -> bool:
        if not self.connected:
            return False
        self.angles[servo_id] = int(logical_angle)
        actual = _clamp(int(logical_angle) + self.offsets[servo_id])
        return self.send_command(f"M{servo_id},{actual}")

    def apply_offset_live(self, servo_id: int) -> bool:
        if not self.connected:
            return False
        logical = self.angles[servo_id]
        actual = _clamp(logical + self.offsets[servo_id])
        return self.send_command(f"M{servo_id},{actual}")

    def move_to_pose_fast(self, pose: dict, min_change: int = 1) -> bool:
        sa = pose.get("servo_angles", {})
        target = [int(round(sa.get(str(i), self.initial_positions[i]))) for i in range(NUM_SERVOS)]
        if not any(abs(target[i] - self.angles[i]) >= min_change for i in range(NUM_SERVOS)):
            return True
        return self.send_batch_command(target)

    def go_to_rest(self):
        if not self.connected:
            return
        self.send_batch_command(list(self.initial_positions))

    def center_all(self):
        if not self.connected:
            return
        self.send_batch_command([90] * NUM_SERVOS)

    # -------------------------------------------------------------------------
    # CALIBRATION
    # -------------------------------------------------------------------------

    def load_calibration(self) -> str:
        try:
            if not os.path.exists(self.calibration_file):
                return "ℹ️ No calibration file — using defaults"
            with open(self.calibration_file, "r") as f:
                data = json.load(f)
            if "initial_positions" in data and len(data["initial_positions"]) == NUM_SERVOS:
                self.initial_positions = data["initial_positions"]
            if "offsets" in data and len(data["offsets"]) == NUM_SERVOS:
                self.offsets = data["offsets"]
            if "mirror_axes" in data and len(data["mirror_axes"]) == EYE_SIZE:
                self.mirror_axes = data["mirror_axes"]
            if "mirror_gains" in data and len(data["mirror_gains"]) == EYE_SIZE:
                self.mirror_gains = [float(g) for g in data["mirror_gains"]]
            return "📂 Calibration loaded"
        except Exception as e:
            return f"❌ Error loading calibration: {e}"

    def save_calibration(self) -> str:
        """Save calibration (initial positions, offsets, mirror config + gains).
        Uses os.replace for Windows compatibility (overwrites existing backups)."""
        try:
            if os.path.exists(self.calibration_file):
                os.replace(self.calibration_file, f"{self.calibration_file}.backup")
            data = {
                "initial_positions": self.initial_positions,
                "offsets": self.offsets,
                "mirror_axes": self.mirror_axes,
                "mirror_gains": self.mirror_gains,
                "servo_names": SERVO_NAMES,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            with open(self.calibration_file, "w") as f:
                json.dump(data, f, indent=2)
            return "💾 Calibration saved"
        except Exception as e:
            return f"❌ Error saving: {e}"

    def set_initial_position(self, servo_id: int, position: int) -> str:
        self.initial_positions[servo_id] = int(position)
        self.save_calibration()
        self.move_servo(servo_id, int(position))
        return f"✅ {SERVO_NAMES[servo_id]} initial → {position}°"

    def set_offset(self, servo_id: int, offset: float) -> str:
        self.offsets[servo_id] = float(offset)
        self.save_calibration()
        self.apply_offset_live(servo_id)
        return f"✅ {SERVO_NAMES[servo_id]} offset → {offset:+.1f}°"

    def reset_to_defaults(self) -> str:
        self.initial_positions = [90] * NUM_SERVOS
        self.offsets = [0] * NUM_SERVOS
        self.mirror_axes = list(DEFAULT_MIRROR_AXES)
        self.mirror_gains = list(DEFAULT_MIRROR_GAINS)
        self.save_calibration()
        self.go_to_rest()
        return "🔄 All reset to defaults"

    # -------------------------------------------------------------------------
    # MIRROR
    # -------------------------------------------------------------------------

    @staticmethod
    def mirror_right_to_left(right_angles: List[int],
                             enabled_axes: Optional[List[bool]] = None,
                             gains: Optional[List[float]] = None,
                             right_neutrals: Optional[List[int]] = None,
                             left_neutrals: Optional[List[int]] = None) -> List[int]:
        """
        Convert 6 right-eye angles to 6 left-eye angles using per-axis flip
        and gain, pivoted on each side's own neutral.
        """
        if enabled_axes is None:
            enabled_axes = DEFAULT_MIRROR_AXES
        if gains is None:
            gains = DEFAULT_MIRROR_GAINS
        if right_neutrals is None:
            right_neutrals = [90] * EYE_SIZE
        if left_neutrals is None:
            left_neutrals = [90] * EYE_SIZE

        left = []
        for i in range(EYE_SIZE):
            r = int(right_angles[i])
            delta = r - int(right_neutrals[i])
            if enabled_axes[i]:
                delta = -delta
            delta *= float(gains[i])
            l = int(round(int(left_neutrals[i]) + delta))
            left.append(max(0, min(180, l)))
        return left

    def _mirror_pose(self, right_6: List[int], mirror: bool = True) -> List[int]:
        right = [int(v) for v in right_6]
        if not mirror:
            return list(right)
        return self.mirror_right_to_left(
            right,
            enabled_axes=self.mirror_axes,
            gains=self.mirror_gains,
            right_neutrals=self.initial_positions[:EYE_SIZE],
            left_neutrals=self.initial_positions[EYE_SIZE:],
        )

    # -------------------------------------------------------------------------
    # EXPRESSIONS
    # -------------------------------------------------------------------------

    def _default_expressions(self) -> dict:
        right_eyes = {
            "Neutral":    [90, 90, 90, 90, 90, 90],
            "Happy":      [90, 85, 70, 110, 80, 75],
            "Sad":        [90, 95, 110, 70, 100, 105],
            "Angry":      [90, 90, 70, 90, 60, 55],
            "Surprised":  [90, 90, 50, 130, 100, 100],
            "Suspicious": [70, 90, 80, 80, 85, 85],
            "Sleepy":     [90, 95, 140, 50, 95, 95],
        }
        out = {}
        for name, r in right_eyes.items():
            l = self._mirror_pose(r, mirror=True)
            out[name] = {
                "servo_angles": {str(i): int(v) for i, v in enumerate(r + l)},
                "timestamp": "default",
            }
        return out

    def load_expressions(self) -> str:
        try:
            if not os.path.exists(self.expressions_file):
                self.expressions = self._default_expressions()
                self.save_expressions_file()
                return "📂 Loaded default expressions"

            with open(self.expressions_file, "r") as f:
                data = json.load(f)
            raw = data.get("expressions", {})

            defaults = self._default_expressions()
            migrated_count = 0
            fixed = {}
            for name, pose in raw.items():
                sa = dict(pose.get("servo_angles", {}))
                fallback = defaults.get(name, defaults["Neutral"])["servo_angles"]
                for i in range(NUM_SERVOS):
                    if str(i) not in sa:
                        sa[str(i)] = fallback[str(i)]
                        migrated_count += 1
                sa = {str(i): int(sa[str(i)]) for i in range(NUM_SERVOS)}
                fixed[name] = {**pose, "servo_angles": sa}

            self.expressions = fixed

            if migrated_count > 0:
                self.save_expressions_file()
                return (f"📂 Loaded {len(self.expressions)} expression(s) "
                        f"— auto-migrated {migrated_count} missing servo value(s)")

            return f"📂 Loaded {len(self.expressions)} expression(s)"
        except Exception as e:
            self.expressions = self._default_expressions()
            return f"❌ Error loading expressions: {e}"

    def save_expressions_file(self) -> str:
        """Save expressions. Uses os.replace for Windows compatibility."""
        try:
            if os.path.exists(self.expressions_file):
                os.replace(self.expressions_file, f"{self.expressions_file}.backup")
            data = {"servo_names": SERVO_NAMES, "expressions": self.expressions}
            with open(self.expressions_file, "w") as f:
                json.dump(data, f, indent=2)
            return "💾 Expressions saved"
        except Exception as e:
            return f"❌ Error saving: {e}"

    def save_expression(self, name: str, *angles) -> Tuple[str, str]:
        name = (name or "").strip()
        if not name:
            return "⚠️ Enter a name first", self.get_expression_library_text()
        if len(angles) < NUM_SERVOS:
            return (f"⚠️ Expected {NUM_SERVOS} angles, got {len(angles)}",
                    self.get_expression_library_text())
        pose = {
            "servo_angles": {str(i): int(angles[i]) for i in range(NUM_SERVOS)},
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        self.expressions[name] = pose
        status = self.save_expressions_file()
        return f"✅ Saved '{name}'  ({status})", self.get_expression_library_text()

    def delete_expression(self, name: str) -> Tuple[str, str]:
        name = (name or "").strip()
        if name not in self.expressions:
            return f"⚠️ No expression '{name}'", self.get_expression_library_text()
        del self.expressions[name]
        status = self.save_expressions_file()
        return f"🗑️ Deleted '{name}'  ({status})", self.get_expression_library_text()

    def apply_expression(self, name: str):
        name = (name or "").strip()
        if name not in self.expressions:
            return (f"⚠️ No expression '{name}'", self.get_expression_library_text(),
                    *([None] * NUM_SERVOS))

        pose = self.expressions[name]
        self.move_to_pose_fast(pose)
        sa = pose["servo_angles"]

        angles = []
        for i in range(NUM_SERVOS):
            v = sa.get(str(i), self.initial_positions[i])
            angles.append(int(v))

        status = f"▶️ Loaded '{name}'" + (" ✅ moved" if self.connected else " (preview)")
        return (status, self.get_expression_library_text(), *angles)

    def get_expression_library_text(self) -> str:
        if not self.expressions:
            return "No expressions saved."
        lines = [f"📚 {len(self.expressions)} saved expression(s):", "-" * 40]
        for name, pose in sorted(self.expressions.items()):
            lines.append(f"  • {name}   ({pose.get('timestamp', '?')})")
        return "\n".join(lines)

    # -------------------------------------------------------------------------
    # PATTERNS
    # -------------------------------------------------------------------------

    def _default_patterns(self) -> dict:
        return {
            "Saccade":     {"description": "Quick eye movements between positions"},
            "Tracking":    {"description": "Smooth eye tracking motion"},
            "Blink":       {"description": "Single blink"},
            "Blink Rapid": {"description": "Multiple rapid blinks"},
            "Look Left":   {"description": "Look to the left"},
            "Look Right":  {"description": "Look to the right"},
            "Look Up":     {"description": "Look up"},
            "Look Down":   {"description": "Look down"},
            "Circle":      {"description": "Circular eye movement"},
            "Dizzy":       {"description": "Dizzy spinning eyes"},
        }

    def load_patterns(self) -> str:
        try:
            if not os.path.exists(self.patterns_file):
                self.patterns = self._default_patterns()
                self.save_patterns_file()
                return "📂 Loaded default patterns"
            with open(self.patterns_file, "r") as f:
                data = json.load(f)
            self.patterns = data.get("patterns", {})
            return f"📂 Loaded {len(self.patterns)} pattern(s)"
        except Exception as e:
            self.patterns = self._default_patterns()
            return f"❌ Error loading patterns: {e}"

    def save_patterns_file(self) -> str:
        """Save patterns. Uses os.replace for Windows compatibility."""
        try:
            if os.path.exists(self.patterns_file):
                os.replace(self.patterns_file, f"{self.patterns_file}.backup")
            data = {"servo_names": SERVO_NAMES, "patterns": self.patterns}
            with open(self.patterns_file, "w") as f:
                json.dump(data, f, indent=2)
            return "💾 Patterns saved"
        except Exception as e:
            return f"❌ Error saving: {e}"

    def _set_both_eyes(self, right_6: List[int], mirror: bool = True):
        right = [int(v) for v in right_6]
        left = self._mirror_pose(right, mirror)
        pose = {"servo_angles": {str(i): (right + left)[i] for i in range(NUM_SERVOS)}}
        self.move_to_pose_fast(pose)

    def _pattern_saccade(self, mirror=True):
        positions = [(90,90),(70,90),(110,90),(90,70),(90,110),
                     (70,70),(110,110),(70,110),(110,70)]
        for pan, tilt in positions:
            if self._stop_event.is_set(): break
            self._set_both_eyes([pan, tilt, 65, 110, 90, 90], mirror=mirror)
            time.sleep(0.3)

    def _pattern_tracking(self, mirror=True):
        for t in np.linspace(0, 2*np.pi, 60):
            if self._stop_event.is_set(): break
            pan = int(90 + 20 * np.sin(t))
            tilt = int(90 + 15 * np.cos(t))
            self._set_both_eyes([pan, tilt, 65, 110, 90, 90], mirror=mirror)
            time.sleep(0.05)

    def _pattern_blink(self, mirror=True):
        self._set_both_eyes([90, 90, 90, 90, 90, 90], mirror=mirror)
        time.sleep(0.15)
        self._set_both_eyes([90, 90, 65, 110, 90, 90], mirror=mirror)

    def _pattern_blink_rapid(self, mirror=True):
        for _ in range(5):
            if self._stop_event.is_set(): break
            self._pattern_blink(mirror=mirror)
            time.sleep(0.2)

    def _pattern_look_left(self, mirror=True):
        self._set_both_eyes([60, 90, 65, 110, 90, 90], mirror=mirror); time.sleep(1.5)

    def _pattern_look_right(self, mirror=True):
        self._set_both_eyes([120, 90, 65, 110, 90, 90], mirror=mirror); time.sleep(1.5)

    def _pattern_look_up(self, mirror=True):
        self._set_both_eyes([90, 70, 65, 110, 90, 90], mirror=mirror); time.sleep(1.5)

    def _pattern_look_down(self, mirror=True):
        self._set_both_eyes([90, 110, 65, 110, 90, 90], mirror=mirror); time.sleep(1.5)

    def _pattern_circle(self, mirror=True):
        for t in np.linspace(0, 2*np.pi, 30):
            if self._stop_event.is_set(): break
            pan = int(90 + 25 * np.sin(t))
            tilt = int(90 + 20 * np.cos(t))
            self._set_both_eyes([pan, tilt, 65, 110, 90, 90], mirror=mirror)
            time.sleep(0.05)

    def _pattern_dizzy(self, mirror=True):
        for _ in range(3):
            for t in np.linspace(0, 2*np.pi, 60):
                if self._stop_event.is_set(): break
                pan = int(90 + 30 * np.sin(t))
                tilt = int(90 + 20 * np.sin(2*t))
                self._set_both_eyes([pan, tilt, 65, 110, 90, 90], mirror=mirror)
                time.sleep(0.03)
            time.sleep(0.2)

    def start_pattern(self, pattern_name: str, mirror: bool = True) -> str:
        if pattern_name not in self.patterns:
            return f"⚠️ No pattern '{pattern_name}'"
        if self._movement_active:
            self.stop_movement()
        self._stop_event.clear()
        self._movement_active = True
        self._movement_thread = threading.Thread(
            target=self._pattern_worker,
            args=(pattern_name, bool(mirror)),
            daemon=True,
        )
        self._movement_thread.start()
        return f"▶️ Started pattern: {pattern_name}" + (" (mirrored)" if mirror else "")

    def stop_movement(self) -> str:
        if not self._movement_active:
            return "ℹ️ No pattern running"
        self._stop_event.set()
        self._movement_active = False
        self.go_to_rest()
        return "⏹ Stopped pattern"

    def _pattern_worker(self, pattern_name: str, mirror: bool):
        try:
            dispatch = {
                "Saccade":     self._pattern_saccade,
                "Tracking":    self._pattern_tracking,
                "Blink":       self._pattern_blink,
                "Blink Rapid": self._pattern_blink_rapid,
                "Look Left":   self._pattern_look_left,
                "Look Right":  self._pattern_look_right,
                "Look Up":     self._pattern_look_up,
                "Look Down":   self._pattern_look_down,
                "Circle":      self._pattern_circle,
                "Dizzy":       self._pattern_dizzy,
            }
            fn = dispatch.get(pattern_name)
            if fn:
                fn(mirror=mirror)
        except Exception as e:
            print(f"Pattern error: {e}")
        finally:
            self._movement_active = False
            if not self._stop_event.is_set():
                self.go_to_rest()

    # -------------------------------------------------------------------------
    # DISPLAY
    # -------------------------------------------------------------------------

    def get_display_text(self) -> str:
        lines = ["📊 CURRENT SETTINGS:", "-" * 66]
        lines.append(f"{'#':>2} {'Name':20} {'Initial':>8} {'Offset':>8} {'Rest Sent':>10}")
        lines.append("-" * 66)
        for i, name in enumerate(SERVO_NAMES):
            sent = _clamp(self.initial_positions[i] + self.offsets[i])
            lines.append(
                f"{i:>2} {name:20} {self.initial_positions[i]:>6}° "
                f"{self.offsets[i]:>+7.1f}° {sent:>8}°"
            )
        lines.append("-" * 66)
        mirror_str = ", ".join(
            f"{n}:{'flip' if c else 'same'}"
            for n, c in zip(
                ["pan", "tilt", "uLid", "lLid", "bIn", "bOut"],
                self.mirror_axes,
            )
        )
        lines.append(f"Mirror flip:   {mirror_str}")
        gain_str = ", ".join(
            f"{n}:{g:.2f}"
            for n, g in zip(
                ["pan", "tilt", "uLid", "lLid", "bIn", "bOut"],
                self.mirror_gains,
            )
        )
        lines.append(f"Mirror gain:   {gain_str}")
        return "\n".join(lines)


# =============================================================================
# GRADIO INTERFACE
# =============================================================================

def create_interface():
    controller = EyeController()

    # NOTE: Gradio 4.x takes theme and css in the Blocks constructor.
    with gr.Blocks(
        title="Dual Eye Control System",
        theme=gr.themes.Soft(),
        css=CUSTOM_CSS,
    ) as demo:
        gr.Markdown("# 👁️👁️ Dual Robot Eye Control System")
        gr.Markdown(
            "Right eye: PCA channels **0-5** &nbsp;·&nbsp; "
            "Left eye: PCA channels **8-13**  \n"
            "Per-axis **flip** and **gain** for mechanical mismatch compensation. "
            "Mirror pivots on each side's **neutral** (Calibration tab)."
        )

        sliders: dict = {}

        with gr.Tabs():

            # =================================================================
            # TAB 1: CONNECTION
            # =================================================================
            with gr.Tab("🔌 Connection"):
                with gr.Group():
                    with gr.Row():
                        port_dropdown = gr.Dropdown(
                            choices=controller.get_ports(),
                            value=(controller.get_ports()[0] if controller.get_ports() else None),
                            label="Port", interactive=True, scale=2,
                        )
                        refresh_btn = gr.Button("🔄 Refresh", scale=1)
                        connect_btn = gr.Button("🔗 Connect", variant="primary", scale=1)
                        disconnect_btn = gr.Button("🔌 Disconnect", variant="stop", scale=1)
                    status_text = gr.Textbox(value="Ready - Click Connect",
                                             label="Status", interactive=False)

                with gr.Row():
                    rest_btn       = gr.Button("🏠 Go to Rest")
                    center_btn     = gr.Button("🎯 Center All")
                    reload_cal_btn = gr.Button("🔄 Reload Calibration")
                    reset_all_btn  = gr.Button("🔄 Reset All to Defaults", variant="stop")

                action_status = gr.Textbox(label="Action Status", interactive=False)
                display_text = gr.Textbox(
                    value=controller.get_display_text(),
                    label="📊 Current Settings",
                    interactive=False, lines=17,
                )

                refresh_btn.click(
                    lambda: gr.update(
                        choices=controller.get_ports(),
                        value=(controller.get_ports()[0] if controller.get_ports() else None),
                    ),
                    outputs=[port_dropdown],
                )

                def _connect(port):
                    ok, msg = controller.connect(port)
                    return msg, gr.update(interactive=not ok), gr.update(interactive=ok)

                connect_btn.click(_connect,
                                  inputs=[port_dropdown],
                                  outputs=[status_text, connect_btn, disconnect_btn])

                def _disconnect():
                    controller.disconnect()
                    return "🔌 Disconnected", gr.update(interactive=True), gr.update(interactive=False)

                disconnect_btn.click(_disconnect,
                                     outputs=[status_text, connect_btn, disconnect_btn])

                rest_btn.click(lambda: (controller.go_to_rest(), "🏠 Rest pose"),
                               outputs=[action_status])
                center_btn.click(lambda: (controller.center_all(), "🎯 Centered"),
                                 outputs=[action_status])
                reload_cal_btn.click(controller.load_calibration, outputs=[action_status])
                reset_all_btn.click(
                    lambda: (controller.reset_to_defaults(), controller.get_display_text()),
                    outputs=[action_status, display_text],
                )

            # =================================================================
            # TAB 2: LIVE CONTROL
            # =================================================================
            with gr.Tab("🎮 Live Control"):
                gr.Markdown(
                    "### Real-time Servo Control\n"
                    "Configure **flip** and **gain** per axis, then click "
                    "**Apply mirror to LEFT eye**."
                )

                with gr.Row():
                    with gr.Column():
                        gr.Markdown("### 👁️ RIGHT EYE")
                        for i in range(0, EYE_SIZE):
                            sliders[i] = gr.Slider(
                                minimum=0, maximum=180, value=90, step=1,
                                label=SERVO_NAMES[i], interactive=True,
                                elem_classes="slider-wrap",
                            )

                    with gr.Column():
                        gr.Markdown("### 👁️ LEFT EYE")
                        for i in range(EYE_SIZE, NUM_SERVOS):
                            sliders[i] = gr.Slider(
                                minimum=0, maximum=180, value=90, step=1,
                                label=SERVO_NAMES[i], interactive=True,
                                elem_classes="slider-wrap",
                            )

                with gr.Group():
                    gr.Markdown(
                        "#### Mirror configuration\n"
                        "**Flip** = invert direction (physically mirrored servo).  \n"
                        "**Gain** = left_delta = flip(right_delta) × gain "
                        "(use < 1 if left moves less, > 1 if it moves more)."
                    )

                    with gr.Row():
                        mirror_cb_pan  = gr.Checkbox(value=controller.mirror_axes[0], label="Flip Pan")
                        mirror_cb_tilt = gr.Checkbox(value=controller.mirror_axes[1], label="Flip Tilt")
                        mirror_cb_uLid = gr.Checkbox(value=controller.mirror_axes[2], label="Flip Upper Lid")
                        mirror_cb_lLid = gr.Checkbox(value=controller.mirror_axes[3], label="Flip Lower Lid")
                        mirror_cb_bIn  = gr.Checkbox(value=controller.mirror_axes[4], label="Flip Brow Inner")
                        mirror_cb_bOut = gr.Checkbox(value=controller.mirror_axes[5], label="Flip Brow Outer")

                    with gr.Row():
                        gain_pan  = gr.Slider(0.30, 2.00, value=controller.mirror_gains[0], step=0.01, label="Pan gain")
                        gain_tilt = gr.Slider(0.30, 2.00, value=controller.mirror_gains[1], step=0.01, label="Tilt gain")
                        gain_uLid = gr.Slider(0.30, 2.00, value=controller.mirror_gains[2], step=0.01, label="Upper Lid gain")
                        gain_lLid = gr.Slider(0.30, 2.00, value=controller.mirror_gains[3], step=0.01, label="Lower Lid gain")
                        gain_bIn  = gr.Slider(0.30, 2.00, value=controller.mirror_gains[4], step=0.01, label="Brow Inner gain")
                        gain_bOut = gr.Slider(0.30, 2.00, value=controller.mirror_gains[5], step=0.01, label="Brow Outer gain")

                with gr.Row():
                    mirror_btn      = gr.Button("🔁 Apply mirror to LEFT eye", variant="secondary")
                    save_mirror_btn = gr.Button("💾 Save mirror config")
                    test_align_btn  = gr.Button("🧪 Test both axes", variant="secondary")

                with gr.Row():
                    test_axis_dd  = gr.Dropdown(choices=MIRROR_LABELS, value="Pan",
                                                label="Single-axis test", scale=2)
                    test_gain_btn = gr.Button("🧪 Test gain on this axis", scale=1)

                live_status = gr.Textbox(label="Live Status", interactive=False, value="Ready")

                all_slider_inputs = [sliders[i] for i in range(NUM_SERVOS)]
                mirror_cb_inputs = [
                    mirror_cb_pan, mirror_cb_tilt, mirror_cb_uLid,
                    mirror_cb_lLid, mirror_cb_bIn, mirror_cb_bOut,
                ]
                gain_inputs = [gain_pan, gain_tilt, gain_uLid, gain_lLid, gain_bIn, gain_bOut]
                mirror_inputs = mirror_cb_inputs + gain_inputs + all_slider_inputs

                def _update_all(*vals):
                    angles = [int(v) for v in vals]
                    if not controller.connected:
                        return "❌ Not connected"
                    controller.send_batch_command(angles)
                    return "✅ Updated all 12 servos"

                for i in range(NUM_SERVOS):
                    sliders[i].change(_update_all,
                                      inputs=all_slider_inputs,
                                      outputs=[live_status])

                def _read_cfg_and_gains(pan_cb, tilt_cb, up_cb, lo_cb, bi_cb, bo_cb,
                                         g_pan, g_tilt, g_uLid, g_lLid, g_bIn, g_bOut):
                    cfg = [bool(pan_cb), bool(tilt_cb), bool(up_cb),
                           bool(lo_cb), bool(bi_cb), bool(bo_cb)]
                    gains = [float(g_pan), float(g_tilt), float(g_uLid),
                             float(g_lLid), float(g_bIn), float(g_bOut)]
                    return cfg, gains

                def _mirror(pan_cb, tilt_cb, up_cb, lo_cb, bi_cb, bo_cb,
                            g_pan, g_tilt, g_uLid, g_lLid, g_bIn, g_bOut, *vals):
                    cfg, gains = _read_cfg_and_gains(
                        pan_cb, tilt_cb, up_cb, lo_cb, bi_cb, bo_cb,
                        g_pan, g_tilt, g_uLid, g_lLid, g_bIn, g_bOut
                    )
                    controller.mirror_axes = cfg
                    controller.mirror_gains = gains

                    right = [int(v) for v in vals[:EYE_SIZE]]
                    left = EyeController.mirror_right_to_left(
                        right,
                        enabled_axes=cfg,
                        gains=gains,
                        right_neutrals=controller.initial_positions[:EYE_SIZE],
                        left_neutrals=controller.initial_positions[EYE_SIZE:],
                    )
                    if controller.connected:
                        controller.send_batch_command(right + left)

                    flip_txt = ", ".join(
                        f"{n}:{'flip' if c else 'same'}"
                        for n, c in zip(MIRROR_LABELS, cfg)
                    )
                    gain_txt = ", ".join(f"{n}:{g:.2f}"
                                         for n, g in zip(MIRROR_LABELS, gains))
                    status = f"Mirrored — flip: {flip_txt}  |  gain: {gain_txt}"
                    return [gr.update(value=float(v)) for v in left] + [status]

                mirror_btn.click(
                    _mirror,
                    inputs=mirror_inputs,
                    outputs=[sliders[i] for i in range(EYE_SIZE, NUM_SERVOS)] + [live_status],
                )

                def _save_mirror_cfg(pan_cb, tilt_cb, up_cb, lo_cb, bi_cb, bo_cb,
                                     g_pan, g_tilt, g_uLid, g_lLid, g_bIn, g_bOut):
                    cfg, gains = _read_cfg_and_gains(
                        pan_cb, tilt_cb, up_cb, lo_cb, bi_cb, bo_cb,
                        g_pan, g_tilt, g_uLid, g_lLid, g_bIn, g_bOut
                    )
                    controller.mirror_axes = cfg
                    controller.mirror_gains = gains
                    status = controller.save_calibration()
                    return f"{status} — mirror config + gains persisted", controller.get_display_text()

                save_mirror_btn.click(
                    _save_mirror_cfg,
                    inputs=mirror_cb_inputs + gain_inputs,
                    outputs=[live_status, display_text],
                )

                # ---- Test both axes (pan then tilt) ----
                def _test_alignment(pan_cb, tilt_cb, up_cb, lo_cb, bi_cb, bo_cb,
                                    g_pan, g_tilt, g_uLid, g_lLid, g_bIn, g_bOut):
                    cfg, gains = _read_cfg_and_gains(
                        pan_cb, tilt_cb, up_cb, lo_cb, bi_cb, bo_cb,
                        g_pan, g_tilt, g_uLid, g_lLid, g_bIn, g_bOut
                    )
                    controller.mirror_axes = cfg
                    controller.mirror_gains = gains
                    rn = controller.initial_positions[:EYE_SIZE]
                    ln = controller.initial_positions[EYE_SIZE:]

                    def run():
                        try:
                            controller.send_batch_command([90] * NUM_SERVOS)
                            time.sleep(0.5)

                            for pan in (60, 120, 90):
                                right = [pan, rn[1], 65, 110, 90, 90]
                                left = EyeController.mirror_right_to_left(
                                    right, enabled_axes=cfg, gains=gains,
                                    right_neutrals=rn, left_neutrals=ln,
                                )
                                controller.send_batch_command(right + left)
                                time.sleep(0.7)

                            for tilt in (70, 110, 90):
                                right = [rn[0], tilt, 65, 110, 90, 90]
                                left = EyeController.mirror_right_to_left(
                                    right, enabled_axes=cfg, gains=gains,
                                    right_neutrals=rn, left_neutrals=ln,
                                )
                                controller.send_batch_command(right + left)
                                time.sleep(0.7)

                            controller.send_batch_command(list(controller.initial_positions))
                        except Exception as e:
                            print(f"Test align error: {e}")

                    threading.Thread(target=run, daemon=True).start()
                    return ("🧪 Test running… watch both eyes.\n"
                            "If the LEFT eye's sweep is smaller → raise its gain; "
                            "if bigger → lower it.")

                test_align_btn.click(
                    _test_alignment,
                    inputs=mirror_cb_inputs + gain_inputs,
                    outputs=[live_status],
                )

                # ---- Test one specific axis (smooth sweep) ----
                def _test_axis(axis_name, pan_cb, tilt_cb, up_cb, lo_cb, bi_cb, bo_cb,
                               g_pan, g_tilt, g_uLid, g_lLid, g_bIn, g_bOut):
                    if axis_name not in MIRROR_LABELS:
                        return f"⚠️ Unknown axis '{axis_name}'"
                    axis_idx = MIRROR_LABELS.index(axis_name)
                    cfg, gains = _read_cfg_and_gains(
                        pan_cb, tilt_cb, up_cb, lo_cb, bi_cb, bo_cb,
                        g_pan, g_tilt, g_uLid, g_lLid, g_bIn, g_bOut
                    )
                    controller.mirror_axes = cfg
                    controller.mirror_gains = gains
                    rn = controller.initial_positions[:EYE_SIZE]
                    ln = controller.initial_positions[EYE_SIZE:]

                    def run():
                        try:
                            right = list(rn)
                            for t in np.linspace(0, 2 * np.pi, 50):
                                if controller._stop_event.is_set():
                                    break
                                delta = 25 * np.sin(t)
                                right[axis_idx] = int(round(rn[axis_idx] + delta))
                                left = EyeController.mirror_right_to_left(
                                    right, enabled_axes=cfg, gains=gains,
                                    right_neutrals=rn, left_neutrals=ln,
                                )
                                controller.send_batch_command(right + left)
                                time.sleep(0.05)
                            controller.send_batch_command(list(controller.initial_positions))
                        except Exception as e:
                            print(f"Test axis error: {e}")

                    threading.Thread(target=run, daemon=True).start()
                    return (f"🧪 Sweeping {axis_name} (±25°)…\n"
                            f"Adjust '{axis_name} gain' until both eyes sweep equally.")

                test_gain_btn.click(
                    _test_axis,
                    inputs=[test_axis_dd] + mirror_cb_inputs + gain_inputs,
                    outputs=[live_status],
                )

            # =================================================================
            # TAB 3: EXPRESSIONS
            # =================================================================
            with gr.Tab("🎭 Expressions"):
                gr.Markdown(
                    "### Expression Presets\n"
                    "Saves/loads all 12 servo values.  \n"
                    "**Note:** after changing mirror gains or neutrals, click "
                    "**♻️ Regenerate Defaults** to refresh the built-in expressions."
                )

                expr_name = gr.Dropdown(
                    choices=list(controller.expressions.keys()),
                    value="Neutral",
                    label="Expression Name",
                    allow_custom_value=True,
                    interactive=True,
                )

                with gr.Row():
                    save_expr_btn   = gr.Button("💾 Save Expression", variant="primary")
                    load_expr_btn   = gr.Button("▶️ Load Expression", variant="secondary")
                    delete_expr_btn = gr.Button("🗑️ Delete Expression", variant="stop")
                    regen_expr_btn  = gr.Button("♻️ Regenerate Defaults",
                                                variant="secondary")

                expr_library = gr.Textbox(
                    value=controller.get_expression_library_text(),
                    label="Saved Expressions",
                    interactive=False, lines=8,
                )

                save_expr_btn.click(
                    controller.save_expression,
                    inputs=[expr_name] + all_slider_inputs,
                    outputs=[action_status, expr_library],
                )

                def _load_expr(name):
                    status, lib, *angles = controller.apply_expression(name)
                    updates = [gr.update(value=float(a)) if a is not None else gr.update()
                               for a in angles]
                    return [status, lib] + updates

                load_expr_btn.click(
                    _load_expr,
                    inputs=[expr_name],
                    outputs=[action_status, expr_library] + all_slider_inputs,
                )

                delete_expr_btn.click(
                    controller.delete_expression,
                    inputs=[expr_name],
                    outputs=[action_status, expr_library],
                )

                def _regen_defaults():
                    controller.expressions = controller._default_expressions()
                    status = controller.save_expressions_file()
                    return (f"♻️ Defaults regenerated ({status})",
                            controller.get_expression_library_text())

                regen_expr_btn.click(
                    _regen_defaults,
                    outputs=[action_status, expr_library],
                )

            # =================================================================
            # TAB 4: PATTERNS
            # =================================================================
            with gr.Tab("🔄 Patterns"):
                gr.Markdown("### Eye Movement Patterns")

                pattern_name = gr.Dropdown(
                    choices=list(controller.patterns.keys()),
                    value="Saccade",
                    label="Pattern Name",
                    interactive=True,
                )
                pattern_desc = gr.Textbox(
                    value=controller.patterns.get("Saccade", {}).get("description", ""),
                    label="Description", interactive=False,
                )
                mirror_pattern_cb = gr.Checkbox(
                    value=True,
                    label="Mirror left eye from right (uses current flip + gain config)",
                )

                with gr.Row():
                    start_pattern_btn = gr.Button("▶️ Start Pattern", variant="primary")
                    stop_pattern_btn  = gr.Button("⏹ Stop Pattern", variant="stop")

                pattern_status = gr.Textbox(label="Pattern Status",
                                            interactive=False, value="Idle")

                pattern_name.change(
                    lambda n: gr.update(value=controller.patterns.get(n, {}).get("description", "")),
                    inputs=[pattern_name], outputs=[pattern_desc],
                )

                start_pattern_btn.click(
                    controller.start_pattern,
                    inputs=[pattern_name, mirror_pattern_cb],
                    outputs=[pattern_status],
                )
                stop_pattern_btn.click(controller.stop_movement, outputs=[pattern_status])

            # =================================================================
            # TAB 5: OFFSETS
            # =================================================================
            with gr.Tab("🎚️ Offsets"):
                gr.Markdown(
                    "### Offset Tuning\n"
                    "`sent = logical + offset`. Drag → live servo move.\n"
                    "**↺** resets one · **💾** saves all · **↺ Reset ALL** zeros everything."
                )

                offset_status = gr.Textbox(
                    label="Offset Status",
                    interactive=False,
                    value="Ready — drag a slider to tune live",
                )

                offset_sliders = []
                offset_info_boxes = []

                def _offset_info_str(idx: int) -> str:
                    logical = controller.angles[idx]
                    off = controller.offsets[idx]
                    sent = _clamp(logical + off)
                    return (f"Base: {controller.initial_positions[idx]}°  ·  "
                            f"Now: {logical}°  ·  Sent: {sent}°")

                for idx in range(NUM_SERVOS):
                    with gr.Row():
                        gr.Markdown(f"**{idx}. {SERVO_NAMES[idx]}**")
                        off_slider = gr.Slider(
                            minimum=-90, maximum=90,
                            value=float(controller.offsets[idx]),
                            step=0.5, label="Offset (°)", scale=4,
                            elem_classes="slider-wrap",
                        )
                        offset_sliders.append(off_slider)
                        info_box = gr.Textbox(
                            value=_offset_info_str(idx),
                            interactive=False, scale=3, show_label=False,
                        )
                        offset_info_boxes.append(info_box)
                        reset_one_btn = gr.Button("↺", scale=1, size="sm", min_width=40)

                    def _make_off_live(i):
                        def _h(val):
                            controller.offsets[i] = float(val)
                            if controller.connected:
                                controller.apply_offset_live(i)
                            status = (f"🎚️ {SERVO_NAMES[i]} offset → {float(val):+.1f}°"
                                      + ("" if controller.connected else "  (not connected)"))
                            return status, _offset_info_str(i)
                        return _h

                    off_slider.change(_make_off_live(idx),
                                      inputs=[off_slider],
                                      outputs=[offset_status, info_box])

                    def _make_off_reset(i):
                        def _h():
                            controller.offsets[i] = 0.0
                            if controller.connected:
                                controller.apply_offset_live(i)
                            return (gr.update(value=0.0),
                                    f"↺ {SERVO_NAMES[i]} offset reset to 0",
                                    _offset_info_str(i))
                        return _h

                    reset_one_btn.click(_make_off_reset(idx),
                                        outputs=[off_slider, offset_status, info_box])

                with gr.Row():
                    save_offsets_btn  = gr.Button("💾 Save all offsets", variant="primary")
                    reset_offsets_btn = gr.Button("↺ Reset ALL offsets to 0", variant="stop")

                def _save_offsets():
                    status = controller.save_calibration()
                    return f"{status} — offsets persisted", controller.get_display_text()

                save_offsets_btn.click(_save_offsets,
                                       outputs=[offset_status, display_text])

                def _reset_all_offsets():
                    controller.offsets = [0.0] * NUM_SERVOS
                    if controller.connected:
                        for i in range(NUM_SERVOS):
                            controller.apply_offset_live(i)
                    slider_updates = [gr.update(value=0.0) for _ in range(NUM_SERVOS)]
                    info_updates = [_offset_info_str(i) for i in range(NUM_SERVOS)]
                    return (slider_updates + info_updates
                            + ["↺ All offsets reset to 0",
                               controller.get_display_text()])

                reset_offsets_btn.click(
                    _reset_all_offsets,
                    outputs=offset_sliders + offset_info_boxes
                            + [offset_status, display_text],
                )

            # =================================================================
            # TAB 6: CALIBRATION
            # =================================================================
            with gr.Tab("⚙️ Calibration"):
                gr.Markdown(
                    "### Initial Position Calibration\n"
                    "Set the **rest / neutral angle** for each servo. "
                    "These neutrals are used as the pivot for mirroring "
                    "(so gains stay valid regardless of where you set rest).  \n"
                    "For offset trims use the **🎚️ Offsets** tab."
                )

                for idx in range(NUM_SERVOS):
                    with gr.Group(elem_id=f"cal_servo_{idx}"):
                        with gr.Row():
                            gr.Markdown(f"**{idx}. {SERVO_NAMES[idx]}**")
                        with gr.Row():
                            cal_slider = gr.Slider(
                                minimum=0, maximum=180,
                                value=float(controller.initial_positions[idx]),
                                step=1, label="Position", interactive=True,
                            )
                        with gr.Row():
                            init_input = gr.Number(
                                value=controller.initial_positions[idx],
                                label="Initial", minimum=0, maximum=180,
                                step=1, precision=0,
                            )
                            set_init_btn = gr.Button("📌 Set Initial", size="sm",
                                                     variant="primary")

                        def _make_set_init(i):
                            def _h(v):
                                s = controller.set_initial_position(i, int(v or 90))
                                return s, controller.get_display_text()
                            return _h

                        def _make_jog(i):
                            def _h(v):
                                val = int(v)
                                if controller.connected:
                                    controller.move_servo(i, val)
                                    return f"✅ {SERVO_NAMES[i]} → {val}°"
                                return "❌ Disconnected"
                            return _h

                        set_init_btn.click(_make_set_init(idx),
                                           inputs=[init_input],
                                           outputs=[action_status, display_text])
                        cal_slider.change(_make_jog(idx),
                                          inputs=[cal_slider],
                                          outputs=[action_status])

    return demo


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    demo = create_interface()
    # Gradio 4.x: theme and css are set on gr.Blocks above, NOT here.
    demo.launch(
        server_name="0.0.0.0",
        server_port=7860,
        share=False,
        debug=False,
    )