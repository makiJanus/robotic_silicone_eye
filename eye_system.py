"""
Unified Eye Control System
==========================
Control robot eyes with 6 servos (expandable to 12):
  - Eye Pan (horizontal rotation)
  - Eye Tilt (vertical rotation)
  - Upper Eyelid
  - Lower Eyelid
  - Eyebrow Inner
  - Eyebrow Middle/Outer

Features:
  - Fast batch angle sending
  - Real-time servo control
  - Multi-touch responsive (sliders work simultaneously)
  - Expression presets (Angry, Surprised, Happy, Sad, etc.)
  - Eye movement patterns (saccades, tracking, blinking)
"""

import gradio as gr
import serial
import serial.tools.list_ports
import time
import json
import os
import threading
import random
import numpy as np
from typing import List, Optional, Tuple, Dict
import plotly.graph_objects as go

# =============================================================================
# CORE CLASSES
# =============================================================================

class EyeController:
    """
    Unified eye controller with:
      - Hardware communication (batch commands)
      - Calibration management
      - Expression presets
      - Eye movement patterns
    """

    def __init__(self):
        # ---- Hardware ----
        self.ser: Optional[serial.Serial] = None
        self.connected = False
        self.port = None

        # ---- Servo definitions ----
        self.servo_names = [
            "Eye Pan",        # 0: Horizontal rotation
            "Eye Tilt",       # 1: Vertical rotation
            "Upper Eyelid",   # 2: Upper eyelid
            "Lower Eyelid",   # 3: Lower eyelid
            "Eyebrow Inner",  # 4: Inner eyebrow
            "Eyebrow Outer",  # 5: Outer/middle eyebrow
        ]
        
        # Servo groups for easier control
        self.EYE_SERVOS = [0, 1]        # Pan & Tilt
        self.EYELID_SERVOS = [2, 3]     # Upper & Lower
        self.EYEBROW_SERVOS = [4, 5]    # Inner & Outer

        self.angles = [90] * 6
        self.offsets = [0] * 6
        self.initial_positions = [90] * 6

        # ---- Files ----
        self.calibration_file = "eye_calibration.json"
        self.expressions_file = "eye_expressions.json"
        self.patterns_file = "eye_patterns.json"

        # ---- Presets ----
        self.expressions = {}
        self.patterns = {}

        # ---- Movement state ----
        self._movement_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._movement_active = False
        self._current_pattern = None

        # ---- Load data ----
        self.load_calibration()
        self.load_expressions()
        self.load_patterns()

    # =========================================================================
    # HARDWARE COMMUNICATION
    # =========================================================================

    def get_ports(self) -> List[str]:
        try:
            return [p.device for p in serial.tools.list_ports.comports()]
        except:
            return []

    def connect(self, port: str) -> Tuple[bool, str]:
        """Connect to Arduino."""
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
            return False, f"❌ Error: {str(e)}"

    def disconnect(self):
        if self._movement_active:
            self.stop_movement()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connected = False
        self.port = None

    def send_command(self, command: str) -> bool:
        """Send a single command."""
        if not self.connected or not self.ser:
            return False
        try:
            self.ser.write(f"{command}\n".encode())
            self.ser.flush()
            time.sleep(0.02)  # Reduced delay for faster response
            return True
        except:
            return False

    def send_batch_command(self, angles: List[int]) -> bool:
        """Send all servo angles in a single command: P,90,85,110,..."""
        if not self.connected or not self.ser:
            return False
        try:
            cmd = "P," + ",".join(str(a) for a in angles)
            self.ser.write(f"{cmd}\n".encode())
            self.ser.flush()
            # No delay needed for batch commands - they're fast
            return True
        except:
            return False

    def move_servo(self, servo_id: int, angle: int) -> bool:
        """Single servo move."""
        if not self.connected:
            return False
        actual = angle + self.offsets[servo_id]
        actual = max(0, min(180, int(actual)))
        self.angles[servo_id] = int(angle)
        return self.send_command(f"M{servo_id},{actual}")

    def move_to_pose_fast(self, pose: dict, min_change: int = 2) -> bool:
        """
        Move all servos to a pose using batch command.
        Only sends if at least one servo changed by >= min_change degrees.
        """
        sa = pose.get("servo_angles", {})
        all_angles = [90] * 6

        for i in range(6):
            all_angles[i] = int(round(sa.get(str(i), self.initial_positions[i])))

        # Apply offsets and clamp
        for i in range(6):
            actual = all_angles[i] + self.offsets[i]
            all_angles[i] = max(0, min(180, int(actual)))

        # Check if any angle changed significantly
        changed = False
        for i in range(6):
            if abs(all_angles[i] - self.angles[i]) >= min_change:
                changed = True
                break
        
        if not changed:
            return True
        
        # Update stored angles and send
        for i in range(6):
            self.angles[i] = all_angles[i]

        return self.send_batch_command(all_angles)

    def go_to_rest(self):
        """Move to calibrated rest/neutral pose."""
        if not self.connected:
            return

        for i in range(6):
            self.angles[i] = self.initial_positions[i]
            actual = self.initial_positions[i] + self.offsets[i]
            actual = max(0, min(180, int(actual)))
            self.send_command(f"M{i},{actual}")
            time.sleep(0.02)

    def center_all(self):
        """Center all servos to 90°."""
        if not self.connected:
            return
        for i in range(6):
            self.angles[i] = 90
            actual = 90 + self.offsets[i]
            actual = max(0, min(180, int(actual)))
            self.send_command(f"M{i},{actual}")
            time.sleep(0.02)
        self.send_command("CENTER")

    # =========================================================================
    # CALIBRATION
    # =========================================================================

    def load_calibration(self) -> str:
        try:
            if not os.path.exists(self.calibration_file):
                return "ℹ️ No calibration file — using defaults"

            with open(self.calibration_file, "r") as f:
                data = json.load(f)

            if "initial_positions" in data and len(data["initial_positions"]) == 6:
                self.initial_positions = data["initial_positions"]
            if "offsets" in data and len(data["offsets"]) == 6:
                self.offsets = data["offsets"]

            return "📂 Calibration loaded"
        except Exception as e:
            return f"❌ Error loading calibration: {e}"

    def save_calibration(self) -> str:
        try:
            if os.path.exists(self.calibration_file):
                os.rename(self.calibration_file, f"{self.calibration_file}.backup")

            data = {
                "initial_positions": self.initial_positions,
                "offsets": self.offsets,
                "servo_names": self.servo_names,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
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
        return f"✅ {self.servo_names[servo_id]} initial → {position}°"

    def set_offset(self, servo_id: int, offset: float) -> str:
        self.offsets[servo_id] = float(offset)
        self.save_calibration()
        self.move_servo(servo_id, self.angles[servo_id])
        return f"✅ {self.servo_names[servo_id]} offset → {offset:+.1f}°"

    def reset_to_defaults(self) -> str:
        self.initial_positions = [90] * 6
        self.offsets = [0] * 6
        self.save_calibration()
        self.go_to_rest()
        return "🔄 All reset to defaults"

    # =========================================================================
    # EXPRESSIONS
    # =========================================================================

    def load_expressions(self) -> str:
        try:
            if not os.path.exists(self.expressions_file):
                self.expressions = self._get_default_expressions()
                self.save_expressions_file()
                return "📂 Loaded default expressions"
            with open(self.expressions_file, "r") as f:
                data = json.load(f)
            self.expressions = data.get("expressions", {})
            return f"📂 Loaded {len(self.expressions)} expression(s)"
        except Exception as e:
            self.expressions = self._get_default_expressions()
            return f"❌ Error loading expressions: {e}"

    def _get_default_expressions(self) -> dict:
        """Return default eye expressions."""
        return {
            "Neutral": {
                "servo_angles": {"0": 90, "1": 90, "2": 90, "3": 90, "4": 90, "5": 90},
                "timestamp": "default"
            },
            "Happy": {
                "servo_angles": {"0": 90, "1": 85, "2": 70, "3": 110, "4": 80, "5": 75},
                "timestamp": "default"
            },
            "Sad": {
                "servo_angles": {"0": 90, "1": 95, "2": 110, "3": 70, "4": 100, "5": 105},
                "timestamp": "default"
            },
            "Angry": {
                "servo_angles": {"0": 90, "1": 90, "2": 70, "3": 90, "4": 60, "5": 55},
                "timestamp": "default"
            },
            "Surprised": {
                "servo_angles": {"0": 90, "1": 90, "2": 50, "3": 130, "4": 100, "5": 100},
                "timestamp": "default"
            },
            "Suspicious": {
                "servo_angles": {"0": 70, "1": 90, "2": 80, "3": 80, "4": 85, "5": 85},
                "timestamp": "default"
            },
            "Sleepy": {
                "servo_angles": {"0": 90, "1": 95, "2": 140, "3": 50, "4": 95, "5": 95},
                "timestamp": "default"
            }
        }

    def save_expressions_file(self) -> str:
        try:
            if os.path.exists(self.expressions_file):
                os.rename(self.expressions_file, f"{self.expressions_file}.backup")
            data = {"servo_names": self.servo_names, "expressions": self.expressions}
            with open(self.expressions_file, "w") as f:
                json.dump(data, f, indent=2)
            return "💾 Expressions saved"
        except Exception as e:
            return f"❌ Error saving: {e}"

    def save_expression(self, name: str, angles: List[int]) -> Tuple[str, str]:
        name = (name or "").strip()
        if not name:
            return "⚠️ Enter a name first", self.get_expression_library_text()

        pose = {
            "servo_angles": {str(i): int(angles[i]) for i in range(6)},
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
                    None, None, None, None, None, None)

        pose = self.expressions[name]
        self.move_to_pose_fast(pose)

        sa = pose["servo_angles"]
        angles = [sa[str(i)] for i in range(6)]
        status = f"▶️ Loaded '{name}'" + (" ✅ moved" if self.connected else " (preview)")
        return (status, self.get_expression_library_text(), 
                *angles)

    def get_expression_library_text(self) -> str:
        if not self.expressions:
            return "No expressions saved."
        lines = [f"📚 {len(self.expressions)} saved expression(s):", "-" * 40]
        for name, pose in sorted(self.expressions.items()):
            ts = pose.get("timestamp", "?")
            lines.append(f"  • {name}   ({ts})")
        return "\n".join(lines)

    # =========================================================================
    # EYE MOVEMENT PATTERNS
    # =========================================================================

    def load_patterns(self) -> str:
        try:
            if not os.path.exists(self.patterns_file):
                self.patterns = self._get_default_patterns()
                self.save_patterns_file()
                return "📂 Loaded default patterns"
            with open(self.patterns_file, "r") as f:
                data = json.load(f)
            self.patterns = data.get("patterns", {})
            return f"📂 Loaded {len(self.patterns)} pattern(s)"
        except Exception as e:
            self.patterns = self._get_default_patterns()
            return f"❌ Error loading patterns: {e}"

    def _get_default_patterns(self) -> dict:
        """Return default eye movement patterns."""
        return {
            "Saccade": {
                "description": "Quick eye movements between positions",
                "duration": 3.0
            },
            "Tracking": {
                "description": "Smooth eye tracking motion",
                "duration": 5.0
            },
            "Blink": {
                "description": "Single blink",
                "duration": 0.5
            },
            "Blink Rapid": {
                "description": "Multiple rapid blinks",
                "duration": 2.0
            },
            "Look Left": {
                "description": "Look to the left",
                "duration": 2.0
            },
            "Look Right": {
                "description": "Look to the right",
                "duration": 2.0
            },
            "Look Up": {
                "description": "Look up",
                "duration": 2.0
            },
            "Look Down": {
                "description": "Look down",
                "duration": 2.0
            },
            "Circle": {
                "description": "Circular eye movement",
                "duration": 4.0
            },
            "Dizzy": {
                "description": "Dizzy spinning eyes",
                "duration": 3.0
            }
        }

    def save_patterns_file(self) -> str:
        try:
            if os.path.exists(self.patterns_file):
                os.rename(self.patterns_file, f"{self.patterns_file}.backup")
            data = {"servo_names": self.servo_names, "patterns": self.patterns}
            with open(self.patterns_file, "w") as f:
                json.dump(data, f, indent=2)
            return "💾 Patterns saved"
        except Exception as e:
            return f"❌ Error saving: {e}"

    def start_pattern(self, pattern_name: str) -> str:
        """Start an eye movement pattern in a background thread."""
        if pattern_name not in self.patterns:
            return f"⚠️ No pattern '{pattern_name}'"
        
        if self._movement_active:
            self.stop_movement()
        
        self._stop_event.clear()
        self._movement_active = True
        self._current_pattern = pattern_name
        self._movement_thread = threading.Thread(
            target=self._pattern_worker, 
            args=(pattern_name,),
            daemon=True
        )
        self._movement_thread.start()
        return f"▶️ Started pattern: {pattern_name}"

    def stop_movement(self) -> str:
        """Stop the current movement pattern."""
        if not self._movement_active:
            return "ℹ️ No pattern running"
        self._stop_event.set()
        self._movement_active = False
        self.go_to_rest()
        return "⏹ Stopped pattern"

    def _pattern_worker(self, pattern_name: str):
        """Worker thread for eye movement patterns."""
        try:
            if pattern_name == "Saccade":
                self._pattern_saccade()
            elif pattern_name == "Tracking":
                self._pattern_tracking()
            elif pattern_name == "Blink":
                self._pattern_blink()
            elif pattern_name == "Blink Rapid":
                self._pattern_blink_rapid()
            elif pattern_name == "Look Left":
                self._pattern_look_left()
            elif pattern_name == "Look Right":
                self._pattern_look_right()
            elif pattern_name == "Look Up":
                self._pattern_look_up()
            elif pattern_name == "Look Down":
                self._pattern_look_down()
            elif pattern_name == "Circle":
                self._pattern_circle()
            elif pattern_name == "Dizzy":
                self._pattern_dizzy()
        except Exception as e:
            print(f"Pattern error: {e}")
        finally:
            self._movement_active = False
            if not self._stop_event.is_set():
                self.go_to_rest()

    def _pattern_saccade(self):
        """Quick eye movements between positions."""
        positions = [
            (90, 90), (70, 90), (110, 90), (90, 70), (90, 110),
            (70, 70), (110, 110), (70, 110), (110, 70)
        ]
        for pan, tilt in positions:
            if self._stop_event.is_set():
                break
            self.move_to_pose_fast({
                "servo_angles": {"0": pan, "1": tilt, "2": 65, "3": 110, "4": 90, "5": 90}
            })
            time.sleep(0.3)

    def _pattern_tracking(self):
        """Smooth eye tracking motion."""
        for t in np.linspace(0, 2*np.pi, 60):
            if self._stop_event.is_set():
                break
            pan = 90 + 20 * np.sin(t)
            tilt = 90 + 15 * np.cos(t)
            self.move_to_pose_fast({
                "servo_angles": {"0": int(pan), "1": int(tilt), "2": 65, "3": 110, "4": 90, "5": 90}
            })
            time.sleep(0.05)

    def _pattern_blink(self):
        """Single blink."""
        # Close
        self.move_to_pose_fast({
            "servo_angles": {"0": 90, "1": 90, "2": 90, "3": 90, "4": 90, "5": 90}
        })
        time.sleep(0.15)
        # Open
        self.move_to_pose_fast({
            "servo_angles": {"0": 90, "1": 90, "2": 65, "3": 110, "4": 90, "5": 90}
        })

    def _pattern_blink_rapid(self):
        """Multiple rapid blinks."""
        for _ in range(5):
            if self._stop_event.is_set():
                break
            self._pattern_blink()
            time.sleep(0.2)

    def _pattern_look_left(self):
        self.move_to_pose_fast({
            "servo_angles": {"0": 60, "1": 90, "2": 65, "3": 110, "4": 90, "5": 90}
        })
        time.sleep(1.5)

    def _pattern_look_right(self):
        self.move_to_pose_fast({
            "servo_angles": {"0": 120, "1": 90, "2": 65, "3": 110, "4": 90, "5": 90}
        })
        time.sleep(1.5)

    def _pattern_look_up(self):
        self.move_to_pose_fast({
            "servo_angles": {"0": 90, "1": 110, "2": 65, "3": 110, "4": 90, "5": 90}
        })
        time.sleep(1.5)

    def _pattern_look_down(self):
        self.move_to_pose_fast({
            "servo_angles": {"0": 90, "1": 70, "2": 65, "3": 110, "4": 90, "5": 90}
        })
        time.sleep(1.5)

    def _pattern_circle(self):
        """Circular eye movement."""
        for t in np.linspace(0, 2*np.pi, 30):
            if self._stop_event.is_set():
                break
            pan = 90 + 25 * np.sin(t)
            tilt = 90 + 20 * np.cos(t)
            self.move_to_pose_fast({
                "servo_angles": {"0": int(pan), "1": int(tilt), "2": 65, "3": 110, "4": 90, "5": 90}
            })
            time.sleep(0.05)

    def _pattern_dizzy(self):
        """Dizzy spinning eyes."""
        for _ in range(3):
            for t in np.linspace(0, 2*np.pi, 60):
                if self._stop_event.is_set():
                    break
                pan = 90 + 30 * np.sin(t)
                tilt = 90 + 20 * np.sin(2*t)
                self.move_to_pose_fast({
                    "servo_angles": {"0": int(pan), "1": int(tilt), "2": 65, "3": 110, "4": 90, "5": 90}
                })
                time.sleep(0.03)
            time.sleep(0.2)

    # =========================================================================
    # GET DISPLAY TEXT
    # =========================================================================

    def get_display_text(self) -> str:
        """Show current settings."""
        lines = ["📊 CURRENT SETTINGS:", "-" * 50]
        lines.append(f"{'#':2} {'Name':18} {'Initial':8} {'Offset':8} {'Actual':8}")
        lines.append("-" * 50)
        for i, name in enumerate(self.servo_names):
            actual = self.initial_positions[i] + self.offsets[i]
            lines.append(f"{i+1:2} {name:18} {self.initial_positions[i]:3d}°    {self.offsets[i]:+5.1f}°   {actual:3d}°")
        return "\n".join(lines)


# =============================================================================
# GRADIO INTERFACE
# =============================================================================

def create_interface():
    controller = EyeController()

    # Custom CSS for better mobile/touch support
    custom_css = """
    .gradio-container {
        max-width: 100% !important;
    }
    .slider-wrap {
        touch-action: none !important;
        -webkit-user-select: none !important;
        user-select: none !important;
    }
    input[type="range"] {
        width: 100% !important;
        height: 30px !important;
        -webkit-appearance: none !important;
        background: transparent !important;
    }
    input[type="range"]::-webkit-slider-runnable-track {
        width: 100% !important;
        height: 8px !important;
        background: #ddd !important;
        border-radius: 4px !important;
    }
    input[type="range"]::-webkit-slider-thumb {
        -webkit-appearance: none !important;
        width: 28px !important;
        height: 28px !important;
        border-radius: 50% !important;
        background: #6c5ce7 !important;
        margin-top: -10px !important;
        cursor: pointer !important;
        touch-action: none !important;
    }
    input[type="range"]::-moz-range-thumb {
        width: 28px !important;
        height: 28px !important;
        border-radius: 50% !important;
        background: #6c5ce7 !important;
        cursor: pointer !important;
        touch-action: none !important;
    }
    .group {
        padding: 10px !important;
        margin: 5px !important;
    }
    .tab-nav {
        flex-wrap: wrap !important;
    }
    """

    with gr.Blocks(
        title="Eye Control System",
        css=custom_css,
        theme=gr.themes.Soft()
    ) as demo:
        gr.Markdown("""
        # 👁️ Robot Eye Control System

        **Control eye, eyelids, and eyebrows with 6 servos**
        
        ---
        """)

        with gr.Tabs():
            # ================================================================
            # TAB 1: CONNECTION
            # ================================================================
            with gr.Tab("🔌 Connection"):
                with gr.Group():
                    with gr.Row():
                        port_dropdown = gr.Dropdown(
                            choices=controller.get_ports(),
                            label="Port",
                            value=controller.get_ports()[0] if controller.get_ports() else None,
                            interactive=True,
                            scale=2
                        )
                        refresh_btn = gr.Button("🔄 Refresh", scale=1)
                        connect_btn = gr.Button("🔗 Connect", variant="primary", scale=1)
                        disconnect_btn = gr.Button("🔌 Disconnect", variant="stop", scale=1)

                    status_text = gr.Textbox(value="Ready - Click Connect", label="Status", interactive=False)

                with gr.Row():
                    rest_btn = gr.Button("🏠 Go to Rest", variant="secondary", scale=1)
                    center_btn = gr.Button("🎯 Center All", variant="secondary", scale=1)
                    reload_cal_btn = gr.Button("🔄 Reload Calibration", scale=1)
                    reset_all_btn = gr.Button("🔄 Reset All to Defaults", variant="stop", scale=1)

                action_status = gr.Textbox(label="Action Status", interactive=False)
                display_text = gr.Textbox(
                    value=controller.get_display_text(),
                    label="📊 Current Settings",
                    interactive=False,
                    lines=10
                )

                # Handlers
                refresh_btn.click(
                    lambda: gr.update(choices=controller.get_ports(),
                                      value=controller.get_ports()[0] if controller.get_ports() else None),
                    outputs=[port_dropdown]
                )

                def _connect(port):
                    ok, msg = controller.connect(port)
                    return msg, gr.update(interactive=not ok), gr.update(interactive=ok)

                connect_btn.click(_connect, inputs=[port_dropdown], outputs=[status_text, connect_btn, disconnect_btn])

                def _disconnect():
                    controller.disconnect()
                    return "🔌 Disconnected", gr.update(interactive=True), gr.update(interactive=False)

                disconnect_btn.click(_disconnect, outputs=[status_text, connect_btn, disconnect_btn])

                rest_btn.click(lambda: (controller.go_to_rest(), "🏠 Rest pose"), outputs=[action_status])
                center_btn.click(lambda: (controller.center_all(), "🎯 Centered all"), outputs=[action_status])
                reload_cal_btn.click(controller.load_calibration, outputs=[action_status])
                reset_all_btn.click(
                    lambda: (controller.reset_to_defaults(), controller.get_display_text()),
                    outputs=[action_status, display_text]
                )

            # ================================================================
            # TAB 2: LIVE CONTROL (Multi-touch friendly)
            # ================================================================
            with gr.Tab("🎮 Live Control"):
                gr.Markdown("""
                ### Real-time Servo Control
                
                Drag any slider - they all work simultaneously with multi-touch support!
                """)

                # Create sliders in a grid layout for better touch support
                slider_dict = {}
                with gr.Row():
                    with gr.Column(scale=1):
                        gr.Markdown("### 👁️ Eye")
                        pan_slider = gr.Slider(
                            minimum=0, maximum=180, value=90, step=1,
                            label="Pan (Horizontal)",
                            interactive=True,
                            elem_classes="slider-wrap"
                        )
                        slider_dict[0] = pan_slider
                        tilt_slider = gr.Slider(
                            minimum=0, maximum=180, value=90, step=1,
                            label="Tilt (Vertical)",
                            interactive=True,
                            elem_classes="slider-wrap"
                        )
                        slider_dict[1] = tilt_slider

                    with gr.Column(scale=1):
                        gr.Markdown("### 👁️ Eyelids")
                        upper_slider = gr.Slider(
                            minimum=0, maximum=180, value=90, step=1,
                            label="Upper Eyelid",
                            interactive=True,
                            elem_classes="slider-wrap"
                        )
                        slider_dict[2] = upper_slider
                        lower_slider = gr.Slider(
                            minimum=0, maximum=180, value=90, step=1,
                            label="Lower Eyelid",
                            interactive=True,
                            elem_classes="slider-wrap"
                        )
                        slider_dict[3] = lower_slider

                    with gr.Column(scale=1):
                        gr.Markdown("### 👀 Eyebrow")
                        inner_slider = gr.Slider(
                            minimum=0, maximum=180, value=90, step=1,
                            label="Inner Eyebrow",
                            interactive=True,
                            elem_classes="slider-wrap"
                        )
                        slider_dict[4] = inner_slider
                        outer_slider = gr.Slider(
                            minimum=0, maximum=180, value=90, step=1,
                            label="Outer Eyebrow",
                            interactive=True,
                            elem_classes="slider-wrap"
                        )
                        slider_dict[5] = outer_slider

                live_status = gr.Textbox(label="Live Status", interactive=False, value="Ready")

                def _update_all(*angles):
                    all_angles = [int(a) for a in angles]
                    if controller.connected:
                        controller.send_batch_command(all_angles)
                        for i in range(6):
                            controller.angles[i] = all_angles[i]
                        return f"✅ Updated all servos"
                    return "❌ Not connected"

                # Group all sliders as inputs for batch update
                slider_inputs = [slider_dict[i] for i in range(6)]
                
                # Update on any slider change
                for i in range(6):
                    slider_dict[i].change(
                        _update_all,
                        inputs=slider_inputs,
                        outputs=[live_status]
                    )

            # ================================================================
            # TAB 3: EXPRESSIONS
            # ================================================================
            with gr.Tab("🎭 Expressions"):
                gr.Markdown("""
                ### Expression Presets
                
                Pose the eye, save as an expression, then load it instantly.
                """)

                expr_name = gr.Dropdown(
                    choices=list(controller.expressions.keys()),
                    value="Neutral",
                    label="Expression Name",
                    allow_custom_value=True,
                    interactive=True,
                )

                # Reuse sliders from live control
                with gr.Row():
                    save_expr_btn = gr.Button("💾 Save Expression", variant="primary")
                    load_expr_btn = gr.Button("▶️ Load Expression", variant="secondary")
                    delete_expr_btn = gr.Button("🗑️ Delete Expression", variant="stop")

                expr_library = gr.Textbox(
                    value=controller.get_expression_library_text(),
                    label="Saved Expressions",
                    interactive=False,
                    lines=6
                )

                # Save
                save_expr_btn.click(
                    controller.save_expression,
                    inputs=[expr_name] + slider_inputs,
                    outputs=[action_status, expr_library]
                )

                # Load
                def _load_expr(name):
                    status, lib_text, *angles = controller.apply_expression(name)
                    updates = [gr.update(value=float(a)) if a is not None else gr.update() for a in angles]
                    return [status, lib_text] + updates

                load_expr_btn.click(
                    _load_expr,
                    inputs=[expr_name],
                    outputs=[action_status, expr_library] + slider_inputs
                )

                # Delete
                delete_expr_btn.click(
                    controller.delete_expression,
                    inputs=[expr_name],
                    outputs=[action_status, expr_library]
                )

            # ================================================================
            # TAB 4: MOVEMENT PATTERNS
            # ================================================================
            with gr.Tab("🔄 Patterns"):
                gr.Markdown("""
                ### Eye Movement Patterns
                
                Run pre-programmed eye movement patterns for expressive behavior.
                """)

                pattern_name = gr.Dropdown(
                    choices=list(controller.patterns.keys()),
                    value="Saccade",
                    label="Pattern Name",
                    interactive=True,
                )

                pattern_desc = gr.Textbox(
                    value=controller.patterns.get("Saccade", {}).get("description", ""),
                    label="Pattern Description",
                    interactive=False,
                )

                with gr.Row():
                    start_pattern_btn = gr.Button("▶️ Start Pattern", variant="primary")
                    stop_pattern_btn = gr.Button("⏹ Stop Pattern", variant="stop")

                pattern_status = gr.Textbox(label="Pattern Status", interactive=False, value="Idle")

                def _update_pattern_desc(name):
                    desc = controller.patterns.get(name, {}).get("description", "")
                    return gr.update(value=desc)

                pattern_name.change(_update_pattern_desc, inputs=[pattern_name], outputs=[pattern_desc])

                start_pattern_btn.click(
                    controller.start_pattern,
                    inputs=[pattern_name],
                    outputs=[pattern_status]
                )

                stop_pattern_btn.click(
                    controller.stop_movement,
                    outputs=[pattern_status]
                )

            # ================================================================
            # TAB 5: CALIBRATION
            # ================================================================
            with gr.Tab("⚙️ Calibration"):
                gr.Markdown("""
                ### Servo Calibration
                
                For each servo:
                - **Slider** — jog the servo live
                - **Initial** — set the rest position
                - **Offset** — hardware trim (saved to `eye_calibration.json`)
                """)

                for idx in range(6):
                    with gr.Group(elem_id=f"servo_{idx}"):
                        with gr.Row():
                            gr.Markdown(f"**{idx+1}. {controller.servo_names[idx]}**")
                            cal_slider = gr.Slider(
                                minimum=0, maximum=180,
                                value=float(controller.initial_positions[idx]),
                                step=1, label="Position", interactive=True
                            )

                        with gr.Row():
                            init_input = gr.Number(
                                value=controller.initial_positions[idx],
                                label="Initial", minimum=0, maximum=180, step=1, precision=0
                            )
                            offset_input = gr.Number(
                                value=controller.offsets[idx],
                                label="Offset", minimum=-90, maximum=90, step=0.5, precision=1
                            )

                        with gr.Row():
                            set_init_btn = gr.Button("📌 Set Initial", size="sm", variant="primary")
                            set_offset_btn = gr.Button("⚙️ Set Offset", size="sm")

                            def _make_set_init(i):
                                def _handler(v):
                                    status = controller.set_initial_position(i, int(v or 90))
                                    return status, controller.get_display_text()
                                return _handler

                            set_init_btn.click(
                                _make_set_init(idx),
                                inputs=[init_input],
                                outputs=[action_status, display_text]
                            )

                            def _make_set_offset(i):
                                def _handler(v):
                                    status = controller.set_offset(i, float(v or 0.0))
                                    return status, controller.get_display_text()
                                return _handler

                            set_offset_btn.click(
                                _make_set_offset(idx),
                                inputs=[offset_input],
                                outputs=[action_status, display_text]
                            )

                        # Live slider update
                        def _make_cal_slider_handler(i):
                            def _handler(v):
                                val = int(v)
                                if controller.connected:
                                    controller.move_servo(i, val)
                                return f"✅ {controller.servo_names[i]} → {val}°" if controller.connected else "❌ Disconnected"
                            return _handler

                        cal_slider.change(
                            _make_cal_slider_handler(idx),
                            inputs=[cal_slider],
                            outputs=[action_status]
                        )

    return demo


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    demo = create_interface()
    demo.launch(
        server_name="0.0.0.0",  # Allow external access
        server_port=7860,
        share=False,
        debug=False
    )