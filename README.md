# 👁️👁️ Robot Eye Control System

A complete control system for dual animatronic robot eyes — 6 servos per eye (12 total). Includes real-time control, per-axis mirror calibration, expression presets, autonomous movement patterns, and a webcam face tracker with random blinking.

## 📋 Table of Contents

- [✨ Features](#-features)
- [🛠️ Hardware Requirements](#️-hardware-requirements)
- [📦 Installation](#-installation)
- [🚀 Quick Start](#-quick-start)
- [📖 Usage Guide](#-usage-guide)
- [🎥 Face Tracker](#-face-tracker)
- [📁 File Structure](#-file-structure)
- [⚙️ Calibration Data](#️-calibration-data)
- [🔧 Troubleshooting](#-troubleshooting)
- [🎯 Performance Tips](#-performance-tips)
- [🔄 Expanding / Modifying](#-expanding--modifying)
- [📝 License](#-license)
- [🤝 Contributing](#-contributing)
- [📧 Support](#-support)

## ✨ Features

- **Dual-eye control** — 12 servos (6 per eye: pan, tilt, upper lid, lower lid, brow inner, brow outer)
- **Real-time control** — all 12 sliders work simultaneously with multi-touch support
- **Per-axis mirror calibration** — configure for each axis:
  - Flip (physically mirrored servos)
  - Gain (compensate for mechanical sweep mismatch: left = neutral + flip(delta) × gain)
- **Fast serial protocol** — one batch command updates all 12 servos (~2 ms)
- **Expression presets** — Happy, Sad, Angry, Surprised, Suspicious, Sleepy, Neutral
- **Autonomous patterns** — Saccade, Tracking, Blink, Blink Rapid, Look L/R/U/D, Circle, Dizzy
- **Dedicated Offsets tab** — live per-servo offset tuning
- **Face tracker** — webcam-driven eye movement with random blinking (uses the same calibration)
- **Responsive UI** — mobile-friendly, custom touch-friendly sliders
- **Persistent config** — calibration, mirror config, gains, expressions and patterns all saved to JSON

## 🛠️ Hardware Requirements

### Components

- Arduino (Uno, Mega, Nano, etc.)
- PCA9685 16-channel PWM servo driver
- 12× servos (MG995 or similar 180° servos)
- External 5 V power supply — 5 V, 4 A minimum recommended for 12 servos

### Servo Layout

Each eye uses 6 servos in the same order:

| Per-eye # | Function | Range |
|---:|---|---:|
| 0 | Eye Pan (horizontal) | 0–180° |
| 1 | Eye Tilt (vertical) | 0–180° |
| 2 | Upper Eyelid | 0–180° |
| 3 | Lower Eyelid | 0–180° |
| 4 | Eyebrow Inner | 0–180° |
| 5 | Eyebrow Outer | 0–180° |

### PCA9685 Channel Mapping

| Servo index (in software) | Eye | PCA9685 channel |
|---|---|---:|
| 0–5 | Right eye | 0–5 |
| 6–11 | Left eye | 8–13 |

Channels 6, 7, 14, 15 are unused.

### Wiring Diagram

#### PCA9685 → Arduino

```text
PCA9685      Arduino
  VCC   →    5V
  GND   →    GND
  SCL   →    A5 (or SCL)
  SDA   →    A4 (or SDA)
```

#### PCA9685 → Servos

Each servo:

```text
  + (Red)     → PCA9685 V+  (external 5V supply)
  - (Brown)   → PCA9685 GND
  S (Orange)  → PCA9685 PWM channel
```

> **Important:** Servos must be powered from an external 5 V supply that shares GND with the Arduino. Do not power 12 servos from the Arduino's 5 V rail.

## 📦 Installation

### Option 1: Conda (Recommended)

```bash
# Create environment
conda create -n eye_control python=3.9 -y

# Activate
conda activate eye_control

# Install dependencies
pip install -r requirements.txt
```

### Option 2: Plain pip / venv

```bash
python -m venv venv
source venv/bin/activate          # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### `requirements.txt`

```text
gradio==4.44.1
huggingface_hub>=0.23,<0.27
pyserial==3.5
numpy==1.24.3
plotly==5.18.0
opencv-python
```

Gradio is pinned to 4.x for Python 3.9 compatibility (Gradio 5/6 require Python 3.10+). On newer Python, you can use Gradio 6 but must move theme= and css= from gr.Blocks(...) to demo.launch(...).

## 🚀 Quick Start

### 1. Upload Arduino Code

Open `eye.ino` in the Arduino IDE.

Select your board and port.

Upload.

### 2. Run the Python Application

```bash
conda activate eye_control
python eye_system.py
```

### 3. Open the UI

Navigate to:

```text
http://localhost:7860
```

### 4. Connect to Hardware

Pick the COM port (Windows) or `/dev/ttyUSB*` (Linux/macOS).

Click 🔗 **Connect**.

Status should read ✅ **Connected to <port>**.

### 5. Calibrate

Recommended order for a fresh setup:

1. ⚙️ **Calibration tab** — set the rest position (Initial) for each of the 12 servos.
2. 🎚️ **Offsets tab** — tune each servo's offset live to align both eyes perfectly at rest.
3. 🎮 **Live Control tab** — flip and gain per axis:
   - Move the right-eye slider; click 🔁 **Apply mirror to LEFT eye**
   - If the left eye moves opposite → toggle that axis's Flip checkbox
   - If the left eye sweeps a different amount → adjust that axis's Gain slider
   - Use 🧪 **Test gain on this axis** for a smooth ±25° sweep on a single axis
   - Click 💾 **Save mirror config** to persist
4. 🎭 **Expressions tab** → click ♻️ **Regenerate Defaults** to rebuild expressions with the new mirror config.

## 📖 Usage Guide

### 🔌 Connection

| Control | Purpose |
|---|---|
| Port dropdown + 🔄 Refresh | Choose and refresh serial ports |
| 🔗 Connect / 🔌 Disconnect | Open/close the serial link |
| 🏠 Go to Rest | Move all 12 servos to their calibrated Initial positions |
| 🎯 Center All | Send 90° to every servo |
| 🔄 Reload Calibration | Re-read `eye_calibration.json` from disk |
| 🔄 Reset All to Defaults | Zero offsets, neutrals back to 90°, mirror config back to defaults |

### 🎮 Live Control

- 12 sliders — 6 right eye, 6 left eye; all work simultaneously
- Flip checkboxes (one per axis) — invert direction when mirroring
- Gain sliders (one per axis) — scale the left eye's sweep vs. right
- 🔁 **Apply mirror to LEFT eye** — copies right-eye pose to left with flip + gain
- 💾 **Save mirror config** — persists flip axes and gains
- 🧪 **Test both axes** — sweeps pan, then tilt on both eyes
- 🧪 **Test gain on this axis** — smooth ±25° sine sweep on a chosen axis (great for tuning gains)

### 🎭 Expressions

- Presets: Neutral, Happy, Sad, Angry, Surprised, Suspicious, Sleepy
- 💾 **Save Expression** — snapshot all 12 angles under a name
- ▶️ **Load Expression** — move to the saved pose
- 🗑️ **Delete Expression** — remove a preset
- ♻️ **Regenerate Defaults** — rebuild built-in expressions with current mirror config and gains

### 🔄 Patterns

| Pattern | Description |
|---|---|
| Saccade | Quick jumps between positions |
| Tracking | Smooth circular-ish motion |
| Blink | Single blink |
| Blink Rapid | Five quick blinks |
| Look Left / Right | Hold eye at an extreme |
| Look Up / Down | Hold at vertical extreme |
| Circle | Smooth circular pan-tilt |
| Dizzy | Layered sin waves, "spinning" look |

Each pattern can be mirrored (checkbox) so the left eye follows using the current flip + gain config.

### 🎚️ Offsets

- 12 sliders — one per servo, live-tune while watching the hardware
- ↺ — reset one offset to 0
- 💾 **Save all offsets** — persist to `eye_calibration.json`
- ↺ **Reset ALL offsets to 0** — zero all offsets in one shot
- Each row shows `Base: X° · Now: Y° · Sent: Z°` so you can see the effective angle

### ⚙️ Calibration

Set Initial (rest/neutral angle) per servo.

These neutrals are used as the pivot point for mirroring, so gains remain valid regardless of where you set rest.

## 🎥 Face Tracker

`face_tracker.py` uses a webcam (OpenCV Haar cascade) to follow a face and adds random blinking between 1–3 seconds.

### Run

```bash
# Preview window (recommended first run)
python face_tracker.py --port COM3

# Headless (e.g. onboard PC on the robot)
python face_tracker.py --port /dev/ttyUSB0 --no-preview
```

Press `q` in the preview or `Ctrl+C` in the terminal to stop. The robot returns to rest on exit.

### Common Options

| Flag | Default | Purpose |
|---|---|---|
| `--port` | (required) | Serial port |
| `--camera` | `0` | Webcam index |
| `--max-pan` | `30.0` | Max pan offset from neutral when face is at the edge of the frame |
| `--max-tilt` | `20.0` | Max tilt offset |
| `--smoothing` | `0.15` | EMA factor (higher = snappier) |
| `--blink-min` | `1.0` | Min seconds between blinks |
| `--blink-max` | `3.0` | Max seconds between blinks |
| `--upper-open` | `65.0` | Right upper lid at full open (matches `_pattern_blink`) |
| `--lower-open` | `110.0` | Right lower lid at full open |
| `--upper-closed` | `90.0` | Right upper lid at full close |
| `--lower-closed` | `90.0` | Right lower lid at full close |
| `--lost-grace` | `1.0` | Seconds before returning to neutral after face lost |
| `--no-preview` | `off` | Run without a camera window |

### Tuning Cheat Sheet

| Want… | Change |
|---|---|
| Eyes to move more per face movement | ↑ `--max-pan` / `--max-tilt` |
| Subtler, calmer tracking | ↓ `--max-pan` to 10–15 |
| Faster reactions | ↑ `--smoothing` to 0.20–0.30 |
| Slower / cinematic | ↓ `--smoothing` to 0.05–0.10 |
| Eyelids close too much | Lower `--upper-closed` (e.g. 85) and/or raise `--lower-closed` (e.g. 95) |

### Calibration Used by the Tracker

The face tracker pulls the following from `eye_calibration.json`:

| Setting | Used? | Where |
|---|---|---|
| Pan/Tilt neutrals | ✅ | pivot for face tracking |
| Brow neutrals | ✅ | brow angles |
| Offsets (all 12) | ✅ | applied on the way to serial |
| Mirror flip (per axis) | ✅ | `mirror_right_to_left` |
| Mirror gains (per axis) | ✅ | `mirror_right_to_left` |

> **Note on eyelids:** the tracker uses its own `DEFAULT_UPPER_OPEN / LOWER_OPEN / UPPER_CLOSED / LOWER_CLOSED` constants (module-level, overridable via CLI) rather than the eyelid neutrals from the calibration file — this is intentional so blinks match the Blink pattern in `eye_system.py`.

## 📁 File Structure

```text
eye_control_system/
├── eye_system.py           # Main Gradio application
├── face_tracker.py         # Webcam face tracker with random blinks
├── eye.ino                 # Arduino firmware (PCA9685 driver)
├── requirements.txt        # Python dependencies
├── README.md               # This file
├── LICENSE                 # License file
├── eye_calibration.json    # Auto-generated: neutrals, offsets, mirror config + gains
├── eye_expressions.json    # Auto-generated: expression presets
└── eye_patterns.json       # Auto-generated: movement patterns
```

## ⚙️ Calibration Data

`eye_calibration.json` is the single source of truth for the whole system — both the app and the face tracker read it on startup.

```json
{
  "initial_positions": [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90],
  "offsets":           [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  "mirror_axes":       [false, true, true, true, true, true],
  "mirror_gains":      [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
  "servo_names":       ["R Eye Pan", "R Eye Tilt", "..."],
  "timestamp":         "2025-01-01 12:00:00"
}
```

### Mirror Formula

For each of the 6 axes (index i = 0..5):

```text
delta       = right_angle - right_neutral[i]
if mirror_axes[i]: delta = -delta
delta      *= mirror_gains[i]
left_angle  = clamp(left_neutral[i] + delta, 0, 180)
```

### What Each Field Controls

- `initial_positions` — rest/neutral angle for each servo (pivots for mirroring)
- `offsets` — added to the logical angle before sending to hardware: `sent = logical + offset`
- `mirror_axes` — flip true/false per axis
- `mirror_gains` — sweep scaling per axis (used only on the left eye)

The file is written with `os.replace` (atomic on Windows and Linux), so backups (`*.json.backup`) are safe to overwrite on every save.

## 🔧 Troubleshooting

### `ImportError: cannot import name 'HfFolder' from 'huggingface_hub'`

Pin `huggingface_hub` to `<0.27`:

```bash
pip install "huggingface_hub>=0.23,<0.27"
```

### `TypeError: __init__() got an unexpected keyword argument 'scale' on gr.Markdown`

You're on Gradio 4.x — `gr.Markdown` doesn't accept `scale`. Wrap it in a `gr.Column(scale=N)` instead, or remove `scale=`.

### `TypeError: launch() got an unexpected keyword argument 'theme'`

You're on Gradio 4.x or 5.x — theme and css belong on `gr.Blocks(...)`, not on `demo.launch(...)`. (On Gradio 6.x it's the opposite.)

### `AttributeError: 'EyeController' object has no attribute 'save_calibration'`

Your `eye_system.py` is missing the `save_calibration` method. Restore it inside the `EyeController` class.

### Saved gains don't persist / second save silently fails on Windows

Cause: using `os.rename` for backups raises `FileExistsError` on Windows when the `.backup` file already exists. Use `os.replace` in `save_calibration`, `save_expressions_file`, and `save_patterns_file`.

### `KeyError: '6'` when loading expressions

Expression file was written when the project was single-eye. Delete `eye_expressions.json` and restart, or open the app and click ♻️ **Regenerate Defaults**.

### Serial Connection Issues

#### Linux — add user to `dialout` group

```bash
sudo usermod -a -G dialout $USER
```

#### Linux — check port permissions

```bash
ls -la /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0      # replace with your port
```

#### Windows — COM port lookup

Open Device Manager → Ports (COM & LPT) and note the port.

#### Port already in use

```bash
# Linux/Mac
lsof /dev/ttyUSB0
kill -9 [PID]
```

Windows: use Device Manager or restart.

### Servo Not Moving

- Verify servos have an external 5 V supply (not the Arduino rail)
- Check PCA9685 I2C address (default `0x40`)
- Verify the servo signal wires go to the channels in the mapping table
- Click 🎯 **Center All** first — if nothing moves, it's a hardware issue

### Conda Environment Issues

```bash
conda update conda
conda env remove -n eye_control
conda create -n eye_control python=3.9 -y
conda activate eye_control
pip install -r requirements.txt
```

## 🎯 Performance Tips

- **Batch protocol** — all 12 servos are updated in a single `P,...` serial command
- **Change detection** — the UI skips serial writes when nothing moved by ≥ 1°
- **Background threads** — patterns and the face tracker run off the UI thread
- **Multi-touch** — sliders don't block each other
- **Smoothing (`--smoothing`)** — tune the tracker's EMA to trade latency for smoothness
- **Offset cache** — offsets are applied in memory before serializing, no extra round-trips

## 🔄 Expanding / Modifying

### Change the Servo Count

Update `NUM_SERVOS` in both `eye.ino` and `eye_system.py`, then extend `SERVO_NAMES`. The batch protocol, calibration JSON and UI all derive from these.

### Change the PCA9685 Channel Mapping

Edit the `SERVO_CHANNELS` array at the top of `eye.ino` — it maps software servo index (0..N−1) to physical PCA9685 channel.

### Add a New Pattern

Add a method `_pattern_<name>(self, mirror=True)` to `EyeController`.

Add it to `_default_patterns()` (description) and to the dispatch dict in `_pattern_worker`.

### Add a New Expression

Just save it from the UI, or add it to `right_eyes` in `_default_expressions()` (left eye is auto-mirrored).

### Multiple Eyes / More Axes

Because the mirror math is generic per-axis, extending to extra eyeballs or extra axes only requires:

- Extending `SERVO_NAMES` and `NUM_SERVOS`
- Extending `mirror_axes` / `mirror_gains` arrays
- Updating `eye.ino`'s `SERVO_CHANNELS` to match the new channel order

## 📝 License

This project is licensed under the MIT License — see the `LICENSE` file for details.

## 🤝 Contributing

Contributions are welcome! Please open a Pull Request or file an issue.

## 📧 Support

- Check the Troubleshooting section first
- Open an issue on GitHub with the full console output and the contents of `eye_calibration.json` (feel free to redact timestamps)
- Confirm your Python version (`python --version`), Gradio version (`pip show gradio`), and OS
