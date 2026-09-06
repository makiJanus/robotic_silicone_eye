# 👁️ Robot Eye Control System

A complete control system for animatronic robot eyes using **6 servos** (expandable to 12). Features real-time control, expression presets, and autonomous movement patterns with multi-touch support.

---

## 📋 Table of Contents

* [✨ Features](#-features)
* [🛠️ Hardware Requirements](#️-hardware-requirements)
* [📦 Installation](#-installation)
* [🚀 Quick Start](#-quick-start)
* [📖 Usage Guide](#-usage-guide)
* [📁 File Structure](#-file-structure)
* [🔧 Troubleshooting](#-troubleshooting)
* [🎯 Performance Tips](#-performance-tips)
* [🔄 Future Expansion (12 Servos)](#-future-expansion-12-servos)
* [📝 License](#-license)
* [🤝 Contributing](#-contributing)
* [📧 Support](#-support)

---

## ✨ Features

* **Real-time Control:** 6 servos controlled simultaneously with multi-touch support
* **Expression Presets:** Pre-configured expressions (Happy, Sad, Angry, Surprised, etc.)
* **Autonomous Patterns:** Eye movement behaviors (saccades, tracking, blinking, circular motion)
* **Fast Communication:** Batch commands eliminate serial delays
* **Calibration System:** Save/load initial positions and offsets
* **Responsive UI:** Mobile-friendly with custom touch controls
* **Expandable:** Ready for 12 servos (2 eyes)

---

## 🛠️ Hardware Requirements

### Components

* Arduino Board (Uno, Mega, or compatible)
* PCA9685 PWM Servo Driver (16-channel)
* MG995 Servos (or similar 180° servos)
* Power Supply (5V 2A minimum for servos)

### Servo Configuration

| Servo # | Function             |  Range | Default |
| ------: | -------------------- | -----: | ------: |
|       0 | Eye Pan (Horizontal) | 0–180° |     90° |
|       1 | Eye Tilt (Vertical)  | 0–180° |     90° |
|       2 | Upper Eyelid         | 0–180° |     90° |
|       3 | Lower Eyelid         | 0–180° |     90° |
|       4 | Inner Eyebrow        | 0–180° |     90° |
|       5 | Outer Eyebrow        | 0–180° |     90° |

### Wiring Diagram

#### PCA9685 → Arduino

```text
PCA9685 → Arduino
  VCC   → 5V
  GND   → GND
  SCL   → A5 (or SCL)
  SDA   → A4 (or SDA)
```

#### PCA9685 → Servos (CH0–CH5)

```text
PCA9685 → Servos (CH0-CH5)

  Each servo:
    + (Red)    → PCA9685 V+ (External 5V)
    - (Brown)  → PCA9685 GND
    S (Orange) → PCA9685 PWM Channel
```

---

## 📦 Installation

### Option 1: Using Conda (Recommended)

1. Clone or download this repository.

2. Create and activate the conda environment:

```bash
# Create environment with Python 3.9
conda create -n eye_control python=3.9

# Activate environment
conda activate eye_control

# Install pip within the environment
conda install pip

# Install required packages
pip install -r requirements.txt
```

---

## 🚀 Quick Start

### 1. Upload Arduino Code

1. Open `eye_control.ino` in Arduino IDE.
2. Select your board and port.
3. Upload the code.

### 2. Run the Python Application

```bash
# With conda environment
conda activate eye_control
python eye_system.py

# Or directly if using system Python
python eye_system.py
```

### 3. Connect to the Interface

Open your browser and navigate to:

```text
http://localhost:7860
```

### 4. Connect to Hardware

1. Select the COM port (Windows) or `/dev/ttyUSB*` (Linux/Mac).
2. Click **"Connect"**.
3. The status will show **"✅ Connected to [port]"**.

---

## 📖 Usage Guide

### Tab 1: Connection

* **Port Selection:** Choose your Arduino port
* **Connect/Disconnect:** Establish or break serial connection
* **Go to Rest:** Move all servos to calibrated rest position
* **Center All:** Move all servos to 90°
* **Reload Calibration:** Reload saved calibration data

### Tab 2: Live Control

* **Multi-touch Sliders:** All 6 servos controlled simultaneously
* **Pan:** Horizontal eye rotation (0–180°)
* **Tilt:** Vertical eye rotation (0–180°)
* **Upper Eyelid:** Controls eyelid opening/closing
* **Lower Eyelid:** Controls lower lid position
* **Inner Eyebrow:** Inner eyebrow angle
* **Outer Eyebrow:** Outer eyebrow angle

### Tab 3: Expressions

* **Presets:** Neutral, Happy, Sad, Angry, Surprised, Suspicious, Sleepy
* **Save Expression:** Save current pose as new expression
* **Load Expression:** Apply saved expression
* **Delete Expression:** Remove saved expression

### Tab 4: Movement Patterns

* **Saccade:** Quick eye movements between positions
* **Tracking:** Smooth eye tracking motion
* **Blink:** Single blink
* **Blink Rapid:** Multiple rapid blinks
* **Look Left/Right:** Directional looking
* **Look Up/Down:** Vertical looking
* **Circle:** Circular eye movement
* **Dizzy:** Spinning eye motion

### Tab 5: Calibration

* **Position Slider:** Jog individual servos
* **Initial:** Set rest position for each servo
* **Offset:** Apply hardware offset calibration
* All settings saved to `eye_calibration.json`

---

## 📁 File Structure

```text
eye_control_system/
├── eye_system.py          # Main Python application
├── eye_control.ino        # Arduino firmware
├── eye_calibration.json   # Calibration data (auto-generated)
├── eye_expressions.json   # Saved expressions (auto-generated)
├── eye_patterns.json      # Movement patterns (auto-generated)
├── environment.yml        # Conda environment file
├── README.md              # This file
└── LICENSE                # License file
```

---

## 🔧 Troubleshooting

### Serial Connection Issues

#### Linux: Add User to `dialout` Group

```bash
sudo usermod -a -G dialout $USER
```

#### Linux: Check Port Permissions

```bash
ls -la /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0  # Replace with your port
```

#### Windows

Check the COM port in **Device Manager**.

---

### Servo Not Moving

* Verify power supply (servos need external power)
* Check PCA9685 I2C address (default `0x40`)
* Verify wiring connections
* Test with center button first

---

### Connection Fails

* Ensure Arduino is plugged in
* Check correct port selected
* Restart Arduino (press reset button)
* Close other serial applications

---

### Conda Environment Issues

```bash
# Update conda
conda update conda

# Remove and recreate environment
conda env remove -n eye_control
conda env create -f environment.yml
```

---

### Port Already in Use

#### Linux/Mac

```bash
lsof /dev/ttyUSB0  # Find process using port
kill -9 [PID]      # Kill the process
```

#### Windows

Use Device Manager or restart computer.

---

## 🎯 Performance Tips

* **Fast Mode:** The system automatically uses batch commands for minimal latency
* **Multi-touch:** All sliders update simultaneously — no blocking
* **Change Detection:** Only sends updates when angles change significantly
* **Background Threads:** Movement patterns don't block the UI

---

## 🔄 Future Expansion (12 Servos)

The system is ready for dual-eye control:

* **Arduino:** Update `NUM_SERVOS` to 12
* **Python:** Add servo definitions for second eye
* **UI:** Add mirror/independent control options

---

## 📝 License

This project is licensed under the MIT License — see the `LICENSE` file for details.

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

## 📧 Support

For issues and questions:

* Open an issue on GitHub
* Check the troubleshooting section
* Ensure all dependencies are installed correctly
