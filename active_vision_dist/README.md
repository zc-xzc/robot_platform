# Active Vision System

PICO 4 head tracking → real-time dual-axis STS3032 gimbal → D415 camera follow → video feedback to headset.

**"Head moves, view moves" — immersive remote vision.**

---

## Hardware

| Component | Model | Notes |
|-----------|-------|-------|
| Headset | PICO 4 | WiFi connection to PC |
| Servos (×2) | STS3032 12V (ST-3032-C062) | ID1=horizontal, ID2=vertical |
| Serial adapter | URT-1 | USB to TTL |
| Camera | Intel RealSense D415 | USB 3.0, mounted on gimbal |
| PC | Windows/Linux | WiFi + USB ports |

The PICO 4 runs [PicoBridge](https://github.com/OpenRobotTech/PicoBridge) APK to stream head tracking data (quaternions + body joints) to the PC via WiFi.

---

## Installation

### 1. Python environment

```bash
conda create -n active_vision python=3.10 -y
conda activate active_vision
```

### 2. Dependencies

```bash
pip install -r requirements.txt

### 4. PicoBridge (head tracking data from PICO)

```bash
pip install pico_bridge-0.2.1-py3-none-any.whl
```
```

### 3. PICO Headset

Install the PicoBridge APK on your PICO 4:
- `PicoBridge_v0.2.1_20260522_release.apk`

Connect PICO and PC to the same WiFi network. Open PicoBridge on the headset.

---

## Quick Start

### Windows

```bash
cd windows
python run.py
```

### Linux

```bash
cd linux
python run.py
```

---

## First-Time Setup

### Step 1: Calibrate servo limits

The servos will scan their physical range, then lock at position 2048 (center).

```bash
python run.py --calibrate
```

**After servos lock at 2048:** Manually rotate the gimbal bracket so the camera points straight forward. Tighten the screws. Press Enter.

This creates `gimbal_config.json` automatically. Do this once after each hardware reassembly.

### Step 2: Tune servo internal PID (eliminate jitter)

```bash
python run.py --tune-servo
```

This writes optimized PID parameters (D=10, dead zone=8) to the servo EEPROM. One-time operation.

### Step 3: Verify direction

```bash
python run.py
```

Put on the headset. Look forward. Wait for `[CALIBRATED]`. Turn your head:

| Head movement | Camera should |
|---------------|---------------|
| Look up | Point up |
| Look down | Point down |
| Turn right | Point right |

If any direction is reversed, add the corresponding flag:

```bash
python run.py --no-inv-yaw            # horizontal reversed
python run.py --no-inv-pitch          # vertical reversed
python run.py --no-inv-yaw --no-inv-pitch  # both reversed
```

---

## Port Configuration

| OS | Default Port | Override |
|----|-------------|----------|
| Windows | COM6 | `--port COM5` |
| Linux | /dev/ttyUSB0 | `--port /dev/ttyUSB1` |

---

## Parameter Tuning

Default parameters (optimized for STS3032 12V via automated step-response testing):

| Parameter | Default | Range | Effect |
|-----------|---------|-------|--------|
| `--acc` | 80 | 0-254 | Acceleration (higher = faster response, 254 = speed cap) |
| `--kp` | 2.0 | 0.1-10 | Tracking aggressiveness (higher = tighter follow) |
| `--kd` | 0.5 | 0-2 | Damping (higher = smoother stops) |

### Real-time tuning (while tracking)

Open a **second terminal** in the project root:

```bash
echo acc 60 > tune_cmd.txt
echo kp 2.5 > tune_cmd.txt
echo kd 0.7 > tune_cmd.txt
echo c    > tune_cmd.txt    # recalibrate center
echo q    > tune_cmd.txt    # quit
```

### Quick presets

```bash
# Aggressive (maximum speed)
python run.py --acc 0 --kp 2.5 --kd 0.7

# Balanced (recommended)
python run.py --acc 80 --kp 2.0 --kd 0.5

# Conservative (smoothest)
python run.py --acc 100 --kp 1.5 --kd 0.3
```

---

## Modes

| Command | Description |
|---------|-------------|
| `python run.py` | Full tracking (default) |
| `python run.py --calibrate` | Servo limit calibration |
| `python run.py --test-head` | Test PICO connection only |
| `python run.py --test-camera` | Test D415 streaming only |
| `python run.py --log data.csv` | Track + save data to CSV |
| `python run.py --no-camera` | Track without camera |
| `python run.py --no-body` | Track without body-relative mode |

---

## Auto-Tuning (optional)

To find optimal KP/KD/ACC values for your specific hardware:

```bash
python autotune.py
```

Tests 100+ parameter combinations with actual servo step-response measurement. Outputs the best command. Takes ~5 minutes.

---

## Data Collection

```bash
python run.py --log experiment_001.csv
```

14-column CSV: timestamp, frame#, raw yaw/pitch/roll, target position, PD-smoothed position, servo steps, speed, KP, KD.

---

## Algorithm

```
PICO quaternion (50Hz)
  → TWIST2 body-relative: q_rel = conjugate(spine) × head
  → YXZ Euler decomposition: yaw, pitch, roll
  → Normalize: yaw/90, pitch/60 → [-1, +1]
  → Jump (|error| > 7.2°): instant position, 0 delay
  → PD (|error| ≤ 7.2°): 2.0×error + 0.5×d(error) + feedforward
  → Servo mapping: position = 2048 + normalized × factor
    (1024 steps = 90° horizontal, 682 steps = 60° vertical)
  → WritePosEx(ID, position, speed, acceleration)
  → STS3032 movement → D415 follows → video back to PICO
```

---

## File Structure

```
active-vision-dist/
├── README.md              ← this file
├── requirements.txt       ← Python dependencies
├── autotune.py            ← auto-parameter optimization tool
├── scservo_sdk/           ← FeiTech servo SDK (do not modify)
├── windows/
│   └── run.py             ← Windows version (default COM6)
└── linux/
    └── run.py             ← Linux version (default /dev/ttyUSB0)
```

Runtime files (auto-generated):
- `gimbal_config.json` — created by `--calibrate`
- `tune_cmd.txt` — real-time tuning commands

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| Servo not responding | Check power supply (12V, 3A+) |
| PICO not connecting | Same WiFi, PicoBridge APK open |
| Jitter / oscillation | `python run.py --acc 100 --kd 0.8` |
| Too slow | `python run.py --acc 50 --kp 2.5` |
| Direction reversed | Add `--no-inv-yaw` or `--no-inv-pitch` |
| Center drifted | Look forward, `echo c > tune_cmd.txt` |
