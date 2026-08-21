# Active Vision System — Technical Reference

## 1. Control Chain & Parameter Map

Every parameter sits at a precise physical location in the pipeline:

```
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 1: PICO Head Tracking (pico_bridge, ~50Hz)               │
│                                                                 │
│  Head quaternion [x,y,z,w]  +  Spine quaternion (joint[3])     │
└──────────────────────────────┬──────────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 2: TWIST2 Body-Relative Tracking (tracker.py)             │
│                                                                 │
│  q_body_rel = conjugate(spine) × head                          │
│    → Head rotation relative to body                            │
│    → Body turns, head still → q_body_rel = identity → no move  │
│    → Head turns, body still → q_body_rel = head rot → move     │
│                                                                 │
│  q_rel = q_body_rel × conjugate(q_cal_offset)                  │
│    → Subtract initial calibration offset                       │
│                                                                 │
│  yaw, pitch, roll = YXZ_Euler(q_rel)                           │
│    → YXZ order matches Unity/PICO coordinate system            │
│                                                                 │
│  DZ = 0.5°  ←──── DEAD ZONE                                    │
│    Physical: absorbs natural head micro-tremor (< 0.5 deg)      │
│    Location: after Euler extraction, before normalization       │
│    Tuning: increase if gimbal twitches when head is still       │
│                                                                 │
│  INV_YAW / INV_PITCH  ←──── DIRECTION INVERT                   │
│    Physical: flips sign when servo mounting orientation differs │
│    Location: after dead zone, before normalization              │
│    Tuning: use --no-inv-yaw or --no-inv-pitch                   │
│                                                                 │
│  Normalization: hv = yaw/90, vv = pitch/60                     │
│    YAW_RANGE=90° → full range mapped to [-1, +1]              │
│    PITCH_RANGE=60° → full range mapped to [-1, +1]             │
│                                                                 │
│  Output: (h,v) ∈ [-1,+1] × [-1,+1]                             │
└──────────────────────────────┬──────────────────────────────────┘
                               │  normalized target
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 3: PD Controller (PDController class)                     │
│                                                                 │
│  ┌─ JUMP_THRESH = 0.08 (7.2°) ─────────────────────────┐       │
│  │ Physical: error threshold for instant position jump   │       │
│  │ Location: first check in PD.update()                  │       │
│  │ If |error| > 0.08: skip PD, jump to target-0.04      │       │
│  │   → 0 delay for large head turns                     │       │
│  │   → Jump leaves 3.6° for PD fine-tuning              │       │
│  │ Tuning: lower = more responsive, higher = smoother     │       │
│  └──────────────────────────────────────────────────────┘       │
│                                                                 │
│  If |error| ≤ 0.08 → PD loop:                                   │
│                                                                 │
│  KP = 2.0  ←──── PROPORTIONAL GAIN                             │
│    Physical: tracking force multiplier                          │
│    Location: PD output term KP × error                          │
│    Effect: KP=2.0 → 1° error produces 2.0° correction           │
│    Tuning: higher = tighter tracking, may oscillate             │
│                                                                 │
│  KD = 0.5  ←──── DERIVATIVE GAIN                               │
│    Physical: braking/damping force                              │
│    Location: PD output term KD × (error - prev_error)           │
│    Effect: error shrinking → negative output → brakes motion    │
│    Tuning: higher = smoother stops, lower = snappier            │
│                                                                 │
│  Feedforward = 0.3 × (target - prev_target)                     │
│    Physical: predicts moving target trajectory                  │
│    Effect: reduces lag when head is continuously turning        │
│                                                                 │
│  PD_CLIP = 0.12  ←──── PER-FRAME POSITION LIMIT                │
│    Physical: max commanded position change per 20ms frame       │
│    0.12 × 90° = 10.8°/frame = 540°/s (at 50Hz)                │
│    Location: after PD output summation, before position update  │
│    Effect: prevents PD from commanding impossible servo speeds  │
│    Tuning: too high → overshoot oscillation                     │
│                                                                 │
│  Output: smoothed position (h,v) ∈ [-1,+1]                      │
└──────────────────────────────┬──────────────────────────────────┘
                               │  smoothed position + speed
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 4: Servo Command (Gimbal class)                           │
│                                                                 │
│  H_FACTOR = 1024  ←──── HORIZONTAL SCALE                       │
│    Physical: normalized-to-step mapping                         │
│    1.0 × 1024 = 1024 steps = 90°                               │
│    Formula: (YAW_RANGE/360) × 4096                              │
│    Location: gimbal.move() position calculation                 │
│                                                                 │
│  V_FACTOR = 682  ←──── VERTICAL SCALE                          │
│    Physical: normalized-to-step mapping                         │
│    1.0 × 682 = 682 steps = 60°                                 │
│    Formula: (PITCH_RANGE/360) × 4096                            │
│                                                                 │
│  SPEED = 2500-4000  ←──── SERVO SPEED                          │
│    Physical: WritePosEx speed parameter (steps/sec unit)        │
│    2500 ≈ 600°/s, 4000 ≈ 900°/s                                │
│    Formula: 2500 + 1500 × min(|error|, 1.0)                    │
│    Location: PD.update() speed output                           │
│    Note: values > ~1000 are all above physical max (948°/s)     │
│                                                                 │
│  ACC = 80  ←──── SERVO ACCELERATION                            │
│    Physical: WritePosEx acc parameter (100 steps/s² per unit)   │
│    80 × 100 = 8000 steps/s²                                    │
│    Location: WritePosEx 7-byte packet, address 0x29             │
│    Effect: controls start/stop smoothness                       │
│    ACC=0  → instant (may jitter), 948°/s possible              │
│    ACC=80 → balanced, ~700°/s                                  │
│    ACC=254 → very smooth, speed capped at 359°/s               │
│    Tuning: higher = smoother but slower                         │
│                                                                 │
│  Output: WritePosEx(ID, position, speed, ACC)                   │
│    → serial packet over URT-1 at 1Mbps                         │
└──────────────────────────────┬──────────────────────────────────┘
                               │  serial (TTL)
                               ▼
┌─────────────────────────────────────────────────────────────────┐
│ LAYER 5: STS3032 Internal Controller (hardware, EEPROM)         │
│                                                                 │
│  P = 32 (0x15)  ←──── INTERNAL PROPORTIONAL                    │
│    Physical: servo motor drive strength                         │
│    Effect: higher = stronger hold, stiffer motion               │
│    We leave at default                                          │
│                                                                 │
│  D = 10 (0x16)  ←──── INTERNAL DERIVATIVE (TUNED!)             │
│    Physical: servo braking force                               │
│    Default=32 → aggressive braking → oscillation               │
│    Tuned to 10 → soft braking, no jitter                       │
│    Set by: python run.py --tune-servo (writes EEPROM)           │
│                                                                 │
│  I = 0 (0x17)  ←──── INTERNAL INTEGRAL                         │
│    Physical: static error correction                            │
│    Manual warns: non-zero can cause jitter                      │
│    We leave at 0                                                │
│                                                                 │
│  DeadZone = 8 (0x1A, 0x1B)  ←──── DEAD BAND (TUNED!)          │
│    Physical: position error < 8 steps (0.7°) → motor off       │
│    Default=0 → motor fights every micro-step → jitter          │
│    Tuned to 8 → ignores sub-degree errors                      │
│    Set by: python run.py --tune-servo (writes EEPROM)           │
│                                                                 │
│  Torque Enable (0x28)                                           │
│    Physical: motor power switch                                 │
│    0=off, 1=on                                                  │
│    Auto-enabled at startup and after --tune-servo               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Complete Program Logic

### 2.1 Startup Sequence (python run.py)

```
1. Load gimbal_config.json (if exists → apply saved limits)
2. Parse CLI arguments (override defaults)
3. Initialize Gimbal:
   a. Open COM port (Windows: COM6, Linux: /dev/ttyUSB0)
   b. Ping servo ID=1, ID=2
   c. Enable torque (write 1 to 0x28)
   d. Center servos at 2048
4. Initialize Camera (D415, auto-degrade resolution)
5. Initialize Tracker (quaternion math ready)
6. Initialize PD Controller (KP=2.0, KD=0.5)
7. Connect to PicoBridge (wait for PICO frame)
8. Enter main tracking loop
```

### 2.2 Calibration (automatic, first frame)

```
1. Read first valid PICO frame
2. If body tracking enabled and Spine3 joint available:
     q_cal = conjugate(spine) × head
   Else:
     q_cal = head (identity offset)
3. Store q_cal as "forward reference"
4. Reset PD controller to position (0,0)
5. Center gimbal at 2048
6. Set cal=True → tracking begins
```

### 2.3 Tracking Loop (every frame, ~50Hz)

```
1. Check for external commands (tune_cmd.txt)
2. Wait for PICO frame (timeout 0.1s)

3. If frame received:
   a. Get head quaternion: frame.head.rotation
   b. Get spine quaternion: frame.body.joints[3, 3:7] (if enabled)

   c. Tracker.update():
      q_body_rel = conjugate(spine) × head              [TWIST2]
      q_rel = q_body_rel × conjugate(q_cal)              [subtract offset]
      yaw, pitch, _ = YXZ_Euler(q_rel)                   [angle extraction]
      If INV: negate                                     [mounting direction]
      If |angle| < DZ: zero                              [dead zone filter]
      h_norm = clip(yaw/90 + H_OFF, -1, +1)             [normalize horizontal]
      v_norm = clip(pitch/60 + V_OFF, -1, +1)            [normalize vertical]

   d. PD Controller.update(h_norm, v_norm):
      err_h = h_norm - current_h
      err_v = v_norm - current_v

      If |err_h| > 0.08 (7.2°):
        current_h = h_norm - 0.04  [JUMP: instant, leave 3.6° for PD]
      Else:
        output = 2.0×err_h + 0.5×(err_h - prev_err_h) + 0.3×(h_norm - prev_h)
        output = clip(output, -0.12, +0.12)
        current_h += output

      [Same for vertical]

      em = max(|err_h|, |err_v|)
      speed = 2500 + 1500 × min(em, 1.0)
      speed = clip(speed, 2500, 4000)

   e. Gimbal.move(current_h, current_v, speed):
      hp = clamp(2048 + current_h × 1024, 0, 4095)     [steps, horizontal]
      vp = clamp(2048 + current_v × 682, 0, 4095)      [steps, vertical]
      WritePosEx(1, hp, speed, ACC=80)
      WritePosEx(2, vp, speed, ACC=80)

   f. Push camera frame to PICO (if camera enabled)
   g. Log to CSV (if --log specified)
   h. Print status (every 20 frames)
```

### 2.4 Calibration Mode (python run.py --calibrate)

```
For each servo (Horizontal ID=1, Vertical ID=2):
  1. Move to 2048
  2. Scan positive direction in 40-step increments
     → Detect stall (position change < 5 steps for 4 readings)
     → Record max position
  3. Return to 2048
  4. Scan negative direction
     → Detect stall
     → Record min position
  5. Return to 2048 (FINAL: servo holds here)
  6. Calculate:
     half_range = (max - min) / 2
     offset = midpoint - 2048
     limit = min(half_range / factor, 1.0)
  7. Save to gimbal_config.json: {limit_h, limit_v, port, h_id, v_id}
  8. Prompt: "Align bracket forward, tighten screws, press Enter"
```

### 2.5 Servo Tuning Mode (python run.py --tune-servo)

```
For each servo:
  1. Read current values (P, D, dead zones)
  2. Unlock EEPROM (write 0 to 0x37)
  3. Write D=10 (0x16)
  4. Write CW dead zone=8 (0x1A)
  5. Write CCW dead zone=8 (0x1B)
  6. Lock EEPROM (write 1 to 0x37)
  7. Verify written values
  8. Enable torque (write 1 to 0x28)
```

---

## 3. Verification Checklist

### Hardware Setup

- [ ] STS3032 servos connected to URT-1 (ID1=horizontal, ID2=vertical)
- [ ] URT-1 USB connected to PC, 12V power connected
- [ ] D415 camera USB3.0 connected, mounted on gimbal
- [ ] PICO 4 on same WiFi as PC
- [ ] PicoBridge APK installed and opened on PICO

### Software Setup

- [ ] `conda activate active_vision`
- [ ] `pip install -r requirements.txt`
- [ ] `pip install pico_bridge-0.2.1-py3-none-any.whl`
- [ ] `cd windows` (or `cd linux`)

### Step 1: Servo Calibration

- [ ] `python run.py --calibrate`
- [ ] Both servos show "OK"
- [ ] Servos scan limits without error
- [ ] Servos lock at 2048
- [ ] Manually align camera forward, tighten screws
- [ ] gimbal_config.json created

### Step 2: Servo Tuning (one-time)

- [ ] `python run.py --tune-servo`
- [ ] Both servos show D=10, DZ=8 confirmed
- [ ] No errors

### Step 3: PICO Connection

- [ ] `python run.py --test-head`
- [ ] Terminal shows yaw/pitch/roll values
- [ ] Values change when head moves
- [ ] FPS ≥ 30

### Step 4: Camera Test

- [ ] `python run.py --test-camera`
- [ ] D415 detected with resolution
- [ ] Frames pushed to PICO

### Step 5: Direction Verification

- [ ] `python run.py`
- [ ] Put on headset, look forward
- [ ] Wait for `[CALIBRATED] Tracking!`
- [ ] Look up → camera points up
- [ ] Look down → camera points down
- [ ] Turn head right → camera turns right
- [ ] If any reversed: add `--no-inv-yaw` / `--no-inv-pitch`

### Step 6: Quality Check

- [ ] Quick head turn → camera follows instantly (no visible delay)
- [ ] Sudden stop → camera stops cleanly (no overshoot bounce)
- [ ] Slow pan → camera moves smoothly (no staircase stepping)
- [ ] Head still → camera absolutely still (no jitter)
- [ ] Body turn, head still → camera does NOT move (body-relative works)

### Step 7: Data Collection (optional)

- [ ] `python run.py --log test.csv`
- [ ] test.csv created with 14 columns
- [ ] Data looks reasonable

---

## 4. Testing Strategy

### 4.1 Speed Calibration

Run `python autotune.py` to objectively measure servo response across parameter combinations. Tests 100 combos of (KP, KD, ACC) with a 45° step input, measuring:

- **Settle time** (ms): time to reach and stay within 1° of target
- **Overshoot** (deg): maximum excursion past target
- **Score** = settle_time + overshoot × 100 (lower = better)

### 4.2 Latency Estimation

```
Total system latency ≈ PICO→PC network + processing + servo motion

PICO→PC (WiFi):          ~5-15ms
Frame processing:         ~2ms
PD controller:            < 1ms
Serial command:           ~1ms (10 bytes @ 1Mbps)
Servo mechanical (90°):   ~95ms (ACC=0, 948°/s)
                           ~130ms (ACC=80, ~700°/s)

Total (90° turn):         ~100-150ms
```

### 4.3 Jitter Diagnosis

| Symptom | Root Cause | Fix |
|---------|-----------|-----|
| High-frequency buzz when still | Servo D too high, DZ too small | `--tune-servo` |
| Oscillation after stop | KP too high, ACC too low | `--kd 0.8 --acc 100` |
| Random twitching | Loose wiring, EMI | Check connections |
| Slow drift | Calibration offset | `echo c > tune_cmd.txt` |

### 4.4 Speed vs Stability Tradeoff

```
More Speed ←──────────────────────────→ More Stability

  ACC=0        ACC=50       ACC=80       ACC=150      ACC=254
  KP=3.0       KP=2.5       KP=2.0       KP=1.5       KP=1.0
  KD=0.2       KD=0.4       KD=0.5       KD=0.7       KD=1.0
  948°/s       ~750°/s      ~700°/s      ~500°/s      359°/s
  may jitter   balanced     optimal      smooth       very smooth
```

---

## 5. Key Design Decisions

### 5.1 TWIST2 Body-Relative Tracking

From TWIST2 paper: use Spine3 (chest) joint as body reference frame. When the user rotates their body but keeps head still relative to body, the gimbal does NOT move. Only head rotation relative to the body triggers gimbal motion.

```
Formula: q_rel = conjugate(spine) × head

Case 1: Body turns 45°, head stays still (relative to body)
  q_spine = rot45, q_head = rot45
  q_rel = conj(rot45) × rot45 = identity → yaw=0 → no move ✓

Case 2: Body still, head turns 30°
  q_spine = identity, q_head = rot30
  q_rel = identity × rot30 = rot30 → yaw=30 → move 30° ✓
```

### 5.2 YXZ Euler Order

Unity/PICO uses YXZ rotation order. The euler extraction formula matches this convention. Using wrong order (XYZ, ZYX) would produce incorrect pitch when yaw is non-zero.

### 5.3 Jump + PD Hybrid Architecture

Large head movements (> 7.2°) bypass the PD controller entirely and jump directly to near-target. This provides 0-latency response for fast head turns. Small movements use PD for smooth tracking.

This is necessary because:
- STS3032 can move 90° in ~100ms
- PICO updates at 50Hz (20ms per frame)
- Without jump: PD would take 3-4 frames (60-80ms) to reach target
- With jump: immediate position update, PD fine-tunes in 1-2 frames

### 5.4 Direct Serial (No UDP Relay)

Previous versions used a UDP relay server between the tracker and servo controller. This added network latency and complexity. The current version connects directly to the servo via serial (URT-1), eliminating an entire network hop.

### 5.5 1:1 Angle Mapping

The servo factor is derived from physical resolution:
```
4096 steps = 360°
1 step = 0.088°
YAW_RANGE = 90° → 1024 steps → H_FACTOR = 1024
PITCH_RANGE = 60° → 682 steps → V_FACTOR = 682
```

This gives exact 1:1 mapping: head turns X°, servo turns X°.

---

## 6. Port Configuration

| OS | Typical Port | Command Override |
|----|-------------|------------------|
| Windows | COM6 | `--port COM5` |
| Linux | /dev/ttyUSB0 | `--port /dev/ttyUSB1` |
| Linux (alternative) | /dev/ttyACM0 | `--port /dev/ttyACM0` |

Find port on Linux: `ls /dev/ttyUSB*` or `ls /dev/ttyACM*`
Find port on Windows: Device Manager → Ports (COM & LPT)

---

## 7. Runtime Files

| File | Created By | Purpose |
|------|-----------|---------|
| `gimbal_config.json` | `--calibrate` | Saved servo limits and port |
| `tune_cmd.txt` | User (terminal 2) | Real-time parameter changes |
| `*.csv` | `--log FILENAME` | Tracking data for analysis |
| `autotune_results.json` | `autotune.py` | Auto-tuning results |

---

## 8. Common Issues

| Problem | Diagnosis | Solution |
|---------|-----------|----------|
| "Servo X no response" | Power or wiring | Check 12V supply, URT-1 connections |
| "Cannot open COM port" | Wrong port or busy | Check Device Manager, close other apps |
| PICO never connects | WiFi or APK | Same network, PicoBridge open, check IP |
| Camera not found | USB or driver | Check USB3.0 connection, reinstall RealSense SDK |
| Gimbal drifts over time | Mechanical slip | Tighten bracket screws, recalibrate |
| Noise/grinding sound | Mechanical binding | Check for cable interference, loosen bracket |
| Python import errors | Missing deps | `pip install -r requirements.txt` |
