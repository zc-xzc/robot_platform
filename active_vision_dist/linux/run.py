#!/usr/bin/env python3
"""Active Vision System v9.0 — Unified single-file deployment
PICO 4 -> body-relative tracking (TWIST2) -> PD -> STS3032 gimbal -> D415

Modes:
  python run.py                      # tracking (default, port auto-detected)
  python run.py --calibrate          # servo calibration (scan limits, lock 2048)
  python run.py --test-head          # PICO head tracking test
  python run.py --test-camera        # camera streaming test
  python run.py --log data.csv       # tracking + CSV logging

Tracking args:
  --kp 2.0 --kd 0.5 --jump-thresh 0.08 --acc 80
  --port COM6 --baud 1000000   (Windows)
  --no-camera --no-body --log PATH

Calibration args:
  --calibrate --port /dev/ttyUSB0 --h-id 1 --v-id 2  (Linux)
  --calibrate --port COM6 --h-id 1 --v-id 2    (Windows)

Keyboard while tracking:
  c = recalibrate   +/- = KP   []=KD   q = quit
"""

import sys, os, time, threading, argparse, csv, json
import numpy as np

# ---- SDK ----
SDK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
if SDK not in sys.path:
    sys.path.append(SDK)
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

# ============================================================================
# Default config
# ============================================================================
H_ID, V_ID = 1, 2
PORT, BAUD, ACC = "/dev/ttyUSB0", 1000000, 80
YAW_RANGE, PITCH_RANGE = 90.0, 60.0
INV_YAW, INV_PITCH, SWAP = True, True, False
LIMIT_H, LIMIT_V = 0.85, 0.85
H_OFF, V_OFF = 0.0, 0.0
KP, KD = 2.0, 0.5
JUMP_THRESH = 0.08
DZ = 0.5
LOG_FILE = None

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gimbal_config.json")

# ============================================================================
# Quaternion (TWIST2)
# ============================================================================

def qmul(a, b):
    w1, x1, y1, z1 = a[3], a[0], a[1], a[2]
    w2, x2, y2, z2 = b[3], b[0], b[1], b[2]
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])

def qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])

def qeuler(q):
    x, y, z, w = q
    yaw = np.degrees(np.arctan2(2*(x*z + w*y), 1 - 2*(y*y + z*z)))
    pitch = np.degrees(np.arcsin(np.clip(-2*(y*z - w*x), -1, 1)))
    roll = np.degrees(np.arctan2(2*(x*y + w*z), 1 - 2*(x*x + z*z)))
    return yaw, pitch, roll

# ============================================================================
# PD Controller
# ============================================================================

class PDController:
    """Hybrid: large-error jump + small-error PD."""

    def __init__(self, kp=1.5, kd=0.3, jump_thresh=0.08):
        self.kp = kp
        self.kd = kd
        self.jump_thresh = jump_thresh
        self._ch = 0.0; self._cv = 0.0
        self._peh = 0.0; self._pev = 0.0
        self._pt_h = 0.0; self._pt_v = 0.0

    def reset(self):
        self._ch = 0.0; self._cv = 0.0
        self._peh = 0.0; self._pev = 0.0
        self._pt_h = 0.0; self._pt_v = 0.0

    def update(self, target_h, target_v):
        eh = target_h - self._ch
        ev = target_v - self._cv

        if abs(eh) > self.jump_thresh:
            self._ch = target_h - np.sign(eh) * self.jump_thresh * 0.5
            self._peh = target_h - self._ch  # reset D-term after jump
        else:
            oh = self.kp * eh + self.kd * (eh - self._peh)
            oh += (target_h - self._pt_h) * 0.3
            self._ch += np.clip(oh, -0.12, 0.12)
            self._peh = eh  # normal: store this frame error for next D-term

        if abs(ev) > self.jump_thresh:
            self._cv = target_v - np.sign(ev) * self.jump_thresh * 0.5
            self._pev = target_v - self._cv  # reset D-term after jump
        else:
            ov = self.kp * ev + self.kd * (ev - self._pev)
            ov += (target_v - self._pt_v) * 0.3
            self._cv += np.clip(ov, -0.12, 0.12)
            self._pev = ev  # normal: store this frame error for next D-term

        self._pt_h, self._pt_v = target_h, target_v

        self._ch = np.clip(self._ch, -LIMIT_H, LIMIT_H)
        self._cv = np.clip(self._cv, -LIMIT_V, LIMIT_V)

        # D=10 + DZ=8 + ACC=50: stable foundation, push speed
        em = max(abs(eh), abs(ev))
        spd = int(3000 + 1000 * min(em, 1.0))
        spd = min(4095, max(2500, spd))
        return self._ch, self._cv, spd

# ============================================================================
# Gimbal
# ============================================================================

# Servo step factor: how many servo steps per normalized unit.
# Derived from: (RANGE/360) * 4096.  This gives 1:1 angle mapping.
H_FACTOR = int(YAW_RANGE / 360.0 * 4096)     # ~1024
V_FACTOR = int(PITCH_RANGE / 360.0 * 4096)    # ~683

class Gimbal:
    def __init__(self, port, baud, acc=0):
        self.acc = acc
        self.h_id, self.v_id = (V_ID, H_ID) if SWAP else (H_ID, V_ID)
        self.port = PortHandler(port)
        self.port.baudrate = baud
        if not self.port.openPort():
            raise RuntimeError("Cannot open " + port)
        self.pkt = sms_sts(self.port)
        time.sleep(0.5)
        for sid in [self.h_id, self.v_id]:
            _, r, _ = self.pkt.ping(sid)
            if r != COMM_SUCCESS:
                raise RuntimeError("Servo %d no response" % sid)
            # Ensure torque is enabled (may be disabled after EEPROM writes)
            self.pkt.write1ByteTxRx(sid, 0x28, 1)
            print("  servo %d OK" % sid)
        self.pkt.WritePosEx(self.h_id, 2048, 500, self.acc)
        self.pkt.WritePosEx(self.v_id, 2048, 500, self.acc)
        print("  gimbal ready")

    def move(self, h, v, speed=4095):
        hp = max(0, min(4095, int(2048 + h * H_FACTOR)))
        vp = max(0, min(4095, int(2048 + v * V_FACTOR)))
        self.pkt.WritePosEx(self.h_id, hp, max(1, min(4095, speed)), self.acc)
        self.pkt.WritePosEx(self.v_id, vp, max(1, min(4095, speed)), self.acc)

    def center(self):
        self.pkt.WritePosEx(self.h_id, 2048, 500, self.acc)
        self.pkt.WritePosEx(self.v_id, 2048, 500, self.acc)

    def close(self):
        self.center()
        time.sleep(0.3)
        self.port.closePort()

# ============================================================================
# Tracker
# ============================================================================

class Tracker:
    def __init__(self):
        self.q_off = np.array([0., 0., 0., 1.])
        self.cal = False

    def calibrate(self, q_head, q_spine=None):
        self.q_off = qmul(qconj(q_spine), q_head) if q_spine is not None \
                     else q_head.copy()
        self.cal = True

    def update(self, q_head, q_spine=None):
        q_body_rel = qmul(qconj(q_spine), q_head) if q_spine is not None else q_head
        q_rel = qmul(q_body_rel, qconj(self.q_off)) if self.cal else q_body_rel
        yaw, pitch, _ = qeuler(q_rel)
        hv, vv = yaw, pitch
        if INV_YAW: hv = -hv
        if INV_PITCH: vv = -vv
        if abs(hv) < DZ: hv = 0.
        if abs(vv) < DZ: vv = 0.
        return (np.clip(hv / YAW_RANGE + H_OFF, -1., 1.),
                np.clip(vv / PITCH_RANGE + V_OFF, -1., 1.))

    def debug_angles(self, q_head):
        if not self.cal: return 0., 0., 0.
        return qeuler(qmul(q_head, qconj(self.q_off)))

# ============================================================================
# Camera
# ============================================================================

class Camera:
    def __init__(self, enabled=True):
        self.pipe = None
        if not enabled: return
        try:
            import pyrealsense2 as rs
            for w, h, fps in [(1280,720,30),(640,480,30),(424,240,30)]:
                try:
                    p = rs.pipeline(); c = rs.config()
                    c.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
                    p.start(c); self.pipe = p
                    print("  D415 %dx%d @ %dfps" % (w, h, fps)); break
                except: continue
        except: pass
        if self.pipe is None: print("  no D415")

    def read(self):
        if not self.pipe: return None
        try:
            f = self.pipe.wait_for_frames(timeout_ms=2000)
            c = f.get_color_frame()
            return np.asanyarray(c.get_data()) if c else None
        except: return None

    def close(self):
        if self.pipe:
            try: self.pipe.stop()
            except: pass

# ============================================================================
# CSV Logger
# ============================================================================

class CSVLogger:
    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(["timestamp","seq","raw_yaw","raw_pitch","raw_roll",
                          "target_h","target_v","pd_h","pd_v",
                          "servo_h","servo_v","speed","kp","kd"])
        self._lk = threading.Lock()
        print("  log -> " + path)

    def write(self, row):
        with self._lk: self._w.writerow(row); self._f.flush()

    def close(self):
        with self._lk: self._f.close()

# ============================================================================
# Threaded keyboard input (works in all terminals)
# ============================================================================

_key_queue = []

def _kb_thread():
    '''Background thread: read stdin line by line, push to queue.'''
    global _key_queue
    while True:
        try:
            line = sys.stdin.readline()
            if not line:
                break
            _key_queue.append(line.strip().lower())
        except:
            break

import threading
_kb_t = threading.Thread(target=_kb_thread, daemon=True)
_kb_t.start()

def kbhit():
    return len(_key_queue) > 0

def getch():
    return _key_queue.pop(0) if _key_queue else ''

# ============================================================================
# MODE: calibrate
# ============================================================================

def find_limit(pkt, sid, start, direction, step=40):
    last = start; stall = 0
    for _ in range(120):
        tgt = last + direction * step
        if tgt < 0 or tgt > 4095: break
        pkt.WritePosEx(sid, tgt, 400, 200)
        time.sleep(0.12)
        actual, _, comm, _ = pkt.ReadPosSpeed(sid)
        if comm != COMM_SUCCESS or actual is None: break
        if abs(actual - last) < 5:
            stall += 1
            if stall >= 4: print("   stalled at %d" % last); return last
        else: stall = 0
        last = actual
    return last

def calibrate_servo(pkt, sid, name):
    print("\n=== %s (ID=%d) ===" % (name, sid))
    pkt.WritePosEx(sid, 2048, 500, 200); time.sleep(1.0)
    print("   at 2048")
    print("   scanning + ..."); p_max = find_limit(pkt, sid, 2048, +1)
    pkt.WritePosEx(sid, 2048, 500, 200); time.sleep(1.0)
    print("   scanning - ..."); p_min = find_limit(pkt, sid, 2048, -1)
    pkt.WritePosEx(sid, 2048, 500, 200); time.sleep(1.0)

    mid = (p_max + p_min) // 2; half = (p_max - p_min) // 2
    deg = 360.0 / 4096; offset = mid - 2048
    print("\n  Limits: %d ~ %d  |  Mid: %d (off=%+d, %.1f deg)  |  Half: %d (%.0f deg)" % (
        p_min, p_max, mid, offset, offset*deg, half, half*deg))
    return half, offset

def mode_calibrate(args):
    port = PortHandler(args.port); port.baudrate = args.baud
    if not port.openPort(): print("ERROR: cannot open " + args.port); return
    pkt = sms_sts(port); time.sleep(0.5)
    for sid in [args.h_id, args.v_id]:
        _, r, _ = pkt.ping(sid)
        if r != COMM_SUCCESS: print("Servo %d no response" % sid); port.closePort(); return
    print("Servos OK")

    h_half, h_off = calibrate_servo(pkt, args.h_id, "Horizontal")
    v_half, v_off = calibrate_servo(pkt, args.v_id, "Vertical")

    h_lim = round(min(h_half / H_FACTOR, 1.0), 3)
    v_lim = round(min(v_half / V_FACTOR, 1.0), 3)

    cfg = {"limit_h": h_lim, "limit_v": v_lim, "port": args.port,
           "h_id": args.h_id, "v_id": args.v_id}
    with open(CONFIG_FILE, "w") as f: json.dump(cfg, f, indent=2)

    print("\n" + "="*55)
    print("Servos locked at 2048. Saved -> %s" % CONFIG_FILE)
    print("  limit_h=%.3f  limit_v=%.3f" % (h_lim, v_lim))
    print("  NOW: align bracket so camera points forward, tighten screws.")
    print("="*55)
    input("Press Enter..."); port.closePort(); print("Done.")

# ============================================================================

# ============================================================================
# MODE: tune-servo (fix internal PID for jitter-free tracking)
# ============================================================================

def mode_tune_servo(args):
    """Reduce servo internal D coefficient and add dead zone to eliminate jitter.
    
    Servo default: P=32, D=32, dead_zone=0
    Head tracking needs: softer D, small dead zone
    
    Changes are written to EEPROM (permanent until next tune).
    """
    print("="*55)
    print("Servo PID Tuning ? Fix Jitter")
    print("="*55)
    print("Current defaults: P=32  D=32  I=0  DeadZone=0")
    print("Target:           P=32  D=10  I=0  DeadZone=8 (0.70 deg)")
    print()
    print("D=10: soft braking, no oscillation")
    print("DeadZone=8: ignore errors < 0.70 deg, no micro-jitter")
    print()
    
    port = PortHandler(args.port)
    port.baudrate = args.baud
    if not port.openPort():
        print("ERROR: Cannot open " + args.port)
        return
    pkt = sms_sts(port)
    time.sleep(0.5)
    
    for sid, name in [(args.h_id, "Horizontal"), (args.v_id, "Vertical")]:
        _, r, _ = pkt.ping(sid)
        if r != COMM_SUCCESS:
            print("ERROR: Servo %d no response" % sid)
            port.closePort()
            return
    
    for sid, name in [(args.h_id, "Horizontal"), (args.v_id, "Vertical")]:
        print("\n--- %s (ID=%d) ---" % (name, sid))
        
        # Read current values
        p_val = pkt.read1ByteTxRx(sid, 0x15)[0]
        d_val = pkt.read1ByteTxRx(sid, 0x16)[0]
        cw_dz = pkt.read1ByteTxRx(sid, 0x1A)[0]
        ccw_dz = pkt.read1ByteTxRx(sid, 0x1B)[0]
        print("  Current: P=%d D=%d CW_DZ=%d CCW_DZ=%d" % (p_val, d_val, cw_dz, ccw_dz))
        
        # Unlock EEPROM
        pkt.write1ByteTxRx(sid, 0x37, 0)
        time.sleep(0.05)
        
        # Set D=16 (softer braking)
        pkt.write1ByteTxRx(sid, 0x16, 10)
        time.sleep(0.05)
        
        # Set dead zone = 5 steps (0.44 deg)
        pkt.write1ByteTxRx(sid, 0x1A, 8)
        time.sleep(0.05)
        pkt.write1ByteTxRx(sid, 0x1B, 8)
        time.sleep(0.05)
        
        # Lock EEPROM
        pkt.write1ByteTxRx(sid, 0x37, 1)
        time.sleep(0.1)
        
        # Verify
        d_new = pkt.read1ByteTxRx(sid, 0x16)[0]
        cw_new = pkt.read1ByteTxRx(sid, 0x1A)[0]
        ccw_new = pkt.read1ByteTxRx(sid, 0x1B)[0]
        print("  New:     P=%d D=%d CW_DZ=%d CCW_DZ=%d" % (p_val, d_new, cw_new, ccw_new))
        print("  OK" if (d_new==10 and cw_new==8 and ccw_new==8) else "  WARNING: verify with FD software")
    
    # Re-enable torque on both servos
    for sid in [args.h_id, args.v_id]:
        pkt.write1ByteTxRx(sid, 0x28, 1)
    
    port.closePort()
    print("\n" + "="*55)
    print("DONE. Servo PID tuned. Torque enabled.")
    print("Restart: python run.py")
    print("To restore defaults: FD software or re-run with --tune-servo-defaults")
    print("="*55)


# ============================================================================
# MODE: test-head
# ============================================================================

def mode_test_head(args):
    from pico_bridge import PicoBridge
    with open("head_tracking_log.csv", "w") as log:
        log.write("seq,yaw,pitch,roll,pos_x,pos_y,pos_z\n")
        with PicoBridge() as pico:
            print("Waiting for PICO..."); cnt = 0
            while True:
                try:
                    f = pico.wait_frame(timeout=5.0)
                    if f is None: continue
                    cnt += 1
                    y, p, r = qeuler(f.head.rotation)
                    pos = f.head.position
                    if cnt % 10 == 0:
                        print("\n#%d  y=%+6.1f  p=%+6.1f  r=%+6.1f  pos=(%.2f,%.2f,%.2f)" % (
                            f.seq, y, p, r, pos[0], pos[1], pos[2]))
                        s = pico.stats()
                        print("  FPS: %.1f  connected: %s" % (s.fps, s.connected))
                    log.write("%d,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n" % (
                        f.seq, y, p, r, pos[0], pos[1], pos[2]))
                except KeyboardInterrupt:
                    print("\nDone. %d frames -> head_tracking_log.csv" % cnt); break

# ============================================================================
# MODE: test-camera
# ============================================================================

def mode_test_camera(args):
    cam = Camera(enabled=True)
    if cam.pipe is None: print("No camera found."); return
    from pico_bridge import PicoBridge
    pushed = [0]
    def push(pico):
        while True:
            f = cam.read()
            if f is not None:
                try: pico.push_video_frame(f); pushed[0] += 1
                except: pass
            time.sleep(0.04)
    with PicoBridge(video="frames") as pico:
        t = threading.Thread(target=push, args=(pico,), daemon=True); t.start()
        print("Streaming... Ctrl+C to stop")
        try:
            while True:
                time.sleep(5)
                print("  %d frames pushed" % pushed[0])
        except KeyboardInterrupt:
            print("\nDone. %d frames pushed." % pushed[0])
    cam.close()

# ============================================================================
# MODE: track (default)
# ============================================================================

def mode_track(args):
    global INV_YAW, INV_PITCH, LIMIT_H, LIMIT_V, H_OFF, V_OFF, DZ
    kp = max(0.1, args.kp); kd = max(0.0, args.kd)
    jt = max(0.03, args.jump_thresh)
    use_cam = not args.no_camera; use_body = not args.no_body
    LIMIT_H = args.limit_h; LIMIT_V = args.limit_v
    H_OFF, V_OFF = args.hoff, args.voff; DZ = args.dead_zone
    if args.no_inv_yaw: INV_YAW = False
    if args.no_inv_pitch: INV_PITCH = False

    print("="*55)
    print("Active Vision v9.0 — Tracking")
    print("="*55)
    print("  KP=%.1f KD=%.1f Jump=%.2f DZ=%.1fdeg" % (kp, kd, jt, DZ))
    print("  H_factor=%d V_factor=%d (1:1 angle)" % (H_FACTOR, V_FACTOR))
    print("  Limits H=+-%.2f V=+-%.2f" % (LIMIT_H, LIMIT_V))
    print("  Body:%s Cam:%s COM:%s@%d" % (
        "ON" if use_body else "OFF", "ON" if use_cam else "OFF", args.port, args.baud))

    global ACC
    ACC = args.acc
    gimbal = Gimbal(args.port, args.baud, acc=ACC)
    cam = Camera(enabled=use_cam); has_cam = cam.pipe is not None

    try:
        from pico_bridge import PicoBridge
    except ImportError:
        print("\n[ERROR] pico_bridge not installed."); gimbal.close(); return

    tracker = Tracker(); pd_ctrl = PDController(kp=kp, kd=kd, jump_thresh=jt)
    logger = CSVLogger(args.log) if args.log else None
    pico_kw = {"video": "frames"} if has_cam else {}

    try:
        with PicoBridge(**pico_kw) as pico:
            if has_cam:
                def push():
                    while True:
                        f = cam.read()
                        if f is not None:
                            try: pico.push_video_frame(f)
                            except: pass
                        time.sleep(0.04)
                threading.Thread(target=push, daemon=True).start()

            print("\n[READY] Headset -> PicoBridge -> look forward")
            print("[CMDS] kp 1.8 | kd 0.5 | acc 30 | kp+ | kd- | c | q\n")

            cal = False; frames = 0; t0 = time.time(); wp = False
            g_kp, g_kd, g_acc = kp, kd, ACC

            while True:
                while kbhit():
                    key = getch()
                    if key in ("q", "quit", "exit"): raise KeyboardInterrupt
                    elif key in ("c", "cal"): cal = False; print("[RECAL]")
                    elif key.startswith("kp "):
                        try:
                            g_kp = float(key.split()[1])
                            g_kp = max(0.1, min(10., g_kp))
                            pd_ctrl.kp = g_kp
                            print("[KP] %.1f" % g_kp)
                        except: print("[KP] invalid")
                    elif key.startswith("kd "):
                        try:
                            g_kd = float(key.split()[1])
                            g_kd = max(0., min(2., g_kd))
                            pd_ctrl.kd = g_kd
                            print("[KD] %.2f" % g_kd)
                        except: print("[KD] invalid")
                    elif key.startswith("acc "):
                        try:
                            g_acc = int(key.split()[1])
                            g_acc = max(0, min(254, g_acc))
                            gimbal.acc = g_acc
                            print("[ACC] %d" % g_acc)
                        except: print("[ACC] invalid")
                    else:
                        print("[?] kp 1.8 | kd 0.5 | acc 20 | c | q")
                try:
                    frame = pico.wait_frame(timeout=0.1)
                except TimeoutError:
                    if not wp: print("[waiting] PicoBridge..."); wp = True
                    continue
                wp = False

                if not cal:
                    q_spine_cal = None
                    if use_body and frame.body.active and frame.body.joints.shape[0] > 3:
                        q_spine_cal = frame.body.joints[3, 3:7]
                    tracker.calibrate(frame.head.rotation, q_spine_cal)
                    pd_ctrl.reset(); gimbal.center(); cal = True
                    print("[CALIBRATED] Tracking!\n"); continue

                frames += 1
                q_spine = None
                if use_body and frame.body.active and frame.body.joints.shape[0] > 3:
                    q_spine = frame.body.joints[3, 3:7]

                th, tv = tracker.update(frame.head.rotation, q_spine)
                ph, pv, spd = pd_ctrl.update(th, tv)
                # Debug: print raw values on first few frames
                if frames <= 5:
                    hp_raw = int(2048 + ph * H_FACTOR)
                    vp_raw = int(2048 + pv * V_FACTOR)
                    print("  [DEBUG f=%d] raw: yaw/pitch=%.2f/%.2f -> norm=%.3f/%.3f -> pos=%d/%d spd=%d acc=%d" % (
                        frames, th*90, tv*60, ph, pv, hp_raw, vp_raw, spd, gimbal.acc))
                gimbal.move(ph, pv, spd)

                if logger and frames % 2 == 0:
                    now = time.time()
                    y, pi, ro = tracker.debug_angles(frame.head.rotation)
                    logger.write(["%.3f"%now, frame.seq,
                        "%.2f"%y,"%.2f"%pi,"%.2f"%ro,
                        "%.4f"%th,"%.4f"%tv,"%.4f"%ph,"%.4f"%pv,
                        int(2048+ph*H_FACTOR), int(2048+pv*V_FACTOR), spd,
                        "%.1f"%g_kp,"%.2f"%g_kd])

                if frames % 20 == 0:
                    y, pi, ro = tracker.debug_angles(frame.head.rotation)
                    fps = frames / (time.time()-t0)
                    print("[%5d] y=%+6.1f p=%+6.1f r=%+6.1f  -> H=%+.3f V=%+.3f  spd=%d  kp=%.1f kd=%.2f acc=%d  %.0ffps" % (
                        frames, y, pi, ro, ph, pv, spd, g_kp, g_kd, g_acc, fps))

    except KeyboardInterrupt: print("\n[EXIT]")
    except Exception as e:
        print("\n[ERROR] "+str(e)); import traceback; traceback.print_exc()

    print("[SHUTDOWN]"); gimbal.close(); cam.close()
    if logger: logger.close(); print("[DONE]")

# ============================================================================
# Entry point
# ============================================================================

def main():
    # Load config from file (if exists), CLI args override
    saved_limits_h, saved_limits_v, saved_port = 0.85, 0.85, PORT
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE) as f:
                cfg = json.load(f)
            saved_limits_h = cfg.get("limit_h", 0.85)
            saved_limits_v = cfg.get("limit_v", 0.85)
            saved_port = cfg.get("port", PORT)
            print("[CONFIG] Loaded %s: H=%.3f V=%.3f port=%s" % (
                CONFIG_FILE, saved_limits_h, saved_limits_v, saved_port))
        except: pass

    p = argparse.ArgumentParser(description="Active Vision v9.0")
    p.add_argument("--calibrate", action="store_true", help="Servo calibration mode")
    p.add_argument("--tune-servo", action="store_true", help="Tune servo internal PID to fix jitter")
    p.add_argument("--test-head", action="store_true", help="PICO head tracking test")
    p.add_argument("--test-camera", action="store_true", help="Camera streaming test")
    p.add_argument("--port", default=saved_port, help="COM port")
    p.add_argument("--baud", type=int, default=BAUD)
    p.add_argument("--acc", type=int, default=ACC, help="Servo acceleration (0=instant, 1-254)")
    p.add_argument("--kp", type=float, default=KP)
    p.add_argument("--kd", type=float, default=KD)
    p.add_argument("--jump-thresh", type=float, default=JUMP_THRESH)
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--no-body", action="store_true")
    p.add_argument("--log", default=LOG_FILE)
    p.add_argument("--limit-h", type=float, default=saved_limits_h)
    p.add_argument("--limit-v", type=float, default=saved_limits_v)
    p.add_argument("--hoff", type=float, default=H_OFF)
    p.add_argument("--voff", type=float, default=V_OFF)
    p.add_argument("--dead-zone", type=float, default=DZ)
    p.add_argument("--no-inv-yaw", action="store_true")
    p.add_argument("--no-inv-pitch", action="store_true")
    p.add_argument("--h-id", type=int, default=H_ID)
    p.add_argument("--v-id", type=int, default=V_ID)
    args = p.parse_args()



    if args.tune_servo:
        mode_tune_servo(args)
    elif args.calibrate:
        mode_calibrate(args)
    elif args.test_head:
        mode_test_head(args)
    elif args.test_camera:
        mode_test_camera(args)
    else:
        mode_track(args)


if __name__ == "__main__":
    main()