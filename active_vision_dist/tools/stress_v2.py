#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""STS3032 + D415 Interactive Stress Test v2
- Live camera feed in dashboard
- Interactive parameter controls (speed/complexity/duration)
- Video recording
- 90+ random patterns at natural head-tracking speeds
"""
import sys, os, time, threading, csv, math, random
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

def find_port():
    from serial.tools import list_ports
    for p in list_ports.comports():
        d = p.description.lower()
        if any(k in d for k in ["ch343","usb-serial","usb enhanced","ttyusb","ttyacm","urt"]):
            return p.device
    for p in list_ports.comports():
        if p.device.startswith("COM") and "bluetooth" not in p.description.lower():
            return p.device
    return None

PORT = find_port()
if PORT is None:
    print("[ERROR] No servo port!"); sys.exit(1)
BAUD = 1000000
print(f"[PORT] {PORT}")

YAW_ID, PITCH_ID = 1, 2
YAW_CENTER, PITCH_CENTER = 2110, 2048
YAW_MIN, YAW_MAX = 1217, 3233
PITCH_MIN, PITCH_MAX = 1162, 2601
TEST_DURATION = 3600
MAX_TEMP_C, HIGH_TEMP_C = 75, 65
MIN_VOLTAGE = 7.0
LOG_DIR = Path(__file__).resolve().parent / "stress_test_logs"

def _clip(v, lo, hi): return max(lo, min(hi, v))

# ---- GIMBAL ----
class Gimbal:
    def __init__(s):
        s.port = PortHandler(PORT); s.port.baudrate = BAUD
        if not s.port.openPort(): raise RuntimeError(f"Cannot open {PORT}")
        s.pkt = sms_sts(s.port); time.sleep(0.5)
        for sid in [YAW_ID, PITCH_ID]:
            _, r, _ = s.pkt.ping(sid)
            if r != COMM_SUCCESS: raise RuntimeError(f"Servo {sid} no response")
            s.pkt.write1ByteTxRx(sid, 0x28, 1)
        s.center(); time.sleep(0.8); print("[GIMBAL] Ready")
    def move(s, yp, pp, sp=2500, ac=100):
        s.pkt.WritePosEx(YAW_ID, int(_clip(yp, YAW_MIN, YAW_MAX)), sp, ac)
        s.pkt.WritePosEx(PITCH_ID, int(_clip(pp, PITCH_MIN, PITCH_MAX)), sp, ac)
    def center(s): s.move(YAW_CENTER, PITCH_CENTER, 500, 50)
    def close(s):
        try: s.center(); time.sleep(0.3)
        except: pass
        try: s.port.closePort()
        except: pass

# ---- CAMERA + RECORDER ----
class CameraMon:
    def __init__(s):
        s.enabled = False; s.pipeline = None; s.frames_total = 0
        s.frames_recent = 0; s.last_fps = 0.0; s.errors = 0
        s.resolution = ""; s.last_check = time.time()
        s.recording = False; s.recorder = None; s.latest_frame = None
        try:
            import pyrealsense2 as rs
            for w, h, fps in [(640, 480, 30), (1280, 720, 30), (424, 240, 30)]:
                try:
                    p = rs.pipeline(); c = rs.config()
                    c.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
                    p.start(c); s.pipeline = p; s.enabled = True
                    s.resolution = f"{w}x{h}@{fps}fps"; s.cam_w, s.cam_h = w, h
                    print(f"[CAMERA] {s.resolution}"); break
                except: continue
        except ImportError: print("[CAMERA] pyrealsense2 not installed")
        except Exception as e: print(f"[CAMERA] Error: {e}")

    def update(s):
        if not s.enabled: return
        try:
            frames = s.pipeline.wait_for_frames(timeout_ms=50)
            if frames:
                s.frames_total += 1; s.frames_recent += 1
                color = frames.get_color_frame()
                if color:
                    import numpy as np
                    s.latest_frame = np.asanyarray(color.get_data())
                    if s.recording and s.recorder:
                        s.recorder.write(s.latest_frame[:, :, ::-1])  # RGB to BGR for cv2
        except: s.errors += 1
        now = time.time()
        if now - s.last_check > 1.0:
            s.last_fps = s.frames_recent / (now - s.last_check + 0.001)
            s.frames_recent = 0; s.last_check = now

    def start_recording(s):
        if s.recording: return
        try:
            import cv2
            LOG_DIR.mkdir(exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = str(LOG_DIR / f"video_{ts}.avi")
            fourcc = cv2.VideoWriter_fourcc(*"XVID")
            s.recorder = cv2.VideoWriter(path, fourcc, 20.0, (s.cam_w, s.cam_h))
            s.recording = True
            print(f"[RECORD] {path}")
        except ImportError: print("[RECORD] opencv-python not installed")
        except Exception as e: print(f"[RECORD] Error: {e}")

    def stop_recording(s):
        s.recording = False
        if s.recorder:
            s.recorder.release(); s.recorder = None
            print("[RECORD] Stopped")

    def close(s):
        s.stop_recording()
        if s.pipeline:
            try: s.pipeline.stop()
            except: pass

# ---- MONITOR ----
class Monitor:
    def __init__(s, g, cam):
        s.g = g; s.cam = cam; s.running = False
        s.lock = threading.Lock(); s.cur = {}; s.hist = []
        s.csv = None; s.wr = None; s.t0 = None
        s.warns = []; s.max_t = 0; s.min_v = 999

    def start(s):
        LOG_DIR.mkdir(exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        s.csv = open(LOG_DIR / f"stress_{ts}.csv", "w", newline="", encoding="utf-8")
        s.wr = csv.writer(s.csv)
        s.wr.writerow(["ts","elapsed","pat","name","yt","pt","yp","pp","ys","ps",
            "yl","pl","v1","v2","t1","t2","c1","c2","sp","ac","cfps","cerr","warns"])
        s.t0 = time.time(); s.running = True
        s.th = threading.Thread(target=s._run, daemon=True); s.th.start()

    def _rd(s, sid, addr, sz):
        try:
            v, c, _ = s.g.pkt.read1ByteTxRx(sid, addr) if sz == 1 else s.g.pkt.read2ByteTxRx(sid, addr)
            return v if c == COMM_SUCCESS else -9999
        except: return -9999

    def _run(s):
        while s.running:
            r = {}
            for sid, nm in [(YAW_ID, "y"), (PITCH_ID, "p")]:
                pos, spd, comm, _ = s.g.pkt.ReadPosSpeed(sid)
                ok = comm == COMM_SUCCESS
                r[f"{nm}p"] = int(pos) if ok else -1
                r[f"{nm}s"] = int(spd) if ok else -1
                r[f"{nm}l"] = s._rd(sid, 0x3C, 2)
                r[f"v{sid}"] = s._rd(sid, 0x3E, 1) / 10.0
                r[f"t{sid}"] = s._rd(sid, 0x3F, 1)
                r[f"c{sid}"] = s._rd(sid, 0x45, 2)
            s.cam.update()
            r["cfps"] = round(s.cam.last_fps, 1); r["cerr"] = s.cam.errors
            for sid in [1, 2]:
                tv = r.get(f"t{sid}", 0)
                if tv and tv > -9000 and tv > s.max_t: s.max_t = tv
                vv = r.get(f"v{sid}", 999)
                if vv and vv > -9000 and vv < s.min_v: s.min_v = vv
            with s.lock: s.cur = r
            if s.wr:
                e = time.time() - s.t0 if s.t0 else 0
                s.wr.writerow([datetime.now().isoformat(), f"{e:.1f}",
                    s.cur.get("pat", 0), s.cur.get("pname", ""),
                    s.cur.get("yt", 0), s.cur.get("pt", 0),
                    r.get("yp", -1), r.get("pp", -1), r.get("ys", -1), r.get("ps", -1),
                    r.get("yl", -9999), r.get("pl", -9999),
                    r.get("v1", -1), r.get("v2", -1), r.get("t1", -1), r.get("t2", -1),
                    r.get("c1", -1), r.get("c2", -1),
                    s.cur.get("sp", 0), s.cur.get("ac", 0),
                    r.get("cfps", 0), r.get("cerr", 0),
                    ";".join(s.warns[-3:]) if s.warns else ""])
            time.sleep(0.08)

    def warn(s, msg):
        s.warns.append(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")

    def stop(s):
        s.running = False
        if hasattr(s, "th"): s.th.join(timeout=2)
        if s.csv: s.csv.close()

# ---- FASTER PATTERN GEN (natural head-tracking speeds) ----
WAVEFORMS = ["sine","triangle","sawtooth","square","sine_var_freq",
    "random_step","random_walk","hold_extreme","figure8","spiral",
    "zigzag","bounce","sine_expand","sine_shrink",
    "sine_phase_90","yaw_sine_pitch_tri","yaw_tri_pitch_sine",
    "dual_random_step","dual_random_walk"]

AXIS_MODES = ["both_sync","both_async","both_opposite",
    "yaw_only","pitch_only","yaw_fast_pitch_slow","yaw_slow_pitch_fast"]

# Speed presets for natural head tracking
SPEED_PRESETS = {
    "slow":   (800, 1500, 30, 80),
    "medium": (1500, 2800, 60, 130),
    "fast":   (2500, 3500, 100, 180),
    "extreme":(3200, 3800, 150, 200),
}

class PatternGen:
    def __init__(s):
        s.idx = 0; s.used = set(); random.seed(int(time.time()))
        s.speed_mode = "fast"  # default
        s.complexity = "mixed"  # simple/mixed/complex
        s._gen()

    def _gen(s):
        while True:
            wf = random.choice(WAVEFORMS)
            ax = random.choice(AXIS_MODES)
            freq = round(random.uniform(0.3, 4.0), 2)
            amp_y = round(random.uniform(0.2, 0.9), 2)
            amp_p = round(random.uniform(0.2, 0.9), 2)
            dur = random.choice([30, 45, 60, 75, 90])
            key = (wf, ax, freq, amp_y, amp_p, dur)
            if key not in s.used:
                s.used.add(key)
                s.wf = wf; s.ax = ax; s.freq = freq
                s.amp_y = amp_y; s.amp_p = amp_p; s.dur = dur
                sm = SPEED_PRESETS[s.speed_mode]
                s.spd = random.randint(sm[0], sm[1])
                s.acc = random.randint(sm[2], sm[3])
                s.name = f"{wf}/{ax} f={freq:.1f}"
                return

    def update(s, t, temp=40):
        tf = max(0.3, 1.0 - (temp - HIGH_TEMP_C) / 15.0) if temp > HIGH_TEMP_C else 1.0
        sp, ac = int(s.spd * tf), int(s.acc * tf)
        ry = (YAW_MAX - YAW_CENTER) * s.amp_y
        rp = (PITCH_MAX - PITCH_CENTER) * s.amp_p
        yt, pt = float(YAW_CENTER), float(PITCH_CENTER)
        wf, ax, f = s.wf, s.ax, s.freq

        def wave(t, f, amp, off, wf):
            phase = 2 * math.pi * f * t
            if wf == "sine": return off + amp * math.sin(phase)
            elif wf == "triangle":
                frac = (t * f) % 1.0
                return off + amp * (4*frac-1 if frac<0.5 else 3-4*frac)
            elif wf == "sawtooth": return off + amp * (2*((t*f)%1.0)-1)
            elif wf == "square": return off + amp * (1 if (t*f)%1.0 <0.5 else -1)
            elif wf == "sine_var_freq":
                vf = f * (0.3 + 1.7*abs(math.sin(2*math.pi*t/20)))
                return off + amp * math.sin(2*math.pi*vf*t)
            elif wf == "sine_expand":
                a = amp * min(1.0, t/10)
                return off + a * math.sin(phase)
            elif wf == "sine_shrink":
                a = amp * max(0.1, 1.0 - t/40)
                return off + a * math.sin(phase)
            return off + amp * math.sin(phase)

        if wf in ("random_step","dual_random_step"):
            if int(t*4)%4==0: yt = YAW_CENTER + random.uniform(-ry, ry)
            if int(t*4+2)%4==0: pt = PITCH_CENTER + random.uniform(-rp, rp)
        elif wf in ("random_walk","dual_random_walk"):
            if int(t*6)%6==0:
                yt = YAW_CENTER + _clip(random.uniform(-ry, ry), -ry, ry)
                pt = PITCH_CENTER + _clip(random.uniform(-rp, rp), -rp, rp)
        elif wf == "hold_extreme":
            pos=[(YAW_MIN+80,PITCH_CENTER),(YAW_MAX-80,PITCH_CENTER),
                 (YAW_CENTER,PITCH_MIN+80),(YAW_CENTER,PITCH_MAX-80)]
            yt, pt = pos[int(t//8)%4]; sp, ac = 300, 30
        elif wf == "figure8":
            yt = YAW_CENTER + ry*0.8*math.sin(2*math.pi*f*t)
            pt = PITCH_CENTER + rp*0.8*math.sin(4*math.pi*f*t)
        elif wf == "spiral":
            rs = min(1.0, t/15); yt = YAW_CENTER + ry*rs*math.sin(2*math.pi*f*t)
            pt = PITCH_CENTER + rp*rs*math.cos(2*math.pi*f*1.3*t)
        elif wf == "zigzag":
            per=1.5/max(f,0.1); frac=(t%per)/per
            if frac<0.25: yt=YAW_CENTER+ry*frac/0.25; pt=PITCH_CENTER+rp*frac/0.25
            elif frac<0.5: yt=YAW_CENTER+ry*(1-(frac-0.25)/0.25); pt=PITCH_CENTER-rp*(frac-0.25)/0.25
            elif frac<0.75: yt=YAW_CENTER-ry*(frac-0.5)/0.25; pt=PITCH_CENTER+rp*(frac-0.5)/0.25
            else: yt=YAW_CENTER-ry*(1-(frac-0.75)/0.25); pt=PITCH_CENTER-rp*(1-(frac-0.75)/0.25)
        elif wf == "bounce":
            per=1.0/max(f,0.1); frac=(t%per)/per
            yt=YAW_CENTER+ry*(1-abs(2*frac-1))*math.sin(2*math.pi*t*0.5)
            pt=PITCH_CENTER+rp*(1-abs(2*frac-1))*math.cos(2*math.pi*t*0.5)
        elif wf == "sine_phase_90":
            yt=YAW_CENTER+ry*math.sin(2*math.pi*f*t)
            pt=PITCH_CENTER+rp*math.cos(2*math.pi*f*t)
        elif wf == "yaw_sine_pitch_tri":
            yt=YAW_CENTER+ry*math.sin(2*math.pi*f*t)
            frac=(t*f)%1.0; pt=PITCH_CENTER+rp*(4*frac-1 if frac<0.5 else 3-4*frac)
        elif wf == "yaw_tri_pitch_sine":
            frac=(t*f)%1.0; yt=YAW_CENTER+ry*(4*frac-1 if frac<0.5 else 3-4*frac)
            pt=PITCH_CENTER+rp*math.sin(2*math.pi*f*t)
        else:
            ph = 2*math.pi*f*t
            if ax == "both_sync": yt=wave(t,f,ry,YAW_CENTER,wf); pt=wave(t,f,rp,PITCH_CENTER,wf)
            elif ax == "both_async": yt=wave(t,f,ry,YAW_CENTER,wf); pt=wave(t,f*0.7,rp,PITCH_CENTER,wf)
            elif ax == "both_opposite": yt=wave(t,f,ry,YAW_CENTER,wf); pt=wave(t,f,-rp,PITCH_CENTER,wf)
            elif ax == "yaw_only": yt=wave(t,f,ry,YAW_CENTER,wf)
            elif ax == "pitch_only": pt=wave(t,f,rp,PITCH_CENTER,wf)
            elif ax == "yaw_fast_pitch_slow": yt=wave(t,f*2,ry,YAW_CENTER,wf); pt=wave(t,f*0.4,rp,PITCH_CENTER,wf)
            elif ax == "yaw_slow_pitch_fast": yt=wave(t,f*0.4,ry,YAW_CENTER,wf); pt=wave(t,f*2,rp,PITCH_CENTER,wf)
            else: yt=wave(t,f,ry,YAW_CENTER,wf); pt=wave(t,f,rp,PITCH_CENTER,wf)

        yt = _clip(yt, YAW_MIN+30, YAW_MAX-30)
        pt = _clip(pt, PITCH_MIN+30, PITCH_MAX-30)
        if t > s.dur: s.idx += 1; s._gen(); return s.update(0, temp)
        return int(yt), int(pt), sp, ac, f"#{s.idx} {s.name}", s.dur - t

class Safety:
    def __init__(s, m): s.m = m; s.paused = False; s.stop = False
    def check(s, e):
        c = s.m.cur
        for sid in [1, 2]:
            if (c.get(f"v{sid}", 12) or 12) < MIN_VOLTAGE:
                s.m.warn(f"STOP: S{sid} volt low!"); s.stop = True; return "stop"
            if (c.get(f"t{sid}", 40) or 40) > MAX_TEMP_C:
                if not s.paused: s.m.warn(f"PAUSE: S{sid} too hot"); s.paused = True
                return "paused"
        if s.paused:
            if all((c.get(f"t{sid}", 0) or 0) < HIGH_TEMP_C - 5 for sid in [1,2]):
                s.paused = False; s.m.warn("Resume")
        return "paused" if s.paused else "ok"

import tkinter as tk
from tkinter import ttk, font
from PIL import Image, ImageTk
import numpy as np

class Dashboard:
    def __init__(s, m, g, cam, pg, sf, state):
        s.m = m; s.g = g; s.cam = cam; s.pg = pg; s.sf = sf; s.state = state
        s.root = tk.Tk(); s.root.title("STS3032 + D415 ? Interactive Stress Test")
        s.root.geometry("1440x900"); s.root.configure(bg="#080810")
        s.root.protocol("WM_DELETE_WINDOW", s._close)

        BG = "#080810"; CB = "#101028"; FG = "#cccccc"
        GR = "#00ff88"; RD = "#ff4444"; OR = "#ff9900"; BL = "#00ccff"
        s.BG, s.CB, s.FG, s.GR, s.RD, s.OR, s.BL = BG, CB, FG, GR, RD, OR, BL

        fb = font.Font(family="Consolas", size=22, weight="bold")
        fv = font.Font(family="Consolas", size=14, weight="bold")
        ft = font.Font(family="Consolas", size=10, weight="bold")
        fs = font.Font(family="Consolas", size=8)
        fn = font.Font(family="Microsoft YaHei", size=10)
        s.fb, s.fv, s.ft, s.fs, s.fn = fb, fv, ft, fs, fn

        # ---- Layout ----
        # Top bar
        top = tk.Frame(s.root, bg=BG, height=36)
        top.pack(fill="x", padx=4, pady=(2, 0)); top.pack_propagate(False)
        s.lel = tk.Label(top, text="0.0 min", fg=GR, bg=BG, font=fb)
        s.lel.pack(side="left", padx=(6, 12))
        s.lph = tk.Label(top, text="---", fg=FG, bg=BG, font=fv)
        s.lph.pack(side="left", padx=6)
        s.lwt = tk.Label(top, text="", fg=RD, bg=BG, font=ft)
        s.lwt.pack(side="right", padx=6)

        # Main: Left (camera + gauges) / Right (controls + stats)
        pan = tk.PanedWindow(s.root, bg=BG, sashwidth=3)
        pan.pack(fill="both", expand=True, padx=4, pady=2)

        left = tk.Frame(pan, bg=BG, width=900)
        right = tk.Frame(pan, bg=BG, width=500)
        pan.add(left); pan.add(right)

        # ---- LEFT PANEL ----
        # Camera feed (big)
        camf = tk.Frame(left, bg=CB, bd=1, relief="solid", highlightbackground="#1a1a40")
        camf.pack(fill="both", expand=True, padx=2, pady=2)
        tk.Label(camf, text="CAMERA FEED", fg=BL, bg=CB, font=ft).pack(pady=(4, 0))
        s.cam_label = tk.Label(camf, bg="#000", text="No Signal", fg="#555", font=fv)
        s.cam_label.pack(fill="both", expand=True, padx=4, pady=(0, 4))

        # Gauges row under camera
        gframe = tk.Frame(left, bg=BG, height=90)
        gframe.pack(fill="x", padx=2, pady=2); gframe.pack_propagate(False)

        def mini_gauge(parent, title, key, unit, mx, cl, row, col):
            f = tk.Frame(parent, bg=CB, bd=1, relief="solid", highlightbackground="#1a1a40")
            f.grid(row=row, column=col, padx=2, pady=1, sticky="nsew")
            tk.Label(f, text=title, fg=cl, bg=CB, font=fs).pack()
            lb = tk.Label(f, text="---", fg="#fff", bg=CB, font=font.Font(family="Consolas", size=13, weight="bold"))
            lb.pack()
            tk.Label(f, text=unit, fg="#555", bg=CB, font=fs).pack()
            setattr(s, f"lb_{key}", lb)

        for i in range(8): gframe.columnconfigure(i, weight=1)

        mini_gauge(gframe, "Temp S1", "t1", "C", 85, OR, 0, 0)
        mini_gauge(gframe, "Temp S2", "t2", "C", 85, RD, 0, 1)
        mini_gauge(gframe, "Volt S1", "v1", "V", 15, BL, 0, 2)
        mini_gauge(gframe, "Volt S2", "v2", "V", 15, "#0099ff", 0, 3)
        mini_gauge(gframe, "Load Yaw", "yl", "", 1500, "#ffcc00", 0, 4)
        mini_gauge(gframe, "Load Pitch", "pl", "", 1500, "#ff6600", 0, 5)
        mini_gauge(gframe, "Curr S1", "c1", "mA", 3000, "#ff9900", 0, 6)
        mini_gauge(gframe, "Curr S2", "c2", "mA", 3000, "#ff4444", 0, 7)

        # ---- RIGHT PANEL ----
        # Position
        for i, (ttl, key, clr) in enumerate([("YAW POS", "y", GR), ("PITCH POS", "p", RD)]):
            f = tk.Frame(right, bg=CB, bd=1, relief="solid", highlightbackground="#1a1a40")
            f.pack(fill="x", padx=2, pady=2)
            tk.Label(f, text=ttl, fg=clr, bg=CB, font=ft).pack(pady=(4, 0))
            lp = tk.Label(f, text="----", fg="#fff", bg=CB, font=fb); lp.pack()
            ld = tk.Label(f, text="---", fg=FG, bg=CB, font=font.Font(family="Consolas", size=12)); ld.pack()
            ls = tk.Label(f, text="Spd: ---", fg="#888", bg=CB, font=fs); ls.pack(pady=(0, 4))
            setattr(s, f"lp_{key}", lp); setattr(s, f"ld_{key}", ld); setattr(s, f"ls_{key}", ls)

        # Command info
        f = tk.Frame(right, bg=CB, bd=1, relief="solid", highlightbackground="#1a1a40")
        f.pack(fill="x", padx=2, pady=2)
        tk.Label(f, text="COMMAND", fg="#ffcc00", bg=CB, font=ft).pack(pady=(4, 0))
        s.lspd = tk.Label(f, text="Speed: ---", fg="#fff", bg=CB, font=fv); s.lspd.pack()
        s.lacc = tk.Label(f, text="Accel: ---", fg="#fff", bg=CB, font=fv); s.lacc.pack()
        s.lyt = tk.Label(f, text="Y Tgt: ---", fg=GR, bg=CB, font=fs); s.lyt.pack()
        s.lpt = tk.Label(f, text="P Tgt: ---", fg="#ff9999", bg=CB, font=fs); s.lpt.pack(pady=(0, 4))

        # Pattern info
        f = tk.Frame(right, bg=CB, bd=1, relief="solid", highlightbackground="#1a1a40")
        f.pack(fill="x", padx=2, pady=2)
        tk.Label(f, text="PATTERN", fg=OR, bg=CB, font=ft).pack(pady=(4, 0))
        s.lpat = tk.Label(f, text="#0 ---", fg=FG, bg=CB, font=fs, justify="left", anchor="w")
        s.lpat.pack(fill="x", padx=6, pady=(0, 4))

        # ---- CONTROLS ----
        ctrl = tk.Frame(right, bg=CB, bd=1, relief="solid", highlightbackground="#1a1a40")
        ctrl.pack(fill="x", padx=2, pady=2)
        tk.Label(ctrl, text="CONTROLS", fg=GR, bg=CB, font=ft).pack(pady=(4, 2))

        # Speed mode
        sfrm = tk.Frame(ctrl, bg=CB); sfrm.pack(fill="x", padx=6, pady=2)
        tk.Label(sfrm, text="Speed:", fg="#888", bg=CB, font=fn).pack(side="left")
        s.spd_var = tk.StringVar(value="fast")
        for txt, val in [("Slow", "slow"), ("Medium", "medium"), ("Fast", "fast"), ("Extreme", "extreme")]:
            tk.Radiobutton(sfrm, text=txt, variable=s.spd_var, value=val,
                          command=s._on_speed, bg=CB, fg=FG, selectcolor=CB,
                          font=fs, activebackground=CB).pack(side="left", padx=3)

        # Record button
        bfrm = tk.Frame(ctrl, bg=CB); bfrm.pack(fill="x", padx=6, pady=4)
        s.btn_rec = tk.Button(bfrm, text="Start Record", command=s._toggle_record,
                             bg="#333", fg="#fff", font=fs, relief="flat", padx=8)
        s.btn_rec.pack(side="left", padx=2)
        s.btn_next = tk.Button(bfrm, text="Next Pattern", command=s._next_pattern,
                              bg="#333", fg="#fff", font=fs, relief="flat", padx=8)
        s.btn_next.pack(side="left", padx=2)
        s.btn_ctr = tk.Button(bfrm, text="Center", command=s._center,
                             bg="#444", fg="#fff", font=fs, relief="flat", padx=8)
        s.btn_ctr.pack(side="left", padx=2)

        # Duration
        dfrm = tk.Frame(ctrl, bg=CB); dfrm.pack(fill="x", padx=6, pady=2)
        tk.Label(dfrm, text="Duration (min):", fg="#888", bg=CB, font=fn).pack(side="left")
        s.dur_var = tk.StringVar(value="60")
        tk.Spinbox(dfrm, textvariable=s.dur_var, from_=1, to=999, width=5,
                  bg=CB, fg=FG, font=fs, bd=1).pack(side="left", padx=4)
        s.btn_dur = tk.Button(dfrm, text="Apply", command=s._apply_dur,
                             bg="#333", fg="#fff", font=fs, relief="flat", padx=6)
        s.btn_dur.pack(side="left")

        # Warnings
        f = tk.Frame(right, bg=CB, bd=1, relief="solid", highlightbackground="#1a1a40")
        f.pack(fill="both", expand=True, padx=2, pady=2)
        tk.Label(f, text="WARNINGS", fg=RD, bg=CB, font=ft).pack(pady=(4, 0))
        s.txw = tk.Text(f, bg=CB, fg="#ff6666", font=fs, height=6, bd=0, state="disabled")
        s.txw.pack(fill="both", expand=True, padx=4, pady=(0, 3))

        # Stats
        f = tk.Frame(right, bg=CB, bd=1, relief="solid", highlightbackground="#1a1a40")
        f.pack(fill="x", padx=2, pady=2)
        tk.Label(f, text="STATS", fg=GR, bg=CB, font=ft).pack(pady=(4, 0))
        s.lst = tk.Label(f, text="", fg=FG, bg=CB, font=fs, justify="left", anchor="w")
        s.lst.pack(fill="x", padx=6, pady=(0, 4))

        s._upd()
        s.root.mainloop()

    def _on_speed(s):
        s.pg.speed_mode = s.spd_var.get()
        sm = SPEED_PRESETS[s.pg.speed_mode]
        s.pg.spd = random.randint(sm[0], sm[1])
        s.pg.acc = random.randint(sm[2], sm[3])
        s.m.warn(f"Speed mode: {s.pg.speed_mode}")

    def _toggle_record(s):
        if s.cam.recording:
            s.cam.stop_recording()
            s.btn_rec.config(text="Start Record", bg="#333")
        else:
            s.cam.start_recording()
            if s.cam.recording:
                s.btn_rec.config(text="Stop Record", bg="#600")
                s.m.warn("Recording started")

    def _next_pattern(s):
        s.state["skip"] = True
        s.m.warn("Skipping to next pattern")

    def _center(s):
        s.g.center()
        s.m.warn("Centering...")

    def _apply_dur(s):
        global TEST_DURATION
        try:
            TEST_DURATION = int(s.dur_var.get()) * 60
            s.m.warn(f"Duration set to {TEST_DURATION//60} min")
        except: pass

    def _upd(s):
        if not s.state.get("running", True): return
        with s.m.lock: c = dict(s.m.cur); w = list(s.m.warns)
        e = c.get("elapsed", 0) or 0

        s.lel.config(text=f"{e/60:.1f} min")
        s.lph.config(text=f"{c.get('pname','---')}")
        t1, t2 = c.get("t1", 0) or 0, c.get("t2", 0) or 0
        s.lwt.config(text=f"HOT {max(t1,t2)}C!" if max(t1,t2) > HIGH_TEMP_C else "")

        # Position gauges
        for k, clr in [("y", s.GR), ("p", s.RD)]:
            p = c.get(f"{k}p"); sp = c.get(f"{k}s")
            if hasattr(s, f"lp_{k}"):
                getattr(s, f"lp_{k}").config(text=str(int(p)) if p else "---")
                getattr(s, f"ld_{k}").config(text=f"{(p or 0)*360/4096:.1f} deg")
                getattr(s, f"ls_{k}").config(text=f"Spd: {int(sp) if sp else '---'}")

        # Mini gauges
        for key in ["t1","t2","v1","v2","yl","pl","c1","c2"]:
            v = c.get(key)
            if hasattr(s, f"lb_{key}") and v is not None and v > -9000:
                fmt = "{:.1f}" if key.startswith("v") else "{}"
                getattr(s, f"lb_{key}").config(text=fmt.format(v))

        s.lspd.config(text=f"Speed: {c.get('sp','---')}")
        s.lacc.config(text=f"Accel: {c.get('ac','---')}")
        s.lyt.config(text=f"Y Tgt: {c.get('yt','---')}")
        s.lpt.config(text=f"P Tgt: {c.get('pt','---')}")
        s.lpat.config(text=f"#{c.get('pat','?')} {s.pg.wf}/{s.pg.ax}\nf={s.pg.freq} ay={s.pg.amp_y} ap={s.pg.amp_p}\nRemain: {c.get('rem','?')}s")

        s.txw.config(state="normal"); s.txw.delete("1.0", "end")
        s.txw.insert("end", "\n".join(w[-8:]) if w else "(none)")
        s.txw.config(state="disabled")

        fps = c.get("cfps", 0) or 0
        s.lst.config(text=(
            f"FPS: {fps:.1f}  Err: {c.get('cerr',0)}\n"
            f"Rec: {'ON' if s.cam.recording else 'OFF'}\n"
            f"Cam: {s.cam.frames_total} frames\n"
            f"Max T: {s.m.max_t}C  Min V: {s.m.min_v:.1f}V\n"
            f"Warns: {len(w)}  Speed: {s.pg.speed_mode}\n"
            f"Progress: {e/60:.0f}/{TEST_DURATION/60:.0f} min"))

        # ---- Camera Feed ----
        if s.cam.latest_frame is not None:
            try:
                img = s.cam.latest_frame
                # Resize to fit (max 700x500)
                h, w = img.shape[:2]
                scale = min(700/w, 500/h, 1.0)
                nw, nh = int(w*scale), int(h*scale)
                img = Image.fromarray(img).resize((nw, nh), Image.LANCZOS)
                photo = ImageTk.PhotoImage(img)
                s.cam_label.config(image=photo, text="")
                s.cam_label.image = photo
            except: pass

        s.root.after(80, s._upd)

    def _close(s):
        s.state["running"] = False
        s.root.quit(); s.root.destroy()

def main():
    print("=" * 60)
    print("  STS3032 + D415 INTERACTIVE STRESS TEST v2")
    print(f"  Port: {PORT}  Default: {TEST_DURATION//60} min")
    print("=" * 60)

    g = Gimbal()
    cam = CameraMon()
    m = Monitor(g, cam)
    pg = PatternGen()
    sf = Safety(m)
    m.start()

    state = {"running": True, "phase_t0": 0, "skip": False}
    print("[READY] Dashboard opening...\n")

    def mover():
        last_pat = -1
        while state["running"] and not sf.stop:
            e = time.time() - m.t0
            if TEST_DURATION > 0 and e > TEST_DURATION:
                state["running"] = False; break
            st = sf.check(e)
            if st == "stop": state["running"] = False; break
            elif st == "paused":
                g.center()
                with m.lock: m.cur["pname"] = "PAUSED"
                time.sleep(1); continue

            pt = e - state["phase_t0"]
            if state.get("skip"):
                state["skip"] = False; pt = 999999  # force skip

            temp = max(m.cur.get("t1", 40) or 40, m.cur.get("t2", 40) or 40)
            yt, pt2, sp, ac, name, rem = pg.update(pt, temp)

            if pg.idx != last_pat:
                last_pat = pg.idx; state["phase_t0"] = e
                print(f"  #{pg.idx}: {pg.wf}/{pg.ax} f={pg.freq} ay={pg.amp_y} ap={pg.amp_p} sp={sp} ac={ac} dur={pg.dur}s")

            with m.lock:
                m.cur["yt"] = yt; m.cur["pt"] = pt2
                m.cur["sp"] = sp; m.cur["ac"] = ac
                m.cur["pname"] = name; m.cur["pat"] = pg.idx
                m.cur["rem"] = int(rem); m.cur["elapsed"] = e

            try: g.move(yt, pt2, sp, ac)
            except Exception as ex: m.warn(f"Move: {ex}")
            time.sleep(0.012)

    th = threading.Thread(target=mover, daemon=True); th.start()
    Dashboard(m, g, cam, pg, sf, state)

    state["running"] = False; th.join(timeout=2)
    cam.stop_recording(); m.stop(); cam.close(); g.close()

    et = time.time() - m.t0 if m.t0 else 0
    print(f"\n{'='*60}\n  DONE  {et/60:.1f}min  MaxT:{m.max_t}C  MinV:{m.min_v:.1f}V")
    print(f"  Cam:{cam.frames_total}fr/{cam.errors}err  Patterns:{pg.idx}")
    print(f"  Warns:{len(m.warns)}\n{'='*60}")

if __name__ == "__main__": main()
