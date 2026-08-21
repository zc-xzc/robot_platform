#!/usr/bin/env python3
"""STS3032 + D415 Final Stress Test
Auto-detect COM port, 90+ random diverse patterns (1/min),
real-time tkinter dashboard, camera monitoring, CSV logging.
"""
import sys, os, time, threading, csv, math, random
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

# ---- Auto-detect port ----
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
    print("[ERROR] No servo port found! Check USB connection.")
    sys.exit(1)
BAUD = 1000000
print(f"[PORT] Auto-detected: {PORT}")

YAW_ID, PITCH_ID = 1, 2
YAW_CENTER, PITCH_CENTER = 2110, 2048
YAW_MIN, YAW_MAX = 1217, 3233
PITCH_MIN, PITCH_MAX = 1162, 2601
TEST_DURATION = 5400  # 90 min
MAX_TEMP_C, HIGH_TEMP_C = 75, 65
MIN_VOLTAGE = 7.0
LOG_DIR = Path(__file__).resolve().parent / "stress_test_logs"

def _clip(v, lo, hi):
    return max(lo, min(hi, v))

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
        print("[GIMBAL] Centering...")
        s.center(); time.sleep(0.8)
        print("[GIMBAL] Ready")
    def move(s, yp, pp, sp=2000, ac=80):
        yp = int(_clip(yp, YAW_MIN, YAW_MAX))
        pp = int(_clip(pp, PITCH_MIN, PITCH_MAX))
        s.pkt.WritePosEx(YAW_ID, yp, sp, ac)
        s.pkt.WritePosEx(PITCH_ID, pp, sp, ac)
    def center(s):
        s.move(YAW_CENTER, PITCH_CENTER, 500, 50)
    def close(s):
        try: s.center(); time.sleep(0.3)
        except: pass
        try: s.port.closePort()
        except: pass

# ---- CAMERA ----
class CameraMon:
    def __init__(s):
        s.enabled = False; s.pipeline = None
        s.frames_total = 0; s.frames_recent = 0
        s.last_fps = 0.0; s.errors = 0
        s.resolution = ""; s.last_check = time.time()
        try:
            import pyrealsense2 as rs
            for w, h, fps in [(1280, 720, 30), (640, 480, 30), (424, 240, 30)]:
                try:
                    p = rs.pipeline(); c = rs.config()
                    c.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
                    p.start(c); s.pipeline = p; s.enabled = True
                    s.resolution = f"{w}x{h}@{fps}fps"
                    print(f"[CAMERA] {s.resolution}"); break
                except: continue
        except ImportError: print("[CAMERA] pyrealsense2 not installed")
        except Exception as e: print(f"[CAMERA] Error: {e}")
    def update(s):
        if not s.enabled or not s.pipeline: return
        now = time.time()
        try:
            frames = s.pipeline.wait_for_frames(timeout_ms=100)
            if frames: s.frames_total += 1; s.frames_recent += 1
        except: s.errors += 1
        if now - s.last_check > 1.0:
            s.last_fps = s.frames_recent / (now - s.last_check + 0.001)
            s.frames_recent = 0; s.last_check = now
    def close(s):
        if s.pipeline:
            try: s.pipeline.stop()
            except: pass

# ---- MONITOR ----
class Monitor:
    def __init__(s, gimbal, camera):
        s.g = gimbal; s.cam = camera
        s.running = False; s.lock = threading.Lock()
        s.cur = {}; s.hist = []; s.csv = None; s.wr = None
        s.t0 = None; s.warns = []; s.max_t = 0; s.min_v = 999
    def start(s):
        LOG_DIR.mkdir(exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        s.csv = open(LOG_DIR / f"stress_{ts}.csv", "w", newline="", encoding="utf-8")
        s.wr = csv.writer(s.csv)
        s.wr.writerow(["ts","elapsed","pattern","name",
            "yt","pt","yp","pp","ys","ps","yl","pl","v1","v2","t1","t2","c1","c2",
            "sp","ac","cfps","cerr","warns"])
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
            with s.lock: s.cur = r; s.hist.append(r)
            if s.wr:
                e = time.time() - s.t0 if s.t0 else 0
                s.wr.writerow([datetime.now().isoformat(), f"{e:.1f}",
                    s.cur.get("pat", 0), s.cur.get("pname", ""),
                    s.cur.get("yt", 0), s.cur.get("pt", 0),
                    r.get("yp", -1), r.get("pp", -1),
                    r.get("ys", -1), r.get("ps", -1),
                    r.get("yl", -9999), r.get("pl", -9999),
                    r.get("v1", -1), r.get("v2", -1),
                    r.get("t1", -1), r.get("t2", -1),
                    r.get("c1", -1), r.get("c2", -1),
                    s.cur.get("sp", 0), s.cur.get("ac", 0),
                    r.get("cfps", 0), r.get("cerr", 0),
                    ";".join(s.warns[-3:]) if s.warns else ""])
            time.sleep(0.08)
    def warn(s, msg):
        t = datetime.now().strftime("%H:%M:%S")
        s.warns.append(f"[{t}] {msg}")
    def stop(s):
        s.running = False
        if hasattr(s, "th"): s.th.join(timeout=2)
        if s.csv: s.csv.close()

# ---- 90+ RANDOM DIVERSE PATTERNS ----
class PatternGen:
    """Generates 1 new random pattern per minute, never repeating."""
    def __init__(s):
        s.idx = 0; s.used = set(); random.seed(int(time.time()))
        s._gen_next()

    WAVEFORMS = [
        "sine", "triangle", "sawtooth", "reverse_saw", "square",
        "sine_expand", "sine_shrink", "sine_var_freq",
        "random_step", "random_walk", "hold_extreme", "hold_random",
        "figure8", "spiral", "zigzag", "bounce",
        "sine_phase_0", "sine_phase_90", "sine_phase_180",
        "yaw_sine_pitch_tri", "yaw_tri_pitch_sine",
        "yaw_step_pitch_sine", "yaw_sine_pitch_step",
        "dual_random_step", "dual_random_walk",
    ]

    AXIS_MODES = [
        "both_sync", "both_async", "both_opposite",
        "yaw_only", "pitch_only",
        "yaw_fast_pitch_slow", "yaw_slow_pitch_fast",
    ]

    def _gen_next(s):
        """Generate a unique random pattern."""
        while True:
            wf = random.choice(s.WAVEFORMS)
            ax = random.choice(s.AXIS_MODES)
            freq = round(random.uniform(0.05, 5.0), 2)
            amp_y = round(random.uniform(0.1, 0.95), 2)
            amp_p = round(random.uniform(0.1, 0.95), 2)
            speed = random.randint(300, 3800)
            accel = random.randint(15, 200)
            dur = random.choice([45, 60, 75, 90, 120])  # seconds
            key = (wf, ax, freq, amp_y, amp_p, speed, accel, dur)
            if key not in s.used:
                s.used.add(key)
                s.waveform = wf; s.axis_mode = ax
                s.freq = freq; s.amp_y = amp_y; s.amp_p = amp_p
                s.speed = speed; s.accel = accel; s.duration = dur
                s.name = f"{wf}/{ax} f={freq:.1f} ay={amp_y:.1f} ap={amp_p:.1f} sp={speed} ac={accel}"
                return

    def update(s, t, temp=40):
        """t = seconds elapsed within this pattern."""
        tf = max(0.3, 1.0 - (temp - HIGH_TEMP_C) / 15.0) if temp > HIGH_TEMP_C else 1.0
        wf, ax = s.waveform, s.axis_mode
        f, ay, ap = s.freq, s.amp_y, s.amp_p
        sp, ac = int(s.speed * tf), int(s.accel * tf)

        ry = (YAW_MAX - YAW_CENTER) * ay
        rp = (PITCH_MAX - PITCH_CENTER) * ap

        def _wave(t, f, amp, offset, wf):
            phase = 2 * math.pi * f * t
            if wf == "sine": return offset + amp * math.sin(phase)
            elif wf == "triangle":
                frac = (t * f) % 1.0
                return offset + amp * (4 * frac - 1 if frac < 0.5 else 3 - 4 * frac)
            elif wf == "sawtooth":
                return offset + amp * (2 * ((t * f) % 1.0) - 1)
            elif wf == "reverse_saw":
                return offset + amp * (1 - 2 * ((t * f) % 1.0))
            elif wf == "square":
                return offset + amp * (1 if (t * f) % 1.0 < 0.5 else -1)
            elif wf == "sine_expand":
                a = amp * min(1.0, t / 15.0)
                return offset + a * math.sin(phase)
            elif wf == "sine_shrink":
                a = amp * max(0.1, 1.0 - t / 60.0)
                return offset + a * math.sin(phase)
            elif wf == "sine_var_freq":
                vf = f * (0.5 + 1.5 * abs(math.sin(2 * math.pi * t / 20.0)))
                return offset + amp * math.sin(2 * math.pi * vf * t)
            return offset + amp * math.sin(phase)  # default sine

        def _rand_step(amp, offset):
            return offset + random.uniform(-amp, amp)

        yt = float(YAW_CENTER); pt = float(PITCH_CENTER)

        if wf in ("random_step", "dual_random_step"):
            if int(t * 3) % 3 == 0:
                yt = _rand_step(ry, YAW_CENTER)
                pt = _rand_step(rp, PITCH_CENTER)
            sp, ac = max(sp, 2500), max(ac, 100)
        elif wf in ("random_walk", "dual_random_walk"):
            if int(t * 5) % 5 == 0:
                yt = YAW_CENTER + _clip(random.uniform(-ry, ry), -(YAW_MAX-YAW_CENTER), YAW_MAX-YAW_CENTER)
                pt = PITCH_CENTER + _clip(random.uniform(-rp, rp), -(PITCH_MAX-PITCH_CENTER), PITCH_MAX-PITCH_CENTER)
            sp, ac = max(sp, 1800), max(ac, 70)
        elif wf == "hold_extreme":
            pos = [(YAW_MIN + 80, PITCH_CENTER), (YAW_MAX - 80, PITCH_CENTER),
                   (YAW_CENTER, PITCH_MIN + 80), (YAW_CENTER, PITCH_MAX - 80),
                   (YAW_MIN + 80, PITCH_MIN + 80), (YAW_MAX - 80, PITCH_MAX - 80)]
            idx = int(t // 10) % len(pos)
            yt, pt = pos[idx]; sp, ac = 300, 30
        elif wf == "hold_random":
            if int(t) % 10 == 0:
                yt = random.uniform(YAW_MIN + 80, YAW_MAX - 80)
                pt = random.uniform(PITCH_MIN + 80, PITCH_MAX - 80)
            sp, ac = 300, 30
        elif wf == "figure8":
            yt = YAW_CENTER + ry * 0.8 * math.sin(2 * math.pi * f * t)
            pt = PITCH_CENTER + rp * 0.8 * math.sin(4 * math.pi * f * t)
        elif wf == "spiral":
            r_scale = min(1.0, t / 20.0)
            yt = YAW_CENTER + ry * r_scale * math.sin(2 * math.pi * f * t)
            pt = PITCH_CENTER + rp * r_scale * math.cos(2 * math.pi * f * 1.3 * t)
        elif wf == "zigzag":
            per = 1.5 / max(f, 0.1); frac = (t % per) / per
            if frac < 0.25:
                yt = YAW_CENTER + ry * (frac / 0.25)
                pt = PITCH_CENTER + rp * (frac / 0.25)
            elif frac < 0.5:
                yt = YAW_CENTER + ry * (1 - (frac - 0.25) / 0.25)
                pt = PITCH_CENTER - rp * ((frac - 0.25) / 0.25)
            elif frac < 0.75:
                yt = YAW_CENTER - ry * ((frac - 0.5) / 0.25)
                pt = PITCH_CENTER + rp * ((frac - 0.5) / 0.25)
            else:
                yt = YAW_CENTER - ry * (1 - (frac - 0.75) / 0.25)
                pt = PITCH_CENTER - rp * (1 - (frac - 0.75) / 0.25)
        elif wf == "bounce":
            per = 1.0 / max(f, 0.1); frac = (t % per) / per
            yt = YAW_CENTER + ry * (1 - abs(2 * frac - 1)) * math.sin(2 * math.pi * t * 0.5)
            pt = PITCH_CENTER + rp * (1 - abs(2 * frac - 1)) * math.cos(2 * math.pi * t * 0.5)
        elif wf == "sine_phase_0":
            yt = YAW_CENTER + ry * math.sin(2 * math.pi * f * t)
            pt = PITCH_CENTER + rp * math.sin(2 * math.pi * f * t)  # in phase
        elif wf == "sine_phase_90":
            yt = YAW_CENTER + ry * math.sin(2 * math.pi * f * t)
            pt = PITCH_CENTER + rp * math.cos(2 * math.pi * f * t)  # 90 deg offset
        elif wf == "sine_phase_180":
            yt = YAW_CENTER + ry * math.sin(2 * math.pi * f * t)
            pt = PITCH_CENTER + rp * math.sin(2 * math.pi * f * t + math.pi)
        elif wf == "yaw_sine_pitch_tri":
            yt = YAW_CENTER + ry * math.sin(2 * math.pi * f * t)
            frac = (t * f) % 1.0
            pt = PITCH_CENTER + rp * (4 * frac - 1 if frac < 0.5 else 3 - 4 * frac)
        elif wf == "yaw_tri_pitch_sine":
            frac = (t * f) % 1.0
            yt = YAW_CENTER + ry * (4 * frac - 1 if frac < 0.5 else 3 - 4 * frac)
            pt = PITCH_CENTER + rp * math.sin(2 * math.pi * f * t)
        elif wf == "yaw_step_pitch_sine":
            if int(t * f * 2) % 2 == 0:
                yt = YAW_CENTER + ry * random.choice([-1, 1]) * random.uniform(0.3, 1.0)
            pt = PITCH_CENTER + rp * math.sin(2 * math.pi * f * t)
        elif wf == "yaw_sine_pitch_step":
            yt = YAW_CENTER + ry * math.sin(2 * math.pi * f * t)
            if int(t * f * 2) % 2 == 0:
                pt = PITCH_CENTER + rp * random.choice([-1, 1]) * random.uniform(0.3, 1.0)
        else:
            # Default: use waveform for both, with axis mode applied
            if ax == "both_sync":
                yt = _wave(t, f, ry, YAW_CENTER, wf)
                pt = _wave(t, f, rp, PITCH_CENTER, wf)
            elif ax == "both_async":
                yt = _wave(t, f, ry, YAW_CENTER, wf)
                pt = _wave(t, f * 0.7, rp, PITCH_CENTER, wf)
            elif ax == "both_opposite":
                yt = _wave(t, f, ry, YAW_CENTER, wf)
                pt = _wave(t, f, -rp, PITCH_CENTER, wf)
            elif ax == "yaw_only":
                yt = _wave(t, f, ry, YAW_CENTER, wf)
            elif ax == "pitch_only":
                pt = _wave(t, f, rp, PITCH_CENTER, wf)
            elif ax == "yaw_fast_pitch_slow":
                yt = _wave(t, f * 2, ry, YAW_CENTER, wf)
                pt = _wave(t, f * 0.4, rp, PITCH_CENTER, wf)
            elif ax == "yaw_slow_pitch_fast":
                yt = _wave(t, f * 0.4, ry, YAW_CENTER, wf)
                pt = _wave(t, f * 2, rp, PITCH_CENTER, wf)
            else:
                yt = _wave(t, f, ry, YAW_CENTER, wf)
                pt = _wave(t, f, rp, PITCH_CENTER, wf)

        # Clamp
        yt = _clip(yt, YAW_MIN + 40, YAW_MAX - 40)
        pt = _clip(pt, PITCH_MIN + 40, PITCH_MAX - 40)

        if t > s.duration:
            s.idx += 1; s._gen_next()
            return s.update(0, temp)  # restart for next pattern

        return int(yt), int(pt), sp, ac, f"#{s.idx} {s.name}"

class Safety:
    def __init__(s, m): s.m = m; s.paused = False; s.stop = False
    def check(s, e):
        c = s.m.cur
        for sid in [1, 2]:
            v = c.get(f"v{sid}", 12)
            if v and v < MIN_VOLTAGE:
                s.m.warn(f"STOP: S{sid} volt {v:.1f}V"); s.stop = True; return "stop"
            t = c.get(f"t{sid}", 40)
            if t and t > MAX_TEMP_C:
                if not s.paused: s.m.warn(f"PAUSE: S{sid} {t}C"); s.paused = True
                return "paused"
        if s.paused:
            ok = all((c.get(f"t{sid}", 0) or 0) < HIGH_TEMP_C - 5 for sid in [1, 2])
            if ok: s.paused = False; s.m.warn("RESUME")
        return "paused" if s.paused else "ok"

import tkinter as tk
from tkinter import font

class Dashboard:
    def __init__(s, m, g, cam):
        s.m = m; s.g = g; s.cam = cam
        s.root = tk.Tk(); s.root.title("STS3032+D415 Stress Test")
        s.root.geometry("1280x820"); s.root.configure(bg="#0a0a14")
        s.root.protocol("WM_DELETE_WINDOW", lambda: (s.root.quit(), s.root.destroy()))
        bg = "#0a0a14"; cb = "#12122a"; fg = "#cccccc"
        gr = "#00ff88"; rd = "#ff4444"; or_ = "#ff9900"
        fb = font.Font(family="Consolas", size=26, weight="bold")
        fv = font.Font(family="Consolas", size=16, weight="bold")
        ft = font.Font(family="Consolas", size=10, weight="bold")
        fs = font.Font(family="Consolas", size=8)

        # Top
        top = tk.Frame(s.root, bg=bg, height=42)
        top.pack(fill="x", padx=6, pady=(4, 0)); top.pack_propagate(False)
        s.lel = tk.Label(top, text="0.0 min", fg=gr, bg=bg, font=fb)
        s.lel.pack(side="left", padx=(6, 15))
        s.lph = tk.Label(top, text="Phase: --", fg=fg, bg=bg, font=fv)
        s.lph.pack(side="left", padx=8)
        s.lwt = tk.Label(top, text="", fg=rd, bg=bg, font=ft)
        s.lwt.pack(side="right", padx=8)

        mf = tk.Frame(s.root, bg=bg)
        mf.pack(fill="both", expand=True, padx=6, pady=3)
        for i in range(4): mf.columnconfigure(i, weight=1)
        for i in range(4): mf.rowconfigure(i, weight=1)

        def card(r, c, rs=1, cs=1):
            f = tk.Frame(mf, bg=cb, bd=1, relief="solid", highlightbackground="#1a1a40", highlightthickness=1)
            f.grid(row=r, column=c, rowspan=rs, columnspan=cs, padx=3, pady=3, sticky="nsew")
            return f

        def pos_card(r, c, t, k):
            f = card(r, c)
            tk.Label(f, text=t, fg=gr, bg=cb, font=ft).pack(pady=(6, 0))
            lp = tk.Label(f, text="----", fg="#fff", bg=cb, font=fb); lp.pack()
            tk.Label(f, text="0-4096", fg="#555", bg=cb, font=fs).pack()
            ld = tk.Label(f, text="---", fg=fg, bg=cb, font=font.Font(family="Consolas", size=13)); ld.pack()
            sf = tk.Frame(f, bg=cb); sf.pack(pady=(6, 2))
            tk.Label(sf, text="Spd:", fg="#888", bg=cb, font=fs).pack(side="left", padx=2)
            ls = tk.Label(sf, text="---", fg=fg, bg=cb, font=fs); ls.pack(side="left")
            setattr(s, f"lp_{k}", lp); setattr(s, f"ld_{k}", ld); setattr(s, f"ls_{k}", ls)

        def gauge(r, c, t, k, u, mx, cl):
            f = card(r, c)
            tk.Label(f, text=t, fg=cl, bg=cb, font=ft).pack(pady=(6, 2))
            lb = tk.Label(f, text="---", fg="#fff", bg=cb, font=fb); lb.pack()
            tk.Label(f, text=u, fg="#555", bg=cb, font=fs).pack()
            cv = tk.Canvas(f, width=160, height=10, bg="#1a1a30", highlightthickness=0)
            cv.pack(pady=(3, 6))
            br = cv.create_rectangle(0, 0, 0, 10, fill=cl, outline="")
            setattr(s, f"lb_{k}", lb); setattr(s, f"br_{k}", (cv, br, mx, cl))

        # Row 0
        pos_card(0, 0, "YAW", "y"); pos_card(0, 1, "PITCH", "p")
        c2 = card(0, 2)
        tk.Label(c2, text="CAMERA", fg="#00ccff", bg=cb, font=ft).pack(pady=(6, 2))
        s.lcf = tk.Label(c2, text="FPS: ---", fg="#fff", bg=cb, font=fv); s.lcf.pack()
        s.lcr = tk.Label(c2, text="", fg="#888", bg=cb, font=fs); s.lcr.pack()
        s.lce = tk.Label(c2, text="Err: 0", fg=rd, bg=cb, font=fs); s.lce.pack(pady=(8, 0))
        s.lct = tk.Label(c2, text="Total: 0", fg=fg, bg=cb, font=fs); s.lct.pack()

        c3 = card(0, 3)
        tk.Label(c3, text="PATTERN", fg=or_, bg=cb, font=ft).pack(pady=(6, 2))
        s.txp = tk.Text(c3, bg=cb, fg=fg, font=fs, height=10, width=30, bd=0, state="disabled", wrap="word")
        s.txp.pack(fill="both", expand=True, padx=4, pady=(0, 4))

        # Row 1
        gauge(1, 0, "Temp S1", "t1", "C", 85, or_)
        gauge(1, 1, "Temp S2", "t2", "C", 85, rd)
        gauge(1, 2, "Volt S1", "v1", "V", 15, "#00ccff")
        gauge(1, 3, "Volt S2", "v2", "V", 15, "#0099ff")

        # Row 2
        gauge(2, 0, "Load Yaw", "yl", "", 1500, "#ffcc00")
        gauge(2, 1, "Load Pitch", "pl", "", 1500, "#ff6600")
        gauge(2, 2, "Curr S1", "c1", "mA", 3000, "#ff9900")
        gauge(2, 3, "Curr S2", "c2", "mA", 3000, "#ff4444")

        # Row 3
        c12 = card(3, 0)
        tk.Label(c12, text="COMMAND", fg="#ffcc00", bg=cb, font=ft).pack(pady=(6, 2))
        s.lsp = tk.Label(c12, text="Spd: ---", fg="#fff", bg=cb, font=fv); s.lsp.pack()
        s.lac = tk.Label(c12, text="Acc: ---", fg="#fff", bg=cb, font=fv); s.lac.pack()
        tk.Label(c12, text="", bg=cb).pack()
        s.lyt = tk.Label(c12, text="YTgt: ---", fg="#00cc66", bg=cb, font=fs); s.lyt.pack()
        s.lpt = tk.Label(c12, text="PTgt: ---", fg="#ff9999", bg=cb, font=fs); s.lpt.pack()

        c13 = card(3, 1, cs=2)
        tk.Label(c13, text="WARNINGS", fg=rd, bg=cb, font=ft).pack(pady=(4, 2))
        s.txw = tk.Text(c13, bg=cb, fg="#ff6666", font=fs, height=4, bd=0, state="disabled")
        s.txw.pack(fill="both", expand=True, padx=4, pady=(0, 3))

        c14 = card(3, 3)
        tk.Label(c14, text="STATS", fg=gr, bg=cb, font=ft).pack(pady=(6, 2))
        s.lst = tk.Label(c14, text="", fg=fg, bg=cb, font=fs, justify="left", anchor="w")
        s.lst.pack(fill="both", expand=True, padx=6, pady=(0, 4))

        s._upd()
        s.root.mainloop()

    def _ub(s, k, v):
        if not hasattr(s, f"br_{k}"): return
        cv, br, mx, cl = getattr(s, f"br_{k}")
        pct = min(1.0, max(0.0, (v or 0) / max(mx, 1)))
        cv.coords(br, 0, 0, int(pct * 160), 10)
        if v and "t" in k:
            if v > MAX_TEMP_C: cv.itemconfig(br, fill="#ff4444")
            elif v > HIGH_TEMP_C: cv.itemconfig(br, fill="#ff9900")
            else: cv.itemconfig(br, fill=cl)

    def _upd(s):
        with s.m.lock: c = dict(s.m.cur); w = list(s.m.warns)
        e = c.get("elapsed", 0) or 0
        s.lel.config(text=f"{e/60:.1f} min")
        s.lph.config(text=f"{c.get('pname','--')}")

        for k in ["y", "p"]:
            p = c.get(f"{k}p"); sp = c.get(f"{k}s")
            if hasattr(s, f"lp_{k}"):
                getattr(s, f"lp_{k}").config(text=str(int(p)) if p else "---")
                getattr(s, f"ld_{k}").config(text=f"{(p or 0)*360/4096:.1f} deg")
                getattr(s, f"ls_{k}").config(text=str(int(sp)) if sp else "---")

        for k in ["t1","t2","v1","v2","yl","pl","c1","c2"]:
            v = c.get(k)
            if hasattr(s, f"lb_{k}") and v is not None and v > -9000:
                fmt = "{:.1f}" if k.startswith("v") else "{}"
                getattr(s, f"lb_{k}").config(text=fmt.format(v))
            s._ub(k, v)

        s.lsp.config(text=f"Spd: {c.get('sp','---')}")
        s.lac.config(text=f"Acc: {c.get('ac','---')}")
        s.lyt.config(text=f"YTgt: {c.get('yt','---')}")
        s.lpt.config(text=f"PTgt: {c.get('pt','---')}")

        fps = c.get("cfps", 0) or 0
        s.lcf.config(text=f"FPS: {fps:.1f}", fg="#00ff88" if fps > 15 else "#ff4444")
        s.lcr.config(text=s.cam.resolution)
        s.lce.config(text=f"Err: {c.get('cerr',0)}")
        s.lct.config(text=f"Total: {s.cam.frames_total}")

        s.txp.config(state="normal"); s.txp.delete("1.0", "end")
        s.txp.insert("end", f"Pattern #{c.get('pat','?')}\n\n")
        s.txp.insert("end", f"Wave: {s.m.cur.get('pwf','?')}\n")
        s.txp.insert("end", f"Axis:  {s.m.cur.get('pax','?')}\n")
        s.txp.insert("end", f"Freq:  {c.get('pfreq','?')} Hz\n")
        s.txp.insert("end", f"Amp Y: {c.get('pampy','?')}  P: {c.get('pampp','?')}\n")
        s.txp.insert("end", f"Speed:{c.get('sp','?')}  Acc:{c.get('ac','?')}\n")
        s.txp.insert("end", f"Time:  {c.get('pdt','?')}s left")
        s.txp.config(state="disabled")

        s.txw.config(state="normal"); s.txw.delete("1.0", "end")
        s.txw.insert("end", "\n".join(w[-8:]) if w else "(none)")
        s.txw.config(state="disabled")

        t1, t2 = c.get("t1", 0) or 0, c.get("t2", 0) or 0
        s.lwt.config(text=f"HOT {max(t1,t2)}C!" if max(t1,t2) > HIGH_TEMP_C else "")

        s.lst.config(text=(
            f"Max Temp: {s.m.max_t}C\nMin Volt: {s.m.min_v:.1f}V\n"
            f"Warnings: {len(w)}\nPattern: #{c.get('pat','?')}\n"
            f"Progress: {e/60:.0f}/{TEST_DURATION/60:.0f} min\n"
            f"Cam: {s.cam.frames_total} fr, {s.cam.errors} err"
        ))
        s.root.after(250, s._upd)

def main():
    print("=" * 60)
    print("  STS3032 + D415 FINAL STRESS TEST")
    print(f"  Port: {PORT}  Duration: {TEST_DURATION}s ({TEST_DURATION/60:.0f} min)")
    print("=" * 60)
    g = Gimbal()
    cam = CameraMon()
    m = Monitor(g, cam)
    pg = PatternGen()
    sf = Safety(m)
    m.start()
    state = {"running": True, "phase_t0": 0}
    print("[READY] Dashboard opening...\n")

    def mover():
        while state["running"] and not sf.stop:
            e = time.time() - m.t0
            if TEST_DURATION > 0 and e > TEST_DURATION:
                state["running"] = False; break
            st = sf.check(e)
            if st == "stop": state["running"] = False; break
            elif st == "paused":
                g.center()
                with m.lock: m.cur["pname"] = "PAUSED (cooling)"
                time.sleep(1); continue

            pt = e - state["phase_t0"]
            temp = max(m.cur.get("t1", 40) or 40, m.cur.get("t2", 40) or 40)
            yt, pt2, sp, ac, name = pg.update(pt, temp)

            with m.lock:
                m.cur["yt"] = yt; m.cur["pt"] = pt2
                m.cur["sp"] = sp; m.cur["ac"] = ac
                m.cur["pname"] = name; m.cur["pat"] = pg.idx
                m.cur["pwf"] = pg.waveform; m.cur["pax"] = pg.axis_mode
                m.cur["pfreq"] = pg.freq; m.cur["pampy"] = pg.amp_y
                m.cur["pampp"] = pg.amp_p
                m.cur["pdt"] = max(0, int(pg.duration - pt))
                m.cur["elapsed"] = e

            if pg.idx > state.get("last_pat", -1):
                state["last_pat"] = pg.idx
                state["phase_t0"] = e
                print(f"  Pattern #{pg.idx}: {pg.waveform}/{pg.axis_mode} f={pg.freq} ay={pg.amp_y} ap={pg.amp_p} sp={sp} ac={ac} dur={pg.duration}s")

            try: g.move(yt, pt2, sp, ac)
            except Exception as ex: m.warn(f"Move: {ex}")
            time.sleep(0.012)

    th = threading.Thread(target=mover, daemon=True); th.start()
    Dashboard(m, g, cam)
    state["running"] = False; th.join(timeout=2)
    m.stop(); cam.close(); g.close()
    et = time.time() - m.t0 if m.t0 else 0
    print(f"\n{'='*60}\n  DONE  Time:{et/60:.1f}min  MaxT:{m.max_t}C  MinV:{m.min_v:.1f}V")
    print(f"  Cam:{cam.frames_total}fr/{cam.errors}err  Warns:{len(m.warns)}")
    print(f"  Patterns run: {pg.idx}\n{'='*60}")

if __name__ == "__main__": main()
