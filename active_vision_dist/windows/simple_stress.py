#!/usr/bin/env python3
"""Servo stress test - clean, working version"""
import sys, os, time, threading, math, random, csv
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

PORT, BAUD = "COM8", 1000000
YC, PC = 2110, 2048
YMIN, YMAX = 1217, 3233
PMIN, PMAX = 1162, 2601
TEST_DUR = 3600

import tkinter as tk
from tkinter import font
import numpy as np
from PIL import Image, ImageTk

# ============ CONNECT ============
print(f"Opening {PORT}...")
ph = PortHandler(PORT); ph.baudrate = BAUD
if not ph.openPort():
    print(f"ERROR: Cannot open {PORT}. Close Feetech software.")
    sys.exit(1)
pkt = sms_sts(ph); time.sleep(0.5)
for sid in [1,2]:
    _, r, _ = pkt.ping(sid)
    if r != COMM_SUCCESS: print(f"ERROR: Servo {sid}"); sys.exit(1)
    pkt.write1ByteTxRx(sid, 0x28, 1)
print("[OK] Servos connected")

# ============ CAMERA ============
cam_pipe = None; cam_enabled = False; cam_frame = None; cam_total = 0; cam_errs = 0
try:
    import pyrealsense2 as rs
    for w,h,fps in [(640,480,30),(424,240,30)]:
        try:
            p = rs.pipeline(); c = rs.config()
            c.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
            p.start(c); cam_pipe = p; cam_enabled = True
            print(f"[OK] Camera: {w}x{h}@{fps}"); break
        except: continue
    if not cam_enabled: print("[WARN] No camera")
except Exception as e: print(f"[WARN] Camera: {e}")

# ============ CENTER ============
pkt.WritePosEx(1, YC, 500, 50); pkt.WritePosEx(2, PC, 500, 50)
time.sleep(1); print("[OK] Centered")

# ============ SHARED STATE ============
state = {
    "running": True, "elapsed": 0,
    "yaw_pos": YC, "pitch_pos": PC, "yaw_spd": 0, "pitch_spd": 0,
    "yaw_tgt": YC, "pitch_tgt": PC, "spd": 0, "acc": 0,
    "t1": 0, "t2": 0, "v1": 0, "v2": 0, "ly": 0, "lp": 0, "c1": 0, "c2": 0,
    "pattern": "", "pat_idx": 0, "warns": [], "fps": 0
}
state_lock = threading.Lock()
history = []
MAX_HIST = 300

# ============ CSV ============
log_dir = Path(__file__).resolve().parent / "stress_test_logs"
log_dir.mkdir(exist_ok=True)
csv_f = open(log_dir / f"log_{datetime.now():%Y%m%d_%H%M%S}.csv", "w", newline="", encoding="utf-8")
csv_w = csv.writer(csv_f)
csv_w.writerow(["ts","elapsed","pat","yt","pt","yp","pp","ys","ps","t1","t2","v1","v2","sp","acc"])

# ============ MONITOR THREAD ============
def monitor_thread():
    t0 = time.time(); cam_lt = t0; cam_fr = 0
    global cam_frame, cam_total, cam_errs
    while state["running"]:
        try:
            # Read servos
            for sid, pk, sk in [(1,"yaw_pos","yaw_spd"), (2,"pitch_pos","pitch_spd")]:
                pos, spd, comm, _ = pkt.ReadPosSpeed(sid)
                if comm == COMM_SUCCESS and pos is not None:
                    state[pk] = int(pos); state[sk] = int(spd) if spd else 0

            # Read temps/voltage/load/current
            for sid, tk, vk, lk, ck in [(1,"t1","v1","ly","c1"), (2,"t2","v2","lp","c2")]:
                try:
                    val, comm, _ = pkt.read1ByteTxRx(sid, 0x3F)
                    if comm == COMM_SUCCESS: state[tk] = val
                except: pass
                try:
                    val, comm, _ = pkt.read1ByteTxRx(sid, 0x3E)
                    if comm == COMM_SUCCESS: state[vk] = val / 10.0
                except: pass
                try:
                    val, comm, _ = pkt.read2ByteTxRx(sid, 0x3C)
                    if comm == COMM_SUCCESS: state[lk] = val
                except: pass
                try:
                    val, comm, _ = pkt.read2ByteTxRx(sid, 0x45)
                    if comm == COMM_SUCCESS: state[ck] = val
                except: pass

            # Camera
            if cam_enabled and cam_pipe:
                try:
                    f = cam_pipe.wait_for_frames(timeout_ms=30)
                    if f:
                        cam_total += 1; cam_fr += 1
                        cf = f.get_color_frame()
                        if cf: cam_frame = np.asanyarray(cf.get_data())
                except: cam_errs += 1

            # FPS
            now = time.time()
            if now - cam_lt > 1.0:
                state["fps"] = cam_fr / (now - cam_lt); cam_fr = 0; cam_lt = now

            state["elapsed"] = now - t0

            # Append to history (reduce lock contention)
            h = {k: state.get(k, 0) for k in ["yaw_pos","pitch_pos","yaw_spd","pitch_spd","t1","t2","v1","v2"]}
            h["t"] = state["elapsed"]
            history.append(h)
            if len(history) > MAX_HIST: history.pop(0)

            # CSV
            csv_w.writerow([datetime.now().isoformat(), f"{state['elapsed']:.1f}",
                state["pat_idx"], state["yaw_tgt"], state["pitch_tgt"],
                state["yaw_pos"], state["pitch_pos"], state["yaw_spd"], state["pitch_spd"],
                state["t1"], state["t2"], state["v1"], state["v2"], state["spd"], state["acc"]])

        except Exception as e:
            state["warns"].append(f"Mon: {e}")
        time.sleep(0.04)

threading.Thread(target=monitor_thread, daemon=True).start()

# ============ PATTERNS (24) ============
patterns = [
    ("01_Warmup",         lambda t: (YC+(YMAX-YC)*0.4*math.sin(2*math.pi*0.3*t), PC+(PMAX-PC)*0.3*math.cos(2*math.pi*0.25*t), 1200, 60)),
    ("02_Yaw_Sweep",      lambda t: (YC+(YMAX-YC)*0.7*math.sin(2*math.pi*0.6*t), PC, 2000, 80)),
    ("03_Pitch_Sweep",    lambda t: (YC, PC+(PMAX-PC)*0.6*math.sin(2*math.pi*0.55*t), 1800, 70)),
    ("04_Dual_Med",       lambda t: (YC+(YMAX-YC)*0.55*math.sin(2*math.pi*0.7*t), PC+(PMAX-PC)*0.45*math.cos(2*math.pi*0.6*t), 2500, 100)),
    ("05_Fast_Sine",      lambda t: (YC+(YMAX-YC)*0.65*math.sin(2*math.pi*2.2*t), PC+(PMAX-PC)*0.5*math.cos(2*math.pi*1.8*t), 3500, 160)),
    ("06_UltraFast",      lambda t: (YC+(YMAX-YC)*0.7*math.sin(2*math.pi*3.5*t), PC+(PMAX-PC)*0.55*math.cos(2*math.pi*3.0*t), 3800, 200)),
    ("07_Small_Glances",  lambda t: (YC+random.uniform(-200,200) if int(t*4)%4==0 else None, PC+random.uniform(-150,150) if int(t*4+1)%4==0 else None, 3200, 150)),
    ("08_Large_Jumps",    lambda t: (random.uniform(YMIN+100,YMAX-100) if int(t*2.5)%3==0 else None, random.uniform(PMIN+100,PMAX-100) if int(t*2.5+1)%3==0 else None, 3700, 180)),
    ("09_Figure8_Med",    lambda t: (YC+(YMAX-YC)*0.55*math.sin(2*math.pi*t/4.5), PC+(PMAX-PC)*0.4*math.sin(4*math.pi*t/4.5), 2600, 110)),
    ("10_Figure8_Fast",   lambda t: (YC+(YMAX-YC)*0.6*math.sin(2*math.pi*t/2.5), PC+(PMAX-PC)*0.45*math.sin(4*math.pi*t/2.5), 3700, 170)),
    ("11_MicroJitter_Lo", lambda t: (YC+4*math.sin(2*math.pi*t*8.0), PC+4*math.cos(2*math.pi*t*8.3), 3500, 200)),
    ("12_MicroJitter_Hi", lambda t: (YC+10*math.sin(2*math.pi*t*7.0), PC+10*math.cos(2*math.pi*t*7.5), 3800, 200)),
    ("13_Triangle_Yaw",   lambda t: (YC+(YMAX-YC)*0.65*(4*(t*0.4%1)-1 if (t*0.4)%1<0.5 else 3-4*(t*0.4)%1), PC, 2200, 80)),
    ("14_Triangle_Pitch", lambda t: (YC, PC+(PMAX-PC)*0.55*(4*(t*0.35%1)-1 if (t*0.35)%1<0.5 else 3-4*(t*0.35)%1), 2000, 70)),
    ("15_Zigzag",         lambda t: (YC+(YMAX-YC)*0.5*math.sin(2*math.pi*t*0.8), PC+(PMAX-PC)*0.45*(2*((t*0.6)%1)-1 if int(t*0.6)%2==0 else 1-2*((t*0.6)%1)), 3000, 140)),
    ("16_Speed_Ramp",     lambda t: (YC+(YMAX-YC)*0.55*math.sin(2*math.pi*(0.3+t/15)*t), PC+(PMAX-PC)*0.4*math.cos(2*math.pi*(0.25+t/15)*t), 1200+int(t/20*2000), 50+int(t/20*120))),
    ("17_Amplitude_Ramp", lambda t: (YC+(YMAX-YC)*min(0.15+t/25,0.8)*math.sin(2*math.pi*1.0*t), PC+(PMAX-PC)*min(0.15+t/25,0.7)*math.cos(2*math.pi*0.9*t), 2800, 120)),
    ("18_Spiral",         lambda t: (YC+(YMAX-YC)*min(0.1+t/20,0.7)*math.sin(2*math.pi*0.7*t), PC+(PMAX-PC)*min(0.1+t/20,0.6)*math.cos(2*math.pi*0.55*t), 3000, 130)),
    ("19_Reversals",      lambda t: (YC+(YMAX-YC)*0.55*math.sin(2*math.pi*1.5*t), PC+(PMAX-PC)*0.4*(1 if math.sin(2*math.pi*1.5*t)>0 else -1), 3800, 200)),
    ("20_Hold_Extreme",   lambda t: ((YMIN+100,YMAX-100,YC,YC)[int(t//10)%4], (PC,PC,PMIN+100,PMAX-100)[int(t//10)%4], 500, 50)),
    ("21_Hold_Random",    lambda t: (state["yaw_pos"] if t%10>0.2 else random.uniform(YMIN+100,YMAX-100), state["pitch_pos"] if t%10>0.2 else random.uniform(PMIN+100,PMAX-100), 2500, 100)),
    ("22_Accel_Stress",   lambda t: (YC+(YMAX-YC)*0.5*math.sin(2*math.pi*1.0*t), PC+(PMAX-PC)*0.4*math.cos(2*math.pi*0.8*t), 2000, 20+int((t%15)/15*180))),
    ("23_Boundary",       lambda t: (YMIN+80+(YMAX-YMIN-160)*(t*0.15%1) if int(t*0.15)%4==0 else YMAX-80 if int(t*0.15)%4==1 else YMAX-80-(YMAX-YMIN-160)*(t*0.15%1) if int(t*0.15)%4==2 else YMIN+80, PMAX-80 if int(t*0.15)%4==0 else PMAX-80-(PMAX-PMIN-160)*(t*0.15%1) if int(t*0.15)%4==1 else PMIN+80 if int(t*0.15)%4==2 else PMIN+80+(PMAX-PMIN-160)*(t*0.15%1), 2000, 70)),
    ("24_Endurance",      lambda t: (YC+(YMAX-YC)*0.6*math.sin(2*math.pi*1.8*t), PC+(PMAX-PC)*0.5*math.cos(2*math.pi*1.5*t), 3500, 160)),
]

# ============ MOVEMENT THREAD ============
def move_thread():
    pat_idx = 0; pat_start = time.time()
    last_yt, last_pt = YC, PC
    while state["running"]:
        elapsed = time.time() - pat_start
        if elapsed > 60:
            pat_idx = (pat_idx + 1) % len(patterns)
            pat_start = time.time()
            print(f"  Pattern {pat_idx}: {patterns[pat_idx][0]}")

        name, func = patterns[pat_idx]
        state["pat_idx"] = pat_idx; state["pattern"] = name

        r0, r1, sp, ac = func(elapsed)
        yt = max(YMIN+30, min(YMAX-30, int(r0 if r0 is not None else last_yt)))
        pt = max(PMIN+30, min(PMAX-30, int(r1 if r1 is not None else last_pt)))
        last_yt, last_pt = yt, pt

        state["yaw_tgt"] = yt; state["pitch_tgt"] = pt
        state["spd"] = sp; state["acc"] = ac

        try:
            pkt.WritePosEx(1, yt, sp, ac); pkt.WritePosEx(2, pt, sp, ac)
        except Exception as e:
            state["warns"].append(f"Move: {e}")

        if max(state.get("t1", 0), state.get("t2", 0)) > 75:
            pkt.WritePosEx(1, YC, 500, 50); pkt.WritePosEx(2, PC, 500, 50)
            state["warns"].append("OVERHEAT - pausing 10s"); time.sleep(10)
            pat_start = time.time()

        time.sleep(0.015)

threading.Thread(target=move_thread, daemon=True).start()

# ============ TKINTER DASHBOARD ============
root = tk.Tk()
root.title("Servo Stress Test | 24 Patterns")
root.geometry("1250x750")
BG, CB, FG = "#080810", "#101028", "#ccc"
GR, RD, OR, BL = "#00e676", "#ff1744", "#ff9100", "#00b0ff"
root.configure(bg=BG)

fH = font.Font(family="Consolas", size=22, weight="bold")
fV = font.Font(family="Consolas", size=13, weight="bold")
fS = font.Font(family="Consolas", size=9)
fB = font.Font(family="Segoe UI", size=10, weight="bold")

# Top Bar
top = tk.Frame(root, bg=CB, height=38); top.pack(fill="x"); top.pack_propagate(False)
lbl_time = tk.Label(top, text="00:00", fg=GR, bg=CB, font=fH); lbl_time.pack(side="left", padx=(10,20))
lbl_pat = tk.Label(top, text="Starting...", fg=FG, bg=CB, font=fV); lbl_pat.pack(side="left")
lbl_spd = tk.Label(top, text="Spd:0", fg=GR, bg=CB, font=fV); lbl_spd.pack(side="right", padx=10)
lbl_acc = tk.Label(top, text="Acc:0", fg=OR, bg=CB, font=fV); lbl_acc.pack(side="right", padx=6)
lbl_fps = tk.Label(top, text="FPS:0", fg=BL, bg=CB, font=fV); lbl_fps.pack(side="right", padx=10)

main = tk.Frame(root, bg=BG); main.pack(fill="both", expand=True, padx=3, pady=1)

# LEFT: Camera
left = tk.Frame(main, bg=CB); left.pack(side="left", fill="both", expand=True)
cam_lbl = tk.Label(left, bg="#000", text="No Signal", fg="#555", font=fV)
cam_lbl.pack(fill="both", expand=True, padx=2, pady=2)

# RIGHT: Charts + Gauges
right = tk.Frame(main, bg=BG, width=400); right.pack(side="right", fill="both", padx=(3,0))

# 3 Charts
chart_configs = []
for i,(title,key,color,ymin,ymax,unit) in enumerate([
    ("POSITION (Yaw)", "yaw_pos", GR, YMIN, YMAX, ""),
    ("SPEED", "yaw_spd", OR, 0, 4000, ""),
    ("TEMPERATURE", "t1", RD, 20, 90, "C"),
]):
    cf = tk.Frame(right, bg=CB); cf.pack(fill="x", pady=1)
    cv = tk.Canvas(cf, width=380, height=110, bg="#0a0a1e", highlightthickness=0); cv.pack(padx=2, pady=2)
    chart_configs.append((cv, key, color, ymin, ymax, unit, title))

# Gauge grid
gf = tk.Frame(right, bg=BG); gf.pack(fill="x", pady=1)
gauges = {}
gdata = [("T1","t1","C",OR),("T2","t2","C",RD),("V1","v1","V",BL),("V2","v2","V","#4fc3f7"),
         ("Y+","yaw_pos","",GR),("P+","pitch_pos","",RD),("LY","ly","","#ffd600"),("LP","lp","","#ff6d00")]
for i,(t,k,u,clr) in enumerate(gdata):
    r,col = i//4, i%4
    f = tk.Frame(gf, bg=CB); f.grid(row=r, column=col, padx=1, pady=1, sticky="nsew")
    tk.Label(f, text=t, fg=clr, bg=CB, font=fB).pack()
    lb = tk.Label(f, text="---", fg="#fff", bg=CB, font=font.Font(family="Consolas", size=11, weight="bold")); lb.pack()
    tk.Label(f, text=u, fg="#666", bg=CB, font=fS).pack(pady=(0,1))
    gauges[k] = lb
for i in range(4): gf.columnconfigure(i, weight=1)

# Log
wf = tk.Frame(right, bg=CB); wf.pack(fill="both", expand=True, pady=1)
tk.Label(wf, text="LOG", fg="#888", bg=CB, font=fB).pack(pady=(2,0))
txt_log = tk.Text(wf, bg=CB, fg="#ff8a80", font=fS, height=4, bd=0, state="disabled"); txt_log.pack(fill="both", expand=True, padx=3)

# Update function
cam_tick = 0
def update():
    global cam_tick
    if not state["running"]: return

    try:
        e = state["elapsed"]
        m, s = int(e//60), int(e%60)
        lbl_time.config(text=f"{m:02d}:{s:02d}")
        lbl_pat.config(text=f"#{state['pat_idx']} {state['pattern']}")
        lbl_spd.config(text=f"Spd:{state['spd']}")
        lbl_acc.config(text=f"Acc:{state['acc']}")
        lbl_fps.config(text=f"FPS:{state.get('fps',0):.0f}")

        # Gauges
        for key, lbl in gauges.items():
            v = state.get(key, 0) or 0
            if key in ("v1","v2"): lbl.config(text=f"{v:.1f}")
            else: lbl.config(text=str(int(v)) if v else "0")

        # Charts (every 2nd tick = 200ms)
        cam_tick += 1
        if cam_tick % 2 == 0 and len(history) > 1:
            for cv, key, color, ymin, ymax, unit, title in chart_configs:
                vals = [h.get(key, 0) for h in history[-200:] if h.get(key) is not None]
                if len(vals) < 2: continue
                cv.delete("all")
                w, h = 380, 110
                # Grid
                for i in range(5):
                    y = 10 + i*(h-20)/4
                    cv.create_line(0, y, w, y, fill="#1a1a35", dash=(2,4))
                # Title
                vmax, vmin = max(vals), min(vals)
                vr = max(vmax-vmin, 1)
                cv.create_text(4, 4, text=f"{title} max:{vmax:.0f}{unit} min:{vmin:.0f}{unit}", anchor="nw", fill=color, font=("Consolas", 8))
                # Line
                pts = []
                for i, v in enumerate(vals):
                    x = 5 + i*(w-10)/max(len(vals)-1,1)
                    y = 10 + (h-20)*(1-(v-vmin)/vr)
                    pts.extend([x, y])
                if len(pts) >= 4:
                    cv.create_line(*pts, fill=color, width=1.5, smooth=True)

        # Camera (every 3rd tick = 300ms)
        if cam_tick % 3 == 0 and cam_frame is not None:
            try:
                h, w = cam_frame.shape[:2]
                scale = min(700/w, 480/h, 1.0)
                img = Image.fromarray(cam_frame).resize((int(w*scale), int(h*scale)), Image.LANCZOS)
                photo = ImageTk.PhotoImage(img)
                cam_lbl.config(image=photo, text=""); cam_lbl.image = photo
            except: pass

        # Log
        ws = state.get("warns", [])
        txt_log.config(state="normal"); txt_log.delete("1.0", "end")
        txt_log.insert("end", "\n".join(ws[-5:]) if ws else "OK")
        txt_log.config(state="disabled")

    except Exception as ex:
        print(f"[UI] {ex}")

    root.after(100, update)

def on_close():
    state["running"] = False
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_close)
root.after(300, update)
print("[OK] Dashboard ready")
root.mainloop()

# Cleanup
csv_f.close()
if cam_pipe: cam_pipe.stop()
ph.closePort()
print("[OK] Done")
