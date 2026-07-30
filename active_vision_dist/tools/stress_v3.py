#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""STS3032 + D415 Stress Test v3 - Human Head Movement Simulation
Models real human head biomechanics: saccades, smooth pursuit, VOR,
fixations, scanning, nodding, shaking. 60+ non-repeating patterns, 1+ hour.
Beautiful tkinter dashboard with camera feed, recording, interactive controls.
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
if PORT is None: print("[ERROR] No port!"); sys.exit(1)
BAUD = 1000000
print(f"[PORT] {PORT}")

YAW_ID, PITCH_ID = 1, 2
YC, PC = 2110, 2048
YAW_MIN, YAW_MAX = 1217, 3233
PITCH_MIN, PITCH_MAX = 1162, 2601
YU = YAW_MAX - YC; YL = YC - YAW_MIN
PU = PITCH_MAX - PC; PL = PC - PITCH_MIN
TEST_DUR = 3600  # 60 min
MAX_T, HIGH_T = 75, 65
MIN_V = 7.0
LOG_DIR = Path(__file__).resolve().parent / "stress_test_logs"

def _clip(v, lo, hi): return max(lo, min(hi, v))

# ---- GIMBAL ----
class Gimbal:
    def __init__(s):
        s.port = PortHandler(PORT); s.port.baudrate = BAUD
        for attempt in range(3):
            try:
                if s.port.openPort(): break
            except Exception as e:
                if attempt < 2:
                    print(f"  [Retry {attempt+1}/3] Port {PORT} busy: {e}")
                    time.sleep(2)
                else:
                    raise RuntimeError(f"Cannot open {PORT} ? port locked by another app. Close Feetech software and retry.") from e
        s.pkt = sms_sts(s.port); time.sleep(0.5)
        for sid in [YAW_ID,PITCH_ID]:
            _,r,_=s.pkt.ping(sid)
            if r!=COMM_SUCCESS: raise RuntimeError(f"Servo {sid} no response")
            s.pkt.write1ByteTxRx(sid,0x28,1)
        s.center(); time.sleep(0.8)
    def move(s,yp,pp,sp=2500,ac=100):
        s.pkt.WritePosEx(YAW_ID,int(_clip(yp,YAW_MIN,YAW_MAX)),sp,ac)
        s.pkt.WritePosEx(PITCH_ID,int(_clip(pp,PITCH_MIN,PITCH_MAX)),sp,ac)
    def center(s): s.move(YC,PC,500,50)
    def close(s):
        try: s.center(); time.sleep(0.3)
        except: pass
        try: s.port.closePort()
        except: pass

# ---- CAMERA ----
class Camera:
    def __init__(s):
        s.enabled=False;s.pipe=None;s.frames=0;s.fr=0;s.fps=0.0
        s.errs=0;s.res="";s.lc=time.time();s.rec=False;s.recorder=None;s.latest=None
        try:
            import pyrealsense2 as rs
            for w,h,fps in[(640,480,30),(1280,720,30),(424,240,30)]:
                try:
                    p=rs.pipeline();c=rs.config()
                    c.enable_stream(rs.stream.color,w,h,rs.format.rgb8,fps)
                    p.start(c);s.pipe=p;s.enabled=True
                    s.res=f"{w}x{h}@{fps}fps";s.cw,s.ch=w,h;break
                except:continue
        except ImportError:print("[CAM] pyrealsense2 missing")
    def update(s):
        if not s.enabled:return
        try:
            f=s.pipe.wait_for_frames(timeout_ms=50)
            if f:
                s.frames+=1;s.fr+=1
                c=f.get_color_frame()
                if c:import numpy as np;s.latest=np.asanyarray(c.get_data())
                if s.rec and s.recorder:s.recorder.write(s.latest[:,:,::-1])
        except:s.errs+=1
        n=time.time()
        if n-s.lc>1.0:s.fps=s.fr/(n-s.lc+0.001);s.fr=0;s.lc=n
    def start_rec(s):
        if s.rec:return
        try:
            import cv2;LOG_DIR.mkdir(exist_ok=True)
            s.recorder=cv2.VideoWriter(str(LOG_DIR/f"video_{datetime.now():%Y%m%d_%H%M%S}.avi"),
                cv2.VideoWriter_fourcc(*"XVID"),20.0,(s.cw,s.ch));s.rec=True
        except:pass
    def stop_rec(s):s.rec=False
    def close(s):s.stop_rec()

# ---- MONITOR ----
class Monitor:
    def __init__(s,g,cam):s.g=g;s.cam=cam;s.r=False;s.l=threading.Lock();s.cur={};s.csv=None;s.wr=None;s.t0=None;s.warns=[];s.mt=0;s.mv=999
    def start(s):
        LOG_DIR.mkdir(exist_ok=True)
        s.csv=open(LOG_DIR/f"stress_{datetime.now():%Y%m%d_%H%M%S}.csv","w",newline="",encoding="utf-8")
        s.wr=csv.writer(s.csv)
        s.wr.writerow(["ts","elapsed","pat","name","yt","pt","yp","pp","ys","ps","yl","pl","v1","v2","t1","t2","c1","c2","sp","ac","cfps","cerr","warns"])
        s.t0=time.time();s.r=True;s.th=threading.Thread(target=s._run,daemon=True);s.th.start()
    def _rd(s,sid,addr,sz):
        try:
            v,c,_=s.g.pkt.read1ByteTxRx(sid,addr)if sz==1 else s.g.pkt.read2ByteTxRx(sid,addr)
            return v if c==COMM_SUCCESS else-9999
        except:return-9999
    def _run(s):
        while s.r:
            r={}
            for sid,nm in[(YAW_ID,"y"),(PITCH_ID,"p")]:
                pos,spd,comm,_=s.g.pkt.ReadPosSpeed(sid);ok=comm==COMM_SUCCESS
                r[f"{nm}p"]=int(pos)if ok else-1;r[f"{nm}s"]=int(spd)if ok else-1
                r[f"{nm}l"]=s._rd(sid,0x3C,2);r[f"v{sid}"]=s._rd(sid,0x3E,1)/10.0
                r[f"t{sid}"]=s._rd(sid,0x3F,1);r[f"c{sid}"]=s._rd(sid,0x45,2)
            s.cam.update();r["cfps"]=round(s.cam.fps,1);r["cerr"]=s.cam.errs
            for sid in[1,2]:
                tv=r.get(f"t{sid}",0)
                if tv and tv>-9000 and tv>s.mt:s.mt=tv
                vv=r.get(f"v{sid}",999)
                if vv and vv>-9000 and vv<s.mv:s.mv=vv
            with s.l:s.cur=r
            if s.wr:
                e=time.time()-s.t0 if s.t0 else 0
                s.wr.writerow([datetime.now().isoformat(),f"{e:.1f}",s.cur.get("pat",0),s.cur.get("pname",""),
                    s.cur.get("yt",0),s.cur.get("pt",0),r.get("yp",-1),r.get("pp",-1),r.get("ys",-1),r.get("ps",-1),
                    r.get("yl",-9999),r.get("pl",-9999),r.get("v1",-1),r.get("v2",-1),r.get("t1",-1),r.get("t2",-1),
                    r.get("c1",-1),r.get("c2",-1),s.cur.get("sp",0),s.cur.get("ac",0),r.get("cfps",0),r.get("cerr",0),
                    ";".join(s.warns[-3:])if s.warns else""])
            time.sleep(0.08)
    def warn(s,msg):s.warns.append(f"[{datetime.now():%H:%M:%S}] {msg}")
    def stop(s):s.r=False

# ---- HUMAN HEAD MOVEMENT PATTERNS ----
# Models real human head biomechanics: fixations, saccades, smooth pursuit etc.
# References: human head saccade velocity 300-700 deg/s, fixation 200-600ms,
# natural range yaw +/-80 deg, pitch +40/-60 deg

class HeadPatternGen:
    """Generates 60+ unique human-like head movement patterns, 1 per minute."""
    def __init__(s):
        s.idx=0;s.used=set();random.seed(int(time.time()))
        s.speed_mode="fast"
        s._gen()

    # Pattern types based on real human head behavior
    PATTERN_TYPES = [
        "reading",        # Small horizontal saccades + line returns (downward reset)
        "conversation",   # Looking between 2-3 fixation points
        "searching",      # Wide scanning: fixation -> saccade -> fixation
        "tracking",       # Smooth pursuit of moving target
        "nodding",        # Vertical yes motion
        "headshake",      # Horizontal no motion
        "neckstretch",    # Figure-8, slow
        "surprise",       # Quick large turn (startle reflex)
        "fatigue",        # Slow downward drift + quick corrective up
        "alert_scan",     # Rapid wide scanning
        "tilt_scan",      # Diagonal scanning
        "reading_return", # Reading with large page returns
        "nod_and_scan",   # Nodding while scanning
        "micro_jitter",   # Tiny fixational movements
        "speed_test",     # Max velocity stress
        "accel_test",     # Max acceleration stress
        "endurance",      # Continuous slow oscillation
        "corner_hold",    # Hold at limit positions
    ]

    def _gen(s):
        while True:
            typ = s.PATTERN_TYPES[s.idx % len(s.PATTERN_TYPES)]
            freq = round(random.uniform(0.5, 5.0), 2)
            amp_y = round(random.uniform(0.15, 0.9), 2)
            amp_p = round(random.uniform(0.15, 0.9), 2)
            dur = 60  # 1 minute per pattern
            key = (typ, freq, amp_y, amp_p, s.idx)
            if key not in s.used:
                s.used.add(key);s.typ=typ;s.freq=freq;s.amp_y=amp_y;s.amp_p=amp_p;s.dur=dur
                s._set_speed()
                s.name=f"[{typ}] f={freq:.1f} ay={amp_y:.1f} ap={amp_p:.1f}"
                return

    def _set_speed(s):
        if s.speed_mode=="slow":s.spd=random.randint(800,1500);s.acc=random.randint(30,80)
        elif s.speed_mode=="medium":s.spd=random.randint(1500,2500);s.acc=random.randint(60,130)
        elif s.speed_mode=="fast":s.spd=random.randint(2500,3500);s.acc=random.randint(100,180)
        else:s.spd=random.randint(3200,3800);s.acc=random.randint(150,200)

    def update(s, t, temp=40):
        tf = max(0.3, 1.0-(temp-HIGH_T)/15.0) if temp>HIGH_T else 1.0
        sp, ac = int(s.spd*tf), int(s.acc*tf)
        typ = s.typ
        ry = YU * s.amp_y; rp = PU * s.amp_p
        yt, pt = float(YC), float(PC)

        # ---- Human head movement implementations ----
        if typ == "reading":
            # Horizontal saccades rightward, then large leftward return (line change)
            line_dur = 2.0  # 2 seconds per line
            n_fix = 4       # fixations per line
            line_t = t % line_dur
            fix_idx = int(line_t / (line_dur / n_fix))
            yt = YC + ry * (fix_idx / (n_fix-1) * 2 - 1)  # left to right within line
            pt = PC - rp * 0.3 * math.sin(2*math.pi*t/line_dur)  # slight downward drift
            sp, ac = 3000, 150  # fast saccades

        elif typ == "conversation":
            # Alternate between 2-3 fixation points
            points = [YC-ry*0.5, YC, YC+ry*0.5]
            fix_t = t % (0.8 + random.random()*0.4)  # variable fixation
            idx = int(t // 1.5) % len(points)
            yt = points[idx]
            pt = PC + rp*0.2*math.sin(2*math.pi*t*0.3)

        elif typ == "searching":
            # Fixation (70% of time) + quick saccade to new position
            fix_dur = 0.3 + random.random()*0.5  # 300-800ms fixation
            if t % (fix_dur*3) < 0.03:  # saccade moment
                yt = YC + random.uniform(-ry, ry)
                pt = PC + random.uniform(-rp*0.6, rp*0.6)
            sp, ac = 3500, 180

        elif typ == "tracking":
            # Smooth sinusoidal pursuit
            yt = YC + ry * math.sin(2*math.pi*s.freq*0.6*t)
            pt = PC + rp * math.cos(2*math.pi*s.freq*0.4*t)
            sp, ac = 1500, 60  # smooth, lower speed

        elif typ == "nodding":
            # Vertical nodding (yes)
            pt = PC + rp * 0.8 * math.sin(2*math.pi*1.5*t)
            yt = YC + ry*0.1*math.sin(2*math.pi*0.3*t)

        elif typ == "headshake":
            # Horizontal shaking (no)
            yt = YC + ry * 0.7 * math.sin(2*math.pi*2.5*t)
            pt = PC + rp*0.1*math.cos(2*math.pi*0.5*t)

        elif typ == "neckstretch":
            # Slow figure-8
            yt = YC + ry*0.6*math.sin(2*math.pi*t/5.0)
            pt = PC + rp*0.5*math.sin(4*math.pi*t/5.0)

        elif typ == "surprise":
            # Keep head still, then quick large turn
            if t % 5.0 < 0.5:
                yt = YC + ry*0.9*random.choice([-1,1])
                pt = PC + rp*0.4*random.choice([-1,1])
            sp, ac = 3800, 200

        elif typ == "fatigue":
            # Slow downward drift, quick correction upward
            cycle = t % 6.0
            if cycle < 4.0:
                pt = PC - rp*0.3*(cycle/4.0)  # drift down
            else:
                pt = PC - rp*0.3 + rp*0.3*((cycle-4.0)/2.0)  # quick up
            sp, ac = (800, 40) if t%6.0<4.0 else (3500, 150)

        elif typ == "alert_scan":
            # Very rapid, wide saccades
            if t % 0.3 < 0.02:
                yt = YC + random.uniform(-ry*0.9, ry*0.9)
                pt = PC + random.uniform(-rp*0.5, rp*0.5)
            sp, ac = 3800, 200

        elif typ == "tilt_scan":
            # Diagonal scanning pattern
            frac = (t % 4.0) / 4.0
            yt = YC + ry*(2*frac-1)
            pt = PC + rp*(2*frac-1)
            sp, ac = 2500, 100

        elif typ == "reading_return":
            # Read rightward, quick return left, move down
            line_t = t % 3.0
            if line_t < 2.5:
                yt = YC + ry*(line_t/2.5*2-1)
            else:
                yt = YC - ry  # return
            pt = PC - rp*0.2*math.floor(t/3.0)
            sp, ac = (3000, 150) if line_t<0.1 or line_t>2.4 else (1000, 50)

        elif typ == "nod_and_scan":
            yt = YC + ry*0.5*math.sin(2*math.pi*0.8*t)
            pt = PC + rp*0.7*math.sin(2*math.pi*1.8*t)
            sp, ac = 2800, 120

        elif typ == "micro_jitter":
            # Tiny high-freq movements (simulating tremor + drift)
            j = 3 + 2*math.sin(2*math.pi*t/1.5)
            yt = YC + j*math.sin(2*math.pi*t*6.0)
            pt = PC + j*math.cos(2*math.pi*t*6.5)
            sp, ac = 3500, 200

        elif typ == "speed_test":
            # Max velocity sinusoidal sweep
            yt = YC + ry*0.8*math.sin(2*math.pi*3.0*t)
            pt = PC + rp*0.5*math.cos(2*math.pi*2.5*t)
            sp, ac = 3800, 180

        elif typ == "accel_test":
            # Rapid direction changes
            per = 1.5; frac = (t%per)/per
            tri = 4*frac-1 if frac<0.5 else 3-4*frac
            yt = YC + ry*tri
            pt = PC + rp*tri
            sp, ac = 3500, 200

        elif typ == "endurance":
            # Continuous oscillation at medium speed
            yt = YC + ry*0.5*math.sin(2*math.pi*1.0*t)
            pt = PC + rp*0.4*math.cos(2*math.pi*1.3*t)
            sp, ac = 2000, 80

        elif typ == "corner_hold":
            # Hold at extreme positions
            corners = [(YC+ry*0.85, PC), (YC-ry*0.85, PC), (YC, PC+rp*0.85), (YC, PC-rp*0.85)]
            idx = int(t//8) % 4
            yt, pt = corners[idx]
            sp, ac = 400, 30

        if t > s.dur:
            s.idx += 1; s._gen(); return s.update(0, temp)
        return int(_clip(yt,YAW_MIN+30,YAW_MAX-30)), int(_clip(pt,PITCH_MIN+30,PITCH_MAX-30)), sp, ac, f"#{s.idx} {s.name}", s.dur-t

class Safety:
    def __init__(s,m):s.m=m;s.paused=False;s.stop=False
    def check(s,e):
        c=s.m.cur
        for sid in[1,2]:
            if(c.get(f"v{sid}",12)or 12)<MIN_V:s.m.warn(f"STOP:S{sid}volt");s.stop=True;return"stop"
            if(c.get(f"t{sid}",40)or 40)>MAX_T:
                if not s.paused:s.m.warn(f"PAUSE:S{sid}hot");s.paused=True
                return"paused"
        if s.paused and all((c.get(f"t{sid}",0)or 0)<HIGH_T-5 for sid in[1,2]):s.paused=False;s.m.warn("Resume")
        return"paused"if s.paused else"ok"

import tkinter as tk
from tkinter import ttk, font
from PIL import Image, ImageTk
import numpy as np

class Dashboard:
    def __init__(s,m,g,cam,pg,sf,state):
        s.m=m;s.g=g;s.cam=cam;s.pg=pg;s.sf=sf;s.state=state
        s.root=tk.Tk();s.root.title("Head-Tracking Stress Test")
        s.root.geometry("1500x880")
        C0="#050510";C1="#0c0c24";C2="#141432";C3="#1c1c3e"
        FG="#d0d0d8";GR="#00e676";RD="#ff1744";OR="#ff9100";BL="#00b0ff";YL="#ffd600"
        s.C0,s.C1,s.C2,s.C3=C0,C1,C2,C3
        s.FG,s.GR,s.RD,s.OR,s.BL,s.YL=FG,GR,RD,OR,BL,YL
        s.root.configure(bg=C0)

        fT=font.Font(family="Segoe UI",size=11,weight="bold")
        fH=font.Font(family="Consolas",size=24,weight="bold")
        fV=font.Font(family="Consolas",size=15,weight="bold")
        fS=font.Font(family="Consolas",size=9)
        fM=font.Font(family="Segoe UI",size=10)

        # ---- TOP BAR ----
        top=tk.Frame(s.root,bg=C2,height=44);top.pack(fill="x");top.pack_propagate(False)
        s.lel=tk.Label(top,text="00:00",fg=GR,bg=C2,font=fH);s.lel.pack(side="left",padx=(12,20))
        s.lph=tk.Label(top,text="Initializing...",fg=FG,bg=C2,font=fV);s.lph.pack(side="left",padx=10)

        # Progress bar
        s.pbar_c=tk.Canvas(top,width=200,height=8,bg=C1,highlightthickness=0)
        s.pbar_c.pack(side="left",padx=(30,10),pady=18)
        s.pbar_r=s.pbar_c.create_rectangle(0,0,0,8,fill=GR,outline="")

        # Speed mode selector
        tk.Label(top,text="Speed:",fg="#888",bg=C2,font=fM).pack(side="right",padx=(10,2))
        s.spd_var=tk.StringVar(value="fast")
        for txt,val in[("Slow","slow"),("Med","medium"),("Fast","fast"),("Ext","extreme")]:
            tk.Radiobutton(top,text=txt,variable=s.spd_var,value=val,
                command=lambda v=val:s._set_speed(v),bg=C2,fg=FG,selectcolor=C3,font=fS,
                indicatoron=0,padx=6,pady=2,bd=0).pack(side="right",padx=1)

        tk.Label(top,text="|",fg="#555",bg=C2,font=fS).pack(side="right",padx=4)
        s.btn_rec=tk.Button(top,text="REC",command=s._toggle_rec,bg=C3,fg="#888",font=fS,
            relief="flat",padx=10,pady=3,bd=0);s.btn_rec.pack(side="right",padx=4)
        s.btn_next=tk.Button(top,text="Next >",command=s._next_pat,bg=C3,fg="#888",font=fS,
            relief="flat",padx=10,pady=3,bd=0);s.btn_next.pack(side="right",padx=2)
        s.btn_ctr=tk.Button(top,text="Center",command=s._center,bg=C3,fg="#888",font=fS,
            relief="flat",padx=10,pady=3,bd=0);s.btn_ctr.pack(side="right",padx=2)

        # ---- MAIN BODY ----
        body=tk.Frame(s.root,bg=C0);body.pack(fill="both",expand=True,padx=6,pady=4)

        # LEFT: Camera + Pattern Timeline
        left=tk.Frame(body,bg=C1);left.pack(side="left",fill="both",expand=True,padx=(0,3))

        # Camera
        camf=tk.Frame(left,bg=C1,bd=1,relief="solid",highlightbackground="#2a2a50")
        camf.pack(fill="both",expand=True,padx=3,pady=3)
        tk.Label(camf,text="CAMERA FEED",fg=BL,bg=C1,font=fT).pack(pady=(4,0))
        s.cam_lbl=tk.Label(camf,bg="#000",text="No Signal",fg="#555",font=fV)
        s.cam_lbl.pack(fill="both",expand=True,padx=3,pady=(0,4))

        # Pattern info below camera
        pinf=tk.Frame(left,bg=C1,bd=1,relief="solid",highlightbackground="#2a2a50")
        pinf.pack(fill="x",padx=3,pady=3)
        tk.Label(pinf,text="CURRENT PATTERN",fg=YL,bg=C1,font=fT).pack(pady=(6,0))
        s.lpat=tk.Label(pinf,text="---",fg=FG,bg=C1,font=fV,justify="left")
        s.lpat.pack(fill="x",padx=10,pady=(0,6))

        # Pattern timeline
        ptlf=tk.Frame(left,bg=C1,bd=1,relief="solid",highlightbackground="#2a2a50")
        ptlf.pack(fill="x",padx=3,pady=3)
        tk.Label(ptlf,text="PATTERN HISTORY",fg="#888",bg=C1,font=fT).pack(pady=(4,0))
        s.txp=tk.Text(ptlf,bg=C1,fg=FG,font=fS,height=6,bd=0,state="disabled",wrap="none")
        s.txp.pack(fill="x",padx=6,pady=(0,4))

        # RIGHT: Stats + Controls
        right=tk.Frame(body,bg=C0,width=420);right.pack(side="right",fill="y",padx=(3,0))
        right.pack_propagate(False)

        # Position cards
        for i,(ttl,key,clr)in enumerate([("YAW POSITION","y",GR),("PITCH POSITION","p",RD)]):
            f=tk.Frame(right,bg=C2,bd=1,relief="solid",highlightbackground="#2a2a50")
            f.pack(fill="x",padx=0,pady=2)
            tk.Label(f,text=ttl,fg=clr,bg=C2,font=fT).pack(pady=(6,0))
            lp=tk.Label(f,text="----",fg="#fff",bg=C2,font=fH);lp.pack()
            ld=tk.Label(f,text="--- deg",fg=FG,bg=C2,font=fV);ld.pack()
            ls=tk.Label(f,text="Spd: ---",fg="#888",bg=C2,font=fS);ls.pack(pady=(0,6))
            setattr(s,f"lp_{key}",lp);setattr(s,f"ld_{key}",ld);setattr(s,f"ls_{key}",ls)

        # Gauges grid (2x4)
        gf=tk.Frame(right,bg=C0);gf.pack(fill="x",pady=2)
        for i,(ttl,key,unit,clr)in enumerate([
            ("T1","t1","C",OR),("T2","t2","C",RD),("V1","v1","V",BL),("V2","v2","V","#4fc3f7"),
            ("LY","yl","","#ffd600"),("LP","pl","","#ff6d00"),("C1","c1","mA","#ffab40"),("C2","c2","mA",RD)]):
            r,c=i//4,i%4
            f=tk.Frame(gf,bg=C3,bd=1,relief="solid",highlightbackground="#2a2a50")
            f.grid(row=r,column=c,padx=1,pady=1,sticky="nsew")
            tk.Label(f,text=ttl,fg=clr,bg=C3,font=fS).pack()
            lb=tk.Label(f,text="---",fg="#fff",bg=C3,font=font.Font(family="Consolas",size=11,weight="bold"))
            lb.pack()
            tk.Label(f,text=unit,fg="#555",bg=C3,font=fS).pack(pady=(0,3))
            setattr(s,f"lb_{key}",lb)

        for i in range(4):gf.columnconfigure(i,weight=1)
        gf.rowconfigure(0,weight=1);gf.rowconfigure(1,weight=1)

        # Command
        cf=tk.Frame(right,bg=C2,bd=1,relief="solid",highlightbackground="#2a2a50")
        cf.pack(fill="x",pady=2)
        tk.Label(cf,text="COMMAND",fg=YL,bg=C2,font=fT).pack(pady=(6,0))
        s.lspd=tk.Label(cf,text="Speed: ---",fg="#fff",bg=C2,font=fV);s.lspd.pack()
        s.lacc=tk.Label(cf,text="Accel: ---",fg="#fff",bg=C2,font=fV);s.lacc.pack()
        s.lyt=tk.Label(cf,text="Y+Ptgt: ---/---",fg=FG,bg=C2,font=fS);s.lyt.pack(pady=(0,6))

        # Warnings
        wf=tk.Frame(right,bg=C1,bd=1,relief="solid",highlightbackground="#2a2a50")
        wf.pack(fill="both",expand=True,pady=2)
        tk.Label(wf,text="WARNINGS",fg=RD,bg=C1,font=fT).pack(pady=(4,0))
        s.txw=tk.Text(wf,bg=C1,fg="#ff8a80",font=fS,height=4,bd=0,state="disabled")
        s.txw.pack(fill="both",expand=True,padx=6,pady=(0,4))

        # Stats
        sf=tk.Frame(right,bg=C1,bd=1,relief="solid",highlightbackground="#2a2a50")
        sf.pack(fill="x",pady=2)
        tk.Label(sf,text="SUMMARY",fg=GR,bg=C1,font=fT).pack(pady=(4,0))
        s.lst=tk.Label(sf,text="",fg=FG,bg=C1,font=fS,justify="left",anchor="w")
        s.lst.pack(fill="x",padx=8,pady=(0,4))

        s.pat_hist=[];s._cam_tick=0
        s.root.after(100,s._upd)
        s.root.mainloop()

    def _set_speed(s,v):s.pg.speed_mode=v;s.pg._set_speed();s.m.warn(f"Speed: {v}")

    def _toggle_rec(s):
        if s.cam.rec:s.cam.stop_rec();s.btn_rec.config(fg="#888",bg=s.C3);s.m.warn("Rec OFF")
        else:s.cam.start_rec();s.btn_rec.config(fg="#fff",bg="#600");s.m.warn("Rec ON")

    def _next_pat(s):s.state["skip"]=True;s.m.warn("Skip pattern")

    def _center(s):s.g.center();s.m.warn("Centering")

    def _upd(s):
        try:
            s._upd_safe()
        except Exception as ex:
            print(f"[UI Error] {ex}")
            import traceback;traceback.print_exc()
        if s.state.get("running",True):s.root.after(100,s._upd)

    _upd_cnt = 0
    def _upd_safe(s):
        s._upd_cnt += 1
        if s._upd_cnt <= 5 or s._upd_cnt % 100 == 0:
            print(f"  [UI tick {s._upd_cnt}] pat={s.m.cur.get('pat','?')} running={s.state.get('running','?')}")
        if not s.state.get("running",True):return
        with s.m.l:c=dict(s.m.cur);w=list(s.m.warns)
        e=c.get("elapsed",0)or 0
        m=int(e/60);sec=int(e%60)
        s.lel.config(text=f"{m:02d}:{sec:02d}")

        # Progress bar
        pct=min(1.0,e/TEST_DUR)
        s.pbar_c.coords(s.pbar_r,0,0,int(pct*200),8)
        pbar_color=s.OR if pct>0.9 else s.GR if pct<0.8 else s.YL
        s.pbar_c.itemconfig(s.pbar_r,fill=pbar_color)

        s.lph.config(text=f"{c.get('pname','---')}")
        t1,t2=c.get("t1",0)or 0,c.get("t2",0)or 0
        if max(t1,t2)>HIGH_T:s.lph.config(fg=s.RD)

        # Positions
        for k in["y","p"]:
            p=c.get(f"{k}p");sp=c.get(f"{k}s")
            if hasattr(s,f"lp_{k}"):
                getattr(s,f"lp_{k}").config(text=str(int(p))if p and p>=0 else"---")
                getattr(s,f"ld_{k}").config(text=f"{(p or 0)*360/4096:.1f} deg")
                getattr(s,f"ls_{k}").config(text=f"Spd: {int(sp) if sp and sp>=0 else '---'}")

        # Mini gauges
        for key in["t1","t2","v1","v2","yl","pl","c1","c2"]:
            v=c.get(key)
            if hasattr(s,f"lb_{key}")and v is not None and v>-9000:
                fmt="{:.1f}"if key.startswith("v")else"{}"
                getattr(s,f"lb_{key}").config(text=fmt.format(v))

        s.lspd.config(text=f"Speed: {c.get('sp','---')}")
        s.lacc.config(text=f"Accel: {c.get('ac','---')}")
        s.lyt.config(text=f"Y/P tgt: {c.get('yt','---')}/{c.get('pt','---')}")

        # Current pattern info
        pat=c.get("pat",0)
        s.lpat.config(text=f"#{pat}  {s.pg.typ.upper()}\nf={s.pg.freq}Hz  Yamp={s.pg.amp_y}  Pamp={s.pg.amp_p}\nRemain: {int(c.get('rem',0))}s  Spd={s.pg.spd}  Acc={s.pg.acc}")

        # Pattern history
        if pat and (not s.pat_hist or s.pat_hist[-1][0]!=pat):
            s.pat_hist.append((pat,s.pg.typ,e))
            if len(s.pat_hist)>20:s.pat_hist=s.pat_hist[-20:]
        s.txp.config(state="normal");s.txp.delete("1.0","end")
        for pi,pt,pe in s.pat_hist[-18:]:
            prefix=">"if pi==pat else" "
            s.txp.insert("end",f"{prefix}#{pi:2d} [{int(pe/60):02d}:{int(pe%60):02d}] {pt}\n")
        s.txp.config(state="disabled");s.txp.see("end")

        # Warnings
        s.txw.config(state="normal");s.txw.delete("1.0","end")
        s.txw.insert("end","\n".join(w[-6:])if w else"OK")
        s.txw.config(state="disabled")

        # Stats
        fps=c.get("cfps",0)or 0
        s.lst.config(text=f"FPS:{fps:.0f} Err:{c.get('cerr',0)} Rec:{'ON'if s.cam.rec else'OFF'}\n"
            f"Cam:{s.cam.frames}fr MaxT:{s.m.mt}C MinV:{s.m.mv:.1f}V\n"
            f"Warns:{len(w)} Prog:{e/60:.0f}/{TEST_DUR//60}min")

        # Camera feed (every 5th tick to avoid UI freeze)
        s._cam_tick = (s._cam_tick + 1) % 5
        if s._cam_tick == 0 and s.cam.latest is not None:
            try:
                img=s.cam.latest;h,w=img.shape[:2]
                scale=min(750/w,450/h,1.0)
                nw,nh=int(w*scale),int(h*scale)
                img=Image.fromarray(img).resize((nw,nh),Image.LANCZOS)
                photo=ImageTk.PhotoImage(img)
                s.cam_lbl.config(image=photo,text="");s.cam_lbl.image=photo
            except:pass

def main():
    print("="*60)
    print("  HEAD MOVEMENT STRESS TEST v3")
    print(f"  Port:{PORT}  Duration:{TEST_DUR//60}min  Patterns:{len(HeadPatternGen.PATTERN_TYPES)} types")
    print("="*60)

    g=Gimbal()
    cam=Camera()
    m=Monitor(g,cam);pg=HeadPatternGen();sf=Safety(m);m.start()

    state={"running":True,"phase_t0":0,"skip":False}

    def mover():
        last_pat=-1
        while state["running"] and not sf.stop:
            e=time.time()-m.t0
            if TEST_DUR>0 and e>TEST_DUR:state["running"]=False;break
            st=sf.check(e)
            if st=="stop":state["running"]=False;break
            elif st=="paused":g.center();time.sleep(1);continue
            pt=e-state["phase_t0"]
            if state.get("skip"):state["skip"]=False;pt=99999
            temp=max(m.cur.get("t1",40)or 40,m.cur.get("t2",40)or 40)
            yt,pt2,sp,ac,name,rem=pg.update(pt,temp)
            if pg.idx!=last_pat:
                last_pat=pg.idx;state["phase_t0"]=e
                print(f"  #{pg.idx}: [{pg.typ}] f={pg.freq} ay={pg.amp_y} ap={pg.amp_p} sp={sp} ac={ac}")
            with m.l:m.cur.update({"yt":yt,"pt":pt2,"sp":sp,"ac":ac,"pname":name,"pat":pg.idx,"rem":int(rem),"elapsed":e})
            try:g.move(yt,pt2,sp,ac)
            except Exception as ex:m.warn(f"Move:{ex}");time.sleep(0.5)
            mc = state.get("move_cnt", 0)
            if mc % 300 == 0: print(f"  [MOVE {mc}] pos=({yt},{pt2}) sp={sp} pat={pg.idx}")
            state["move_cnt"] = mc + 1
            time.sleep(0.012)

    print("[MAIN] Starting movement thread...")
    th=threading.Thread(target=mover,daemon=True);th.start()
    print("[MAIN] Starting dashboard...")
    Dashboard(m,g,cam,pg,sf,state)
    state["running"]=False;th.join(timeout=2)
    cam.stop_rec();m.stop();cam.close();g.close()
    et=time.time()-m.t0 if m.t0 else 0
    print(f"\n{'='*60}\n  DONE {et/60:.1f}min MaxT:{m.mt}C MinV:{m.mv:.1f}V")
    print(f"  Cam:{cam.frames}fr/{cam.errs}err Pats:{pg.idx} Warns:{len(m.warns)}")
    print(f"{'='*60}")

if __name__=="__main__":main()
