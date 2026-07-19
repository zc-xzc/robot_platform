import sys, os, time, threading, csv, math, random
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

PORT="COM8";BAUD=1000000;YAW_ID,PITCH_ID=1,2
YAW_CENTER,PITCH_CENTER=2110,2048
YAW_MIN,YAW_MAX=1217,3233;PITCH_MIN,PITCH_MAX=1162,2601
TEST_DURATION=5400;MAX_TEMP_C=75;HIGH_TEMP_C=65;MIN_VOLTAGE=7.0
MONITOR_INTERVAL=0.08
LOG_DIR=Path(__file__).resolve().parent/"stress_test_logs"

def _clip(v,lo,hi):return max(lo,min(hi,v))

class Gimbal:
    def __init__(s):
        s.port=PortHandler(PORT);s.port.baudrate=BAUD
        if not s.port.openPort():raise RuntimeError(f"Cannot open {PORT}")
        s.pkt=sms_sts(s.port);time.sleep(0.5)
        for sid in[YAW_ID,PITCH_ID]:
            _,r,_=s.pkt.ping(sid)
            if r!=COMM_SUCCESS:raise RuntimeError(f"Servo {sid} no response")
            s.pkt.write1ByteTxRx(sid,0x28,1)
        s.center();print("[GIMBAL] Ready")
    def move(s,yp,pp,sp=2000,ac=80):
        yp=int(_clip(yp,YAW_MIN,YAW_MAX));pp=int(_clip(pp,PITCH_MIN,PITCH_MAX))
        s.pkt.WritePosEx(YAW_ID,yp,sp,ac);s.pkt.WritePosEx(PITCH_ID,pp,sp,ac)
    def center(s):s.move(YAW_CENTER,PITCH_CENTER,500,50)
    def close(s):
        try:s.center();time.sleep(0.3)
        except:pass
        try:s.port.closePort()
        except:pass

class CameraMonitor:
    def __init__(s):
        s.enabled=False;s.pipeline=None;s.frames_total=0;s.frames_recent=0
        s.last_fps=0.0;s.errors=0;s.resolution="";s.frame_times=[];s.last_check=0
        try:
            import pyrealsense2 as rs
            for w,h,fps in[(1280,720,30),(640,480,30),(424,240,30)]:
                try:
                    p=rs.pipeline();c=rs.config()
                    c.enable_stream(rs.stream.color,w,h,rs.format.rgb8,fps)
                    p.start(c);s.pipeline=p;s.enabled=True
                    s.resolution=f"{w}x{h}@{fps}fps";s.last_check=time.time()
                    print(f"[CAMERA] {s.resolution}");break
                except:continue
        except ImportError:print("[CAMERA] pyrealsense2 not installed")
        except Exception as e:print(f"[CAMERA] Error: {e}")
    def update_stats(s):
        if not s.enabled or not s.pipeline:return
        now=time.time()
        try:
            frames=s.pipeline.wait_for_frames(timeout_ms=100)
            if frames:
                s.frames_total+=1;s.frames_recent+=1;s.frame_times.append(now)
                if len(s.frame_times)>50:s.frame_times=s.frame_times[-50:]
        except:s.errors+=1
        if now-s.last_check>1.0:
            s.last_fps=s.frames_recent/(now-s.last_check+0.001)
            s.frames_recent=0;s.last_check=now
    def close(s):
        if s.pipeline:
            try:s.pipeline.stop()
            except:pass

class Monitor:
    def __init__(s,gimbal,camera):
        s.gimbal=gimbal;s.camera=camera;s.running=False
        s.lock=threading.Lock();s.current={};s.history=[]
        s.csv_file=None;s.csv_writer=None;s.start_time=None
        s.warnings=[];s.max_temp=0;s.min_voltage=999;s.anomaly_count=0
    def start(s):
        LOG_DIR.mkdir(exist_ok=True)
        ts=datetime.now().strftime("%Y%m%d_%H%M%S")
        s.csv_file=open(LOG_DIR/f"stress_{ts}.csv","w",newline="",encoding="utf-8")
        s.csv_writer=csv.writer(s.csv_file)
        s.csv_writer.writerow(["timestamp","elapsed_s","phase","phase_name",
            "yaw_tgt","pitch_tgt","yaw_pos","pitch_pos","yaw_spd","pitch_spd",
            "yaw_load","pitch_load","v1","v2","t1","t2","cur1","cur2",
            "spd_cmd","acc_cmd","cam_fps","cam_errs","warnings"])
        s.start_time=time.time();s.running=True
        s.thread=threading.Thread(target=s._run,daemon=True);s.thread.start()
    def _read(s,sid,addr,size):
        try:
            val,comm,_=(s.gimbal.pkt.read1ByteTxRx(sid,addr)if size==1
                      else s.gimbal.pkt.read2ByteTxRx(sid,addr))
            return val if comm==COMM_SUCCESS else-9999
        except:return-9999
    def _run(s):
        while s.running:
            r={}
            for sid,name in[(YAW_ID,"yaw"),(PITCH_ID,"pitch")]:
                pos,speed,comm,_=s.gimbal.pkt.ReadPosSpeed(sid)
                ok=comm==COMM_SUCCESS
                r[f"{name}_pos"]=int(pos)if ok else-1
                r[f"{name}_speed"]=int(speed)if ok else-1
                r[f"{name}_load"]=s._read(sid,0x3C,2)
                r[f"v{sid}"]=s._read(sid,0x3E,1)/10.0
                r[f"t{sid}"]=s._read(sid,0x3F,1)
                r[f"cur{sid}"]=s._read(sid,0x45,2)
            s.camera.update_stats()
            r["cam_fps"]=round(s.camera.last_fps,1)
            r["cam_errs"]=s.camera.errors
            for sid in[1,2]:
                tv=r.get(f"t{sid}",0)
                if tv>s.max_temp:s.max_temp=tv
                vv=r.get(f"v{sid}",999)
                if vv<s.min_voltage:s.min_voltage=vv
            with s.lock:
                s.current=r;s.history.append(r)
                if len(s.history)>800:s.history=s.history[-800:]
            elapsed=time.time()-s.start_time if s.start_time else 0
            if s.csv_writer:
                s.csv_writer.writerow([datetime.now().isoformat(),f"{elapsed:.1f}",
                    s.current.get("phase",0),s.current.get("phase_name",""),
                    s.current.get("yaw_tgt",0),s.current.get("pitch_tgt",0),
                    r.get("yaw_pos",-1),r.get("pitch_pos",-1),
                    r.get("yaw_speed",-1),r.get("pitch_speed",-1),
                    r.get("yaw_load",-9999),r.get("pitch_load",-9999),
                    r.get("v1",-1),r.get("v2",-1),r.get("t1",-1),r.get("t2",-1),
                    r.get("cur1",-1),r.get("cur2",-1),
                    s.current.get("spd_cmd",0),s.current.get("acc_cmd",0),
                    r.get("cam_fps",0),r.get("cam_errs",0),
                    ";".join(s.warnings[-3:])if s.warnings else""])
            time.sleep(MONITOR_INTERVAL)
    def add_warning(s,msg):
        t=datetime.now().strftime("%H:%M:%S")
        s.warnings.append(f"[{t}] {msg}");s.anomaly_count+=1
    def stop(s):
        s.running=False
        if hasattr(s,"thread"):s.thread.join(timeout=2)
        if s.csv_file:s.csv_file.close()

PATTERNS=[(180,"Warm-up Slow Sine","warmup"),(300,"Yaw Speed Ramp 0.1->2Hz","speed"),
(300,"Pitch Speed Ramp 0.1->2Hz","speed"),(360,"Acceleration Stress 20->200","accel"),
(420,"Random Small Steps 5-20deg","random"),(420,"Random Large Steps 30-60deg","random"),
(360,"Direction Reversal Stress","reversal"),(420,"Figure-8 Medium Speed","coordinated"),
(300,"Figure-8 High Speed","speed"),(360,"Triangle Wave Yaw","const_vel"),
(300,"Triangle Wave Pitch","const_vel"),(240,"Micro-Jitter low amp","stability"),
(240,"Micro-Jitter high amp","stability"),(360,"Sine Sweep Expanding Amp","range"),
(360,"Zigzag Diagonal","coordinated"),(300,"Boundary Trace Full","range"),
(300,"Corner-to-Corner Stress","range"),(240,"Hold Yaw Min","static"),
(240,"Hold Yaw Max","static"),(240,"Hold Pitch Min","static"),
(240,"Hold Pitch Max","static"),(420,"Speed+Accel Combined Max","speed"),
(420,"Random Sine (variable freq)","random"),(600,"Full Envelope Random","random"),
(9999,"Mixed Extreme Endurance","endurance")]

class PatternGen:
    def __init__(s):s.phase=0;s._switched=False;random.seed(42)
    def update(s,phase_dur,temp=40):
        tf=max(0.3,1.0-(temp-HIGH_TEMP_C)/15.0)if temp>HIGH_TEMP_C else 1.0
        p=s.phase%len(PATTERNS);dur,name,cat=PATTERNS[p]
        sp,ac=2000,80;yt,pt=float(YAW_CENTER),float(PITCH_CENTER);t=phase_dur;sub=0
        if p==0:freq=0.2;yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.4*math.sin(2*math.pi*freq*t);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.3*math.cos(2*math.pi*freq*t);sp,ac=1000,40
        elif p==1:freq=0.1+(t/dur)*1.9;yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.6*math.sin(2*math.pi*freq*t);sp,ac=int(800+(t/dur)*2700),60
        elif p==2:freq=0.1+(t/dur)*1.9;pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.5*math.sin(2*math.pi*freq*t);sp,ac=int(800+(t/dur)*2700),60
        elif p==3:freq=0.5;yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.5*math.sin(2*math.pi*freq*t);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.4*math.cos(2*math.pi*1.3*freq*t);ac=int(20+(t/dur)*180);sp=2500
        elif p==4:
            if int(t*5)%5==0:yt=YAW_CENTER+random.uniform(-200,200);pt=PITCH_CENTER+random.uniform(-150,150)
            sp,ac=3000,100
        elif p==5:
            if int(t*3)%3==0:yt=random.uniform(YAW_MIN+100,YAW_MAX-100);pt=random.uniform(PITCH_MIN+100,PITCH_MAX-100)
            sp,ac=3500,140
        elif p==6:freq=1.2;yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.5*math.sin(2*math.pi*freq*t);frac=(t%(1/freq))/(1/freq);sp,ac=(3800,180)if(frac<0.02 or abs(frac-0.5)<0.02)else(2000,60)
        elif p==7:yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.55*math.sin(2*math.pi*t/5.0);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.4*math.sin(4*math.pi*t/5.0);sp,ac=2500,80
        elif p==8:yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.65*math.sin(2*math.pi*t/2.5);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.45*math.sin(4*math.pi*t/2.5);sp,ac=3800,140
        elif p==9:per=4.0;frac=(t%per)/per;tri=4*frac-1 if frac<0.5 else 3-4*frac;yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.6*tri;sp,ac=1800,50
        elif p==10:per=3.5;frac=(t%per)/per;tri=4*frac-1 if frac<0.5 else 3-4*frac;pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.5*tri;sp,ac=1600,50
        elif p==11:j=3+2*math.sin(2*math.pi*t/1.5);yt=YAW_CENTER+j*math.sin(2*math.pi*t*8.0);pt=PITCH_CENTER+j*math.cos(2*math.pi*t*8.5);sp,ac=3500,200
        elif p==12:j=10+6*math.sin(2*math.pi*t/1.0);yt=YAW_CENTER+j*math.sin(2*math.pi*t*6.0);pt=PITCH_CENTER+j*math.cos(2*math.pi*t*6.3);sp,ac=3800,200
        elif p==13:freq=0.3;amp=0.1+(t/dur)*0.7;yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*amp*math.sin(2*math.pi*freq*t);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*amp*math.cos(2*math.pi*freq*1.3*t);sp,ac=2200,90
        elif p==14:per=1.5;frac=(t%per)/per
        if p==14:
            if frac<0.33:yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.4*(frac/0.33);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.4*(frac/0.33)
            elif frac<0.66:yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.4*(1-(frac-0.33)/0.33);pt=PITCH_CENTER
            else:yt=YAW_CENTER;pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.4*((frac-0.66)/0.34)
            sp,ac=2800,120
        elif p==15:frac=(t%dur)/dur
        if p==15:
            if frac<0.25:yt=YAW_MIN+(YAW_MAX-YAW_MIN)*frac*4;pt=PITCH_MAX-100
            elif frac<0.5:yt=YAW_MAX-100;pt=PITCH_MAX-(PITCH_MAX-PITCH_MIN)*(frac-0.25)*4
            elif frac<0.75:yt=YAW_MAX-(YAW_MAX-YAW_MIN)*(frac-0.5)*4;pt=PITCH_MIN+100
            else:yt=YAW_MIN+100;pt=PITCH_MIN+(PITCH_MAX-PITCH_MIN)*(frac-0.75)*4
            sp,ac=2000,70
        elif p==16:corners=[(YAW_MIN+80,PITCH_MIN+80),(YAW_MAX-80,PITCH_MIN+80),(YAW_MAX-80,PITCH_MAX-80),(YAW_MIN+80,PITCH_MAX-80)];idx=int(t//4)%4;ni=(idx+1)%4;frac=(t%4)/4;cy,cp=corners[idx];ny,np=corners[ni];yt=cy+(ny-cy)*frac;pt=cp+(np-cp)*frac;sp,ac=2800,130
        elif p==17:yt,pt=YAW_MIN+80,PITCH_CENTER;sp,ac=500,50
        elif p==18:yt,pt=YAW_MAX-80,PITCH_CENTER;sp,ac=500,50
        elif p==19:yt,pt=YAW_CENTER,PITCH_MIN+80;sp,ac=500,50
        elif p==20:yt,pt=YAW_CENTER,PITCH_MAX-80;sp,ac=500,50
        elif p==21:yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.7*math.sin(2*math.pi*t/2.0);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.5*math.cos(2*math.pi*t/1.7);sp,ac=3800,200
        elif p==22:fy=0.3+1.7*abs(math.sin(2*math.pi*t/30.0));fp=0.3+1.7*abs(math.cos(2*math.pi*t/25.0));yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.6*math.sin(2*math.pi*fy*t);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.5*math.sin(2*math.pi*fp*t);sp,ac=3000,120
        elif p==23:
            if int(t*4)%4==0:yt=random.uniform(YAW_MIN+80,YAW_MAX-80);pt=random.uniform(PITCH_MIN+80,PITCH_MAX-80)
            if int(t*8)%8==0:sp=random.randint(500,3800);ac=random.randint(20,200)
        else:sub=int(t//120)%6
        if p>=24:
            if sub==0:yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.7*math.sin(2*math.pi*t/2.0);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.5*math.cos(2*math.pi*t/1.7);sp,ac=3500,150
            elif sub==1:
                if int(t*2)%2==0:yt=random.uniform(YAW_MIN+80,YAW_MAX-80)
                if int(t*2)%2==0:pt=random.uniform(PITCH_MIN+80,PITCH_MAX-80)
                sp,ac=3500,140
            elif sub==2:yt,pt=YAW_CENTER,PITCH_MAX-100;sp,ac=400,40
            elif sub==3:yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.6*math.sin(2*math.pi*t/2.5);pt=PITCH_CENTER+(PITCH_MAX-PITCH_CENTER)*0.4*math.sin(4*math.pi*t/2.5);sp,ac=3500,140
            elif sub==4:j=8+4*math.sin(2*math.pi*t/2.0);yt=YAW_CENTER+j*math.sin(2*math.pi*t*7.0);pt=PITCH_CENTER+j*math.cos(2*math.pi*t*7.3);sp,ac=3800,180
            else:per=4.0;frac=(t%per)/per;tri=4*frac-1 if frac<0.5 else 3-4*frac;yt=YAW_CENTER+(YAW_MAX-YAW_CENTER)*0.6*tri;sp,ac=2000,60
        sp,ac=int(sp*tf),int(ac*tf)
        if phase_dur>dur:s._next()
        return int(yt),int(pt),sp,ac,f"[{p}] {name}"
    def _next(s):
        if s._switched:return
        s._switched=True;s.phase+=1

class Safety:
    def __init__(s,monitor):s.monitor=monitor;s.paused=False;s.stop=False
    def check(s,elapsed):
        c=s.monitor.current
        for sid in[1,2]:
            v=c.get(f"v{sid}",12)
            if v and v<MIN_VOLTAGE:s.monitor.add_warning(f"STOP: S{sid} volt {v:.1f}V");s.stop=True;return"stop"
            t=c.get(f"t{sid}",40)
            if t and t>MAX_TEMP_C:
                if not s.paused:s.monitor.add_warning(f"OVERHEAT: S{sid} {t}C. Pause");s.paused=True
                return"paused"
        if s.paused:
            ok=all((c.get(f"t{sid}",0)or 0)<HIGH_TEMP_C-5 for sid in[1,2])
            if ok:s.paused=False;s.monitor.add_warning("Resume")
        return"paused"if s.paused else"ok"

import tkinter as tk
from tkinter import font

class Dashboard:
    def __init__(s,monitor,gimbal,camera):
        s.monitor=monitor;s.gimbal=gimbal;s.camera=camera
        s.root=tk.Tk();s.root.title("STS3032 + D415 Stress Test")
        s.root.geometry("1280x800");s.root.configure(bg="#0a0a14")
        s.root.protocol("WM_DELETE_WINDOW",s._close)
        s.bg="#0a0a14";s.card_bg="#12122a";s.fg="#cccccc";s.accent="#00ff88"
        s.warn="#ff4444";s.highlight="#ff9900"
        s.f_big=font.Font(family="Consolas",size=28,weight="bold")
        s.f_val=font.Font(family="Consolas",size=18,weight="bold")
        s.f_title=font.Font(family="Consolas",size=11,weight="bold")
        s.f_small=font.Font(family="Consolas",size=9)
        s.f_label=font.Font(family="Microsoft YaHei",size=10)
        s._build();s._update()
    def _card(s,parent,row,col,rs=1,cs=1):
        f=tk.Frame(parent,bg=s.card_bg,bd=1,relief="solid",highlightbackground="#1a1a40",highlightthickness=1)
        f.grid(row=row,column=col,rowspan=rs,columnspan=cs,padx=4,pady=4,sticky="nsew");return f
    def _build(s):
        top=tk.Frame(s.root,bg=s.bg,height=45);top.pack(fill="x",padx=8,pady=(6,0));top.pack_propagate(False)
        s.lbl_elapsed=tk.Label(top,text="0.0 min",fg=s.accent,bg=s.bg,font=s.f_big);s.lbl_elapsed.pack(side="left",padx=(8,20))
        s.lbl_phase=tk.Label(top,text="Phase: --",fg=s.fg,bg=s.bg,font=s.f_val);s.lbl_phase.pack(side="left",padx=10)
        s.lbl_warn_top=tk.Label(top,text="",fg=s.warn,bg=s.bg,font=s.f_title);s.lbl_warn_top.pack(side="right",padx=10)
        main=tk.Frame(s.root,bg=s.bg);main.pack(fill="both",expand=True,padx=8,pady=4)
        for i in range(4):main.columnconfigure(i,weight=1)
        for i in range(4):main.rowconfigure(i,weight=1)
        c0=s._card(main,0,0);s._pos_gauge(c0,"YAW","yaw")
        c1=s._card(main,0,1);s._pos_gauge(c1,"PITCH","pitch")
        c2=s._card(main,0,2);s._cam_panel(c2)
        c3=s._card(main,0,3);s._phase_list(c3)
        c4=s._card(main,1,0);s._gauge(c4,"Temp S1","t1","C",85,s.highlight)
        c5=s._card(main,1,1);s._gauge(c5,"Temp S2","t2","C",85,s.warn)
        c6=s._card(main,1,2);s._gauge(c6,"Voltage S1","v1","V",15,"#00ccff")
        c7=s._card(main,1,3);s._gauge(c7,"Voltage S2","v2","V",15,"#0099ff")
        c8=s._card(main,2,0);s._gauge(c8,"Load Yaw","yaw_load","",1500,"#ffcc00")
        c9=s._card(main,2,1);s._gauge(c9,"Load Pitch","pitch_load","",1500,"#ff6600")
        c10=s._card(main,2,2);s._gauge(c10,"Current S1","cur1","mA",3000,"#ff9900")
        c11=s._card(main,2,3);s._gauge(c11,"Current S2","cur2","mA",3000,"#ff4444")
        c12=s._card(main,3,0);s._cmd_panel(c12)
        c13=s._card(main,3,1,cs=2);s._warn_panel(c13)
        c14=s._card(main,3,3);s._stats_panel(c14)
    def _pos_gauge(s,parent,title,key):
        tk.Label(parent,text=title,fg=s.accent,bg=s.card_bg,font=s.f_title).pack(pady=(8,0))
        lp=tk.Label(parent,text="----",fg="#ffffff",bg=s.card_bg,font=s.f_big);lp.pack()
        tk.Label(parent,text="0-4096",fg="#555",bg=s.card_bg,font=s.f_small).pack()
        ld=tk.Label(parent,text="---",fg=s.fg,bg=s.card_bg,font=font.Font(family="Consolas",size=14));ld.pack()
        sf=tk.Frame(parent,bg=s.card_bg);sf.pack(pady=(8,2))
        tk.Label(sf,text="Spd:",fg="#888",bg=s.card_bg,font=s.f_small).pack(side="left",padx=3)
        ls=tk.Label(sf,text="---",fg=s.fg,bg=s.card_bg,font=s.f_small);ls.pack(side="left")
        setattr(s,f"lbl_{key}_pos",lp);setattr(s,f"lbl_{key}_deg",ld);setattr(s,f"lbl_{key}_spd",ls)
    def _gauge(s,parent,title,key,unit,max_val,color):
        tk.Label(parent,text=title,fg=color,bg=s.card_bg,font=s.f_title).pack(pady=(8,2))
        lbl=tk.Label(parent,text="---",fg="#ffffff",bg=s.card_bg,font=s.f_big);lbl.pack()
        tk.Label(parent,text=unit,fg="#555",bg=s.card_bg,font=s.f_small).pack()
        canvas=tk.Canvas(parent,width=180,height=12,bg="#1a1a30",highlightthickness=0);canvas.pack(pady=(4,8))
        bar=canvas.create_rectangle(0,0,0,12,fill=color,outline="")
        setattr(s,f"lbl_{key}",lbl);setattr(s,f"bar_{key}",(canvas,bar,max_val,color))
    def _cam_panel(s,parent):
        tk.Label(parent,text="CAMERA",fg="#00ccff",bg=s.card_bg,font=s.f_title).pack(pady=(8,2))
        s.lbl_cam_fps=tk.Label(parent,text="FPS: ---",fg="#ffffff",bg=s.card_bg,font=s.f_val);s.lbl_cam_fps.pack()
        s.lbl_cam_res=tk.Label(parent,text="",fg="#888",bg=s.card_bg,font=s.f_small);s.lbl_cam_res.pack()
        s.lbl_cam_err=tk.Label(parent,text="Errors: 0",fg=s.warn,bg=s.card_bg,font=s.f_small);s.lbl_cam_err.pack(pady=(10,0))
        s.lbl_cam_total=tk.Label(parent,text="Total: 0",fg=s.fg,bg=s.card_bg,font=s.f_small);s.lbl_cam_total.pack()
    def _phase_list(s,parent):
        tk.Label(parent,text="SCHEDULE",fg=s.highlight,bg=s.card_bg,font=s.f_title).pack(pady=(8,2))
        s.txt_phases=tk.Text(parent,bg=s.card_bg,fg=s.fg,font=s.f_small,height=12,width=28,bd=0,state="disabled",wrap="none")
        s.txt_phases.pack(fill="both",expand=True,padx=6,pady=(0,6))
        s.txt_phases.config(state="normal")
        for i,(dur,name,cat)in enumerate(PATTERNS):s.txt_phases.insert("end",f"{i:2d}. [{dur//60:2d}m] {name}\n")
        s.txt_phases.config(state="disabled")
    def _cmd_panel(s,parent):
        tk.Label(parent,text="COMMAND",fg="#ffcc00",bg=s.card_bg,font=s.f_title).pack(pady=(8,2))
        f1=tk.Frame(parent,bg=s.card_bg);f1.pack()
        tk.Label(f1,text="Speed:",fg="#888",bg=s.card_bg,font=s.f_small).pack(side="left",padx=3)
        s.lbl_spd=tk.Label(f1,text="---",fg="#ffffff",bg=s.card_bg,font=s.f_val);s.lbl_spd.pack(side="left")
        f2=tk.Frame(parent,bg=s.card_bg);f2.pack()
        tk.Label(f2,text="Accel:",fg="#888",bg=s.card_bg,font=s.f_small).pack(side="left",padx=3)
        s.lbl_acc=tk.Label(f2,text="---",fg="#ffffff",bg=s.card_bg,font=s.f_val);s.lbl_acc.pack(side="left")
        f3=tk.Frame(parent,bg=s.card_bg);f3.pack(pady=(8,0))
        tk.Label(f3,text="Y Tgt:",fg="#00cc66",bg=s.card_bg,font=s.f_small).pack(side="left",padx=3)
        s.lbl_yt=tk.Label(f3,text="---",fg=s.fg,bg=s.card_bg,font=s.f_small);s.lbl_yt.pack(side="left")
        f4=tk.Frame(parent,bg=s.card_bg);f4.pack()
        tk.Label(f4,text="P Tgt:",fg="#ff9999",bg=s.card_bg,font=s.f_small).pack(side="left",padx=3)
        s.lbl_pt=tk.Label(f4,text="---",fg=s.fg,bg=s.card_bg,font=s.f_small);s.lbl_pt.pack(side="left")
    def _warn_panel(s,parent):
        tk.Label(parent,text="WARNINGS & LOG",fg=s.warn,bg=s.card_bg,font=s.f_title).pack(pady=(4,2))
        s.txt_warn=tk.Text(parent,bg=s.card_bg,fg="#ff6666",font=s.f_small,height=5,bd=0,state="disabled")
        s.txt_warn.pack(fill="both",expand=True,padx=6,pady=(0,4))
    def _stats_panel(s,parent):
        tk.Label(parent,text="STATISTICS",fg=s.accent,bg=s.card_bg,font=s.f_title).pack(pady=(8,2))
        s.lbl_stats=tk.Label(parent,text="",fg=s.fg,bg=s.card_bg,font=s.f_small,justify="left",anchor="w")
        s.lbl_stats.pack(fill="both",expand=True,padx=8,pady=(0,4))
    def _update_bar(s,key,value):
        if not hasattr(s,f"bar_{key}"):return
        canvas,bar,max_val,color=getattr(s,f"bar_{key}")
        pct=min(1.0,max(0.0,(value or 0)/max(max_val,1)))
        canvas.coords(bar,0,0,int(pct*180),12)
        if value and "t" in key:
            if value>MAX_TEMP_C:canvas.itemconfig(bar,fill=s.warn)
            elif value>HIGH_TEMP_C:canvas.itemconfig(bar,fill=s.highlight)
            else:canvas.itemconfig(bar,fill=color)
    def _update(s):
        with s.monitor.lock:c=dict(s.monitor.current);warns=list(s.monitor.warnings)
        elapsed=c.get("elapsed",0)or 0
        s.lbl_elapsed.config(text=f"{elapsed/60:.1f} min")
        s.lbl_phase.config(text=f"Phase {c.get('phase_name','--')}")
        for key in["yaw","pitch"]:
            pos=c.get(f"{key}_pos");spd=c.get(f"{key}_speed")
            if hasattr(s,f"lbl_{key}_pos"):
                getattr(s,f"lbl_{key}_pos").config(text=str(int(pos))if pos else"---")
                getattr(s,f"lbl_{key}_deg").config(text=f"{(pos or 0)*360/4096:.1f} deg")
                getattr(s,f"lbl_{key}_spd").config(text=str(int(spd))if spd else"---")
        for key in["t1","t2","v1","v2","yaw_load","pitch_load","cur1","cur2"]:
            val=c.get(key)
            if hasattr(s,f"lbl_{key}"):
                if val is not None and val>-9000:
                    fmt="{:.1f}"if key.startswith("v")else"{}"
                    getattr(s,f"lbl_{key}").config(text=fmt.format(val))
            s._update_bar(key,val)
        s.lbl_spd.config(text=str(c.get("spd_cmd","---")))
        s.lbl_acc.config(text=str(c.get("acc_cmd","---")))
        s.lbl_yt.config(text=str(c.get("yaw_tgt","---")))
        s.lbl_pt.config(text=str(c.get("pitch_tgt","---")))
        s.lbl_cam_fps.config(text=f"FPS: {c.get('cam_fps',0):.1f}",fg=s.accent if(c.get('cam_fps',0)or 0)>15 else s.warn)
        s.lbl_cam_res.config(text=s.camera.resolution)
        s.lbl_cam_err.config(text=f"Errors: {c.get('cam_errs',0)}")
        s.lbl_cam_total.config(text=f"Total: {s.camera.frames_total}")
        cur_p=c.get("phase",0)%len(PATTERNS)
        s.txt_phases.config(state="normal");s.txt_phases.delete("1.0","end")
        for i,(dur,name,cat)in enumerate(PATTERNS):
            prefix=">>"if i==cur_p else"  "
            s.txt_phases.insert("end",f"{prefix}{i:2d}. [{dur//60:2d}m] {name}\n")
            if i==cur_p:s.txt_phases.tag_add(f"p{i}",f"{i+1}.0",f"{i+1}.end");s.txt_phases.tag_config(f"p{i}",foreground=s.accent,background="#1a1a40")
        s.txt_phases.config(state="disabled")
        s.txt_warn.config(state="normal");s.txt_warn.delete("1.0","end")
        s.txt_warn.insert("end","\n".join(warns[-10:])if warns else"(none)")
        s.txt_warn.config(state="disabled")
        t1,t2=c.get("t1",0)or 0,c.get("t2",0)or 0
        s.lbl_warn_top.config(text=f"HOT! {max(t1,t2)}C"if max(t1,t2)>HIGH_TEMP_C else"")
        em=elapsed/60
        stats=(f"Max Temp: {s.monitor.max_temp}C\nMin Volt: {s.monitor.min_voltage:.1f}V\n"
               f"Warnings: {len(warns)}\nProgress: {em:.0f}/{TEST_DURATION/60:.0f} min\n"
               f"Cam Frames: {s.camera.frames_total}\nCam Errs: {s.camera.errors}")
        s.lbl_stats.config(text=stats)
        s.root.after(250,s._update)
    def _close(s):s.root.quit();s.root.destroy()
    def run(s):s.root.mainloop()

def movement_loop(gimbal,monitor,camera,patterns,safety,state):
    last_phase=-1
    while state["running"]and not safety.stop:
        elapsed=time.time()-monitor.start_time
        if TEST_DURATION>0 and elapsed>TEST_DURATION:state["running"]=False;break
        status=safety.check(elapsed)
        if status=="stop":state["running"]=False;break
        elif status=="paused":
            gimbal.center()
            with monitor.lock:monitor.current["phase_name"]="PAUSED";monitor.current["spd_cmd"]=0;monitor.current["acc_cmd"]=0
            time.sleep(1);continue
        if patterns.phase!=last_phase:
            last_phase=patterns.phase;patterns._switched=False;state["phase_start"]=elapsed
            dur,name,cat=PATTERNS[patterns.phase%len(PATTERNS)]
            print(f"\n>>> Phase {patterns.phase%len(PATTERNS)}: {name} ({dur}s)\n")
        phase_dur=elapsed-state.get("phase_start",elapsed)
        temp=max(monitor.current.get("t1",40)or 40,monitor.current.get("t2",40)or 40)
        yt,pt,sp,ac,name=patterns.update(phase_dur,temp)
        with monitor.lock:
            monitor.current["yaw_tgt"]=yt;monitor.current["pitch_tgt"]=pt
            monitor.current["spd_cmd"]=sp;monitor.current["acc_cmd"]=ac
            monitor.current["phase_name"]=name
            monitor.current["phase"]=patterns.phase%len(PATTERNS)
            monitor.current["elapsed"]=elapsed
        try:gimbal.move(yt,pt,sp,ac)
        except Exception as e:monitor.add_warning(f"Move: {e}")
        time.sleep(0.012)

def main():
    print("="*65)
    print("  STS3032 + D415 COMPREHENSIVE STRESS TEST")
    print(f"  Duration: {TEST_DURATION}s ({TEST_DURATION/60:.0f} min)")
    print(f"  Patterns: {len(PATTERNS)} phases  Port: {PORT}")
    print("="*65)
    gimbal=Gimbal()
    camera=CameraMonitor()
    monitor=Monitor(gimbal,camera)
    patterns=PatternGen()
    safety=Safety(monitor)
    monitor.start()
    state={"running":True,"phase_start":0}
    mthread=threading.Thread(target=movement_loop,args=(gimbal,monitor,camera,patterns,safety,state),daemon=True)
    mthread.start()
    dashboard=Dashboard(monitor,gimbal,camera)
    dashboard.run()
    state["running"]=False;mthread.join(timeout=2)
    monitor.stop();camera.close();gimbal.close()
    elapsed=time.time()-monitor.start_time if monitor.start_time else 0
    print(f"\n{'='*65}\n  TEST COMPLETE  Time:{elapsed/60:.1f} min")
    print(f"  Max Temp:{monitor.max_temp}C  Min Volt:{monitor.min_voltage:.1f}V")
    print(f"  Cam:{camera.frames_total} frames, {camera.errors} errors")
    print(f"  Warnings:{len(monitor.warnings)}\n{'='*65}")

if __name__=="__main__":main()
