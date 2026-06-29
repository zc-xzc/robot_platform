#!/usr/bin/env python3
"""Active Vision — minimal tracking (STS3032 12V, tuned params)"""
import sys, os, time, numpy as np

# ---- fixed config ----
PORT = "COM5"          # change to COM5 / /dev/ttyUSB0 as needed
H_ID, V_ID = 1, 2
BAUD = 1000000
ACC = 80
KP, KD = 2.0, 0.5
JUMP = 0.08
CLIP = 0.12
DZ_DEG = 0.5
YAW_RANGE, PITCH_RANGE = 90.0, 60.0
H_FACTOR = int(YAW_RANGE / 360 * 4096)
V_FACTOR = int(PITCH_RANGE / 360 * 4096)
INV_YAW, INV_PITCH = True, True
H_OFF, V_OFF = 0.0, 0.0

# ---- SDK ----
SDK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.append(SDK)
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

# ---- quaternion math ----
def qmul(a, b):
    w1, x1, y1, z1 = a[3], a[0], a[1], a[2]
    w2, x2, y2, z2 = b[3], b[0], b[1], b[2]
    return np.array([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2])

def qconj(q): return np.array([-q[0], -q[1], -q[2], q[3]])

def qeuler(q):
    x, y, z, w = q
    yaw   = np.degrees(np.arctan2(2*(x*z+w*y), 1-2*(y*y+z*z)))
    pitch = np.degrees(np.arcsin(np.clip(-2*(y*z-w*x), -1, 1)))
    roll  = np.degrees(np.arctan2(2*(x*y+w*z), 1-2*(x*x+z*z)))
    return yaw, pitch, roll

# ---- gimbal ----
class Gimbal:
    def __init__(self):
        self.port = PortHandler(PORT); self.port.baudrate = BAUD
        if not self.port.openPort(): raise RuntimeError("Cannot open " + PORT)
        self.pkt = sms_sts(self.port); time.sleep(0.5)
        for sid in [H_ID, V_ID]:
            _, r, _ = self.pkt.ping(sid)
            if r != COMM_SUCCESS: raise RuntimeError("Servo %d no response" % sid)
            self.pkt.write1ByteTxRx(sid, 0x28, 1)  # torque on
        self.pkt.WritePosEx(H_ID, 2048, 500, ACC)
        self.pkt.WritePosEx(V_ID, 2048, 500, ACC)
        print("gimbal ready")

    def move(self, h, v):
        hp = max(0, min(4095, int(2048 + h * H_FACTOR)))
        vp = max(0, min(4095, int(2048 + v * V_FACTOR)))
        self.pkt.WritePosEx(H_ID, hp, 3500, ACC)
        self.pkt.WritePosEx(V_ID, vp, 3500, ACC)

    def center(self):
        self.pkt.WritePosEx(H_ID, 2048, 500, ACC)
        self.pkt.WritePosEx(V_ID, 2048, 500, ACC)

    def close(self):
        self.center(); time.sleep(0.3); self.port.closePort()

# ---- tracker ----
class Tracker:
    def __init__(self): self.q_off = np.array([0.,0.,0.,1.]); self.ok = False
    def calibrate(self, q_head, q_spine=None):
        self.q_off = qmul(qconj(q_spine), q_head) if q_spine is not None else q_head.copy()
        self.ok = True
    def update(self, q_head, q_spine=None):
        q_body = qmul(qconj(q_spine), q_head) if q_spine is not None else q_head
        q_rel = qmul(q_body, qconj(self.q_off)) if self.ok else q_body
        y, p, _ = qeuler(q_rel)
        if INV_YAW: y = -y
        if INV_PITCH: p = -p
        if abs(y) < DZ_DEG: y = 0
        if abs(p) < DZ_DEG: p = 0
        return (np.clip(y/YAW_RANGE+H_OFF, -1,1), np.clip(p/PITCH_RANGE+V_OFF, -1,1))

# ---- PD controller ----
class PD:
    def __init__(self):
        self.ch = 0.0; self.cv = 0.0; self.peh = 0.0; self.pev = 0.0
        self.pth = 0.0; self.ptv = 0.0

    def reset(self):
        self.ch = 0.0; self.cv = 0.0; self.peh = 0.0; self.pev = 0.0
        self.pth = 0.0; self.ptv = 0.0

    def update(self, th, tv):
        eh, ev = th - self.ch, tv - self.cv

        if abs(eh) > JUMP:
            self.ch = th - np.sign(eh) * JUMP * 0.5
            self.peh = th - self.ch
        else:
            oh = KP*eh + KD*(eh-self.peh) + (th-self.pth)*0.3
            self.ch += np.clip(oh, -CLIP, CLIP)
            self.peh = eh

        if abs(ev) > JUMP:
            self.cv = tv - np.sign(ev) * JUMP * 0.5
            self.pev = tv - self.cv
        else:
            ov = KP*ev + KD*(ev-self.pev) + (tv-self.ptv)*0.3
            self.cv += np.clip(ov, -CLIP, CLIP)
            self.pev = ev

        self.pth, self.ptv = th, tv
        return self.ch, self.cv

# ---- main ----
def main():
    gimbal = Gimbal()
    cam = None
    try:
        import pyrealsense2 as rs
        for w, h, fps in [(1280,720,30),(640,480,30),(424,240,30)]:
            try:
                p = rs.pipeline(); c = rs.config()
                c.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
                p.start(c); cam = p
                print("D415 %dx%d @ %dfps" % (w,h,fps)); break
            except: continue
    except ImportError: print("no D415 (pyrealsense2 not installed)")
    except Exception as e: print(f"no D415 ({e})")

    from pico_bridge import PicoBridge
    tracker = Tracker(); pd = PD()

    if cam:
        def push():
            while True:
                try:
                    f = cam.wait_for_frames(timeout_ms=2000)
                    img = np.asanyarray(f.get_color_frame().get_data())
                    pico.push_video_frame(img)
                except: pass
                time.sleep(0.04)
        import threading
        threading.Thread(target=push, daemon=True).start()

    with PicoBridge(video="frames" if cam else None) as pico:
        print("[READY] look forward...")
        cal = False; frames = 0; t0 = time.time(); wp = False

        while True:
            try:
                frame = pico.wait_frame(timeout=0.1)
            except TimeoutError:
                if not wp: print("[waiting] PicoBridge..."); wp = True
                continue
            wp = False

            if not cal:
                q_spine = None
                if frame.body.active and frame.body.joints.shape[0] > 3:
                    q_spine = frame.body.joints[3, 3:7]
                tracker.calibrate(frame.head.rotation, q_spine)
                pd.reset(); gimbal.center(); cal = True
                print("[CALIBRATED]\n"); continue

            frames += 1
            q_spine = None
            if frame.body.active and frame.body.joints.shape[0] > 3:
                q_spine = frame.body.joints[3, 3:7]

            th, tv = tracker.update(frame.head.rotation, q_spine)
            ph, pv = pd.update(th, tv)
            gimbal.move(ph, pv)

            if frames % 50 == 0:
                fps = frames / (time.time()-t0)
                print("[%5d] %.0f fps  H=%+.3f V=%+.3f" % (frames, fps, ph, pv))

    print("[DONE]")

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\n[EXIT]")
    except Exception as e: print("\n[ERROR] "+str(e)); import traceback; traceback.print_exc()
