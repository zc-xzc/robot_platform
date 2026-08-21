#!/usr/bin/env python3
"""Auto-tune v3: Full pipeline test.
Simulates head turn -> PD controller -> servo command -> measure response.
Tests BOTH servos, 90 deg step, all (KP, KD, ACC) combos.
"""
import sys, os, time, json
import numpy as np

SDK = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "FTServo", "FTServo_Python-main")
sys.path.append(SDK)
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

PORT = "COM6"; BAUD = 1000000
CENTER = 2048
H_FACTOR = 1024  # 90 deg = 1024 steps
STEP_NORM = 0.5   # 45 deg step (half of max yaw range)
SPEED = 3500

class PD:
    def __init__(self, kp, kd):
        self.kp = kp; self.kd = kd
        self.pos = 0.0; self.pe = 0.0; self.pt = 0.0
    
    def update(self, target):
        err = target - self.pos
        out = self.kp * err + self.kd * (err - self.pe) + (target - self.pt) * 0.3
        self.pos += np.clip(out, -0.25, 0.25)
        self.pe = err; self.pt = target
        return self.pos

def test_full_pipeline(pkt, sid, kp, kd, acc):
    """Full pipeline: PD step response -> servo -> measure settling."""
    pd = PD(kp, kd)
    target = STEP_NORM  # 45 deg step
    
    # Center servo
    pkt.WritePosEx(sid, CENTER, 1000, 50)
    time.sleep(0.3)
    
    t0 = time.perf_counter()
    settle_time = None
    max_err = 0
    
    # Run PD loop at ~50Hz
    for frame in range(150):  # max 3 seconds
        pos_norm = pd.update(target)
        pos_raw = int(CENTER + pos_norm * H_FACTOR)
        pkt.WritePosEx(sid, pos_raw, SPEED, acc)
        
        # Read actual position
        actual, _, comm, _ = pkt.ReadPosSpeed(sid)
        if comm == COMM_SUCCESS:
            err = abs(actual - (CENTER + int(target * H_FACTOR)))
            if err > max_err:
                max_err = err
            
            if settle_time is None and err < 12:  # within ~1 deg
                settle_time = (time.perf_counter() - t0) * 1000
        
        time.sleep(0.018)  # ~50Hz
    
    if settle_time is None:
        settle_time = 3000
    
    overshoot = max(0, max_err - 12) * 360.0 / 4096
    return settle_time, overshoot

def main():
    port = PortHandler(PORT); port.baudrate = BAUD
    if not port.openPort(): print("ERROR: Cannot open " + PORT); return
    pkt = sms_sts(port); time.sleep(0.5)
    
    for sid in [1, 2]:
        _, r, _ = pkt.ping(sid)
        if r != COMM_SUCCESS:
            print("ERROR: Servo %d no response" % sid); port.closePort(); return
    print("Both servos OK\n")
    
    kp_list  = [1.0, 1.5, 2.0, 2.5, 3.0]
    kd_list  = [0.2, 0.4, 0.6, 0.8, 1.0]
    acc_list = [20, 35, 50, 80]
    
    total = len(kp_list) * len(kd_list) * len(acc_list)
    results = []; n = 0
    
    print(f"Testing {total} combos x 2 servos (PD + servo full pipeline, 45 deg step)")
    print()
    
    for acc in acc_list:
        for kp in kp_list:
            for kd in kd_list:
                n += 1
                s1, o1 = test_full_pipeline(pkt, 1, kp, kd, acc)
                s2, o2 = test_full_pipeline(pkt, 2, kp, kd, acc)
                
                s_avg = (s1 + s2) / 2; o_avg = (o1 + o2) / 2
                score = s_avg + o_avg * 100
                
                results.append({"kp":kp,"kd":kd,"acc":acc,"s_avg":round(s_avg,1),"o_avg":round(o_avg,2),"score":round(score,1)})
                
                bar = "="*(n*30//total) + "-"*(30-n*30//total)
                print(f"\r[{n:3d}/{total}] {bar} kp={kp:.1f} kd={kd:.2f} acc={acc} | {s_avg:.0f}ms {o_avg:.2f}deg", end="")
    
    print("\n")
    results.sort(key=lambda x: x["score"])
    
    print("=" * 65)
    print("RESULTS: Full pipeline (PD + servo), 45 deg step, both servos")
    print("=" * 65)
    print(f"{'#':<4} {'KP':<5} {'KD':<5} {'ACC':<5} {'Settle':<9} {'Over':<8} {'Score':<7}")
    print("-" * 65)
    for i, r in enumerate(results[:10]):
        print(f"{i+1:<4} {r['kp']:<5.1f} {r['kd']:<5.2f} {r['acc']:<5} {r['s_avg']:<9.0f} {r['o_avg']:<8.2f} {r['score']:<7.0f}")
    
    best = results[0]
    print(f"\n>>> BEST: python run.py --kp {best['kp']:.1f} --kd {best['kd']:.2f} --acc {best['acc']}")
    
    with open("autotune_results.json","w") as f: json.dump(results[:20], f, indent=2)
    for sid in [1,2]: pkt.WritePosEx(sid, CENTER, 1000, 50)
    time.sleep(0.5); port.closePort()

if __name__ == "__main__":
    main()
