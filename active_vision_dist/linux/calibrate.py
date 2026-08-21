#!/usr/bin/env python3
"""STS3032 gimbal calibration — scan limits, lock at 2048, save config.
Run once after hardware assembly/reassembly."""
import sys, os, time, json

PORT = "/dev/ttyUSB0"          # Linux: /dev/ttyUSB0
H_ID, V_ID = 1, 2
BAUD = 1000000
H_FACTOR = 1024        # 90 deg
V_FACTOR = 682         # 60 deg

SDK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.append(SDK)
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

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

def calibrate_one(pkt, sid, name):
    print("\n=== %s (ID=%d) ===" % (name, sid))
    pkt.WritePosEx(sid, 2048, 500, 200); time.sleep(1.0)
    print("   scanning + ..."); p_max = find_limit(pkt, sid, 2048, +1)
    pkt.WritePosEx(sid, 2048, 500, 200); time.sleep(1.0)
    print("   scanning - ..."); p_min = find_limit(pkt, sid, 2048, -1)
    pkt.WritePosEx(sid, 2048, 500, 200); time.sleep(1.0)
    mid = (p_max + p_min) // 2
    half = (p_max - p_min) // 2
    deg = 360.0 / 4096
    offset = mid - 2048
    print("   Limits: %d ~ %d  Mid: %d (off=%+d, %.1fdeg)  Half: %d (%.0fdeg)" % (
        p_min, p_max, mid, offset, offset*deg, half, half*deg))
    return half

def main():
    port = PortHandler(PORT); port.baudrate = BAUD
    if not port.openPort(): print("ERROR: cannot open " + PORT); return
    pkt = sms_sts(port); time.sleep(0.5)
    for sid in [H_ID, V_ID]:
        _, r, _ = pkt.ping(sid)
        if r != COMM_SUCCESS: print("Servo %d no response" % sid); port.closePort(); return
    print("Servos OK")

    h_half = calibrate_one(pkt, H_ID, "Horizontal")
    v_half = calibrate_one(pkt, V_ID, "Vertical")

    h_lim = round(min(h_half / H_FACTOR, 1.0), 3)
    v_lim = round(min(v_half / V_FACTOR, 1.0), 3)

    cfg = {"limit_h": h_lim, "limit_v": v_lim, "port": PORT, "h_id": H_ID, "v_id": V_ID}
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "gimbal_config.json"), "w") as f:
        json.dump(cfg, f, indent=2)

    print("\n" + "=" * 55)
    print("Servos locked at 2048. Config saved.")
    print(">>> NOW: align bracket so camera points FORWARD, tighten screws.")
    print("=" * 55)
    input("Press Enter to finish...")
    port.closePort()
    print("Done.")

if __name__ == "__main__":
    main()
