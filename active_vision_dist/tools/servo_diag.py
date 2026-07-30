# -*- coding: utf-8 -*-
"""STS3032 Servo Diagnostic Tool
Reads all EEPROM registers + live values, compares against defaults/tuned values,
flags anomalies that could indicate damage.
"""
import sys, os, time
sys.path.insert(0, r"D:\westlake_Work\code\robot_platform\active_vision_dist")
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS

PORT, BAUD = "COM8", 1000000
SERVO_IDS = [1, 2]

# ---- STS3032 Register Map & Defaults ----
REGISTERS = {
    # (address, size, name, factory_default, tuned_value, category, description)
    (0x05, 1, "ID",           1,    None, "identity", "Servo ID, should not change"),
    (0x06, 1, "BaudRate",     0,    None, "identity", "0=1M, 1=0.5M, 2=250K..."),
    (0x09, 2, "MinAngle",     0,    None, "limits",   "Min angle limit, should stay 0"),
    (0x0B, 2, "MaxAngle",     4095, None, "limits",   "Max angle limit, should stay 4095"),
    (0x15, 1, "P_Gain",       32,   32,   "pid",      "Proportional gain. Factory=32"),
    (0x16, 1, "D_Gain",       32,   10,   "pid",      "Damping. Factory=32, Tuned=10 (softer)"),
    (0x17, 1, "I_Gain",       0,    0,    "pid",      "Integral gain. Should be 0"),
    (0x18, 1, "Punch",        32,   None, "pid",      "Minimum drive current. Factory=32"),
    (0x1A, 1, "CW_DeadZone",  0,    8,    "deadzone", "CW dead zone. Factory=0, Tuned=8"),
    (0x1B, 1, "CCW_DeadZone", 0,    8,    "deadzone", "CCW dead zone. Factory=0, Tuned=8"),
    (0x1F, 2, "Offset",       0,    None, "critical", "Zero offset. Changes = potentiometer/encoder shifted! MECHANICAL DAMAGE INDICATOR"),
    (0x21, 1, "Mode",         0,    0,    "control",  "0=position mode, 1=wheel mode"),
}

LIVE_VALUES = [
    # (address, size, name, unit, normal_range, description)
    (0x3E, 1, "Voltage",       "V",  (7.0, 14.0), "Input voltage. Low=power issue"),
    (0x3F, 1, "Temperature",   "C",  (10, 60),     "Temperature. High=friction/stress"),
    (0x3C, 2, "Load",          "raw", (-500, 500), "Load at rest. High=mechanical bind"),
    (0x45, 2, "Current",       "mA", (0, 2000),    "Current draw. High idle=motor issue"),
]

def read_register(pkt, sid, addr, size):
    """Read a register, return raw value or None on failure."""
    try:
        if size == 1:
            val, comm, err = pkt.read1ByteTxRx(sid, addr)
        elif size == 2:
            val, comm, err = pkt.read2ByteTxRx(sid, addr)
        else:
            return None
        if comm != COMM_SUCCESS:
            return None
        return val
    except:
        return None

def read_model(pkt, sid):
    """Read model number (addr 3-4, 2 bytes)."""
    try:
        val, comm, err = pkt.read2ByteTxRx(sid, 3)
        if comm == COMM_SUCCESS:
            return val
    except:
        pass
    return None

# ---- Connect ----
port = PortHandler(PORT)
port.baudrate = BAUD
if not port.openPort():
    print(f"[ERROR] Cannot open {PORT}")
    sys.exit(1)
pkt = sms_sts(port)
time.sleep(0.5)

# Ping & read model
for sid in SERVO_IDS:
    _, r, _ = pkt.ping(sid)
    if r != COMM_SUCCESS:
        print(f"[ERROR] Servo {sid} no response")
        port.closePort()
        sys.exit(1)

print("=" * 72)
print("  STS3032 Servo Diagnostic Report")
print("=" * 72)

for sid in SERVO_IDS:
    model = read_model(pkt, sid)
    model_str = f"0x{model:04X}" if model else "N/A"
    print(f"\n{'='*72}")
    print(f"  SERVO {sid}  |  Model: {model_str}")
    print(f"{'='*72}")

    # ---- EEPROM Registers ----
    print(f"\n  {'Register':<16} {'Addr':>5} {'Value':>8} {'Default':>8} {'Tuned':>8} {'Status':<12}  Notes")
    print(f"  {'-'*16} {'-'*5} {'-'*8} {'-'*8} {'-'*8} {'-'*12}  {'-'*30}")

    anomalies = []

    for addr, size, name, default, tuned, category, desc in REGISTERS:
        val = read_register(pkt, sid, addr, size)

        if val is None:
            status = "READ ERR!"
            anomalies.append((category, name, f"Cannot read register 0x{addr:02X}"))
        else:
            # Determine status
            reasons = []
            if tuned is not None and val == tuned:
                status = "OK (tuned)"
            elif val == default:
                status = "OK"
            else:
                # Value differs from both default and tuned
                if category == "critical":
                    status = "CHANGED!!"
                    reasons.append("Critical! Possible mechanical damage")
                elif category == "pid":
                    status = "MODIFIED"
                    reasons.append("PID was retuned")
                elif category == "identity":
                    status = "CHANGED!"
                    reasons.append("Should NOT change")
                elif category == "limits":
                    status = "CHANGED!"
                    reasons.append("Angle limits changed")
                else:
                    status = "MODIFIED"
                    reasons.append("Value differs from defaults")

            if reasons:
                anomalies.append((category, name, "; ".join(reasons)))

        def_str = str(default) if default is not None else "-"
        tun_str = str(tuned) if tuned is not None else "-"
        print(f"  {name:<16} 0x{addr:02X}  {val if val is not None else 'ERR':>8} {def_str:>8} {tun_str:>8} {status:<12}  {desc}")

    # ---- Live Values ----
    print(f"\n  {'Live Value':<16} {'Addr':>5} {'Value':>10} {'Normal':>18} {'Status':<12}  Notes")
    print(f"  {'-'*16} {'-'*5} {'-'*10} {'-'*18} {'-'*12}  {'-'*30}")

    for addr, size, name, unit, normal, desc in LIVE_VALUES:
        val = read_register(pkt, sid, addr, size)
        if val is None:
            print(f"  {name:<16} 0x{addr:02X}  {'ERR':>10} {'-':>18} {'READ ERR':<12}  {desc}")
            continue

        if name == "Voltage":
            display = f"{val/10:.1f}V"
            ok = normal[0] <= val/10 <= normal[1]
        elif name == "Temperature":
            display = f"{val}C"
            ok = normal[0] <= val <= normal[1]
        elif name == "Load":
            display = f"{val}"
            ok = normal[0] <= val <= normal[1]
        elif name == "Current":
            display = f"{val}mA"
            ok = normal[0] <= val <= normal[1]
        else:
            display = str(val)
            ok = True

        if ok:
            status = "OK"
        else:
            status = "WARNING!"
            anomalies.append(("live", name, f"Out of normal range: {display}"))

        normal_str = f"{normal[0]}-{normal[1]}{unit}"
        print(f"  {name:<16} 0x{addr:02X}  {display:>10} {normal_str:>18} {status:<12}  {desc}")

    # ---- Position ----
    pos, _, comm, _ = pkt.ReadPosSpeed(sid)
    if comm == COMM_SUCCESS and pos is not None:
        deg = pos * 360.0 / 4096
        print(f"  {'Position':<16} {'-':>5}  {pos:>5} raw   {'-':>18} {'-':<12}  {deg:.1f} deg (0-4095)")

    # ---- Summary ----
    if anomalies:
        print(f"\n  *** ANOMALIES DETECTED ({len(anomalies)}) ***")
        for cat, name, reason in anomalies:
            print(f"    [{cat}] {name}: {reason}")
    else:
        print(f"\n  *** No anomalies detected ***")

# ---- Cross-servo comparison ----
print(f"\n{'='*72}")
print(f"  Cross-Servo Comparison")
print(f"{'='*72}")
print(f"  {'Register':<16} {'Servo 1':>8} {'Servo 2':>8} {'Match?':>8}")
print(f"  {'-'*16} {'-'*8} {'-'*8} {'-'*8}")

mismatches = []
for addr, size, name, default, tuned, category, desc in REGISTERS:
    v1 = read_register(pkt, 1, addr, size)
    v2 = read_register(pkt, 2, addr, size)
    match = "YES" if v1 == v2 else "NO!"
    if v1 != v2:
        mismatches.append((name, v1, v2))
    print(f"  {name:<16} {str(v1) if v1 is not None else 'ERR':>8} {str(v2) if v2 is not None else 'ERR':>8} {match:>8}")

if mismatches:
    print(f"\n  Mismatches may indicate:")
    print(f"    - Different factory calibration (normal for Offset)")
    print(f"    - One servo was tuned, the other wasn't")
    print(f"    - EEPROM corruption in one servo")

# ---- Damage Indicators Summary ----
print(f"\n{'='*72}")
print(f"  Damage Indicator Summary")
print(f"{'='*72}")
print(f"""
  KEY INDICATORS OF MECHANICAL DAMAGE:
  ------------------------------------
  1. Offset (0x1F-0x20) CHANGED from factory:
     -> Potentiometer or magnetic encoder physically shifted
     -> Cause: strong impact, drop, or excessive torque
     -> Fix: Re-calibrate with Feetch software

  2. Load (0x3C-0x3D) HIGH at idle:
     -> Mechanical binding, bent shaft, gear damage
     -> Check: Can you rotate servo freely by hand (power off)?

  3. Temperature (0x3F) ABOVE 60C:
     -> Excessive friction, motor overstress, bearing failure

  4. Current (0x45-0x46) HIGH at idle (above ~100mA):
     -> Motor winding short, driver damage

  5. P/D/I values CHANGED unexpectedly:
     -> EEPROM corruption from power glitch
     -> Or intentional tuning (run.py --tune-servo)

  VALUES THAT SHOULD NEVER CHANGE:
  --------------------------------
  ID (0x05), BaudRate (0x06), MinAngle (0x09-0A), MaxAngle (0x0B-0C)
  If any of these changed -> EEPROM corruption or wrong servo configured
""")

port.closePort()
print("[DONE]")
