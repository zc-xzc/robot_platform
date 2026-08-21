import sys, os, time
sys.path.insert(0, r"D:\westlake_Work\code\robot_platform\active_vision_dist")
from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
import tkinter as tk
from tkinter import font

PORT, BAUD = "COM8", 1000000

port = PortHandler(PORT); port.baudrate = BAUD
if not port.openPort():
    print("Cannot open " + PORT); sys.exit(1)
pkt = sms_sts(port); time.sleep(0.3)

root = tk.Tk()
root.title("Servo Position Monitor")
root.geometry("520x420")
root.configure(bg="#1a1a2e")

big_font = font.Font(family="Consolas", size=30, weight="bold")
mid_font = font.Font(family="Consolas", size=16, weight="bold")
label_font = font.Font(family="Microsoft YaHei", size=12)
small_font = font.Font(family="Consolas", size=11)

running = True

def on_close():
    global running
    running = False
    root.destroy()
root.protocol("WM_DELETE_WINDOW", on_close)

# ---- Data storage ----
class ServoData:
    def __init__(self):
        self.init_pos = None
        self.init_deg = None
        self.min_pos = 9999
        self.max_pos = -1
        self.min_deg = 999.0
        self.max_deg = -999.0

data = [ServoData(), ServoData()]  # index 0 = servo1, 1 = servo2

def create_servo_col(col, title_text, sid):
    idx = sid - 1
    r = 0
    tk.Label(root, text=title_text, fg="#e94560", bg="#1a1a2e", font=label_font).grid(row=r, column=col, padx=15, pady=(15,0)); r+=1

    # Raw position
    raw_lbl = tk.Label(root, text="---", fg="#ffffff", bg="#1a1a2e", font=big_font)
    raw_lbl.grid(row=r, column=col, padx=15); r+=1
    tk.Label(root, text="0-4096", fg="#888888", bg="#1a1a2e", font=small_font).grid(row=r, column=col); r+=1

    # Degrees
    deg_lbl = tk.Label(root, text="---", fg="#0f3460", bg="#16213e", font=big_font, width=7)
    deg_lbl.grid(row=r, column=col, padx=15); r+=1
    tk.Label(root, text="degrees", fg="#888888", bg="#1a1a2e", font=small_font).grid(row=r, column=col); r+=1

    # Separator
    tk.Label(root, text="", bg="#1a1a2e").grid(row=r, column=col); r+=1

    # Min / Init / Max row
    info_frame = tk.Frame(root, bg="#1a1a2e")
    info_frame.grid(row=r, column=col, padx=10, pady=(5,0)); r+=1

    tk.Label(info_frame, text="MIN", fg="#00ff88", bg="#1a1a2e", font=small_font).grid(row=0, column=0, padx=3)
    tk.Label(info_frame, text="INIT", fg="#ffcc00", bg="#1a1a2e", font=small_font).grid(row=0, column=1, padx=3)
    tk.Label(info_frame, text="MAX", fg="#ff6666", bg="#1a1a2e", font=small_font).grid(row=0, column=2, padx=3)

    min_lbl = tk.Label(info_frame, text="---", fg="#00ff88", bg="#1a1a2e", font=mid_font)
    min_lbl.grid(row=1, column=0, padx=3)
    init_lbl = tk.Label(info_frame, text="---", fg="#ffcc00", bg="#1a1a2e", font=mid_font)
    init_lbl.grid(row=1, column=1, padx=3)
    max_lbl = tk.Label(info_frame, text="---", fg="#ff6666", bg="#1a1a2e", font=mid_font)
    max_lbl.grid(row=1, column=2, padx=3)

    # Min/Init/Max degrees
    min_deg_lbl = tk.Label(info_frame, text="", fg="#00aa66", bg="#1a1a2e", font=small_font)
    min_deg_lbl.grid(row=2, column=0, padx=3)
    init_deg_lbl = tk.Label(info_frame, text="", fg="#cc9900", bg="#1a1a2e", font=small_font)
    init_deg_lbl.grid(row=2, column=1, padx=3)
    max_deg_lbl = tk.Label(info_frame, text="", fg="#cc4444", bg="#1a1a2e", font=small_font)
    max_deg_lbl.grid(row=2, column=2, padx=3)

    return raw_lbl, deg_lbl, min_lbl, init_lbl, max_lbl, min_deg_lbl, init_deg_lbl, max_deg_lbl

raw1, deg1, min1, init1, max1, min_deg1, init_deg1, max_deg1 = create_servo_col(0, "Servo 1 (Yaw / ID=1)", 1)
raw2, deg2, min2, init2, max2, min_deg2, init_deg2, max_deg2 = create_servo_col(1, "Servo 2 (Pitch / ID=2)", 2)

# Reset button
def reset_stats():
    for d in data:
        d.min_pos = 9999; d.max_pos = -1
        d.min_deg = 999.0; d.max_deg = -999.0

reset_btn = tk.Button(root, text="Reset Min/Max", command=reset_stats,
                      bg="#333355", fg="#cccccc", font=small_font, relief="flat", padx=10)
reset_btn.grid(row=10, column=0, columnspan=2, pady=(5,0))

status = tk.Label(root, text="Monitoring...", fg="#666666", bg="#1a1a2e", font=("Microsoft YaHei", 10))
status.grid(row=11, column=0, columnspan=2, pady=(5,10))

root.grid_columnconfigure(0, weight=1)
root.grid_columnconfigure(1, weight=1)

def update_display(raw_lbl, deg_lbl, min_lbl, init_lbl, max_lbl,
                   min_deg_lbl, init_deg_lbl, max_deg_lbl, pos, comm_ok, d_idx):
    d = data[d_idx]
    if comm_ok and pos is not None:
        deg = pos * 360.0 / 4096
        raw_lbl.config(text=str(pos))
        deg_lbl.config(text=f"{deg:.1f}")

        # Init
        if d.init_pos is None:
            d.init_pos = pos; d.init_deg = deg
            init_lbl.config(text=str(pos))
            init_deg_lbl.config(text=f"{deg:.1f}")

        # Min
        if pos < d.min_pos:
            d.min_pos = pos; d.min_deg = deg
            min_lbl.config(text=str(pos))
            min_deg_lbl.config(text=f"{deg:.1f}")

        # Max
        if pos > d.max_pos:
            d.max_pos = pos; d.max_deg = deg
            max_lbl.config(text=str(pos))
            max_deg_lbl.config(text=f"{deg:.1f}")

        # Always refresh init display
        if d.init_pos is not None:
            init_lbl.config(text=str(d.init_pos))
            init_deg_lbl.config(text=f"{d.init_deg:.1f}")
        if d.min_pos != 9999:
            min_lbl.config(text=str(d.min_pos))
            min_deg_lbl.config(text=f"{d.min_deg:.1f}")
        if d.max_pos != -1:
            max_lbl.config(text=str(d.max_pos))
            max_deg_lbl.config(text=f"{d.max_deg:.1f}")
    else:
        raw_lbl.config(text="ERR"); deg_lbl.config(text="ERR")

def update():
    if not running:
        port.closePort()
        return
    try:
        pos1, _, comm1, _ = pkt.ReadPosSpeed(1)
        pos2, _, comm2, _ = pkt.ReadPosSpeed(2)
        t = time.strftime("%H:%M:%S")

        update_display(raw1, deg1, min1, init1, max1, min_deg1, init_deg1, max_deg1,
                       pos1, comm1 == COMM_SUCCESS, 0)
        update_display(raw2, deg2, min2, init2, max2, min_deg2, init_deg2, max_deg2,
                       pos2, comm2 == COMM_SUCCESS, 1)

        status.config(text=f"Updated {t}  |  Reset clears min/max  |  Close to stop")
    except Exception as e:
        status.config(text=f"Error: {e}")
    root.after(80, update)

root.after(200, update)
root.mainloop()
port.closePort()
print("Monitor closed.")
