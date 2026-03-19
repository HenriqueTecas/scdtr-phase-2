"""
SCDTR Luminaire Controller - Live Tuning Plot
=============================================
Type commands in the box at the bottom of the window and press Enter.

Same commands as Arduino Serial Monitor:
    r 1 20        set reference to 20 LUX
    f 1 1         enable feedback
    f 1 0         disable feedback
    c p 1 0.8     set kp = 0.8
    c i 1 0.05    set ki = 0.05
    c B 1 0.5     set setpoint weight b
    a 1 2         set anti-windup mode (0/1/2)
    s y 1         start LUX stream
    S y 1         stop LUX stream
    g E 1         get energy metric
    g V 1         get visibility error
    g F 1         get flicker metric
    clear         clear plot history

Usage:
    python live_plot.py [PORT]
    python live_plot.py COM3
    python live_plot.py /dev/ttyACM0
"""

import sys
import time
import threading
import re
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import TextBox, Button
from matplotlib.animation import FuncAnimation
from collections import deque
import serial
import serial.tools.list_ports

# ── Configuration ──────────────────────────────────────────────────────────────
LUMINAIRE  = None   # auto-detected from "THIS NODE = X" boot message
BAUD_RATE  = 115200
HISTORY_S  = 30
MAX_POINTS = HISTORY_S * 100
UPDATE_MS  = 150

# ── Auto-detect port ───────────────────────────────────────────────────────────
def find_pico_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc or k in hwid for k in
               ["pico", "rp2040", "2e8a", "usbmodem", "ttyacm"]):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None

# ── Shared state ───────────────────────────────────────────────────────────────
lock      = threading.Lock()
times     = deque(maxlen=MAX_POINTS)
lux_meas  = deque(maxlen=MAX_POINTS)
lux_ref   = deque(maxlen=MAX_POINTS)
duty_vals = deque(maxlen=MAX_POINTS)
log_lines = deque(maxlen=6)

state = {
    "r":         0.0,
    "connected": False,
    "status":    "Connecting...",
    "last_lux":  0.0,
    "last_duty": 0.0,
}

ser     = None
t_start = time.time()

# ── Serial thread ──────────────────────────────────────────────────────────────
def serial_thread(port):
    global ser, LUMINAIRE
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(0.5)
        # Do NOT call reset_input_buffer() — it would discard the boot messages
        # containing "THIS NODE = X" that we need to detect the node ID.

        with lock:
            state["connected"] = True
            state["status"] = f"Connected: {port} — waiting for node ID..."
        print(f"[serial] Connected to {port}")

        # ── Phase 1: read boot output until we see "THIS NODE = X" ──────────
        # Scan for up to 25 s; the Pico prints the ready message after CAN discovery
        # (up to 15 s idle exit when alone, faster when peers are present).
        deadline = time.time() + 25
        while time.time() < deadline:
            try:
                line = ser.readline().decode(errors="ignore").strip()
            except serial.SerialException:
                break
            if not line:
                continue
            with lock:
                log_lines.append(line)
            print(f"[boot] {line}")

            m = re.search(r'THIS NODE\s*=\s*(\d+)', line)
            if m:
                LUMINAIRE = int(m.group(1))
                break

        if LUMINAIRE is None:
            # Boot message missed (e.g. Serial Monitor was open earlier).
            # Probe each node index with a benign "get lux" query and take
            # the first one that replies with a stream line.
            print("[serial] Boot message missed — probing nodes 1/2/3...")
            with lock:
                log_lines.append("Boot msg missed — probing nodes...")
            for probe_node in (1, 2, 3):
                ser.write(f"g y {probe_node}\n".encode())
                # Wait up to 1 s and collect all bytes the Pico sends back
                text = ""
                deadline2 = time.time() + 1.0
                while time.time() < deadline2:
                    if ser.in_waiting:
                        text += ser.read(ser.in_waiting).decode(errors="ignore")
                    time.sleep(0.05)
                # Valid response: a line "y N <float>" where N == probe_node
                # (not an error/CAN message which would not start with "y N ")
                if re.search(rf'(?m)^y {probe_node} [\d.]+', text):
                    LUMINAIRE = probe_node
                    break
            if LUMINAIRE is None:
                LUMINAIRE = 1
                with lock:
                    log_lines.append("Probe failed — defaulting to node 1")
                print("[serial] Probe failed, defaulting to node 1")
            else:
                with lock:
                    log_lines.append(f"Probe found node {LUMINAIRE}")
                print(f"[serial] Probe detected node {LUMINAIRE}")

        with lock:
            state["status"] = f"Connected: {port} — node {LUMINAIRE}"
        print(f"[serial] This is node {LUMINAIRE}")

        # ── Phase 2: start streams now that we know the node number ─────────
        ser.write(f"s y {LUMINAIRE}\n".encode())
        time.sleep(0.05)
        ser.write(f"s u {LUMINAIRE}\n".encode())

        # ── Phase 3: normal streaming loop ──────────────────────────────────
        while True:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) >= 4 and parts[0] == "s" and parts[2].isdigit():
                    var = parts[1]
                    val = float(parts[3])
                    t   = time.time() - t_start
                    with lock:
                        if var == "y":
                            state["last_lux"] = val
                            times.append(t)
                            lux_meas.append(val)
                            lux_ref.append(state["r"])
                        elif var == "u":
                            state["last_duty"] = val
                            duty_vals.append(val)
                else:
                    with lock:
                        log_lines.append(line)
            except (ValueError, IndexError):
                pass
            except serial.SerialException:
                with lock:
                    state["status"] = "Disconnected"
                break
    except serial.SerialException as e:
        with lock:
            state["status"] = f"Cannot open {port}: {e}"
        print(f"[serial] {e}")


def send(cmd):
    if ser and ser.is_open:
        ser.write((cmd.strip() + "\n").encode())
        with lock:
            log_lines.append(f"> {cmd.strip()}")
        print(f"[cmd] {cmd.strip()}")
    else:
        with lock:
            log_lines.append("Not connected")


# ── Figure ─────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(13, 9))
fig.patch.set_facecolor("#1e1e2e")
gs = gridspec.GridSpec(4, 1, figure=fig,
                       height_ratios=[3, 2, 2, 1],
                       hspace=0.5, top=0.93, bottom=0.14)

ax_lux  = fig.add_subplot(gs[0])
ax_duty = fig.add_subplot(gs[1])
ax_err  = fig.add_subplot(gs[2])
ax_log  = fig.add_subplot(gs[3])

for ax in [ax_lux, ax_duty, ax_err, ax_log]:
    ax.set_facecolor("#13131f")
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_edgecolor("#444466")
    ax.grid(True, alpha=0.15, color="#444466")

ax_log.set_xticks([])
ax_log.set_yticks([])
ax_log.set_title("Serial Log", color="#888", fontsize=8, pad=2)

line_ref,  = ax_lux.plot([], [], "--", color="#f38ba8", lw=1.5, label="Reference r")
line_meas, = ax_lux.plot([], [], "-",  color="#89dceb", lw=1.2, label="Measured LUX")
line_duty, = ax_duty.plot([], [], "-", color="#a6e3a1", lw=1.2)
line_err,  = ax_err.plot([], [], "-",  color="#fab387", lw=1.0)

ax_lux.set_ylabel("LUX", color="white", fontsize=9)
ax_lux.set_title("Reference vs Measured LUX", color="white", fontsize=10)
ax_lux.legend(loc="upper left", facecolor="#2a2a3e", labelcolor="white", fontsize=8)
ax_duty.set_ylabel("Duty [0-1]", color="white", fontsize=9)
ax_duty.set_title("Control Signal u", color="white", fontsize=10)
ax_duty.set_ylim(-0.05, 1.05)
ax_err.set_ylabel("Error [LUX]", color="white", fontsize=9)
ax_err.set_xlabel("Time [s]", color="white", fontsize=9)
ax_err.set_title("Tracking Error  e = r - y", color="white", fontsize=10)
ax_err.axhline(0, color="#585b70", lw=0.8, linestyle="--")

status_txt = fig.text(0.01, 0.97, "Connecting...", fontsize=8,
                      color="#cdd6f4", va="top", fontfamily="monospace")
vals_txt   = fig.text(0.50, 0.97, "", fontsize=8,
                      color="#cba6f7", va="top", fontfamily="monospace")
log_txt    = ax_log.text(0.01, 0.5, "", transform=ax_log.transAxes,
                         fontsize=7.5, color="#a6e3a1",
                         va="center", fontfamily="monospace")

# ── Command box ────────────────────────────────────────────────────────────────
ax_box     = fig.add_axes([0.08, 0.03, 0.68, 0.048])
ax_btn     = fig.add_axes([0.77, 0.03, 0.10, 0.048])
ax_save    = fig.add_axes([0.88, 0.03, 0.10, 0.048])

textbox = TextBox(ax_box, "  CMD: ", initial="",
                  color="#2a2a3e", hovercolor="#3a3a5e", label_pad=0.01)
textbox.label.set_color("white")
textbox.text_disp.set_color("#cdd6f4")
textbox.text_disp.set_fontfamily("monospace")
textbox.text_disp.set_fontsize(10)

btn = Button(ax_btn, "Send ↵", color="#313244", hovercolor="#45475a")
btn.label.set_color("white")

btn_save = Button(ax_save, "Save", color="#1e3a2f", hovercolor="#2d5a45")
btn_save.label.set_color("#a6e3a1")

fig.text(0.08, 0.005,
    "Examples:  r 1 20  │  f 1 1  │  f 1 0  │  c p 1 0.8  │  c i 1 0.05  │  a 1 2  │  clear",
    fontsize=7, color="#585b70", va="bottom", fontfamily="monospace")


def save_data(_=None):
    """Save current plot data to a CSV that plot_report.py can load directly."""
    with lock:
        t   = list(times)
        y   = list(lux_meas)
        ref = list(lux_ref)
        u   = list(duty_vals)

    # Align lengths (lux and duty are streamed separately, may differ by ±1)
    n = min(len(t), len(y), len(u))
    if n < 2:
        with lock:
            log_lines.append("Save failed: not enough data")
        return

    t   = t[:n];  y = y[:n];  ref = ref[:n];  u = u[:n]

    fname = time.strftime("data_%Y%m%d_%H%M%S.csv")
    with open(fname, "w") as f:
        f.write("# SCDTR live_plot.py export\n")
        f.write("# time_s,reference_lux,lux_measured,duty_cycle\n")
        for i in range(n):
            f.write(f"{t[i]:.4f},{ref[i]:.4f},{y[i]:.4f},{u[i]:.4f}\n")

    with lock:
        log_lines.append(f"Saved {n} pts → {fname}")
    print(f"[save] {fname}  ({n} samples)")


btn_save.on_clicked(save_data)


def submit(text):
    text = text.strip()
    if not text:
        return
    if text.lower() == "clear":
        with lock:
            times.clear(); lux_meas.clear()
            lux_ref.clear(); duty_vals.clear()
        with lock:
            log_lines.append("Plot cleared")
    else:
        parts = text.split()
        # Track reference from explicit r command
        if parts[0] == "r" and len(parts) >= 3:
            try:
                state["r"] = float(parts[2])
            except ValueError:
                pass
        # Track reference from occupancy command — must match OCC_REF in main.ino
        elif parts[0] == "o" and len(parts) >= 3:
            OCC_REF = [10.0, 20.0, 30.0]   # keep in sync with main.ino
            try:
                mode = int(parts[2])
                if 0 <= mode <= 2:
                    state["r"] = OCC_REF[mode]
            except ValueError:
                pass
        send(text)
    textbox.set_val("")


textbox.on_submit(submit)
btn.on_clicked(lambda _: submit(textbox.text))


# ── Animation ──────────────────────────────────────────────────────────────────
def update(_):
    with lock:
        t   = list(times)
        y   = list(lux_meas)
        ref = list(lux_ref)
        u   = list(duty_vals)
        s   = dict(state)
        log = list(log_lines)

    if len(t) < 2:
        return

    n = min(len(t), len(y), len(u))
    t   = t[-n:]; y = y[-n:]; ref = ref[-n:]; u = u[-n:]
    err = [ref[i] - y[i] for i in range(n)]

    line_ref.set_data(t, ref)
    line_meas.set_data(t, y)
    line_duty.set_data(t, u)
    line_err.set_data(t, err)

    t0, t1 = t[0], max(t[-1], t[0] + 5)
    for ax in [ax_lux, ax_duty, ax_err]:
        ax.set_xlim(t0, t1)

    if y and ref:
        lo = min(min(y), min(ref)) - 2
        hi = max(max(y), max(ref)) + 2
        ax_lux.set_ylim(max(0, lo), hi)

    if err:
        e_abs = max(abs(e) for e in err) + 1
        ax_err.set_ylim(-e_abs, e_abs)

    conn = "●" if s["connected"] else "○"
    status_txt.set_text(f"{conn} {s['status']}")
    vals_txt.set_text(
        f"LUX={s['last_lux']:.2f}  duty={s['last_duty']:.3f}  r={s['r']:.1f} LUX"
    )
    log_txt.set_text("  │  ".join(log[-4:]) if log else "waiting for data...")
    fig.canvas.draw_idle()


ani = FuncAnimation(fig, update, interval=UPDATE_MS, cache_frame_data=False)

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else find_pico_port()
    if port is None:
        print("ERROR: No serial port found.")
        print("Usage: python live_plot.py COM3")
        sys.exit(1)
    print(f"Using port: {port}")
    print("Make sure Arduino Serial Monitor is CLOSED before running.\n")
    threading.Thread(target=serial_thread, args=(port,), daemon=True).start()
    plt.show()