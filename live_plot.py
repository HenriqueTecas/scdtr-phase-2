#!/usr/bin/env python3
"""
SCDTR Phase 2 — 3-Node Live Dashboard
=======================================
Optimized version of the original functional dashboard.
"""

import sys
import time
import threading
from collections import deque

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import TextBox, Button
from matplotlib.animation import FuncAnimation

import serial
import serial.tools.list_ports

# ── Configuration ────────────────────────────────────────────────────────────
BAUD_RATE  = 115200
HISTORY_S  = 30
MAX_POINTS = HISTORY_S * 100  # internal buffer
UPDATE_MS  = 250              # ~4 FPS (Lowering this saves massive CPU)
N_NODES    = 3
MAX_PLOT_PTS = 120            # Decimation limit for rendering

# Colors (Catppuccin Mocha-ish)
BG      = "#1e1e2e"
PANEL   = "#181825"
FG      = "#cdd6f4"
DIM     = "#6c7086"
WHITE   = "#ffffff"
GRID    = "#313244"
ACCENT  = "#cba6f7"

COLOURS = {
    "lux_meas": ["#89dceb", "#a6e3a1", "#f9e2af"], # Sky, Green, Yellow
    "lux_ref":  ["#f38ba8", "#eba0ac", "#fab387"], # Red, Maroon, Peach
    "duty":     ["#74c7ec", "#94e2d5", "#b4befe"], # Sapphire, Teal, Lavender
}

OCC_REF = [0.0, 10.0, 20.0]

# ── Auto-detect Ports ────────────────────────────────────────────────────────
def find_pico_ports(n=N_NODES):
    found = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc or k in hwid for k in ["pico", "rp2040", "2e8a", "usbmodem", "ttyacm"]):
            found.append(p.device)
    found.sort()
    return found[:n]

# ── Node State ───────────────────────────────────────────────────────────────
class NodeState:
    def __init__(self, port, idx):
        self.port     = port
        self.idx      = idx
        self.node_id  = "?"
        self.lock     = threading.Lock()
        
        # Buffers
        self.times     = deque(maxlen=MAX_POINTS)
        self.lux_meas  = deque(maxlen=MAX_POINTS)
        self.lux_ref   = deque(maxlen=MAX_POINTS)
        self.duty_vals = deque(maxlen=MAX_POINTS)
        self.log_lines = deque(maxlen=4)
        
        self.connected = False
        self.status    = "Connecting…"
        self.last_lux  = 0.0
        self.last_duty = 0.0
        self.r         = 0.0
        self.ser       = None
        self.t_start   = time.time()

def serial_thread(ns: NodeState):
    try:
        ns.ser = serial.Serial(ns.port, BAUD_RATE, timeout=0.1)
        time.sleep(1.5)
        ns.ser.reset_input_buffer()
        
        # Tell node to start streaming
        for lum in range(1, N_NODES + 1):
            ns.ser.write(f"s y {lum}\n".encode())
            time.sleep(0.05)
            ns.ser.write(f"s u {lum}\n".encode())
            time.sleep(0.05)

        with ns.lock:
            ns.connected = True
            ns.status = "Connected"

        while True:
            line = ns.ser.readline().decode(errors="ignore").strip()
            if not line:
                continue

            # Auto-detect node ID from boot or messages
            if "[BOOT] LUMINAIRE=" in line:
                nid = line.split("LUMINAIRE=")[1].split()[0]
                with ns.lock: ns.node_id = nid
            
            parts = line.split()
            # Stream format: s <var> <id> <val> <ts_ms>
            if len(parts) >= 4 and parts[0] == "s" and parts[2].isdigit():
                var = parts[1]
                nid = parts[2]
                val = float(parts[3])
                t = time.time() - ns.t_start

                with ns.lock:
                    if ns.node_id == "?": ns.node_id = nid
                    if var == "y":
                        ns.last_lux = val
                        ns.times.append(t)
                        ns.lux_meas.append(val)
                        ns.lux_ref.append(ns.r)
                    elif var == "u":
                        ns.last_duty = val
                        ns.duty_vals.append(val)
            else:
                # Log regular messages
                with ns.lock:
                    ns.log_lines.append(line)
                    
    except Exception as e:
        with ns.lock:
            ns.status = f"Error: {str(e)[:20]}"
            ns.connected = False

# ── UI Setup ─────────────────────────────────────────────────────────────────
_graphs_on = True
if "--no-graphs" in sys.argv:
    _graphs_on = False
    sys.argv.remove("--no-graphs")

fig = plt.figure(figsize=(14, 8), facecolor=BG)
fig.canvas.manager.set_window_title("SCDTR Phase 2 Dashboard")
gs = gridspec.GridSpec(3, N_NODES, height_ratios=[3, 2, 0.7], hspace=0.4, wspace=0.25)

ax_lux  = []
ax_duty = []
ax_log  = []
lines_meas = []
lines_ref  = []
lines_duty = []
status_txts = []
vals_txts   = []
log_texts   = []

for col in range(N_NODES):
    # LUX Plot
    al = fig.add_subplot(gs[0, col], facecolor=PANEL)
    al.set_title(f"Node {col+1}", color=FG, fontsize=9, pad=10)
    al.grid(True, color=GRID, alpha=0.3)
    al.tick_params(colors=DIM, labelsize=7)
    for spine in al.spines.values(): spine.set_color(GRID)
    
    lm, = al.plot([], [], color=COLOURS["lux_meas"][col], lw=1.2, label="y")
    lr, = al.plot([], [], "--", color=COLOURS["lux_ref"][col], lw=1.0, alpha=0.8, label="ref")
    
    # DUTY Plot
    ad = fig.add_subplot(gs[1, col], facecolor=PANEL)
    ad.grid(True, color=GRID, alpha=0.3)
    ad.tick_params(colors=DIM, labelsize=7)
    for spine in ad.spines.values(): spine.set_color(GRID)
    ld, = ad.plot([], [], color=COLOURS["duty"][col], lw=1.2)
    
    # LOG Panel
    alog = fig.add_subplot(gs[2, col], facecolor=PANEL)
    alog.set_xticks([]); alog.set_yticks([])
    for spine in alog.spines.values(): spine.set_color(GRID)
    lt = alog.text(0.02, 0.5, "waiting…", color="#a6e3a1", fontsize=7.5, 
                   va="center", family="monospace")
    
    ax_lux.append(al); ax_duty.append(ad); ax_log.append(alog)
    lines_meas.append(lm); lines_ref.append(lr); lines_duty.append(ld)
    log_texts.append(lt)
    
    st = fig.text(0.05 + col * 0.32, 0.97, "…", fontsize=8, color=WHITE, weight="bold")
    vt = fig.text(0.05 + col * 0.32, 0.95, "", fontsize=8, color=ACCENT, family="monospace")
    status_txts.append(st); vals_txts.append(vt)

# ── Controls ─────────────────────────────────────────────────────────────────
ax_box = fig.add_axes([0.05, 0.03, 0.45, 0.045])
textbox = TextBox(ax_box, "  CMD: ", initial="", color="#313244", label_pad=0.01)
textbox.label.set_color(WHITE)
textbox.text_disp.set_color(FG)

ax_btn = fig.add_axes([0.52, 0.03, 0.08, 0.045])
btn = Button(ax_btn, "Send", color="#45475a", hovercolor="#585b70")
btn.label.set_color(WHITE)

ax_tog = fig.add_axes([0.62, 0.03, 0.12, 0.045])
btn_tog = Button(ax_tog, "📊 ON" if _graphs_on else "📊 OFF", color="#1a1b26")
btn_tog.label.set_color(WHITE)

nodes = []

def toggle_graphs(_=None):
    global _graphs_on
    _graphs_on = not _graphs_on
    btn_tog.label.set_text("📊 ON" if _graphs_on else "📊 OFF")
    for i in range(N_NODES):
        ax_lux[i].set_visible(_graphs_on)
        ax_duty[i].set_visible(_graphs_on)
    fig.canvas.draw_idle()
btn_tog.on_clicked(toggle_graphs)

def submit(text):
    text = text.strip()
    if not text: return
    parts = text.split()
    # Send to all if no index or index matches
    for ns in nodes:
        if ns.ser: 
            ns.ser.write((text + "\n").encode())
            if parts[0] == "o" and len(parts) >= 3:
                try: ns.r = OCC_REF[int(parts[2])]
                except: pass
    textbox.set_val("")

textbox.on_submit(submit)
btn.on_clicked(lambda _: submit(textbox.text))

# ── Animation Loop ───────────────────────────────────────────────────────────
_frame_count = 0

def update(frame):
    global _frame_count
    _frame_count += 1
    
    for i, ns in enumerate(nodes):
        with ns.lock:
            conn = ns.connected
            status = f"Node {ns.node_id} ({ns.port})" if conn else f"Disconnected ({ns.port})"
            last_y, last_u, r_val = ns.last_lux, ns.last_duty, ns.r
            logs = list(ns.log_lines)
            
            if _graphs_on:
                t = list(ns.times)
                y = list(ns.lux_meas)
                ref = list(ns.lux_ref)
                u = list(ns.duty_vals)
        
        # 1. Update text (Cheap)
        if status_txts[i].get_text() != status: status_txts[i].set_text(status)
        new_vals = f"y={last_y:.1f} lux  u={last_u:.2f}  r={r_val:.1f}"
        if vals_txts[i].get_text() != new_vals: vals_txts[i].set_text(new_vals)
        
        new_log = "\n".join(logs) if logs else "waiting…"
        if log_texts[i].get_text() != new_log: log_texts[i].set_text(new_log)

        if not _graphs_on: continue

        # 2. Update Graphs (Expensive)
        if len(t) > 1:
            # DECIMATION: only plot a subset of points to save CPU
            stride = max(1, len(t) // MAX_PLOT_PTS)
            tp, yp, rp, up = t[::stride], y[::stride], ref[::stride], u[::stride]
            
            lines_meas[i].set_data(tp, yp)
            lines_ref[i].set_data(tp, rp)
            lines_duty[i].set_data(tp, up)
            
            # Throttled Rescaling
            if _frame_count % 5 == 0:
                ax_lux[i].set_xlim(tp[0], max(tp[-1], tp[0] + 5))
                ax_duty[i].set_xlim(tp[0], max(tp[-1], tp[0] + 5))
                
                hi = max(max(yp), max(rp)) + 5
                ax_lux[i].set_ylim(0, max(45, hi))
                ax_duty[i].set_ylim(-0.05, 1.05)

    return []

ani = FuncAnimation(fig, update, interval=UPDATE_MS, cache_frame_data=False)

if __name__ == "__main__":
    ports = find_pico_ports()
    for i in range(N_NODES):
        port = ports[i] if i < len(ports) else None
        ns = NodeState(port or "N/A", i)
        nodes.append(ns)
        if port:
            threading.Thread(target=serial_thread, args=(ns,), daemon=True).start()
    
    if not _graphs_on:
        toggle_graphs() # trigger visibility off
        _graphs_on = False # reset state
        
    plt.show()
