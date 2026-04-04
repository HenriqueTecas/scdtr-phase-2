#!/usr/bin/env python3
"""
SCDTR Phase 2 — HIGH-PERFORMANCE Parallel Dashboard
===================================================
Uses Multiprocessing for serial and Optimized rendering for speed.
"""

import sys
import time
import multiprocessing as mp
from collections import deque
import numpy as np

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
UPDATE_MS  = 100              # 10 FPS
N_NODES    = 3
MAX_PLOT_PTS = 150            # Rendering decimation
OCC_REF = [0.0, 10.0, 20.0]

# Catppuccin Mocha Palette
BG      = "#1e1e2e"
PANEL   = "#181825"
FG      = "#cdd6f4"
WHITE   = "#ffffff"
ACCENT  = "#cba6f7"
GREEN   = "#a6e3a1"
RED     = "#f38ba8"

COLOURS = {
    "lux_meas": ["#89dceb", "#a6e3a1", "#f9e2af"],
    "lux_ref":  ["#f38ba8", "#eba0ac", "#fab387"],
    "duty":     ["#74c7ec", "#94e2d5", "#b4befe"],
}

# ── Serial Worker Process ────────────────────────────────────────────────────
def serial_worker(port, port_idx, queue, cmd_queue):
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.05)
        time.sleep(1.0)
        ser.reset_input_buffer()
        
        # Identity check
        ser.write(b"i\n")
        
        t0 = time.time()
        while True:
            # 1. Handle outgoing commands
            while not cmd_queue.empty():
                cmd = cmd_queue.get_nowait()
                ser.write((cmd + "\n").encode())

            # 2. Read incoming data
            line = ser.readline().decode(errors="ignore").strip()
            if not line: continue

            parts = line.split()
            # Format: s <var> <nid> <val> <ts>
            if len(parts) >= 4 and parts[0] == "s" and parts[2].isdigit():
                var, nid, val = parts[1], int(parts[2]), float(parts[3])
                queue.put((port_idx, var, val, time.time() - t0, nid))
            elif len(parts) == 2 and parts[0] == "i" and parts[1].isdigit():
                # identity response: i <nid>
                queue.put((port_idx, "id", int(parts[1]), 0, int(parts[1])))
            else:
                queue.put((port_idx, "log", line, 0, None))
                
    except Exception as e:
        queue.put((port_idx, "err", str(e), 0, None))

# ── GUI Dashboard ────────────────────────────────────────────────────────────
class Dashboard:
    def __init__(self, ports):
        self.ports = ports
        self.data_queue = mp.Queue()
        self.cmd_queues = [mp.Queue() for _ in range(len(ports))]
        
        # Internal buffers (indexed by logical Node ID - 1)
        self.times     = [deque(maxlen=2000) for _ in range(N_NODES)]
        self.lux_meas  = [deque(maxlen=2000) for _ in range(N_NODES)]
        self.lux_ref   = [deque(maxlen=2000) for _ in range(N_NODES)]
        self.duty_vals = [deque(maxlen=2000) for _ in range(N_NODES)]
        self.logs      = [deque(maxlen=4) for _ in range(N_NODES)]
        
        self.last_y = [0.0] * N_NODES
        self.last_u = [0.0] * N_NODES
        self.last_r = [0.0] * N_NODES
        self.port_names = ["?"] * N_NODES

        self.setup_ui()

    def setup_ui(self):
        self.fig = plt.figure(figsize=(14, 8), facecolor=BG)
        self.fig.canvas.manager.set_window_title("SCDTR Phase 2 — Parallel Dashboard")
        gs = gridspec.GridSpec(3, N_NODES, height_ratios=[3, 2, 0.7], hspace=0.4, wspace=0.25)

        self.ax_lux = []
        self.ax_duty = []
        self.lines_meas = []
        self.lines_ref = []
        self.lines_duty = []
        self.status_txts = []
        self.vals_txts = []
        self.log_texts = []

        for i in range(N_NODES):
            # LUX plot
            al = self.fig.add_subplot(gs[0, i], facecolor=PANEL)
            al.set_title(f"Node {i+1}", color=FG, fontsize=9, pad=10)
            al.tick_params(colors="#6c7086", labelsize=7)
            lm, = al.plot([], [], color=COLOURS["lux_meas"][i], lw=1.5, label="y")
            lr, = al.plot([], [], "--", color=COLOURS["lux_ref"][i], lw=1.0, alpha=0.8, label="ref")
            al.set_ylim(0, 45); al.set_xlim(0, 30)
            self.ax_lux.append(al); self.lines_meas.append(lm); self.lines_ref.append(lr)

            # DUTY plot
            ad = self.fig.add_subplot(gs[1, i], facecolor=PANEL)
            ad.tick_params(colors="#6c7086", labelsize=7)
            ld, = ad.plot([], [], color=COLOURS["duty"][i], lw=1.2)
            ad.set_ylim(-0.05, 1.05); ad.set_xlim(0, 30)
            self.ax_duty.append(ad); self.lines_duty.append(ld)

            # Log Panel
            alog = self.fig.add_subplot(gs[2, i], facecolor=PANEL)
            alog.axis("off")
            lt = alog.text(0.02, 0.5, "waiting…", color=GREEN, fontsize=7, va="center", family="monospace")
            self.log_texts.append(lt)

            st = self.fig.text(0.05 + i*0.32, 0.97, f"Node {i+1}", color=WHITE, weight="bold", fontsize=8)
            vt = self.fig.text(0.05 + i*0.32, 0.95, "", color=ACCENT, fontsize=8, family="monospace")
            self.status_txts.append(st); self.vals_txts.append(vt)

        # CMD Box
        ax_box = self.fig.add_axes([0.05, 0.03, 0.45, 0.04])
        self.textbox = TextBox(ax_box, "  CMD: ", initial="", color="#313244", label_pad=0.01)
        self.textbox.label.set_color(WHITE)
        self.textbox.text_disp.set_color(FG)
        self.textbox.on_submit(self.submit)

    def submit(self, text):
        text = text.strip()
        if not text: return
        # Send to all workers
        for q in self.cmd_queues:
            q.put(text)
        self.textbox.set_val("")

    def update(self, frame):
        # 1. Drain Queue
        while not self.data_queue.empty():
            try:
                port_idx, var, val, t, nid = self.data_queue.get_nowait()
                
                # Determine which column this belongs to
                if nid is not None and 1 <= nid <= N_NODES:
                    col = nid - 1
                    self.port_names[col] = self.ports[port_idx]
                else:
                    # Log message or unknown - use port_idx as fallback
                    col = port_idx if port_idx < N_NODES else 0

                if var == "y":
                    self.times[col].append(t)
                    self.lux_meas[col].append(val)
                    self.lux_ref[col].append(self.last_r[col])
                    self.last_y[col] = val
                elif var == "u":
                    self.duty_vals[col].append(val)
                    self.last_u[col] = val
                elif var == "log":
                    self.logs[col].append(str(val))
                elif var == "id":
                    self.port_names[col] = self.ports[port_idx]
            except: break

        # 2. Update Artists
        for i in range(N_NODES):
            if len(self.times[i]) > 1:
                t = np.array(self.times[i])
                y = np.array(self.lux_meas[i])
                r = np.array(self.lux_ref[i])
                
                # Decimate
                step = max(1, len(t) // MAX_PLOT_PTS)
                self.lines_meas[i].set_data(t[::step], y[::step])
                self.lines_ref[i].set_data(t[::step], r[::step])
                
                if len(self.duty_vals[i]) == len(self.times[i]):
                    u = np.array(self.duty_vals[i])
                    self.lines_duty[i].set_data(t[::step], u[::step])

                # Scroll
                cur_t = t[-1]
                if cur_t > self.ax_lux[i].get_xlim()[1]:
                    self.ax_lux[i].set_xlim(cur_t - 25, cur_t + 5)
                    self.ax_duty[i].set_xlim(cur_t - 25, cur_t + 5)

            self.status_txts[i].set_text(f"Node {i+1} ({self.port_names[i]})")
            self.vals_txts[i].set_text(f"y={self.last_y[i]:.1f}  u={self.last_u[i]:.2f}")
            self.log_texts[i].set_text("\n".join(self.logs[i]))

        return []

    def start(self):
        for i, port in enumerate(self.ports):
            p = mp.Process(target=serial_worker, args=(port, i, self.data_queue, self.cmd_queues[i]))
            p.daemon = True
            p.start()
        
        self.ani = FuncAnimation(self.fig, self.update, interval=UPDATE_MS, cache_frame_data=False)
        plt.show()

def find_ports():
    found = []
    for p in serial.tools.list_ports.comports():
        if any(k in (p.description or "").lower() or k in (p.hwid or "").lower() 
               for k in ["pico", "rp2040", "2e8a", "usbmodem", "ttyacm"]):
            found.append(p.device)
    found.sort()
    return found[:N_NODES]

if __name__ == "__main__":
    mp.set_start_method('spawn')
    p = find_ports()
    if not p:
        print("No Picos detected!")
        sys.exit(1)
    dash = Dashboard(p)
    dash.start()
