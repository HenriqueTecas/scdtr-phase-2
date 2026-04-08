#!/usr/bin/env python3
"""
SCDTR Phase 2 — High-Performance GUI Dashboard (Text-Only)
=========================================================
GUI with 3 separate terminals and 3 separate command inputs.
Uses Multiprocessing for serial and Matplotlib for the layout/UI.
Includes calibration and identity logic from test_suite.py.
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
from matplotlib.widgets import TextBox

import serial
import serial.tools.list_ports
import re

# ── Configuration (Matched to live_plot.py and test_suite.py) ────────────────
BAUD_RATE  = 115200
UPDATE_MS  = 100              # 10 FPS
N_NODES    = 3

# Catppuccin Mocha Palette (Matched to live_plot.py)
BG      = "#1e1e2e"
PANEL   = "#181825"
FG      = "#cdd6f4"
WHITE   = "#ffffff"
ACCENT  = "#cba6f7"
GREEN   = "#a6e3a1"
RED     = "#f38ba8"
CYAN    = "#89dceb"
YELLOW  = "#f9e2af"

# ── Serial Worker Process (Enhanced with test_suite.py logic) ────────────────
def serial_worker(port, port_idx, queue, cmd_queue):
    try:
        # Use a longer timeout for the initial connection like test_suite.py
        ser = serial.Serial(port, BAUD_RATE, timeout=0.2)
        
        # Capture [BOOT] message if it happens on reset
        boot_deadline = time.time() + 2.5
        while time.time() < boot_deadline:
            try:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    queue.put((port_idx, "log", line, 0, None))
                    m = re.search(r'\[BOOT\] LUMINAIRE=(\d+)', line)
                    if m:
                        nid = int(m.group(1))
                        queue.put((port_idx, "id", nid, 0, nid))
            except:
                break

        ser.reset_input_buffer()
        ser.timeout = 0.05 # Back to fast polling
        
        # Explicitly request identity
        ser.write(b"i\n")
        
        t0 = time.time()
        while True:
            # 1. Handle outgoing commands
            while not cmd_queue.empty():
                cmd = cmd_queue.get_nowait()
                ser.write((cmd.strip() + "\n").encode())
                # Small delay like test_suite.py for stability
                time.sleep(0.06)

            # 2. Read incoming data
            line = ser.readline().decode(errors="ignore").strip()
            if not line: continue

            parts = line.split()
            # Format: s <var> <nid> <val> <ts>
            if len(parts) >= 4 and parts[0] == "s" and parts[2].isdigit():
                var, nid, val = parts[1], int(parts[2]), float(parts[3])
                queue.put((port_idx, var, val, time.time() - t0, nid))
            
            # Identity response: i <nid>
            elif len(parts) == 2 and parts[0] == "i" and parts[1].isdigit():
                nid = int(parts[1])
                queue.put((port_idx, "id", nid, 0, nid))
            
            # Catch other useful messages from test_suite.py
            else:
                if "[CAL] Calibration complete" in line:
                    queue.put((port_idx, "cal", True, 0, None))
                
                # Try to extract NID from any line containing a known NID-like pattern
                # if we don't have it yet.
                m = re.search(r'LUMINAIRE=(\d+)', line)
                if m:
                    nid = int(m.group(1))
                    queue.put((port_idx, "id", nid, 0, nid))
                
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
        self.lux       = [0.0] * N_NODES
        self.duty      = [0.0] * N_NODES
        self.logs      = [deque(maxlen=40) for _ in range(N_NODES)]
        self.port_names = ["?"] * N_NODES
        self.last_upd  = [0.0] * N_NODES
        self.cal_done  = [False] * N_NODES
        
        # Mapping: node_id (1-3) -> port_idx
        self.id_to_port = {}

        self.setup_ui()

    def setup_ui(self):
        self.fig = plt.figure(figsize=(15, 10), facecolor=BG)
        self.fig.canvas.manager.set_window_title("SCDTR Phase 2 — Parallel Terminal GUI")
        
        # 3 columns for 3 nodes
        # Row 0: Large LUX/Duty display
        # Row 1: Scrolling Terminal
        # Row 2: CMD Input (per node)
        gs = gridspec.GridSpec(3, N_NODES, height_ratios=[1, 4, 0.4], hspace=0.3, wspace=0.15)

        self.val_texts = []
        self.log_texts = []
        self.status_txts = []
        self.textboxes = []

        for i in range(N_NODES):
            # Readout Panel
            ax_val = self.fig.add_subplot(gs[0, i], facecolor=PANEL)
            ax_val.axis("off")
            
            # Big Node Label
            st = ax_val.text(0.5, 0.85, f"NODE {i+1}", color=WHITE, weight="bold", 
                            fontsize=14, ha="center", transform=ax_val.transAxes)
            port_t = ax_val.text(0.5, 0.7, "WAITING FOR ID...", color=ACCENT, 
                                fontsize=9, ha="center", transform=ax_val.transAxes)
            
            # Big Numbers
            lux_t = ax_val.text(0.3, 0.35, "0.0", color=CYAN, fontsize=24, 
                               weight="bold", ha="center", transform=ax_val.transAxes)
            ax_val.text(0.3, 0.15, "LUX", color=FG, fontsize=10, ha="center", transform=ax_val.transAxes)
            
            duty_t = ax_val.text(0.7, 0.35, "0.00", color=YELLOW, fontsize=24, 
                                weight="bold", ha="center", transform=ax_val.transAxes)
            ax_val.text(0.7, 0.15, "DUTY", color=FG, fontsize=10, ha="center", transform=ax_val.transAxes)
            
            self.status_txts.append((st, port_t))
            self.val_texts.append((lux_t, duty_t))

            # Terminal Panel
            ax_log = self.fig.add_subplot(gs[1, i], facecolor=PANEL)
            ax_log.axis("off")
            ax_log.set_title(f"TERMINAL NODE {i+1}", color=FG, fontsize=9, loc="left", pad=5)
            
            lt = ax_log.text(0.02, 0.98, "", color=GREEN, fontsize=8, va="top", 
                            family="monospace", transform=ax_log.transAxes)
            self.log_texts.append(lt)

            # CMD Box per node
            ax_box = self.fig.add_subplot(gs[2, i])
            # Adjusting margins for the textbox to fit in the gridspec cell
            tb = TextBox(ax_box, f"CMD {i+1}: ", initial="", color="#313244", label_pad=0.1)
            tb.label.set_color(WHITE)
            tb.text_disp.set_color(FG)
            # Use a closure to capture the index i
            def make_submit(idx):
                return lambda text: self.submit(text, idx)
            tb.on_submit(make_submit(i))
            self.textboxes.append(tb)

    def submit(self, text, node_idx):
        text = text.strip()
        if not text: return
        
        # Send to the specific worker mapped to this logical node_idx
        port_idx = self.id_to_port.get(node_idx + 1)
        if port_idx is not None:
            self.cmd_queues[port_idx].put(text)
        else:
            # If mapping not yet established, broadcast to all
            for q in self.cmd_queues:
                q.put(text)
        
        self.logs[node_idx].append(f">>> {text}")
        self.textboxes[node_idx].set_val("")

    def update(self, frame):
        # 1. Drain Queue
        while not self.data_queue.empty():
            try:
                port_idx, var, val, t, nid = self.data_queue.get_nowait()
                
                col = None
                if nid is not None and 1 <= nid <= N_NODES:
                    col = nid - 1
                    self.id_to_port[nid] = port_idx
                    self.port_names[col] = self.ports[port_idx]
                
                if col is not None:
                    self.last_upd[col] = time.time()
                    if var == "y":
                        self.lux[col] = val
                    elif var == "u":
                        self.duty[col] = val
                    elif var == "log":
                        self.logs[col].append(str(val))
                    elif var == "id":
                        self.port_names[col] = self.ports[port_idx]
                    elif var == "cal":
                        self.cal_done[col] = True
                        self.logs[col].append("[UI] Calibration confirmed.")
                elif var == "log":
                    # Try to find which col this port_idx currently maps to
                    found_col = None
                    for nid, p_idx in self.id_to_port.items():
                        if p_idx == port_idx:
                            found_col = nid - 1
                            break
                    if found_col is not None:
                        self.logs[found_col].append(str(val))
            except: break

        # 2. Update Artists
        for i in range(N_NODES):
            lux_t, duty_t = self.val_texts[i]
            lux_t.set_text(f"{self.lux[i]:.1f}")
            duty_t.set_text(f"{self.duty[i]:.2f}")
            
            st, port_t = self.status_txts[i]
            online = (time.time() - self.last_upd[i] < 3.0)
            
            if self.port_names[i] == "?":
                port_t.set_text("WAITING FOR ID...")
                st.set_color(RED)
            else:
                status_str = f"{self.port_names[i]}"
                if self.cal_done[i]:
                    status_str += " [CAL]"
                port_t.set_text(status_str)
                st.set_color(GREEN if online else RED)
            
            # Combine logs into a single string
            self.log_texts[i].set_text("\n".join(list(self.logs[i])))

        return []

    def start(self):
        processes = []
        for i, port in enumerate(self.ports):
            p = mp.Process(target=serial_worker, args=(port, i, self.data_queue, self.cmd_queues[i]))
            p.daemon = True
            p.start()
            processes.append(p)
        
        # Send streaming commands automatically to start data flow
        def startup_pings():
            time.sleep(2.0)
            for q in self.cmd_queues:
                for nid in range(1, 4):
                    q.put(f"s y {nid}")
                    q.put(f"s u {nid}")

        import threading
        threading_thread = threading.Thread(target=startup_pings)
        threading_thread.daemon = True
        threading_thread.start()

        from matplotlib.animation import FuncAnimation
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
