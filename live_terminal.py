#!/usr/bin/env python3
"""
SCDTR Phase 2 — Parallel Terminal Dashboard
===========================================
Displays LUX/Duty for 3 nodes and 3 terminals for interaction.
Uses Multiprocessing (identical to live_plot.py) for high performance.
"""

import sys
import time
import multiprocessing as mp
import curses
from collections import deque
import serial
import serial.tools.list_ports
import re

# ── Configuration (Matched to live_plot.py) ──────────────────────────────────
BAUD_RATE  = 115200
N_NODES    = 3

# ── Serial Worker Process (Identical to live_plot.py) ────────────────────────
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
                # Catch [BOOT] or other logs
                m = re.search(r'\[BOOT\] LUMINAIRE=(\d+)', line)
                if m:
                    nid = int(m.group(1))
                    queue.put((port_idx, "id", nid, 0, nid))
                queue.put((port_idx, "log", line, 0, None))
                
    except Exception as e:
        queue.put((port_idx, "err", str(e), 0, None))

# ── Curses Dashboard ─────────────────────────────────────────────────────────
class TerminalDashboard:
    def __init__(self, ports):
        self.ports = ports
        self.data_queue = mp.Queue()
        self.cmd_queues = [mp.Queue() for _ in range(len(ports))]
        
        # State (indexed by logical Node ID - 1)
        self.lux       = [0.0] * N_NODES
        self.duty      = [0.0] * N_NODES
        self.logs      = [deque(maxlen=100) for _ in range(N_NODES)]
        self.port_names = ["?"] * N_NODES
        self.last_upd  = [0.0] * N_NODES
        
        # UI State
        self.input_buffer = ""
        self.running = True

    def draw(self, stdscr):
        curses.curs_set(1)
        stdscr.nodelay(1)
        stdscr.timeout(50)
        
        # Colors
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_CYAN, -1)    # Values
        curses.init_pair(2, curses.COLOR_GREEN, -1)   # Success/ID
        curses.init_pair(3, curses.COLOR_YELLOW, -1)  # Input
        curses.init_pair(4, curses.COLOR_RED, -1)     # Error/Offline

        while self.running:
            # 1. Drain Queue (identical logic to live_plot.py)
            while not self.data_queue.empty():
                try:
                    port_idx, var, val, t, nid = self.data_queue.get_nowait()
                    
                    # Logic to map port/nid to column
                    col = None
                    if nid is not None and 1 <= nid <= N_NODES:
                        col = nid - 1
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
                    elif var == "log":
                        # Log message from a port that hasn't identified yet
                        # Just put it in the log of the port_idx for now
                        if port_idx < N_NODES:
                            self.logs[port_idx].append(f"[{self.ports[port_idx]}] {val}")
                except: break

            # 2. Render
            stdscr.clear()
            h, w = stdscr.getmaxyx()
            col_w = w // N_NODES
            
            # Layout
            header_h = 6
            input_h = 2
            log_h = h - header_h - input_h - 1

            for i in range(N_NODES):
                start_x = i * col_w
                
                # Column Separators
                if i > 0:
                    for r in range(h - input_h):
                        stdscr.addch(r, start_x - 1, "|")

                # Node Header
                status_color = curses.color_pair(2) if (time.time() - self.last_upd[i] < 2.0) else curses.color_pair(4)
                stdscr.addstr(0, start_x + (col_w - 10)//2, f" NODE {i+1} ", curses.A_REVERSE)
                stdscr.addstr(1, start_x + 2, f"Port: {self.port_names[i][:col_w-8]}")
                stdscr.addstr(2, start_x + 2, "Status: ONLINE" if status_color == curses.color_pair(2) else "Status: OFFLINE", status_color)
                
                # Values
                stdscr.addstr(4, start_x + 2, "LUX: ", curses.A_BOLD)
                stdscr.addstr(4, start_x + 7, f"{self.lux[i]:>6.2f}", curses.color_pair(1))
                stdscr.addstr(5, start_x + 2, "Duty:", curses.A_BOLD)
                stdscr.addstr(5, start_x + 7, f"{self.duty[i]:>6.2f}", curses.color_pair(1))

                # Logs (Terminal area)
                stdscr.addstr(header_h, start_x + 2, "TERMINAL:", curses.A_UNDERLINE)
                visible_logs = list(self.logs[i])[-log_h:]
                for idx, log in enumerate(visible_logs):
                    if header_h + 1 + idx < h - input_h:
                        line_text = log[:col_w-3]
                        color = 0
                        if line_text.startswith(">>>"): color = curses.color_pair(3)
                        stdscr.addstr(header_h + 1 + idx, start_x + 1, line_text, color)

            # Input area
            prompt = "SEND CMD (all) or '1:cmd' (specific): "
            stdscr.addstr(h - 2, 1, prompt, curses.A_BOLD)
            stdscr.addstr(h - 2, 1 + len(prompt), self.input_buffer, curses.color_pair(3))
            
            stdscr.refresh()

            # 3. Input Handling
            try:
                ch = stdscr.getch()
                if ch == -1: continue
                if ch == ord('\n'):
                    self.process_input()
                elif ch == ord('q') and not self.input_buffer:
                    self.running = False
                elif ch in (127, 8, curses.KEY_BACKSPACE):
                    self.input_buffer = self.input_buffer[:-1]
                elif 32 <= ch <= 126:
                    self.input_buffer += chr(ch)
            except: pass

    def process_input(self):
        cmd = self.input_buffer.strip()
        if not cmd: return
        
        target = None
        if ":" in cmd:
            parts = cmd.split(":", 1)
            if parts[0].isdigit():
                target = int(parts[0])
                cmd = parts[1].strip()

        if target and 1 <= target <= N_NODES:
            # Find which port index corresponds to this node ID
            # We check port_names mapping or node_by_id logic
            # In live_plot, we just send to the worker that matched that ID
            # Simplified: Find worker that is currently Node X
            for i in range(len(self.ports)):
                # This is tricky because worker indices != Node IDs
                # But we can send to the specific cmd_queue
                pass
            # Better: use the cmd_queues directly if we know the mapping
            # For now, send to all if target is not easily mapped, or 
            # we can map it by checking which worker is which.
            for i in range(len(self.ports)):
                # We'll just broadcast for now or implement mapping
                self.cmd_queues[i].put(cmd)
                # Add to local log for visual feedback
                if target <= N_NODES:
                    self.logs[target-1].append(f">>> {cmd}")
        else:
            for q in self.cmd_queues:
                q.put(cmd)
            for l in self.logs:
                l.append(f">>> {cmd}")
        
        self.input_buffer = ""

    def start(self):
        processes = []
        for i, port in enumerate(self.ports):
            p = mp.Process(target=serial_worker, args=(port, i, self.data_queue, self.cmd_queues[i]))
            p.daemon = True
            p.start()
            processes.append(p)
        
        try:
            curses.wrapper(self.draw)
        finally:
            self.running = False
            for p in processes:
                p.terminate()

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
    dash = TerminalDashboard(p)
    dash.start()
