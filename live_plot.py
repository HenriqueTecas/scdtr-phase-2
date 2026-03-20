"""
SCDTR Phase 2 — Live Monitor
=============================
Visualises CAN traffic between nodes, per-node serial logs,
telemetry (LUX / duty), and the calibration coupling matrix.

Layout
------
  Row 0 │ Illuminance (all 3 nodes)  │  Control signal u
  Row 1 │ CAN sequence diagram        │  Coupling-gain heatmap
  Row 2 │ Node 1 serial log │ Node 2  │  Node 3 serial log
  Row 3 │ command bar

Usage
-----
  python live_plot.py                          # auto-detect Picos
  python live_plot.py /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2
"""

import sys
import time
import threading
import re
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.widgets import TextBox, Button
from matplotlib.animation import FuncAnimation
from collections import deque
import serial
import serial.tools.list_ports

# ── tunables ──────────────────────────────────────────────────────────────────
N_NODES      = 3
BAUD_RATE    = 115200
MAX_POINTS   = 100 * 60     # 60 s at 100 Hz
UPDATE_MS    = 160
MAX_SEQ_EV   = 50           # events kept in sequence diagram
LOG_LINES    = 14           # lines per node serial log

# ── palette (Catppuccin Mocha) ─────────────────────────────────────────────
BG           = "#1e1e2e"
PANEL_BG     = "#181825"
TEXT         = "#cdd6f4"
GRID         = "#313244"
NODE_C       = ["#89dceb", "#a6e3a1", "#fab387"]   # cyan / green / peach
NODE_N       = ["Node 1",  "Node 2",  "Node 3"]

MSG_C = {
    "SYN":      "#89b4fa",   # blue
    "SYN_ACK":  "#74c7ec",   # sky
    "ACK_DONE": "#a6e3a1",   # green
    "CAL_ON":   "#fab387",   # peach
    "CAL_LUX":  "#f9e2af",   # yellow
    "CAL_DONE": "#cba6f7",   # mauve
    "DATA":     "#6c7086",   # surface2
}
LOG_C = {
    "boot":   "#89b4fa",
    "wakeup": "#74c7ec",
    "cal":    "#fab387",
    "reply":  "#cba6f7",
    "cmd":    "#f9e2af",
    "misc":   "#6c7086",
    "disc":   "#f38ba8",
}

# ── shared state ──────────────────────────────────────────────────────────────
lock = threading.Lock()

times      = {i: deque(maxlen=MAX_POINTS) for i in range(1, N_NODES + 1)}
lux_meas   = {i: deque(maxlen=MAX_POINTS) for i in range(1, N_NODES + 1)}
lux_ref    = {i: deque(maxlen=MAX_POINTS) for i in range(1, N_NODES + 1)}
duty_vals  = {i: deque(maxlen=MAX_POINTS) for i in range(1, N_NODES + 1)}
node_logs  = {i: deque(maxlen=LOG_LINES)  for i in range(1, N_NODES + 1)}
can_events = deque(maxlen=MAX_SEQ_EV)

coupling   = [[None] * N_NODES for _ in range(N_NODES)]   # 0-indexed
bg_lux     = [None]   # mutable box

state = {
    "connected":    {i: False for i in range(1, N_NODES + 1)},
    "phase":        {i: "—"   for i in range(1, N_NODES + 1)},
    "last_lux":     {i: 0.0   for i in range(1, N_NODES + 1)},
    "last_duty":    {i: 0.0   for i in range(1, N_NODES + 1)},
    "ref":          {i: 0.0   for i in range(1, N_NODES + 1)},
}

serial_conns   = {}   # port → Serial
node_from_port = {}   # port → node id
uid_to_node    = {}   # hex addr string → node id
t_start        = time.time()


# ── helpers ───────────────────────────────────────────────────────────────────
def _addr_node(hex_str):
    return uid_to_node.get(hex_str.lower())


def _log(node_id, ts, text, cat):
    if 1 <= node_id <= N_NODES:
        node_logs[node_id].append((ts, text, cat))


def _ev(t, src, dst, mtype, label):
    can_events.append({"t": t, "src": src, "dst": dst, "type": mtype, "lbl": label})


# ── line parser (called with lock held) ───────────────────────────────────────
def parse_line(line, node_id):
    t  = time.time() - t_start
    ts = f"{t:6.1f}s"

    # streaming  s y 1 25.3200 12345
    m = re.match(r"^s ([yu]) (\d+) ([\d.]+)", line)
    if m:
        var, src, val = m.group(1), int(m.group(2)), float(m.group(3))
        if var == "y":
            state["last_lux"][src] = val
            times[src].append(t)
            lux_meas[src].append(val)
            lux_ref[src].append(state["ref"][src])
        elif var == "u":
            state["last_duty"][src] = val
            duty_vals[src].append(val)
        return

    # get reply  y 1 25.4 / r 2 30.00 …
    m = re.match(r"^([yurvoafpEVFOULC]) (\d+) ([\d.eE+\-.]+)", line)
    if m:
        var, src, val = m.group(1), int(m.group(2)), float(m.group(3))
        if var == "r" and 1 <= src <= N_NODES:
            state["ref"][src] = val
        _log(node_id, ts, line, "reply")
        return

    # discovery print
    m = re.search(r"UID last byte = 0x([0-9a-fA-F]+) \((\d+) decimal\)", line)
    if m:
        _log(node_id, ts, f"UID=0x{m.group(1)} ({m.group(2)} dec)", "disc")
        return

    # [BOOT] node_address=0xXX  LUMINAIRE=N
    m = re.search(r"\[BOOT\].*node_address=0x([0-9a-fA-F]+)\s+LUMINAIRE=(\d+)", line)
    if m:
        addr, lum = m.group(1), int(m.group(2))
        uid_to_node[addr.lower()] = lum
        state["phase"][node_id] = "BOOT"
        _log(node_id, ts, line, "boot")
        return

    m = re.search(r"LUMINAIRE\s*=\s*(\d+)", line)
    if m:
        state["phase"][node_id] = "READY"
        _log(node_id, ts, line, "boot")
        return

    # [WAKEUP] messages
    if "[WAKEUP]" in line:
        state["phase"][node_id] = "WAKEUP"

        m = re.search(r"SYN from 0x([0-9a-fA-F]+)", line)
        if m:
            peer = _addr_node(m.group(1))
            _ev(t, peer, node_id, "SYN", f"SYN→N{node_id}")
            _log(node_id, ts, line, "wakeup")
            return

        if "sending SYN-ACK" in line:
            m = re.search(r"SYN from 0x([0-9a-fA-F]+)", line)
            peer = _addr_node(m.group(1)) if m else None
            _ev(t, node_id, peer, "SYN_ACK", f"N{node_id}→SYN-ACK")
            _log(node_id, ts, line, "wakeup")
            return

        m = re.search(r"SYN-ACK from 0x([0-9a-fA-F]+)", line)
        if m:
            peer = _addr_node(m.group(1))
            _ev(t, peer, node_id, "SYN_ACK", f"SYN-ACK→N{node_id}")
            _log(node_id, ts, line, "wakeup")
            return

        if "sending ACK_DONE" in line:
            m = re.search(r"0x([0-9a-fA-F]+)", line)
            peer = _addr_node(m.group(1)) if m else None
            _ev(t, node_id, peer, "ACK_DONE", f"N{node_id}→ACK-DONE")
            _log(node_id, ts, line, "wakeup")
            return

        if "Handshake complete" in line:
            m = re.search(r"0x([0-9a-fA-F]+)", line)
            peer = _addr_node(m.group(1)) if m else None
            _ev(t, node_id, peer, "ACK_DONE", f"✓N{node_id}↔N{peer}")
            _log(node_id, ts, line, "wakeup")
            return

        if "READY" in line:
            state["phase"][node_id] = "READY"

        _log(node_id, ts, line, "wakeup")
        return

    # [CAL] messages
    if "[CAL]" in line:
        state["phase"][node_id] = "CAL"

        m = re.search(r"Background = ([\d.]+)", line)
        if m:
            bg_lux[0] = float(m.group(1))
            _log(node_id, ts, line, "cal")
            return

        m = re.search(r"Round (\d+): node 0x([0-9a-fA-F]+) is active", line)
        if m:
            rnd, addr = int(m.group(1)), m.group(2)
            active = _addr_node(addr)
            _ev(t, active, None, "CAL_ON", f"CAL r{rnd} N{active} ON")
            _log(node_id, ts, line, "cal")
            return

        m = re.search(r"(?:Self|Cross)-gain\[(\d+)\]\[(\d+)\] = ([\d.]+)", line)
        if m:
            i, j, v = int(m.group(1)) - 1, int(m.group(2)) - 1, float(m.group(3))
            if 0 <= i < N_NODES and 0 <= j < N_NODES:
                coupling[i][j] = v
            _log(node_id, ts, line, "cal")
            return

        if "Calibration complete" in line:
            state["phase"][node_id] = "READY"
            _ev(t, node_id, None, "CAL_DONE", f"N{node_id} done")

        _log(node_id, ts, line, "cal")
        return

    _log(node_id, ts, line, "misc")


# ── serial thread ─────────────────────────────────────────────────────────────
def serial_thread(port):
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(0.3)
        with lock:
            serial_conns[port] = ser
        print(f"[serial] {port} opened")

        node_id = None
        deadline = time.time() + 25
        while time.time() < deadline:
            try:
                raw = ser.readline().decode(errors="ignore").strip()
            except serial.SerialException:
                return
            if not raw:
                continue
            with lock:
                parse_line(raw, node_id or 0)
            m = re.search(r"LUMINAIRE\s*=\s*(\d+)", raw)
            if m:
                node_id = int(m.group(1))
                with lock:
                    node_from_port[port] = node_id
                    state["connected"][node_id] = True
                    uid_to_node  # already updated via parse_line
                print(f"[serial] {port} → Node {node_id}")
                break

        # fallback probe
        if node_id is None:
            for pid in (1, 2, 3):
                ser.write(f"g y {pid}\n".encode())
                time.sleep(0.15)
            buf = ""
            t2 = time.time() + 2
            while time.time() < t2:
                if ser.in_waiting:
                    buf += ser.read(ser.in_waiting).decode(errors="ignore")
                time.sleep(0.05)
            for pid in (1, 2, 3):
                if re.search(rf"(?m)^y {pid} ", buf):
                    node_id = pid
                    break
            if node_id is None:
                node_id = (len(node_from_port) % N_NODES) + 1
            with lock:
                node_from_port[port] = node_id
                state["connected"][node_id] = True
            print(f"[serial] {port} → Node {node_id} (fallback)")

        # auto-start streams
        ser.write(f"s y {node_id}\n".encode())
        time.sleep(0.05)
        ser.write(f"s u {node_id}\n".encode())

        while True:
            try:
                raw = ser.readline().decode(errors="ignore").strip()
                if raw:
                    with lock:
                        parse_line(raw, node_id)
            except serial.SerialException:
                with lock:
                    state["connected"][node_id] = False
                break

    except serial.SerialException as e:
        print(f"[serial] {port}: {e}")


# ── send command ──────────────────────────────────────────────────────────────
def send_cmd(cmd):
    cmd = cmd.strip()
    if not cmd:
        return
    parts = cmd.split()

    with lock:
        if parts[0] == "r" and len(parts) >= 3:
            try:
                state["ref"][int(parts[1])] = float(parts[2])
            except (ValueError, IndexError):
                pass

        conns = list(serial_conns.values())
        node_id_for_log = next(
            (nid for p, nid in node_from_port.items()
             if serial_conns.get(p) and serial_conns[p].is_open), 1
        )

    sent = False
    for ser in conns:
        try:
            if ser and ser.is_open:
                ser.write((cmd + "\n").encode())
                sent = True
                break
        except serial.SerialException:
            pass

    with lock:
        cat = "cmd"
        _log(node_id_for_log, "CMD", f"> {cmd}", cat)
    if not sent:
        print("[cmd] not connected")


# ── figure layout ─────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(17, 11))
fig.patch.set_facecolor(BG)

outer = gridspec.GridSpec(
    4, 1, figure=fig,
    height_ratios=[3.0, 3.2, 2.6, 0.5],
    hspace=0.55, top=0.945, bottom=0.06
)

# row 0 — LUX | duty
row0 = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=outer[0], wspace=0.28)
ax_lux  = fig.add_subplot(row0[0])
ax_duty = fig.add_subplot(row0[1])

# row 1 — CAN sequence | coupling matrix
row1 = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=outer[1],
                                         wspace=0.30, width_ratios=[2.2, 1])
ax_seq = fig.add_subplot(row1[0])
ax_mat = fig.add_subplot(row1[1])

# row 2 — 3 serial logs
row2 = gridspec.GridSpecFromSubplotSpec(1, 3, subplot_spec=outer[2], wspace=0.08)
ax_log = [fig.add_subplot(row2[k]) for k in range(3)]

def _style(ax, grid=False):
    ax.set_facecolor(PANEL_BG)
    ax.tick_params(colors=TEXT, labelsize=7)
    for sp in ax.spines.values():
        sp.set_edgecolor(GRID)
    if grid:
        ax.grid(True, alpha=0.15, color=GRID)

for ax in [ax_lux, ax_duty, ax_seq, ax_mat] + ax_log:
    _style(ax, grid=(ax in [ax_lux, ax_duty]))

# telemetry lines
line_ref  = {}
line_meas = {}
line_duty = {}
for i in range(1, N_NODES + 1):
    (lr,) = ax_lux.plot([], [], "--", color="#f38ba8", lw=1.2,
                        label="Ref" if i == 1 else "")
    line_ref[i] = lr
    (lm,) = ax_lux.plot([], [], "-",  color=NODE_C[i-1], lw=1.3,
                        label=NODE_N[i-1])
    line_meas[i] = lm
    (ld,) = ax_duty.plot([], [], "-", color=NODE_C[i-1], lw=1.3,
                         label=NODE_N[i-1])
    line_duty[i] = ld

ax_lux.set_ylabel("LUX", color=TEXT, fontsize=8)
ax_lux.set_xlabel("Time [s]", color=TEXT, fontsize=8)
ax_lux.set_title("Illuminance — Reference vs Measured", color=TEXT, fontsize=9)
ax_lux.legend(loc="upper left", facecolor="#2a2a3e", labelcolor=TEXT,
              fontsize=7, ncol=2)
ax_duty.set_ylabel("Duty [0–1]", color=TEXT, fontsize=8)
ax_duty.set_xlabel("Time [s]", color=TEXT, fontsize=8)
ax_duty.set_title("Control Signal u", color=TEXT, fontsize=9)
ax_duty.set_ylim(-0.05, 1.05)
ax_duty.legend(loc="upper left", facecolor="#2a2a3e", labelcolor=TEXT, fontsize=7)

# static seq / log labels (cleared every frame but title set in draw functions)
ax_seq.set_xticks([]); ax_seq.set_yticks([])
for ax in ax_log:
    ax.set_xticks([]); ax.set_yticks([])

# status bar
status_txt = fig.text(0.01, 0.975, "connecting…", fontsize=8,
                      color=TEXT, va="top", fontfamily="monospace")
vals_txt   = fig.text(0.28, 0.975, "",           fontsize=8,
                      color="#cba6f7", va="top", fontfamily="monospace")

# command bar
ax_cmd  = fig.add_axes([0.06, 0.012, 0.64, 0.036])
ax_send = fig.add_axes([0.71, 0.012, 0.075, 0.036])
ax_save = fig.add_axes([0.79, 0.012, 0.075, 0.036])
ax_clr  = fig.add_axes([0.87, 0.012, 0.075, 0.036])
ax_rst  = fig.add_axes([0.95, 0.012, 0.042, 0.036])

tbox = TextBox(ax_cmd, " CMD ", initial="", color="#2a2a3e",
               hovercolor="#3a3a5e", label_pad=0.01)
tbox.label.set_color(TEXT)
tbox.text_disp.set_color("#cdd6f4")
tbox.text_disp.set_fontfamily("monospace")
tbox.text_disp.set_fontsize(9)

def _btn(ax, lbl, bg, hov, fg=TEXT):
    b = Button(ax, lbl, color=bg, hovercolor=hov)
    b.label.set_color(fg); b.label.set_fontsize(8)
    return b

btn_send = _btn(ax_send, "Send",  "#313244", "#45475a")
btn_save = _btn(ax_save, "Save",  "#1e3a2f", "#2d5a45", "#a6e3a1")
btn_clr  = _btn(ax_clr,  "Clear", "#3a1e1e", "#5a2d2d", "#f38ba8")
btn_rst  = _btn(ax_rst,  "R",     "#2a1e3a", "#3d2d5a", "#cba6f7")

fig.text(0.06, 0.003,
    "r <n> <lux>  o <n> <0/1/2>  f <n> <0/1>  s/S y/u <n>  "
    "c p/i <n> <v>  g y/u/r/E/V/F <n>  O/U/C <n> <v>  R",
    fontsize=6, color="#45475a", va="bottom", fontfamily="monospace")


# ── save ──────────────────────────────────────────────────────────────────────
def save_data(_=None):
    fname = time.strftime("scdtr_%Y%m%d_%H%M%S.csv")
    with lock:
        with open(fname, "w") as f:
            f.write("# SCDTR Phase 2\n# time_s,node,ref,lux,duty\n")
            for n in range(1, N_NODES + 1):
                tl = list(times[n]); yl = list(lux_meas[n])
                rl = list(lux_ref[n]); ul = list(duty_vals[n])
                k = min(len(tl), len(yl), len(rl), len(ul))
                for i in range(k):
                    f.write(f"{tl[i]:.4f},{n},{rl[i]:.4f},{yl[i]:.4f},{ul[i]:.4f}\n")
        nid = next((v for v in node_from_port.values()), 1)
        _log(nid, "", f"Saved {fname}", "misc")
    print(f"[save] {fname}")

btn_save.on_clicked(save_data)


# ── submit ────────────────────────────────────────────────────────────────────
def submit(text):
    text = text.strip()
    if not text:
        return
    if text.lower() == "clear":
        with lock:
            for i in range(1, N_NODES + 1):
                times[i].clear(); lux_meas[i].clear()
                lux_ref[i].clear(); duty_vals[i].clear()
            can_events.clear()
    else:
        send_cmd(text)
    tbox.set_val("")

tbox.on_submit(submit)
btn_send.on_clicked(lambda _: submit(tbox.text))
btn_clr.on_clicked(lambda _: submit("clear"))
btn_rst.on_clicked(lambda _: submit("R"))


# ── sequence diagram ──────────────────────────────────────────────────────────
_LANE = {1: 0.15, 2: 0.50, 3: 0.85}   # x per node

def draw_sequence(events, s):
    ax_seq.cla()
    _style(ax_seq)
    ax_seq.set_xlim(0, 1); ax_seq.set_ylim(0, 1)
    ax_seq.set_xticks([]); ax_seq.set_yticks([])
    ax_seq.set_title("CAN Message Flow  (newest → top)", color=TEXT,
                     fontsize=9, pad=4)

    # node lane headers
    for n, x in _LANE.items():
        conn  = s["connected"][n]
        phase = s["phase"][n]
        col   = NODE_C[n-1] if conn else "#444466"
        ax_seq.axvline(x=x, ymin=0.06, ymax=0.92,
                       color=col, lw=0.9, alpha=0.45, linestyle="--")
        ax_seq.text(x, 0.955, NODE_N[n-1], ha="center", va="bottom",
                    color=col, fontsize=8, fontweight="bold",
                    transform=ax_seq.transAxes)
        ax_seq.text(x, 0.005, phase, ha="center", va="bottom",
                    color=col, fontsize=6,
                    transform=ax_seq.transAxes)

    if not events:
        ax_seq.text(0.5, 0.5, "waiting for CAN traffic…",
                    ha="center", va="center", color="#45475a",
                    fontsize=9, transform=ax_seq.transAxes)
    else:
        n_ev = len(events)
        y_top, y_bot = 0.90, 0.08
        for idx, ev in enumerate(reversed(events)):   # newest at top
            y = y_top - idx * (y_top - y_bot) / max(n_ev, 1)
            if y < y_bot:
                break
            src  = ev["src"]
            dst  = ev["dst"]
            mtyp = ev["type"]
            lbl  = ev["lbl"]
            col  = MSG_C.get(mtyp, "#888")
            x0   = _LANE.get(src, 0.05) if isinstance(src, int) else 0.05
            x1   = _LANE.get(dst, None) if isinstance(dst, int) else None

            if x1 is None:
                # broadcast — right-pointing arrow across all lanes
                ax_seq.annotate("",
                    xy=(0.92, y), xytext=(x0, y),
                    xycoords="axes fraction", textcoords="axes fraction",
                    arrowprops=dict(arrowstyle="->", color=col,
                                    lw=0.9, alpha=0.65))
            else:
                ax_seq.annotate("",
                    xy=(x1, y), xytext=(x0, y),
                    xycoords="axes fraction", textcoords="axes fraction",
                    arrowprops=dict(arrowstyle="->", color=col, lw=1.1))

            mid = (x0 + (x1 if x1 else 0.92)) / 2
            ax_seq.text(mid, y + 0.014, lbl, ha="center", va="bottom",
                        color=col, fontsize=5.5, clip_on=True,
                        transform=ax_seq.transAxes)

    # legend
    patches = [mpatches.Patch(color=v, label=k)
               for k, v in MSG_C.items() if k != "DATA"]
    ax_seq.legend(handles=patches, loc="lower right",
                  facecolor="#2a2a3e", labelcolor=TEXT,
                  fontsize=5.5, ncol=2, framealpha=0.85)


# ── coupling matrix ───────────────────────────────────────────────────────────
def draw_matrix(mat, bg):
    ax_mat.cla()
    _style(ax_mat)
    ax_mat.set_title("Coupling Gains G[i][j]  [LUX]", color=TEXT, fontsize=8, pad=4)

    has_data = any(mat[i][j] is not None for i in range(3) for j in range(3))
    if not has_data:
        ax_mat.set_xticks([]); ax_mat.set_yticks([])
        ax_mat.text(0.5, 0.58, "Coupling matrix\nappears after calibration",
                    ha="center", va="center", color="#45475a",
                    fontsize=7.5, transform=ax_mat.transAxes)
        if bg is not None:
            ax_mat.text(0.5, 0.28, f"Background: {bg:.2f} LUX",
                        ha="center", va="center", color="#74c7ec",
                        fontsize=7.5, transform=ax_mat.transAxes)
        return

    data = np.array([[mat[i][j] if mat[i][j] is not None else 0.0
                      for j in range(3)] for i in range(3)])
    ax_mat.imshow(data, cmap="YlOrRd", aspect="auto",
                  vmin=0, vmax=max(data.max(), 1))
    lbls = ["N1", "N2", "N3"]
    ax_mat.set_xticks(range(3)); ax_mat.set_xticklabels(lbls, color=TEXT, fontsize=7)
    ax_mat.set_yticks(range(3)); ax_mat.set_yticklabels(lbls, color=TEXT, fontsize=7)
    ax_mat.set_xlabel("Active node  j", color=TEXT, fontsize=7)
    ax_mat.set_ylabel("Sensor node  i", color=TEXT, fontsize=7)
    thresh = data.max() * 0.55
    for i in range(3):
        for j in range(3):
            v = data[i][j]
            fc = "black" if v > thresh else TEXT
            ax_mat.text(j, i, f"{v:.1f}", ha="center", va="center",
                        color=fc, fontsize=7.5, fontweight="bold")
    if bg is not None:
        ax_mat.set_xlabel(f"j  (bg={bg:.1f} LUX)", color=TEXT, fontsize=7)


# ── per-node serial logs ──────────────────────────────────────────────────────
def draw_logs(logs_snap, s):
    for k, ax in enumerate(ax_log):
        node = k + 1
        ax.cla()
        _style(ax)
        ax.set_xticks([]); ax.set_yticks([])
        conn  = s["connected"][node]
        phase = s["phase"][node]
        tc    = NODE_C[k] if conn else "#444466"
        dot   = "●" if conn else "○"
        ax.set_title(f"{dot} Node {node}  [{phase}]",
                     color=tc, fontsize=8, pad=3)

        lines = list(logs_snap[node])
        if not lines:
            ax.text(0.5, 0.5,
                    "no data" if conn else "not connected",
                    ha="center", va="center", color="#444466",
                    fontsize=7.5, transform=ax.transAxes)
            continue

        n = len(lines)
        for j, (ts, text, cat) in enumerate(lines):
            y = 1.0 - (j + 0.5) / n
            col = LOG_C.get(cat, "#888")
            txt = text if len(text) <= 52 else text[:49] + "…"
            prefix = f"{ts} " if ts else ""
            ax.text(0.01, y, prefix + txt,
                    transform=ax.transAxes,
                    fontsize=5.5, color=col, va="center",
                    fontfamily="monospace", clip_on=True)


# ── animation ─────────────────────────────────────────────────────────────────
def update(_):
    with lock:
        # snapshot everything needed for rendering
        s         = {k: dict(v) if isinstance(v, dict) else v
                     for k, v in state.items()}
        events    = list(can_events)
        mat       = [row[:] for row in coupling]
        bg        = bg_lux[0]
        logs_snap = {i: list(node_logs[i]) for i in range(1, N_NODES + 1)}
        t_now     = {i: list(times[i])     for i in range(1, N_NODES + 1)}
        y_now     = {i: list(lux_meas[i])  for i in range(1, N_NODES + 1)}
        r_now     = {i: list(lux_ref[i])   for i in range(1, N_NODES + 1)}
        u_now     = {i: list(duty_vals[i]) for i in range(1, N_NODES + 1)}

    # telemetry
    all_t = []
    for i in range(1, N_NODES + 1):
        tl = t_now[i]; yl = y_now[i]; rl = r_now[i]; ul = u_now[i]
        n = min(len(tl), len(yl))
        if n >= 2:
            all_t += tl
            line_meas[i].set_data(tl[:n], yl[:n])
            line_ref[i].set_data(tl[:n], rl[:n])
            nu = min(len(tl), len(ul))
            line_duty[i].set_data(tl[:nu], ul[:nu])
        else:
            line_meas[i].set_data([], [])
            line_ref[i].set_data([], [])
            line_duty[i].set_data([], [])

    if all_t:
        t0 = min(all_t); t1 = max(max(all_t), t0 + 10)
        ax_lux.set_xlim(t0, t1)
        ax_duty.set_xlim(t0, t1)
        all_y   = [v for i in range(1, N_NODES+1) for v in y_now[i]]
        all_ref = [v for i in range(1, N_NODES+1) for v in r_now[i]]
        combined = all_y + all_ref
        if combined:
            lo = min(combined) - 2; hi = max(combined) + 2
            ax_lux.set_ylim(max(0, lo), hi)

    # status bar
    conn_str = "  ".join(
        ("●" if s["connected"][i] else "○") + NODE_N[i-1]
        for i in range(1, N_NODES + 1)
    )
    status_txt.set_text(conn_str)
    vals_str = "   │   ".join(
        f"N{i}: {s['last_lux'][i]:.1f} LUX  u={s['last_duty'][i]:.2f}"
        for i in range(1, N_NODES + 1) if s["connected"][i]
    )
    vals_txt.set_text(vals_str)

    draw_sequence(events, s)
    draw_matrix(mat, bg)
    draw_logs(logs_snap, s)

    fig.canvas.draw_idle()


ani = FuncAnimation(fig, update, interval=UPDATE_MS, cache_frame_data=False)

# ── entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    ports = sys.argv[1:] if len(sys.argv) > 1 else []
    if not ports:
        ports = []
        for p in serial.tools.list_ports.comports():
            d = (p.description or "").lower()
            h = (p.hwid or "").lower()
            if any(k in d or k in h for k in ["pico","rp2040","2e8a","ttyacm"]):
                ports.append(p.device)
        ports.sort()

    if not ports:
        print("No Pico ports found.  Usage: python live_plot.py [PORT ...]")
        sys.exit(1)

    print(f"Ports: {ports}")
    print("Close any Arduino serial monitors first.\n")

    for port in ports:
        threading.Thread(target=serial_thread, args=(port,), daemon=True).start()

    plt.show()
