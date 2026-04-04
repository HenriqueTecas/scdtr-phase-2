#!/usr/bin/env python3
"""
SCDTR Phase 2 — Automated Test Suite
======================================
Connects to all 3 Pico nodes, waits for calibration, then runs 10 test
scenarios covering occupancy sweeps and cost-vector sweeps.

Produces:
  • Console table (steady-state lux, duty, ΔE, V, F per node per test)
  • plot_steady_state_occupancy.png  — lux & duty vs occupancy config
  • plot_cost_comparison.png         — duty & energy vs cost vector
  • plot_metrics.png                 — E / V / F across all scenarios
  • plot_ts_<scenario>.png           — time-series for selected tests

Usage:
    python test_suite.py                                       # auto-detect ports
    python test_suite.py /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2
"""

import sys, os, time, threading, re
from collections import deque
import numpy as np
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Tuneable parameters ────────────────────────────────────────────────────────
BAUD           = 115200
N_NODES        = 3
CAL_TIMEOUT_S  = 180   # max wait for calibration
ID_TIMEOUT_S   = 40    # max wait for node ID detection
ADMM_WAIT_S    = 12    # wait after triggering ADMM (convergence + PI settle)
COLLECT_S      = 10    # streaming data window per scenario
MAX_POINTS     = 5000  # per-node stream buffer size

# Physical constants (from firmware)
MAXIMUM_POWER  = 0.0132   # W per node at duty=1
SAMPLE_H       = 0.010    # s (10 ms control period)

# ── Test scenarios: (label, occ1, occ2, occ3, c1, c2, c3) ─────────────────────
# occ: 0=off, 1=low(10lux), 2=high(20lux)
# c:   energy cost coefficient

CONVERGENCE_STEPS = [5, 10, 20, 40, 80, 150]

SCENARIOS = [
    # ── Occupancy sweep, uniform costs ──────────────────────────────────────
    ("Occ(0,0,0) C=[1,1,1]",  0,0,0,  1, 1, 1),
    ("Occ(2,0,0) C=[1,1,1]",  2,0,0,  1, 1, 1),
    ("Occ(0,2,0) C=[1,1,1]",  0,2,0,  1, 1, 1),
    ("Occ(0,0,2) C=[1,1,1]",  0,0,2,  1, 1, 1),
    ("Occ(2,2,2) C=[1,1,1]",  2,2,2,  1, 1, 1),
    ("Occ(1,2,1) C=[1,1,1]",  1,2,1,  1, 1, 1),
    # ── Cost sweep, all HIGH occupancy ───────────────────────────────────────
    ("Occ(2,2,2) C=[1,10,1]", 2,2,2,  1,10, 1),
    ("Occ(2,2,2) C=[10,1,10]",2,2,2, 10, 1,10),
    ("Occ(2,2,2) C=[1,20,1]", 2,2,2,  1,20, 1),
    ("Occ(2,2,2) C=[5,1,5]",  2,2,2,  0.25, 0.05, 0.25),
]

# ── Per-node connection state ──────────────────────────────────────────────────
class Node:
    def __init__(self, port):
        self.port      = port
        self.node_id   = None      # 1/2/3, learnt from serial
        self.ser       = None
        self.lock      = threading.Lock()
        self._lines    = deque(maxlen=MAX_POINTS)  # all raw lines
        self.lux_buf   = []        # [(t, val), …] filled during collect()
        self.duty_buf  = []
        self.connected = False
        self.cal_done  = False
        self.admm_done = False
        self._t0       = time.time()

    # ── Serial helpers ─────────────────────────────────────────────────────
    def write(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd.strip() + "\n").encode())
            time.sleep(0.06)

    def flush(self):
        with self.lock:
            self._lines.clear()

    def wait_line(self, pattern: str, timeout=5.0) -> "str | None":
        """Return first matching line or None after timeout."""
        pat = re.compile(pattern)
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self.lock:
                for line in self._lines:
                    if pat.search(line):
                        return line
            time.sleep(0.05)
        return None

    def last_match(self, pattern: str) -> "str | None":
        pat = re.compile(pattern)
        with self.lock:
            for line in reversed(list(self._lines)):
                if pat.search(line):
                    return line
        return None

    def query(self, var: str, timeout=4.0) -> "float | None":
        """Send 'g <var> <node_id>' locally, parse 'VAR <id> <val>'."""
        if self.node_id is None:
            return None
        self.flush()
        self.write(f"g {var} {self.node_id}")
        line = self.wait_line(rf"^{var} {self.node_id}\b", timeout=timeout)
        if line:
            try:
                return float(line.split()[2])
            except (IndexError, ValueError):
                pass
        return None

    def query_str(self, var: str, timeout=4.0) -> "str | None":
        """Like query() but returns the raw line."""
        if self.node_id is None:
            return None
        self.flush()
        self.write(f"g {var} {self.node_id}")
        return self.wait_line(rf"^{var} {self.node_id}\b", timeout=timeout)

# ── Serial reader threads ──────────────────────────────────────────────────────
def reader_thread(node: Node):
    try:
        node.ser = serial.Serial(node.port, BAUD, timeout=1)
        time.sleep(2)
        node.ser.reset_input_buffer()
        node.connected = True
        print(f"  [{node.port}] connected")

        while True:
            try:
                raw = node.ser.readline().decode(errors="ignore").strip()
                if not raw:
                    continue
                t = time.time() - node._t0
                with node.lock:
                    node._lines.append(raw)

                    # ── Node ID from boot message ───────────────────────────
                    if node.node_id is None:
                        m = re.search(r'\[BOOT\] LUMINAIRE=(\d+)', raw)
                        if m:
                            node.node_id = int(m.group(1))
                            print(f"  [{node.port}] → Node {node.node_id}")
                        elif raw.startswith("i "):
                            parts = raw.split()
                            if len(parts) >= 2 and parts[1].isdigit():
                                node.node_id = int(parts[1])
                                print(f"  [{node.port}] → Node {node.node_id} (identity)")
                        elif raw.startswith("s "):
                            parts = raw.split()
                            if (len(parts) >= 4 and parts[1] in ('y','u')
                                    and parts[2].isdigit()):
                                node.node_id = int(parts[2])
                                print(f"  [{node.port}] → Node {node.node_id} (stream)")

                    # ── Calibration complete ───────────────────────────────
                    if "[CAL] Calibration complete" in raw:
                        node.cal_done = True

                    # ── ADMM done ─────────────────────────────────────────
                    if any(tag in raw for tag in
                           ("[ADMM] converged", "[ADMM] done",
                            "[ADMM] maxiter", "[ADMM] hardmax")):
                        node.admm_done = True

                    # ── Stream data (stored during collect window) ─────────
                    if raw.startswith("s "):
                        parts = raw.split()
                        if len(parts) >= 4:
                            try:
                                var = parts[1]
                                val = float(parts[3])
                                if var == 'y':
                                    node.lux_buf.append((t, val))
                                elif var == 'u':
                                    node.duty_buf.append((t, val))
                            except ValueError:
                                pass

            except serial.SerialException:
                node.connected = False
                break
            except Exception:
                pass

    except Exception as e:
        print(f"  [{node.port}] error: {e}")


# ── Global node registry ───────────────────────────────────────────────────────
nodes: list[Node] = []
node_by_id: dict[int, Node] = {}   # {1: Node, 2: Node, 3: Node}

def send(nid: int, cmd: str):
    """Send command to node nid (hub-forward if needed)."""
    n = node_by_id.get(nid)
    if n and n.connected:
        n.write(cmd)
    else:
        # Fall back: send to first available node; hub forwards via CAN
        for nn in nodes:
            if nn.connected:
                nn.write(cmd)
                return

def wait_admm(timeout=ADMM_WAIT_S) -> bool:
    """Wait for any node to signal ADMM completion."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if any(n.admm_done for n in nodes):
            return True
        time.sleep(0.1)
    return False

def collect(duration_s: float) -> dict[int, dict]:
    """Enable streaming, collect for duration_s, stop, return per-node data."""
    # Clear buffers first
    for n in nodes:
        n.lux_buf.clear()
        n.duty_buf.clear()

    # Start streaming
    for n in nodes:
        if n.node_id:
            n.write(f"s y {n.node_id}")
            n.write(f"s u {n.node_id}")

    time.sleep(duration_s)

    # Stop streaming
    for n in nodes:
        if n.node_id:
            n.write(f"S y {n.node_id}")
            n.write(f"S u {n.node_id}")
    time.sleep(0.3)

    result = {}
    for n in nodes:
        if n.node_id:
            with n.lock:
                result[n.node_id] = {
                    't_lux':  [x[0] for x in n.lux_buf],
                    'lux':    [x[1] for x in n.lux_buf],
                    't_duty': [x[0] for x in n.duty_buf],
                    'duty':   [x[1] for x in n.duty_buf],
                }
    return result

# ── Metric computation from streaming data ─────────────────────────────────────
def compute_metrics(lux_data: list, duty_data: list,
                    L: float, d: float, G: float,
                    h: float = SAMPLE_H) -> tuple:
    """
    Compute E, V, F from streaming data using equations (14)-(16) from PDF.

    E = Σ u_k · P_max · h
    V = (1/T) Σ max(0, L − x̂_k)  where x̂_k = d + G·u_k
    F = 1/(T·h) · Σ_{Δu_k·Δu_{k−1}<0} (|Δu_k| + |Δu_{k−1}|)
    """
    T = len(duty_data)
    if T == 0:
        return 0.0, 0.0, 0.0

    u = np.array(duty_data, dtype=float)

    # Energy (J)
    E = float(np.sum(u) * MAXIMUM_POWER * h)

    # Visibility error
    x_hat = d + G * u
    V = float(np.mean(np.maximum(0.0, L - x_hat))) if T > 0 else 0.0

    # Flicker
    if T >= 3:
        du = np.diff(u)
        sign_reversal = du[1:] * du[:-1] < 0
        F = float(np.sum((np.abs(du[1:]) + np.abs(du[:-1])) * sign_reversal) / (T * h))
    else:
        F = 0.0

    return E, V, F

def steady_state_mean(data: dict, last_s: float = 3.0) -> tuple:
    """Return (mean_lux, mean_duty) over the last last_s seconds."""
    if not data['lux']:
        return 0.0, 0.0
    t_end = data['t_lux'][-1] if data['t_lux'] else 0.0
    t_cut = t_end - last_s
    lux_ss  = [v for t, v in zip(data['t_lux'],  data['lux'])  if t >= t_cut]
    duty_ss = [v for t, v in zip(data['t_duty'], data['duty']) if t >= t_cut]
    return (float(np.mean(lux_ss))  if lux_ss  else 0.0,
            float(np.mean(duty_ss)) if duty_ss else 0.0)

def parse_cal_params(node: Node) -> tuple:
    """Query 'c ?' and parse gain K and background d. Returns (G, d)."""
    node.flush()
    node.write("c ?")
    time.sleep(0.5)
    G, d = None, None
    with node.lock:
        for line in node._lines:
            m = re.search(r'gain K\s*=\s*([\d.]+)', line)
            if m:
                G = float(m.group(1))
            m = re.search(r'background\s*=\s*([\d.]+)', line)
            if m:
                d = float(m.group(1))
    return (G or 1.0, d or 0.0)

# ── Run tests ──────────────────────────────────────────────────────────────────
def run_tests() -> list:
    results = []

    # Fetch calibration parameters once per node (they don't change)
    cal_params = {}
    for n in nodes:
        if n.node_id:
            G, d = parse_cal_params(n)
            cal_params[n.node_id] = (G, d)
            print(f"  Node {n.node_id}: G={G:.3f} d={d:.2f}")

    for sc_idx, (label, occ1, occ2, occ3, c1, c2, c3) in enumerate(SCENARIOS):
        print(f"\n{'━'*62}")
        print(f"  Test {sc_idx+1}/{len(SCENARIOS)}: {label}")
        print(f"{'━'*62}")

        occs  = {1: occ1, 2: occ2, 3: occ3}
        costs = {1: c1,   2: c2,   3: c3}

        # ── 1. Reset ADMM flags before issuing commands ────────────────────
        for n in nodes:
            n.admm_done = False

        # ── 2. Apply cost coefficients ─────────────────────────────────────
        for nid in range(1, N_NODES + 1):
            send(nid, f"C {nid} {costs[nid]}")
        time.sleep(0.5)

        # ── 3. Apply occupancy (also triggers ADMM broadcast) ─────────────
        for nid in range(1, N_NODES + 1):
            send(nid, f"o {nid} {occs[nid]}")
        time.sleep(0.3)

        # ── 4. Wait for ADMM + PI settle ───────────────────────────────────
        converged = wait_admm(ADMM_WAIT_S)
        print(f"  ADMM: {'converged ✓' if converged else 'timeout (continuing)'}")
        # Extra settle for PI to reach steady state
        time.sleep(4)

        # ── 5. Snapshot firmware energy before collection ──────────────────
        e0 = {n.node_id: n.query('E') for n in nodes if n.node_id}

        # ── 6. Collect streaming data ──────────────────────────────────────
        print(f"  Collecting {COLLECT_S}s …", end="", flush=True)
        stream = collect(COLLECT_S)
        print(" done")

        # ── 7. Snapshot firmware metrics after collection ──────────────────
        e1 = {n.node_id: n.query('E') for n in nodes if n.node_id}
        v1 = {n.node_id: n.query('V') for n in nodes if n.node_id}
        f1 = {n.node_id: n.query('F') for n in nodes if n.node_id}

        # ── 8. Gather lower-bound L per node ──────────────────────────────
        L_vals = {n.node_id: n.query('L') for n in nodes if n.node_id}

        # ── 9. Compute per-node results ────────────────────────────────────
        sc_result = {
            'label':  label,
            'occ':    (occ1, occ2, occ3),
            'costs':  (c1, c2, c3),
            'nodes':  {}
        }

        for nid in range(1, N_NODES + 1):
            d_ss = stream.get(nid, {'lux':[], 'duty':[], 't_lux':[], 't_duty':[]})
            lux_ss, duty_ss = steady_state_mean(d_ss)

            G, d_bg = cal_params.get(nid, (1.0, 0.0))
            L       = L_vals.get(nid) or 0.0
            E_fw    = max(0.0, (e1.get(nid) or 0) - (e0.get(nid) or 0))
            V_fw    = v1.get(nid) or 0.0
            F_fw    = f1.get(nid) or 0.0

            # Also compute E/V/F from streaming data (cross-check)
            E_s, V_s, F_s = compute_metrics(
                d_ss['lux'], d_ss['duty'], L, d_bg, G)

            sc_result['nodes'][nid] = {
                'lux_ss':  lux_ss,
                'duty_ss': duty_ss,
                'E_fw':    E_fw,    # firmware ΔE
                'V_fw':    V_fw,    # firmware running-avg V
                'F_fw':    F_fw,    # firmware running-avg F
                'E':       E_s,     # computed from stream
                'V':       V_s,
                'F':       F_s,
                'L':       L,
                'stream':  d_ss,
                'occ':     occs.get(nid, 0),
                'cost':    costs.get(nid, 1),
            }

            print(f"  Node {nid}: lux_ss={lux_ss:.2f}  duty_ss={duty_ss:.3f}  "
                  f"L={L:.1f}  E={E_s:.4f}J  V={V_s:.4f}  F={F_s:.4f}")

        results.append(sc_result)

    return results

# ── Console table ──────────────────────────────────────────────────────────────
def print_table(results: list):
    W = 110
    print("\n" + "═"*W)
    print("  RESULTS TABLE — SCDTR Phase 2 Consensus ADMM")
    print("═"*W)
    hdr = (f"  {'Scenario':<28}  {'N':>2}  {'Occ':>3}  {'Cost':>5}  "
           f"{'LUX_ss':>7}  {'Duty_ss':>7}  {'L[lux]':>7}  "
           f"{'E[J]':>8}  {'V[lux]':>8}  {'F':>9}")
    print(hdr)
    print("─"*W)
    for r in results:
        for nid in range(1, N_NODES+1):
            nd   = r['nodes'].get(nid, {})
            occ  = nd.get('occ', '-')
            cost = nd.get('cost', '-')
            row = (f"  {r['label']:<28}  {nid:>2}  {occ:>3}  {cost:>5}  "
                   f"{nd.get('lux_ss',0):>7.2f}  {nd.get('duty_ss',0):>7.3f}  "
                   f"{nd.get('L',0):>7.1f}  "
                   f"{nd.get('E',0):>8.5f}  {nd.get('V',0):>8.5f}  {nd.get('F',0):>9.5f}")
            print(row)
        print("─"*W)

# ── Plotting helpers ───────────────────────────────────────────────────────────
DARK   = "#1e1e2e"
PANEL  = "#13131f"
GRID   = "#444466"
FG     = "#cdd6f4"
PAL    = ["#89dceb", "#a6e3a1", "#f9e2af"]    # node 1/2/3
RED    = "#f38ba8"
ORG    = "#fab387"

def _style(ax):
    ax.set_facecolor(PANEL)
    ax.tick_params(colors=FG, labelsize=8)
    for sp in ax.spines.values():
        sp.set_edgecolor(GRID)
    ax.grid(True, alpha=0.18, color=GRID)
    ax.xaxis.label.set_color(FG)
    ax.yaxis.label.set_color(FG)
    ax.title.set_color(FG)

def _bar_group(ax, x, w, results, metric, node_ids=(1,2,3)):
    for i, nid in enumerate(node_ids):
        vals = [r['nodes'].get(nid,{}).get(metric, 0) for r in results]
        ax.bar(x + (i-1)*w, vals, w, label=f'Node {nid}',
               color=PAL[i], alpha=0.85, edgecolor=GRID, linewidth=0.5)

def _legend(ax):
    ax.legend(facecolor='#2a2a3e', labelcolor=FG, fontsize=8)

# ── Plot 1: Steady-state lux & duty vs occupancy (uniform costs) ───────────────
def plot_occupancy(results, out_dir):
    sc = [r for r in results if r['costs'] == (1,1,1)]
    if not sc:
        print("  [skip] plot_steady_state_occupancy — no uniform-cost scenarios")
        return

    labels = [r['label'].replace(' C=[1,1,1]', '') for r in sc]
    x = np.arange(len(labels)); w = 0.25

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.patch.set_facecolor(DARK)
    fig.suptitle("Steady State vs Occupancy Configuration  (c=[1,1,1])",
                 color=FG, fontsize=11)

    for ax, metric, ylabel, title in [
        (axes[0], 'lux_ss',  'Illuminance [lux]',
         'Steady-State Illuminance'),
        (axes[1], 'duty_ss', 'Duty Cycle  [0–1]',
         'Steady-State Duty Cycle'),
    ]:
        _style(ax)
        _bar_group(ax, x, w, sc, metric)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=25, ha='right', color=FG, fontsize=7.5)
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=10)
        _legend(ax)

    # Reference lines on lux plot
    axes[0].axhline(20, color=RED, lw=1.2, ls='--', alpha=0.7, label='ref_high=20')
    axes[0].axhline(10, color=ORG, lw=1.2, ls='--', alpha=0.7, label='ref_low=10')
    _legend(axes[0])

    plt.tight_layout()
    path = os.path.join(out_dir, "plot_steady_state_occupancy.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")

# ── Plot 2: Duty & energy vs cost vector (all HIGH occ) ───────────────────────
def plot_costs(results, out_dir):
    sc = [r for r in results if r['occ'] == (2,2,2)]
    if not sc:
        print("  [skip] plot_cost_comparison — no all-HIGH scenarios")
        return

    labels = [r['label'].replace('Occ(2,2,2) ', '') for r in sc]
    x = np.arange(len(labels)); w = 0.25

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    fig.patch.set_facecolor(DARK)
    fig.suptitle("Effect of Cost Vector  (all nodes HIGH occupancy, L=20 lux)",
                 color=FG, fontsize=11)

    for ax, metric, ylabel, title in [
        (axes[0], 'duty_ss', 'Duty Cycle',        'Duty Distribution'),
        (axes[1], 'lux_ss',  'Illuminance [lux]', 'Achieved Illuminance'),
        (axes[2], 'E',       'Energy [J]',         f'Energy  ({COLLECT_S}s window)'),
    ]:
        _style(ax)
        _bar_group(ax, x, w, sc, metric)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=20, ha='right', color=FG, fontsize=7.5)
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=10)
        _legend(ax)

    axes[1].axhline(25, color=RED, lw=1, ls='--', alpha=0.6)

    plt.tight_layout()
    path = os.path.join(out_dir, "plot_cost_comparison.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")

# ── Plot 3: E / V / F per scenario ────────────────────────────────────────────
def plot_metrics(results, out_dir):
    x = np.arange(len(results)); w = 0.25

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.patch.set_facecolor(DARK)
    fig.suptitle("Performance Metrics per Scenario", color=FG, fontsize=11)

    for ax, metric, ylabel, title in [
        (axes[0], 'E', f'Energy [J]  ({COLLECT_S}s)',     'Energy'),
        (axes[1], 'V', 'Visibility Error [lux]',           'Visibility Error  V'),
        (axes[2], 'F', 'Flicker  [1/s]',                   'Flicker  F'),
    ]:
        _style(ax)
        _bar_group(ax, x, w, results, metric)
        ax.set_xticks(x)
        ax.set_xticklabels([f"T{i+1}" for i in range(len(results))],
                           color=FG, fontsize=9)
        ax.set_ylabel(ylabel)
        ax.set_title(title, fontsize=10)
        _legend(ax)

    # Legend table below (map T1…T10 → label)
    legend_text = "\n".join(
        f"T{i+1}: {r['label']}" for i, r in enumerate(results))
    fig.text(0.01, -0.04, legend_text, fontsize=6.5, color=FG,
             fontfamily='monospace', va='top')

    plt.tight_layout()
    path = os.path.join(out_dir, "plot_metrics.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")

# ── Plot 4: Time series (lux + duty) for selected scenarios ───────────────────


def plot_timeseries(results, out_dir):
    """Generates time-series plots for each scenario."""
    for idx, r in enumerate(results):
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True, facecolor=DARK)
        fig.suptitle(f"Time Series — {r['label']}", color=FG, fontsize=12)
        
        # LUX plot
        axes[0].set_facecolor(PANEL)
        for nid in range(1, N_NODES + 1):
            node_res = r['nodes'].get(nid)
            if node_res:
                data = node_res['stream']
                if data and data['t_lux']:
                    axes[0].plot(data['t_lux'], data['lux'], label=f"Node {nid}  occ={node_res['occ']}  c={node_res['cost']}", alpha=0.9)
        
        axes[0].axhline(20, color=RED, lw=0.9, ls='--', alpha=0.6, label='ref_high=20')
        axes[0].axhline(10, color=ORG, lw=0.9, ls='--', alpha=0.6, label='ref_low=10')
        axes[0].set_ylabel("Illuminance [lux]", color=FG)
        axes[0].set_title("LUX (dashed = lower bounds 10 / 20)", fontsize=9, color=FG)
        axes[0].legend(loc='upper right', fontsize=8, facecolor=DARK, labelcolor=FG)
        axes[0].grid(True, color=GRID, alpha=0.3)
        axes[0].tick_params(colors=FG)

        # Duty plot
        axes[1].set_facecolor(PANEL)
        for nid in range(1, N_NODES + 1):
            node_res = r['nodes'].get(nid)
            if node_res:
                data = node_res['stream']
                if data and data['t_duty']:
                    axes[1].plot(data['t_duty'], data['duty'], label=f"Node {nid}", alpha=0.9)
        axes[1].set_ylabel("Duty Cycle", color=FG)
        axes[1].set_xlabel("Time [s]", color=FG)
        axes[1].set_ylim(-0.05, 1.05)
        axes[1].set_title("Duty Cycles", fontsize=9, color=FG)
        axes[1].grid(True, color=GRID, alpha=0.3)
        axes[1].tick_params(colors=FG)

        plt.tight_layout()
        safe_label = r['label'].replace(" ", "_").replace("(", "").replace(")", "").replace("=", "").replace(",", "").replace("[", "").replace("]", "")
        fname = os.path.join(out_dir, f"plot_ts_{safe_label}.png")
        fig.savefig(fname, dpi=150, bbox_inches='tight', facecolor=DARK)
        plt.close(fig)
        print(f"  Saved: {fname}")



def run_convergence_study(out_dir):
    print("\n" + "="*60)
    print("  RESEARCH: ADMM Convergence Study (Occ 2,2,2)")
    print("="*60)
    
    # Fetch calibration parameters once per node
    cal_params = {}
    for n in nodes:
        if n.node_id:
            G, d = parse_cal_params(n)
            cal_params[n.node_id] = (G, d)

    # Set all nodes to High Occupancy
    for n in nodes:
        n.write(f"o {n.node_id} 2")
    time.sleep(1.0)
    
    convergence_results = []
    
    for iters in CONVERGENCE_STEPS:
        print(f"  Testing {iters} iterations...")
        for n in nodes: n.admm_done = False
        
        # Trigger ADMM with specific iteration count
        node_by_id[1].write(f"T {iters}")
        
        if wait_admm(ADMM_WAIT_S):
            time.sleep(2) # settle PI
            stream = collect(5.0) # short collection
            
            total_v = 0.0
            total_e = 0.0
            for nid in node_by_id:
                d_ss = stream.get(nid, {'lux':[], 'duty':[], 't_lux':[], 't_duty':[]})
                G, d_bg = cal_params.get(nid, (1.0, 0.0))
                # Get L for this node
                n = node_by_id[nid]
                L = n.query('L') or 20.0
                
                E_s, V_s, F_s = compute_metrics(d_ss['lux'], d_ss['duty'], L, d_bg, G)
                total_v += V_s
                total_e += E_s
            
            convergence_results.append({
                'iters': iters,
                'visibility_error': total_v,
                'energy': total_e
            })
            print(f"    Total V={total_v:.2f}  Total E={total_e:.4f}")
        else:
            print("    Timeout!")

    # Plot results
    if not convergence_results: return
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    fig.patch.set_facecolor(DARK)
    ax1.set_facecolor(PANEL)
    
    x = [r['iters'] for r in convergence_results]
    y_v = [r['visibility_error'] for r in convergence_results]
    y_e = [r['energy'] for r in convergence_results]
    
    color = '#f38ba8' # red
    ax1.set_xlabel('ADMM Iterations', color=FG)
    ax1.set_ylabel('Total Visibility Error [lux]', color=color)
    ax1.plot(x, y_v, 'o-', color=color, lw=2, label='Visibility Error')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.grid(True, color=GRID, alpha=0.3)
    
    ax2 = ax1.twinx()
    color = '#89dceb' # sky
    ax2.set_ylabel('Total Energy [J]', color=color)
    ax2.plot(x, y_e, 's--', color=color, lw=1.5, alpha=0.7, label='Energy')
    ax2.tick_params(axis='y', labelcolor=color)
    
    plt.title("ADMM Convergence Study: Iterations vs. Performance", color=FG)
    for ax in [ax1, ax2]:
        ax.tick_params(colors=FG)
        for spine in ax.spines.values(): spine.set_color(GRID)

    fname = os.path.join(out_dir, "plot_convergence_research.png")
    fig.savefig(fname, dpi=150, facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {fname}")
    # Pick one interesting scenario from each group
    picks = []
    for label_key in ["Occ(2,2,2) C=[1,1,1]",
                      "Occ(2,2,2) C=[1,10,1]",
                      "Occ(1,2,1) C=[1,1,1]"]:
        for r in results:
            if r['label'] == label_key:
                picks.append(r)
                break

    for sc in picks:
        fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
        fig.patch.set_facecolor(DARK)
        fig.suptitle(f"Time Series — {sc['label']}", color=FG, fontsize=11)

        for ax in axes:
            _style(ax)

        has_data = False
        for i, nid in enumerate([1, 2, 3]):
            nd = sc['nodes'].get(nid, {})
            d  = nd.get('stream', {})
            occ  = nd.get('occ', 0)
            cost = nd.get('cost', 1)

            if d.get('t_lux') and d.get('lux'):
                t0 = d['t_lux'][0]
                t  = [x - t0 for x in d['t_lux']]
                axes[0].plot(t, d['lux'],
                             color=PAL[i], lw=1.2,
                             label=f'Node {nid}  occ={occ}  c={cost}')
                has_data = True

            if d.get('t_duty') and d.get('duty'):
                t0 = d['t_duty'][0]
                t  = [x - t0 for x in d['t_duty']]
                axes[1].plot(t, d['duty'],
                             color=PAL[i], lw=1.2, label=f'Node {nid}')

        axes[0].axhline(20, color=RED, lw=0.9, ls='--', alpha=0.6, label='ref_high=20')
        axes[0].axhline(10, color=ORG, lw=0.9, ls='--', alpha=0.6, label='ref_low=10')
        axes[0].set_ylabel('Illuminance [lux]')
        axes[0].set_title('LUX (dashed = lower bounds 10 / 20)', fontsize=9)
        axes[0].set_ylim(bottom=0)
        _legend(axes[0])

        axes[1].set_ylabel('Duty Cycle')
        axes[1].set_xlabel('Time  [s]')
        axes[1].set_title('Duty Cycles', fontsize=9)
        axes[1].set_ylim(-0.02, 1.05)
        _legend(axes[1])

        if not has_data:
            plt.close(fig)
            continue

        plt.tight_layout()
        safe = (sc['label']
                .replace(' ', '_').replace(',', '').replace('=','')
                .replace('[','').replace(']','').replace('(','').replace(')',''))
        path = os.path.join(out_dir, f"plot_ts_{safe}.png")
        fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
        plt.close(fig)
        print(f"  Saved: {path}")

# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    out_dir = os.path.dirname(os.path.abspath(__file__))

    # ── 1. Resolve ports ───────────────────────────────────────────────────
    if len(sys.argv) > 1:
        ports = sys.argv[1:]
    else:
        ports = sorted([
            p.device for p in serial.tools.list_ports.comports()
            if any(k in (p.description or "").lower()
                      or k in (p.hwid or "").lower()
                   for k in ["pico", "rp2040", "2e8a", "ttyacm"])
        ])[:N_NODES]

    if not ports:
        print("ERROR: No Pico ports detected.")
        print("Usage: python test_suite.py [/dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2]")
        sys.exit(1)

    print(f"\nPorts: {ports}")
    print("Starting serial readers…")

    for p in ports:
        n = Node(p)
        nodes.append(n)
        threading.Thread(target=reader_thread, args=(n,), daemon=True).start()

    # ── 2. Wait for node IDs ────────────────────────────────────────────────
    print(f"\nWaiting for node IDs (up to {ID_TIMEOUT_S}s)…")
    deadline = time.time() + ID_TIMEOUT_S
    while time.time() < deadline and not all(n.node_id for n in nodes):
        # Use 'i' command to get local ID (bypasses hub_forward)
        for n in nodes:
            if n.node_id is None and n.connected:
                n.ser.write(b"\n") # clear partial commands
                time.sleep(0.1)
                n.write("i")
        time.sleep(1)

    for n in nodes:
        if n.node_id:
            node_by_id[n.node_id] = n
            print(f"  {n.port} → Node {n.node_id}")
        else:
            print(f"  {n.port} → UNKNOWN (continuing anyway)")

    if not node_by_id:
        print("ERROR: No node IDs detected. Exiting.")
        sys.exit(1)

    # ── 3. Wait for calibration ────────────────────────────────────────────
    print(f"\nWaiting for calibration (up to {CAL_TIMEOUT_S}s)…")
    # Ask if already calibrated
    for n in nodes:
        if n.node_id:
            n.write("c ?")
    time.sleep(1.0)

    # Unconditionally trigger 3-node calibration
    print("  Triggering calibration...")
    for n in nodes:
        n.cal_done = False
    
    # Repeatedly kick calibration to ensure Node 1 catches it
    for _ in range(5):
        node_by_id[1].write("c")
        time.sleep(1.0)
    
    CAL_TIMEOUT_S = 180 
    deadline = time.time() + CAL_TIMEOUT_S
    while time.time() < deadline:
        if all(n.cal_done for n in nodes):
            print("\n  Calibration complete.")
            # Verify non-zero gains
            node_by_id[1].write("c ?")
            time.sleep(1.0)
            break
        remaining = int(deadline - time.time())
        print(f"  Calibrating… {CAL_TIMEOUT_S - remaining}s elapsed", end="\r")
        time.sleep(1)
    else:
        print("\n  WARNING: Calibration timed out.")

    # ── 4. Enable feedback on all nodes ────────────────────────────────────
    print("Enabling feedback…")
    for nid in range(1, N_NODES + 1):
        send(nid, f"f {nid} 1")
    time.sleep(1.5)

    # ── 5. Run test scenarios ───────────────────────────────────────────────
    print(f"\nRunning {len(SCENARIOS)} scenarios "
          f"({ADMM_WAIT_S}s ADMM+settle + {COLLECT_S}s collect each)\n"
          f"Estimated total: ~{len(SCENARIOS)*(ADMM_WAIT_S+COLLECT_S)//60+1} min")

    results = run_tests()

    # ── 6. Print results table ──────────────────────────────────────────────
    print_table(results)

    # ── 7. Generate plots ───────────────────────────────────────────────────
    print("\nGenerating plots…")
    plot_occupancy(results, out_dir)
    plot_costs(results, out_dir)
    plot_metrics(results, out_dir)
    plot_timeseries(results, out_dir)

    run_convergence_study(out_dir)
    print(f"\n✓ Done. Plots saved to: {out_dir}")
    print("Files: plot_steady_state_occupancy.png  plot_cost_comparison.png"
          "  plot_metrics.png  plot_ts_*.png")
