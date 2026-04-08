#!/usr/bin/env python3
"""
SCDTR Phase 2 — Automated Test Suite
======================================
Connects to one or more Pico nodes, uses the first connected node as a CAN hub
for any nodes that are not USB-connected, waits for calibration, then runs 10
test scenarios covering occupancy sweeps and cost-vector sweeps.

Produces:
  • Console table (steady-state lux, duty, ΔE, V, F per node per test)
  • plot_steady_state_occupancy.png  — lux & duty vs occupancy config
  • plot_cost_comparison.png         — duty & energy vs cost vector
  • plot_metrics.png                 — E / V / F across all scenarios
  • plot_ts_<scenario>.png           — time-series for selected tests

Usage:
    python test_suite.py                                       # auto-detect ports
    python test_suite.py /dev/ttyACM0                          # one USB hub
    python test_suite.py /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2
    python test_suite.py --reboot /dev/ttyACM0                  # reboot all nodes first
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
REBOOT_SYSTEM  = False

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
    ("Occ(2,2,2) C=[5,1,5]",  2,2,2,  5, 1, 5),
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
        open_deadline = time.time() + 10.0
        last_error = None
        while time.time() < open_deadline:
            try:
                node.ser = serial.Serial(node.port, BAUD, timeout=0.2)
                break
            except Exception as e:
                last_error = e
                time.sleep(0.5)
        else:
            print(f"  [{node.port}] error: {last_error}")
            return

        # Read for 2.5 s instead of sleeping blindly: this captures the
        # [BOOT] LUMINAIRE=N line the Pico prints on serial open (a DTR
        # toggle can trigger a reset).  reset_input_buffer() would erase it.
        boot_deadline = time.time() + 2.5
        while time.time() < boot_deadline:
            try:
                raw = node.ser.readline().decode(errors="ignore").strip()
            except Exception:
                break
            if not raw:
                continue
            with node.lock:
                node._lines.append(raw)
                if raw.startswith(("[BOOT]", "[WAKEUP]", "[RX] WAKEUP", "[CAL]", "[ADMM]", "[CTRL]")):
                    print(f"  [{node.port}] {raw}")
                if node.node_id is None:
                    m = re.search(r'\[BOOT\] LUMINAIRE=(\d+)', raw)
                    if m:
                        node.node_id = int(m.group(1))
                        print(f"  [{node.port}] → Node {node.node_id}")
                if "[CAL] Calibration complete" in raw:
                    node.cal_done = True
                if any(tag in raw for tag in
                       ("[ADMM] converged", "[ADMM] done",
                        "[ADMM] maxiter", "[ADMM] hardmax")):
                    node.admm_done = True
        node.ser.timeout = 1  # normal blocking readline timeout
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

                    if raw.startswith(("[BOOT]", "[WAKEUP]", "[RX] WAKEUP", "[CAL]", "[ADMM]", "[CTRL]")):
                        print(f"  [{node.port}] {raw}")

                    # ── Node ID from boot message ───────────────────────────
                    if node.node_id is None:
                        m = re.search(r'\[BOOT\] LUMINAIRE=(\d+)', raw)
                        if m:
                            node.node_id = int(m.group(1))
                            print(f"  [{node.port}] → Node {node.node_id}")

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
                                src_id = int(parts[2])
                                val = float(parts[3])
                                if 1 <= src_id <= N_NODES:
                                    with stream_lock:
                                        if var == 'y':
                                            stream_buffers[src_id]['lux'].append((t, val))
                                        elif var == 'u':
                                            stream_buffers[src_id]['duty'].append((t, val))
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
stream_lock = threading.Lock()
stream_buffers = {
    nid: {'lux': [], 'duty': []}
    for nid in range(1, N_NODES + 1)
}

def hub_node() -> "Node | None":
    """Return the serial-connected node used as the CAN hub."""
    for n in nodes:
        if n.connected:
            return n
    return None

def route_node(nid: int) -> "Node | None":
    """Use direct serial if available, otherwise send via the connected hub."""
    n = node_by_id.get(nid)
    if n and n.connected:
        return n
    return hub_node()

def clear_stream_buffers():
    with stream_lock:
        for b in stream_buffers.values():
            b['lux'].clear()
            b['duty'].clear()
    for n in nodes:
        n.lux_buf.clear()
        n.duty_buf.clear()

def stream_snapshot() -> dict[int, dict]:
    with stream_lock:
        return {
            nid: {
                't_lux':  [x[0] for x in b['lux']],
                'lux':    [x[1] for x in b['lux']],
                't_duty': [x[0] for x in b['duty']],
                'duty':   [x[1] for x in b['duty']],
            }
            for nid, b in stream_buffers.items()
        }

def start_streams():
    for nid in range(1, N_NODES + 1):
        send(nid, f"s y {nid}")
        send(nid, f"s u {nid}")

def stop_streams():
    for nid in range(1, N_NODES + 1):
        send(nid, f"S y {nid}")
        send(nid, f"S u {nid}")

def send(nid: int, cmd: str):
    """Send command to node nid (hub-forward if needed)."""
    n = route_node(nid)
    if n:
        n.write(cmd)
    else:
        print(f"  [warn] no connected hub available for command: {cmd}")

def query_node(nid: int, var: str, timeout=4.0) -> "float | None":
    """Query any node through direct serial if present, otherwise through hub CAN forwarding."""
    n = route_node(nid)
    if n is None:
        return None
    n.flush()
    n.write(f"g {var} {nid}")
    line = n.wait_line(rf"^{var} {nid}\b", timeout=timeout)
    if line:
        try:
            return float(line.split()[2])
        except (IndexError, ValueError):
            pass
    return None

def query_str_node(nid: int, var: str, timeout=4.0) -> "str | None":
    n = route_node(nid)
    if n is None:
        return None
    n.flush()
    n.write(f"g {var} {nid}")
    return n.wait_line(rf"^{var} {nid}\b", timeout=timeout)

def send_reboot_to_ports(ports: list[str]):
    """Send the firmware R command, then close serial so Windows can re-enumerate."""
    print("Sending R reboot command before starting serial readers...")
    for port in ports:
        try:
            with serial.Serial(port, BAUD, timeout=1.0) as ser:
                # Opening the Pico serial port can reset the RP2040. Wait long
                # enough for the firmware to start reading serial before
                # sending R, otherwise the command can be lost and only the USB
                # node reboots.
                time.sleep(3.0)
                while ser.in_waiting:
                    ser.readline()
                ser.write(b"R\n")
                ser.flush()
                print(f"  [{port}] R sent")
                time.sleep(0.5)
        except Exception as e:
            print(f"  [{port}] could not send R: {e}")
    # Keep the port closed while Windows re-enumerates and the CAN nodes reboot.
    time.sleep(4.0)

def trigger_admm_via_cost(costs: dict = None, target_nid: int = 1):
    """Trigger ADMM using an implemented parameter command.

    The firmware starts ADMM when `C <i> <cost>` is applied.  Re-sending the
    already-staged cost gives a clean final trigger after all scenario
    parameters have been staged, without using any test-only command.
    """
    if costs and target_nid in costs:
        cost = costs[target_nid]
    else:
        cost = query_node(target_nid, 'C') or 1.0
    send(target_nid, f"C {target_nid} {cost}")

def establish_coordinated_state(occs: dict, costs: dict, settle_s: float = 3.0):
    """Stage a complete scenario and let the implemented ADMM path settle."""
    for n in nodes:
        n.admm_done = False
    for nid in range(1, N_NODES + 1):
        send(nid, f"C {nid} {costs[nid]}")
    time.sleep(0.5)
    for nid in range(1, N_NODES + 1):
        send(nid, f"o {nid} {occs[nid]}")
    time.sleep(0.3)
    wait_admm(ADMM_WAIT_S)
    time.sleep(0.5)
    for n in nodes:
        n.admm_done = False
    trigger_admm_via_cost(costs, target_nid=1)
    wait_admm(ADMM_WAIT_S)
    time.sleep(settle_s)

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
    clear_stream_buffers()
    start_streams()

    time.sleep(duration_s)

    stop_streams()
    time.sleep(0.3)

    return stream_snapshot()

# ── Metric computation from streaming data ─────────────────────────────────────
def compute_metrics(lux_data: list, duty_data: list,
                    L: float, d: float, G: float,
                    h: float = SAMPLE_H) -> tuple:
    """
    Compute E, V, F from streaming data using equations (14)-(16) from PDF.

    E = Σ u_k · P_max · h
    V = (1/T) Σ max(0, L − ŷ_k)  where ŷ_k is MEASURED lux (includes cross-coupling)
    F = 1/(T·h) · Σ_{Δu_k·Δu_{k−1}<0} (|Δu_k| + |Δu_{k−1}|)

    V uses actual measured lux (not the single-node estimate d+G·u) so that spill
    from neighbouring LEDs is correctly accounted for.
    """
    T = len(duty_data)
    if T == 0:
        return 0.0, 0.0, 0.0

    u = np.array(duty_data, dtype=float)

    # Energy (J)
    E = float(np.sum(u) * MAXIMUM_POWER * h)

    # Visibility error — use actual measured lux when available
    if lux_data:
        y = np.array(lux_data[:T], dtype=float)   # align to duty length
        V = float(np.mean(np.maximum(0.0, L - y)))
    else:
        x_hat = d + G * u
        V = float(np.mean(np.maximum(0.0, L - x_hat)))

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
    time.sleep(1.0)
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

def parse_cal_params_node(nid: int) -> tuple:
    """Return (self gain, background) for a node.

    Full `c ?` output is only local firmware output, so with a single USB hub
    this is exact for the connected hub node and a safe fallback for remote
    nodes. Metrics use measured lux when available, so these fallbacks only
    affect model-line annotations and missing-lux fallback calculations.
    """
    n = node_by_id.get(nid)
    if n and n.connected:
        return parse_cal_params(n)
    d = query_node(nid, 'd') or 0.0
    return (1.0, d)

def dump_coupling_gains():
    """Print the full 3x3 coupling-gain matrix (k[i][j]) from each node.
    Uses 'g k <nid>' which prints: k <nid> <k[nid][1]> <k[nid][2]> <k[nid][3]>
    This is essential for diagnosing ADMM infeasibility when a node can't reach its target.
    """
    print("\n  Coupling-gain matrix  k[i][j]  (node i's sensor, node j's LED):")
    for nid in range(1, N_NODES + 1):
        n = node_by_id.get(nid)
        if not n:
            print(f"    Node {nid}: unavailable through generic MSG_GET (connect USB for full k row)")
            continue
        n.flush()
        n.write(f"g k {nid}")
        line = n.wait_line(rf"^k {nid}\b", timeout=3.0)
        if line:
            parts = line.split()
            if len(parts) >= 5:
                try:
                    k1, k2, k3 = float(parts[2]), float(parts[3]), float(parts[4])
                    print(f"    Node {nid}: k[{nid}][1]={k1:.4f}  k[{nid}][2]={k2:.4f}  k[{nid}][3]={k3:.4f}  "
                          f"(self-gain k[{nid}][{nid}]={[k1,k2,k3][nid-1]:.4f})")
                except ValueError:
                    print(f"    Node {nid}: parse error: {line}")
        else:
            print(f"    Node {nid}: no response")

# ── Run tests ──────────────────────────────────────────────────────────────────
def run_tests() -> list:
    results = []

    # Let any in-progress ADMM from a previous run drain before starting
    print("  Waiting for nodes to idle…")
    for nid in range(1, N_NODES + 1):
        send(nid, f"o {nid} 0")  # all-off occupancy → ADMM will run and settle
    time.sleep(ADMM_WAIT_S + 2)  # wait for ADMM + PI to reach steady state

    # Fetch calibration parameters once per node (they don't change)
    cal_params = {}
    for nid in range(1, N_NODES + 1):
        G, d = parse_cal_params_node(nid)
        cal_params[nid] = (G, d)
        print(f"  Node {nid}: G={G:.3f} d={d:.2f}")

    for sc_idx, (label, occ1, occ2, occ3, c1, c2, c3) in enumerate(SCENARIOS):
        print(f"\n{'━'*62}")
        print(f"  Test {sc_idx+1}/{len(SCENARIOS)}: {label}")
        print(f"{'━'*62}")

        occs  = {1: occ1, 2: occ2, 3: occ3}
        costs = {1: c1,   2: c2,   3: c3}

        # ── 1. Reset ADMM flags before issuing commands ────────────────────
        for n in nodes:
            n.admm_done = False

        # ── 2. Stage cost coefficients ─────────────────────────────────────
        # Any C/o command can start ADMM immediately in the firmware.  That
        # first run may see only part of this scenario's staged parameters, so
        # we drain it below and then re-trigger with a final C command.
        for nid in range(1, N_NODES + 1):
            send(nid, f"C {nid} {costs[nid]}")
        time.sleep(0.5)

        # ── 3. Stage occupancy ─────────────────────────────────────────────
        for nid in range(1, N_NODES + 1):
            send(nid, f"o {nid} {occs[nid]}")
        time.sleep(0.3)

        # Drain any stale ADMM run, then start a clean run through C.
        wait_admm(ADMM_WAIT_S)
        time.sleep(0.5)   # brief settle before re-trigger

        # ── 3b. Re-trigger ADMM using C — now every node has the correct state
        for n in nodes:
            n.admm_done = False
        trigger_admm_via_cost(costs, target_nid=1)

        # ── 4. Wait for correct ADMM + PI settle ──────────────────────────
        converged = wait_admm(ADMM_WAIT_S)
        print(f"  ADMM: {'converged ✓' if converged else 'timeout (continuing)'}")
        # Extra settle for PI to reach steady state
        time.sleep(4)

        # ── 5. Snapshot firmware energy before collection ──────────────────
        e0 = {nid: query_node(nid, 'E') for nid in range(1, N_NODES + 1)}

        # ── 6. Collect streaming data ──────────────────────────────────────
        print(f"  Collecting {COLLECT_S}s …", end="", flush=True)
        stream = collect(COLLECT_S)
        print(" done")

        # ── 7. Snapshot firmware metrics after collection ──────────────────
        e1 = {nid: query_node(nid, 'E') for nid in range(1, N_NODES + 1)}
        v1 = {nid: query_node(nid, 'V') for nid in range(1, N_NODES + 1)}
        f1 = {nid: query_node(nid, 'F') for nid in range(1, N_NODES + 1)}

        # ── 8. Gather lower-bound L per node ──────────────────────────────
        L_vals = {nid: query_node(nid, 'L') for nid in range(1, N_NODES + 1)}

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

    axes[1].axhline(20, color=RED, lw=1, ls='--', alpha=0.6, label='L=20')
    _legend(axes[1])

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
                col = PAL[nid - 1]
                if data and data['t_lux']:
                    axes[0].plot(data['t_lux'], data['lux'], color=col, lw=1.2,
                                 label=f"Node {nid}  occ={node_res['occ']}  c={node_res['cost']}",
                                 alpha=0.9)
                L = node_res.get('L', 0)
                if L > 0:
                    axes[0].axhline(L, color=col, lw=0.9, ls='--', alpha=0.5,
                                    label=f"L{nid}={L:.0f} lux")
        axes[0].set_ylabel("Illuminance [lux]", color=FG)
        axes[0].set_title("LUX (dashed = lower bound L per node)", fontsize=9, color=FG)
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
                    axes[1].plot(data['t_duty'], data['duty'], color=PAL[nid - 1],
                                 lw=1.2, label=f"Node {nid}", alpha=0.9)
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



def run_single_admm(occs: dict, costs: dict, label: str) -> dict:
    """
    Run one fresh ADMM scenario and return results in the same format as run_tests().
    Used to get a freshly-collected ADMM result for back-to-back comparison.
    """
    cal_params_local = {}
    for nid in range(1, N_NODES + 1):
        cal_params_local[nid] = parse_cal_params_node(nid)

    for n in nodes:
        n.admm_done = False

    for nid in range(1, N_NODES + 1):
        send(nid, f"C {nid} {costs[nid]}")
    time.sleep(0.5)

    # Stage occupancy, drain any stale ADMM, then re-trigger with all parameters
    # staged. Same protocol as run_tests().
    for nid in range(1, N_NODES + 1):
        send(nid, f"o {nid} {occs[nid]}")
    time.sleep(0.3)
    wait_admm(ADMM_WAIT_S)
    time.sleep(0.5)

    for n in nodes:
        n.admm_done = False
    trigger_admm_via_cost(costs, target_nid=1)
    wait_admm(ADMM_WAIT_S)
    # Do not send f <i> 1 here. admm_apply_result() already sets the local
    # reference and seeds the PID integrator with the ADMM consensus duty.
    time.sleep(6)

    stream = collect(COLLECT_S)
    L_vals = {nid: query_node(nid, 'L') for nid in range(1, N_NODES + 1)}

    result = {'label': label,
              'occ':   tuple(occs[i]  for i in range(1, 4)),
              'costs': tuple(costs[i] for i in range(1, 4)),
              'nodes': {}}

    for nid in range(1, N_NODES + 1):
        d_ss = stream.get(nid, {'lux': [], 'duty': [], 't_lux': [], 't_duty': []})
        lux_ss, duty_ss = steady_state_mean(d_ss)
        G, d_bg = cal_params_local.get(nid, (1.0, 0.0))
        L = L_vals.get(nid) or 0.0
        E_s, V_s, F_s = compute_metrics(d_ss['lux'], d_ss['duty'], L, d_bg, G)
        result['nodes'][nid] = {
            'lux_ss': lux_ss, 'duty_ss': duty_ss,
            'E': E_s, 'V': V_s, 'F': F_s, 'L': L,
            'stream': d_ss, 'occ': occs[nid], 'cost': costs[nid],
        }
    return result


def run_baseline(occs: dict, label: str, costs: dict = None) -> dict:
    """
    Non-coordinated control: PI only, reference set directly from occupancy.
    No ADMM is triggered. Used as a comparison baseline.

    costs — the cost vector for the scenario (stored in results for weighted-cost
            computation; PI does not use costs to control, but we need them to
            apply the same Σ cᵢ·Eᵢ weighting as the ADMM result).
    """
    if costs is None:
        costs = {nid: 1 for nid in range(1, N_NODES + 1)}

    REF_HIGH = 20.0
    REF_LOW  = 10.0
    SETTLE_S = 8.0

    # Set reference then re-enable feedback with a fresh integrator seed.
    # r <i> <ref> is sent first so that the subsequent f <i> 1 finds the correct
    # reference when it calls apply_feedforward(r).
    for nid in range(1, N_NODES + 1):
        ref = REF_HIGH if occs[nid] == 2 else (REF_LOW if occs[nid] == 1 else 0.0)
        send(nid, f"r {nid} {ref}")   # set reference first so apply_feedforward uses it
        send(nid, f"f {nid} 1")       # re-enable feedback + seed integrator
    time.sleep(SETTLE_S)

    stream = collect(COLLECT_S)
    # Baseline deliberately uses `r` instead of `o` so it does not trigger
    # ADMM. Therefore `g L` may still reflect the previous occupancy state; use
    # the intended lower bound from this baseline scenario for metrics/plots.
    L_vals = {
        nid: REF_HIGH if occs[nid] == 2 else (REF_LOW if occs[nid] == 1 else 0.0)
        for nid in range(1, N_NODES + 1)
    }

    result = {'label': label, 'occ': tuple(occs[i] for i in range(1, 4)),
              'costs': tuple(costs[i] for i in range(1, 4)), 'nodes': {}}

    cal_params_local = {}
    for nid in range(1, N_NODES + 1):
        cal_params_local[nid] = parse_cal_params_node(nid)

    for nid in range(1, N_NODES + 1):
        d_ss  = stream.get(nid, {'lux': [], 'duty': [], 't_lux': [], 't_duty': []})
        lux_ss, duty_ss = steady_state_mean(d_ss)
        G, d_bg = cal_params_local.get(nid, (1.0, 0.0))
        L = L_vals.get(nid) or 0.0
        E_s, V_s, F_s = compute_metrics(d_ss['lux'], d_ss['duty'], L, d_bg, G)
        result['nodes'][nid] = {
            'lux_ss': lux_ss, 'duty_ss': duty_ss,
            'E': E_s, 'V': V_s, 'F': F_s, 'L': L,
            'stream': d_ss, 'occ': occs[nid], 'cost': costs[nid],
        }
    return result


def plot_comparison(admm_result: dict, baseline_result: dict, out_dir: str):
    """2×2 grid: LUX and duty for ADMM (left) vs PI-only (right)."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 9), sharex='col')
    fig.patch.set_facecolor(DARK)
    fig.suptitle(
        f"Coordinated (ADMM) vs Non-coordinated (PI only)  —  {admm_result['label']}",
        color=FG, fontsize=12)

    col_data = [admm_result, baseline_result]
    col_titles = ["ADMM (coordinated)", "PI only (non-coordinated)"]

    for col, (res, title) in enumerate(zip(col_data, col_titles)):
        ax_lux  = axes[0][col]
        ax_duty = axes[1][col]
        ax_lux.set_facecolor(PANEL)
        ax_duty.set_facecolor(PANEL)
        _style(ax_lux)
        _style(ax_duty)

        total_E = 0.0
        sum_V   = 0.0
        sum_F   = 0.0
        n_nodes = 0

        for i, nid in enumerate(range(1, N_NODES + 1)):
            nd  = res['nodes'].get(nid, {})
            d   = nd.get('stream', {})
            col_c = PAL[i]

            if d.get('t_lux') and d.get('lux'):
                t0 = d['t_lux'][0]
                t  = [x - t0 for x in d['t_lux']]
                ax_lux.plot(t, d['lux'], color=col_c, lw=1.2,
                            label=f"Node {nid}  occ={nd.get('occ',0)}  c={nd.get('cost',1)}")
            L = nd.get('L', 0)
            if L > 0:
                ax_lux.axhline(L, color=col_c, lw=0.9, ls='--', alpha=0.5,
                               label=f"L{nid}={L:.0f} lux")

            if d.get('t_duty') and d.get('duty'):
                t0 = d['t_duty'][0]
                t  = [x - t0 for x in d['t_duty']]
                ax_duty.plot(t, d['duty'], color=col_c, lw=1.2, label=f"Node {nid}")

            total_E += nd.get('E', 0)
            sum_V   += nd.get('V', 0)
            sum_F   += nd.get('F', 0)
            n_nodes += 1

        mean_V = sum_V / max(n_nodes, 1)
        mean_F = sum_F / max(n_nodes, 1)
        summary = f"E={total_E:.4f} J\nV={mean_V:.4f} lux\nF={mean_F:.4f} /s"
        ax_lux.text(0.97, 0.97, summary, transform=ax_lux.transAxes,
                    fontsize=7.5, va='top', ha='right', color=FG,
                    bbox=dict(facecolor='#2a2a3e', alpha=0.7, edgecolor=GRID, pad=3))

        ax_lux.set_title(title, fontsize=10, color=FG)
        ax_lux.set_ylabel("Illuminance [lux]" if col == 0 else "", color=FG)
        ax_lux.set_ylim(bottom=0)
        _legend(ax_lux)

        ax_duty.set_ylabel("Duty Cycle" if col == 0 else "", color=FG)
        ax_duty.set_xlabel("Time [s]", color=FG)
        ax_duty.set_ylim(-0.05, 1.1)
        _legend(ax_duty)

    plt.tight_layout()
    path = os.path.join(out_dir, "plot_comparison_coordinated_vs_baseline.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")


def run_disturbance_test(disturber_nid: int, out_dir: str):
    """
    Disturbance rejection test.
    Assumes all nodes are already in coordinated (ADMM) steady state.

    Timeline:
      0 – 5 s   : steady-state collection
      5 – 11 s  : disturber duty forced to 0.8 (feedback off)
      11 – 17 s : feedback restored on disturber, PI recovers
    """
    print("\n" + "=" * 60)
    print(f"  DISTURBANCE REJECTION TEST  (disturber = Node {disturber_nid})")
    print("=" * 60)

    T_STEADY  = 5.0
    T_DISTURB = 6.0
    T_RECOVER = 6.0
    t_disturb_rel = T_STEADY
    t_restore_rel = T_STEADY + T_DISTURB

    # Clear buffers and start streaming through the hub/direct serial routes.
    clear_stream_buffers()
    start_streams()

    # Phase 1: steady state
    print(f"  Collecting {T_STEADY:.0f}s steady state…")
    time.sleep(T_STEADY)

    # Phase 2: apply disturbance (disable feedback → force duty)
    print(f"  Disturbance ON — Node {disturber_nid} duty → 0.8")
    send(disturber_nid, f"f {disturber_nid} 0")
    time.sleep(0.05)
    send(disturber_nid, f"u {disturber_nid} 0.8")
    time.sleep(T_DISTURB)

    # Phase 3: restore feedback
    print(f"  Disturbance OFF — restoring feedback on Node {disturber_nid}")
    send(disturber_nid, f"f {disturber_nid} 1")
    time.sleep(T_RECOVER)

    stop_streams()
    time.sleep(0.3)

    stream = stream_snapshot()

    L_vals = {nid: (query_node(nid, 'L') or 0.0) for nid in range(1, N_NODES + 1)}

    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.patch.set_facecolor(DARK)
    fig.suptitle(
        f"Disturbance Rejection — Node {disturber_nid} duty forced to 0.8 at t≈{t_disturb_rel:.0f}s",
        color=FG, fontsize=12)

    for ax in axes:
        _style(ax)

    for i, nid in enumerate(range(1, N_NODES + 1)):
        d   = stream.get(nid, {})
        col = PAL[i]
        if d.get('t_lux') and d.get('lux'):
            t0 = d['t_lux'][0]
            t  = [x - t0 for x in d['t_lux']]
            axes[0].plot(t, d['lux'], color=col, lw=1.2, label=f"Node {nid}")
            L = L_vals.get(nid, 0.0)
            if L > 0:
                axes[0].axhline(L, color=col, lw=0.8, ls='--', alpha=0.5,
                                label=f"L{nid}={L:.0f} lux")
        if d.get('t_duty') and d.get('duty'):
            t0 = d['t_duty'][0]
            t  = [x - t0 for x in d['t_duty']]
            axes[1].plot(t, d['duty'], color=col, lw=1.2, label=f"Node {nid}")

    for ax in axes:
        ax.axvline(t_disturb_rel, color=RED, lw=1.2, ls='--', alpha=0.8,
                   label=f"disturbance on (t={t_disturb_rel:.0f}s)")
        ax.axvline(t_restore_rel, color=ORG, lw=1.2, ls='--', alpha=0.8,
                   label=f"disturbance off (t={t_restore_rel:.0f}s)")

    axes[0].set_ylabel("Illuminance [lux]")
    axes[0].set_title("LUX — dashed horizontal = lower bound L, dashed vertical = disturbance events",
                      fontsize=9)
    axes[0].set_ylim(bottom=0)
    _legend(axes[0])

    axes[1].set_ylabel("Duty Cycle")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_title("Duty Cycles", fontsize=9)
    axes[1].set_ylim(-0.05, 1.1)
    _legend(axes[1])

    plt.tight_layout()
    path = os.path.join(out_dir, "plot_disturbance_rejection.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")


def run_convergence_study(out_dir, results):
    print("\n" + "="*60)
    print("  RESEARCH: ADMM Convergence Study (Occ 2,2,2)")
    print("="*60)
    print("  [skip] Variable-iteration study is not exposed by this firmware build.")
    return
    
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
        # Current firmware has a fixed ADMM_MAXITER, so no iteration-budget
        # command is available here.
        for n in nodes:
            if n.node_id:
                pass
        time.sleep(0.1)
        for n in nodes: n.admm_done = False

        trigger_admm_via_cost({1: 1, 2: 1, 3: 1}, target_nid=1)
        
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

    # No iteration budget restore is needed with the current fixed-budget firmware.

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

        for i, nid in enumerate([1, 2, 3]):
            nd = sc['nodes'].get(nid, {})
            L = nd.get('L', 0)
            if L > 0:
                axes[0].axhline(L, color=PAL[i], lw=0.9, ls='--', alpha=0.5,
                                label=f"L{nid}={L:.0f} lux")
        axes[0].set_ylabel('Illuminance [lux]')
        axes[0].set_title('LUX (dashed = lower bound L per node)', fontsize=9)
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

# ── New Plots ──────────────────────────────────────────────────────────────────

def collect_model_fit_data():
    print("\n" + "=" * 60)
    print("  COLLECTING DATA FOR MODEL FIT PLOT")
    print("=" * 60)
    data = {}
    
    # Pre-fetch k_ii and background from each node
    node_params = {}
    for i in range(1, N_NODES + 1):
        n = node_by_id.get(i)
        k_ii = 1.0
        d_i = 0.0

        # Full k-row and c ? output are local-only text commands in the firmware.
        # Use them when the node has direct USB; otherwise still collect measured
        # points through the hub and let the fitted line show the model.
        if n and n.connected:
            n.flush()
            n.write(f"g k {i}")
            line_k = n.wait_line(rf"^k {i}\b", timeout=3.0)
        else:
            line_k = None
        if line_k:
            parts = line_k.split()
            if len(parts) >= 2 + i:
                k_ii = float(parts[1 + i])

        if n and n.connected:
            n.flush()
            n.write("c ?")
            time.sleep(1.0) # Longer wait for full 'c ?' output
            with n.lock:
                for line in n._lines:
                    m = re.search(r'background\s*=\s*([\d.]+)', line)
                    if m:
                        d_i = float(m.group(1))
                        break
        else:
            d_i = query_node(i, 'd') or 0.0
        
        node_params[i] = (k_ii, d_i)
        print(f"  Node {i} Params: k_ii={k_ii:.3f}, d_i={d_i:.2f}")

    # Sweep each node
    for i in range(1, N_NODES + 1):
        print(f"  Sweeping Node {i}...")
        # Set feedback off for all
        for nid in range(1, N_NODES + 1):
            send(nid, f"f {nid} 0")
        time.sleep(0.5)
        
        # Set all to duty 0
        for nid in range(1, N_NODES + 1):
            send(nid, f"u {nid} 0")
        time.sleep(0.5)
        
        node_data = []
        # Sweep duty from 0 to 1 in 10 steps
        for u_val in np.linspace(0.0, 1.0, 11):
            send(i, f"u {i} {u_val:.2f}")
            time.sleep(0.8) # Wait for LDR to settle (0.8s)
            y = query_node(i, 'y')
            if y is not None:
                node_data.append((u_val, y))
                print(f"    u={u_val:.1f} -> y={y:.2f} lux", end="\r")
        print(f"\n    Node {i} sweep complete.")
        
        k_ii, d_i = node_params.get(i, (1.0, 0.0))
        data[i] = {
            'measured': node_data,
            'k_ii': k_ii,
            'd_i': d_i
        }
    
    # Restore feedback on
    print("  Restoring feedback on all nodes.")
    for nid in range(1, N_NODES + 1):
        send(nid, f"f {nid} 1")
    time.sleep(1.0)
    
    return data

def plot_model_fit(data, out_dir):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    fig.patch.set_facecolor(DARK)
    fig.suptitle("Model Fit — Linear illuminance model vs measured data", color=FG, fontsize=12)
    
    for i in range(1, N_NODES + 1):
        ax = axes[i-1]
        _style(ax)
        node_results = data.get(i)
        if not node_results: 
            ax.set_title(f"Node {i} - No Data", color=FG)
            continue
        
        u_meas = np.array([p[0] for p in node_results['measured']])
        y_meas = np.array([p[1] for p in node_results['measured']])
        k_ii = node_results['k_ii']
        d_i = node_results['d_i']
        
        # Scatter measured
        ax.scatter(u_meas, y_meas, color=PAL[i-1], label='Measured', zorder=3, s=40)

        u_model = np.linspace(0, 1, 100)

        # Firmware calibration model line
        y_fw = k_ii * u_model + d_i
        ax.plot(u_model, y_fw, color=FG, ls='--', alpha=0.55,
                label=f'Firmware: {k_ii:.2f}u + {d_i:.2f}')

        # Linear regression fitted to the measured sweep data
        if len(u_meas) > 1:
            A = np.vstack([u_meas, np.ones(len(u_meas))]).T
            k_fit, d_fit = np.linalg.lstsq(A, y_meas, rcond=None)[0]
            y_fit = k_fit * u_model + d_fit
            ax.plot(u_model, y_fit, color=PAL[i-1], ls='-', lw=1.5, alpha=0.85,
                    label=f'Fitted:   {k_fit:.2f}u + {d_fit:.2f}')

            # R² for firmware model (calibration accuracy)
            y_pred_fw = k_ii * u_meas + d_i
            ss_res_fw = np.sum((y_meas - y_pred_fw) ** 2)
            ss_tot    = np.sum((y_meas - np.mean(y_meas)) ** 2)
            r2_fw = 1 - ss_res_fw / ss_tot if ss_tot != 0 else 0

            # R² for fitted line (linearity of the physical system)
            y_pred_fit = k_fit * u_meas + d_fit
            ss_res_fit = np.sum((y_meas - y_pred_fit) ** 2)
            r2_fit = 1 - ss_res_fit / ss_tot if ss_tot != 0 else 0

            ann = (f"Firmware R² = {r2_fw:.4f}  (calibration accuracy)\n"
                   f"Fitted R²   = {r2_fit:.4f}  (system linearity)")
            ax.text(0.04, 0.97, ann, transform=ax.transAxes, color=FG,
                    va='top', fontsize=8,
                    bbox=dict(facecolor=DARK, alpha=0.6, edgecolor=GRID, pad=3))
        
        ax.set_title(f"Node {i}", color=FG)
        ax.set_xlabel("Duty Cycle", color=FG)
        ax.set_ylabel("Illuminance [lux]", color=FG)
        ax.set_ylim(bottom=0)
        _legend(ax)
        
    plt.tight_layout()
    path = os.path.join(out_dir, "model_fit.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")

def plot_constraint_satisfaction_heatmap(results, out_dir):
    scenarios = [r['label'] for r in results]
    data_matrix = []
    for r in results:
        row = []
        for nid in range(1, N_NODES + 1):
            nd = r['nodes'].get(nid, {})
            val = nd.get('lux_ss', 0) - nd.get('L', 0)
            row.append(val)
        data_matrix.append(row)
    
    data_matrix = np.array(data_matrix)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.patch.set_facecolor(DARK)
    ax.set_facecolor(PANEL)
    
    # diverging colormap centered at 0
    v_ext = max(abs(data_matrix.min()), abs(data_matrix.max()), 0.1)
    im = ax.imshow(data_matrix, cmap='RdYlGn', aspect='auto', vmin=-v_ext, vmax=v_ext)
    
    # Annotate
    for i in range(len(scenarios)):
        for j in range(N_NODES):
            ax.text(j, i, f"{data_matrix[i, j]:.1f}", ha="center", va="center", color="black", fontsize=9)

    ax.set_xticks(np.arange(N_NODES))
    ax.set_xticklabels([f"Node {i+1}" for i in range(N_NODES)], color=FG)
    ax.set_yticks(np.arange(len(scenarios)))
    ax.set_yticklabels(scenarios, color=FG, fontsize=8)
    
    ax.set_title("Constraint Satisfaction — lux_ss minus lower bound L [lux]", color=FG, pad=20)
    cbar = plt.colorbar(im, ax=ax)
    cbar.ax.tick_params(colors=FG)
    
    plt.tight_layout()
    path = os.path.join(out_dir, "constraint_satisfaction_heatmap.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")

def plot_duty_allocation_heatmap(results, out_dir):
    scenarios = [r['label'] for r in results]
    data_matrix = []
    for r in results:
        row = []
        for nid in range(1, N_NODES + 1):
            nd = r['nodes'].get(nid, {})
            row.append(nd.get('duty_ss', 0))
        data_matrix.append(row)
    
    data_matrix = np.array(data_matrix)
    
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.patch.set_facecolor(DARK)
    ax.set_facecolor(PANEL)
    
    im = ax.imshow(data_matrix, cmap='viridis', aspect='auto', vmin=0, vmax=1)
    
    # Annotate
    for i in range(len(scenarios)):
        for j in range(N_NODES):
            text_col = "white" if data_matrix[i,j] < 0.5 else "black"
            ax.text(j, i, f"{data_matrix[i, j]:.3f}", ha="center", va="center", color=text_col, fontsize=9)

    ax.set_xticks(np.arange(N_NODES))
    ax.set_xticklabels([f"Node {i+1}" for i in range(N_NODES)], color=FG)
    ax.set_yticks(np.arange(len(scenarios)))
    ax.set_yticklabels(scenarios, color=FG, fontsize=8)
    
    ax.set_title("Steady-State Duty Allocation per Scenario", color=FG, pad=20)
    cbar = plt.colorbar(im, ax=ax)
    cbar.ax.tick_params(colors=FG)
    
    plt.tight_layout()
    path = os.path.join(out_dir, "duty_allocation_heatmap.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")

def run_disturbance_test_improved(disturber_nid: int, out_dir: str):
    print("\n" + "=" * 60)
    print(f"  IMPROVED DISTURBANCE REJECTION TEST  (disturber = Node {disturber_nid})")
    print("=" * 60)

    T_STEADY  = 5.0
    T_DISTURB = 6.0
    T_RECOVER = 10.0
    t_disturb_rel = T_STEADY
    t_restore_rel = T_STEADY + T_DISTURB

    clear_stream_buffers()
    start_streams()

    # Phase 1: steady state
    print(f"  Collecting {T_STEADY:.0f}s steady state…")
    time.sleep(T_STEADY)

    # Phase 2: apply disturbance (disable feedback → force duty)
    print(f"  Disturbance ON — Node {disturber_nid} duty → 0.8")
    send(disturber_nid, f"f {disturber_nid} 0")
    time.sleep(0.05)
    send(disturber_nid, f"u {disturber_nid} 0.8")
    time.sleep(T_DISTURB)

    # Phase 3: restore feedback
    print(f"  Disturbance OFF — restoring feedback on Node {disturber_nid}")
    send(disturber_nid, f"f {disturber_nid} 1")
    time.sleep(T_RECOVER)

    stop_streams()
    time.sleep(0.3)

    stream = stream_snapshot()

    L_vals = {nid: (query_node(nid, 'L') or 0.0) for nid in range(1, N_NODES + 1)}

    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(13, 10), sharex=True)
    fig.patch.set_facecolor(DARK)
    fig.suptitle("Disturbance Rejection — Constraint violation (max(0, L−lux))", color=FG, fontsize=12)

    for ax in axes:
        _style(ax)

    max_err_val = 0
    max_err_node = 1
    max_err_time = 0
    recovery_times = {}

    for i, nid in enumerate(range(1, N_NODES + 1)):
        d   = stream.get(nid, {})
        col = PAL[i]
        L = L_vals.get(nid, 0.0)
        
        if d.get('t_lux') and d.get('lux'):
            t0 = d['t_lux'][0]
            t  = np.array([x - t0 for x in d['t_lux']])
            lux = np.array(d['lux'])
            err = np.maximum(0, L - lux)
            
            axes[0].plot(t, err, color=col, lw=1.5, label=f"Node {nid} (L={L:.0f} lux)")
            
            local_max_idx = np.argmax(err)
            local_max_err = err[local_max_idx]
            if local_max_err > max_err_val:
                max_err_val = local_max_err
                max_err_node = nid
                max_err_time = t[local_max_idx]
            
            after_restore_mask = t > t_restore_rel
            t_after = t[after_restore_mask]
            err_after = err[after_restore_mask]
            
            rec_idx = np.where(err_after < 0.5)[0]
            if len(rec_idx) > 0:
                rec_time = t_after[rec_idx[0]] - t_restore_rel
                recovery_times[nid] = rec_time

        if d.get('t_duty') and d.get('duty'):
            t0 = d['t_duty'][0]
            t  = [x - t0 for x in d['t_duty']]
            axes[1].plot(t, d['duty'], color=col, lw=1.2, label=f"Node {nid}")

    for ax in axes:
        ax.axvline(t_disturb_rel, color=RED, lw=1.2, ls='--', alpha=0.8)
        ax.axvline(t_restore_rel, color=ORG, lw=1.2, ls='--', alpha=0.8)
    
    axes[0].text(t_disturb_rel, axes[0].get_ylim()[1]*0.8, ' Disturbance ON', color=RED, fontsize=9, fontweight='bold')
    axes[0].text(t_restore_rel, axes[0].get_ylim()[1]*0.8, ' Disturbance OFF', color=ORG, fontsize=9, fontweight='bold')

    if max_err_val > 0:
        axes[0].annotate(f"Max Violation: {max_err_val:.1f} lux (Node {max_err_node})",
                         xy=(max_err_time, max_err_val), xytext=(max_err_time + 1, max_err_val + 2),
                         color=FG, fontsize=9, arrowprops=dict(arrowstyle='->', color=FG))

    rec_text = "Recovery Times (error < 0.5 lux):\n" + "\n".join([f"Node {nid}: {t:.2f}s" for nid, t in recovery_times.items()])
    axes[0].text(0.98, 0.05, rec_text, transform=axes[0].transAxes, color=FG, fontsize=9, ha='right', va='bottom', 
                 bbox=dict(facecolor='#2a2a3e', alpha=0.7, edgecolor=GRID))

    axes[0].set_ylabel("Visibility Error [lux] — max(0, L - y)")
    axes[0].set_title("Constraint Violation Over Time", fontsize=10, color=FG)
    _legend(axes[0])

    axes[1].set_ylabel("Duty Cycle")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_title("Duty Cycles Over Time", fontsize=10, color=FG)
    axes[1].set_ylim(-0.05, 1.1)
    _legend(axes[1])

    plt.tight_layout()
    path = os.path.join(out_dir, "disturbance_rejection_improved.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")

def run_residual_study(out_dir):
    print("\n" + "=" * 60)
    print("  RUNNING ADMM CONVERGENCE RESIDUAL STUDY")
    print("=" * 60)

    # 1. Start from a known coordinated state.
    base_occs = {1: 2, 2: 2, 3: 2}
    base_costs = {1: 1, 2: 1, 3: 1}
    establish_coordinated_state(base_occs, base_costs, settle_s=2)

    # 2. Trigger one fresh ADMM run through an implemented command. Changing
    # node 2's cost gives a meaningful convergence transient without using T or
    # any test-only command.
    target_costs = {1: 1, 2: 10, 3: 1}
    for n in nodes:
        n.admm_done = False
    print("  Triggering residual run with: C 2 10")
    trigger_admm_via_cost(target_costs, target_nid=2)

    # 3. Poll residuals until ADMM completes or timeout. Firmware exposes the
    # latest residuals as g K <i> and g J <i>; it does not expose the iteration
    # counter, so the x-axis is sample order during the ADMM run.
    data = []
    start_time = time.time()
    deadline = start_time + ADMM_WAIT_S + 5.0
    query_timeout = 0.35

    # Track last values to deduplicate
    last_res = {nid: (None, None) for nid in range(1, N_NODES+1)}

    print("  Polling residuals through g K / g J...")
    while time.time() < deadline:
        sample = {'time': time.time() - start_time}
        any_new = False

        for nid in range(1, N_NODES + 1):
            K = query_node(nid, 'K', timeout=query_timeout)
            J = query_node(nid, 'J', timeout=query_timeout)

            if K is not None and J is not None:
                if (K, J) != last_res[nid]:
                    any_new = True
                    last_res[nid] = (K, J)

                sample[f'K{nid}'] = K
                sample[f'J{nid}'] = J

        if any_new:
            data.append(sample)
            print(f"    Sample {len(data)}: K1={sample.get('K1',0):.4f} J1={sample.get('J1',0):.4f}", end="\r")

        if any(n.admm_done for n in nodes):
            print("\n  ADMM signaled completion.")
            break

        time.sleep(0.05)
    else:
        print(f"\n  Polling timed out after {ADMM_WAIT_S + 5.0:.0f}s.")

    if not data:
        print("  [skip] plot_residuals_convergence — no data collected")
        return

    # Deduplicate and align into iteration indices
    # Since nodes might be slightly out of sync in iteration count, 
    # we'll just use the per-node distinct sequences.
    per_node_K = {nid: [] for nid in range(1, N_NODES + 1)}
    per_node_J = {nid: [] for nid in range(1, N_NODES + 1)}
    
    for s in data:
        for nid in range(1, N_NODES + 1):
            k_val = s.get(f'K{nid}')
            j_val = s.get(f'J{nid}')
            if k_val is not None:
                if not per_node_K[nid] or k_val != per_node_K[nid][-1]:
                    per_node_K[nid].append(k_val)
                    per_node_J[nid].append(j_val)

    non_empty_counts = [len(v) for v in per_node_K.values() if v]
    if not non_empty_counts:
        print("  [skip] plot_residuals_convergence — no residual sequences collected")
        return
    if max(non_empty_counts) < 3:
        print(f"  [warning] Few distinct residual points collected (max={max(non_empty_counts)}). Plotting anyway.")

    plot_residuals_convergence(per_node_K, per_node_J, out_dir)

def plot_residuals_convergence(per_node_K, per_node_J, out_dir):
    fig, axes = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
    fig.patch.set_facecolor(DARK)
    fig.suptitle("ADMM Convergence — Residuals for Occ(2,2,2), cost step C=[1,1,1]→[1,10,1]", color=FG, fontsize=12)
    
    for ax in axes:
        _style(ax)
        ax.set_yscale('log')
        
    # Top: Primal Residual
    for nid in range(1, N_NODES + 1):
        y_raw = per_node_K[nid]
        if not y_raw:
            continue
        y = np.maximum(np.array(y_raw, dtype=float), 1e-6)
        x = np.arange(len(y))
        col = PAL[nid-1]
        axes[0].plot(x, y, color=col, lw=1.5, marker='o', markersize=3, label=f"Node {nid}")
        if len(y) > 0:
            axes[0].axhline(y[-1], color=col, ls='--', alpha=0.4)
            
    axes[0].set_ylabel(r"Primal Residual $\|x_i - \bar{u}\|$")
    axes[0].set_title("Primal Residual Convergence", fontsize=10, color=FG)
    _legend(axes[0])
    
    # Bottom: Dual Residual
    for nid in range(1, N_NODES + 1):
        y_raw = per_node_J[nid]
        if not y_raw:
            continue
        y = np.maximum(np.array(y_raw, dtype=float), 1e-6)
        x = np.arange(len(y))
        col = PAL[nid-1]
        axes[1].plot(x, y, color=col, lw=1.5, marker='s', markersize=3, label=f"Node {nid}")
        if len(y) > 0:
            axes[1].axhline(y[-1], color=col, ls='--', alpha=0.4)
            
    axes[1].set_ylabel(r"Dual Residual $\|\rho(\bar{u}^k - \bar{u}^{k-1})\|$")
    axes[1].set_xlabel("Residual sample during ADMM run")
    axes[1].set_title("Dual Residual Convergence", fontsize=10, color=FG)
    _legend(axes[1])
    
    plt.tight_layout()
    path = os.path.join(out_dir, "residuals_convergence.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")

# ── Priority 1: E / V / F comparison bar chart — ADMM vs PI-only ─────────────

def plot_metrics_comparison(admm_results, baseline_results, sc_labels, out_dir):
    """
    4-panel grouped bar chart: total-E, weighted-cost, mean-V, mean-F.

    Weighted cost = Σ cᵢ·Eᵢ  (proportional to the ADMM objective Σ cᵢ·uᵢ).
    For C=[1,1,1] this equals total energy.  For asymmetric costs ADMM trades
    raw energy for lower weighted cost — panel 1 may show ADMM higher, panel 2
    will always show ADMM lower or equal.
    """
    n_sc = len(admm_results)
    x = np.arange(n_sc)
    w = 0.35

    fig, axes = plt.subplots(1, 4, figsize=(20, 6))
    fig.patch.set_facecolor(DARK)
    fig.suptitle(
        "ADMM vs PI-only — Raw Energy · Weighted Cost · Visibility Error · Flicker\n"
        "Note: ADMM minimises weighted cost (Σ cᵢ·uᵢ), not raw energy",
        color=FG, fontsize=11)

    ADMM_C = "#89dceb"   # sky
    PI_C   = "#f9e2af"   # yellow

    def _weighted_cost(result):
        return sum(
            result['nodes'][nid].get('E', 0) * result['nodes'][nid].get('cost', 1)
            for nid in range(1, N_NODES + 1) if nid in result['nodes'])

    panel_cfg = [
        # (ax, compute_fn, ylabel, title, note, highlight_lower)
        (axes[0],
         lambda ar, br: (sum(ar['nodes'][nid].get('E', 0) for nid in range(1, N_NODES+1)),
                         sum(br['nodes'][nid].get('E', 0) for nid in range(1, N_NODES+1))),
         f'Total Energy [J]  ({COLLECT_S}s)',
         'Raw Energy  Σ Eᵢ',
         'Higher raw E expected when\nADMM shifts load to cheaper nodes',
         False),
        (axes[1],
         lambda ar, br: (_weighted_cost(ar), _weighted_cost(br)),
         f'Weighted Cost [J]  ({COLLECT_S}s)',
         'Weighted Cost  Σ cᵢ·Eᵢ  ← ADMM objective',
         'ADMM minimises this — should be ≤ PI-only',
         True),
        (axes[2],
         lambda ar, br: (
             float(np.mean([ar['nodes'][nid].get('V', 0) for nid in range(1, N_NODES+1) if nid in ar['nodes']])),
             float(np.mean([br['nodes'][nid].get('V', 0) for nid in range(1, N_NODES+1) if nid in br['nodes']]))),
         'Mean Visibility Error [lux]',
         'Visibility Error  V  ← strongest result',
         'Lower is better — coordinated ref improves constraint satisfaction',
         True),
        (axes[3],
         lambda ar, br: (
             float(np.mean([ar['nodes'][nid].get('F', 0) for nid in range(1, N_NODES+1) if nid in ar['nodes']])),
             float(np.mean([br['nodes'][nid].get('F', 0) for nid in range(1, N_NODES+1) if nid in br['nodes']]))),
         'Mean Flicker  [1/s]', 'Flicker  F', '', False),
    ]

    for ax, compute_fn, ylabel, title, note, highlight_lower in panel_cfg:
        _style(ax)
        admm_vals, pi_vals = [], []
        for ar, br in zip(admm_results, baseline_results):
            av, bv = compute_fn(ar, br)
            admm_vals.append(av)
            pi_vals.append(bv)

        b_admm = ax.bar(x - w / 2, admm_vals, w, label='ADMM',
                        color=ADMM_C, alpha=0.85, edgecolor=GRID, linewidth=0.5)
        b_pi   = ax.bar(x + w / 2, pi_vals,   w, label='PI-only',
                        color=PI_C,   alpha=0.85, edgecolor=GRID, linewidth=0.5)

        for bar in list(b_admm) + list(b_pi):
            h = bar.get_height()
            if h > 0:
                ax.text(bar.get_x() + bar.get_width() / 2, h * 1.01,
                        f'{h:.4f}', ha='center', va='bottom', color=FG, fontsize=7)

        # Annotate percentage improvement where ADMM < PI-only
        if highlight_lower:
            for xi, (av, bv) in enumerate(zip(admm_vals, pi_vals)):
                if bv > 0 and av < bv:
                    pct = (bv - av) / bv * 100
                    ax.annotate(f'−{pct:.0f}%',
                                xy=(xi - w / 2, av), xytext=(xi, av * 1.18),
                                ha='center', fontsize=8, color="#a6e3a1",
                                fontweight='bold',
                                arrowprops=dict(arrowstyle='->', color="#a6e3a1",
                                                lw=0.8))

        ax.set_xticks(x)
        ax.set_xticklabels(sc_labels, color=FG, fontsize=9)
        ax.set_ylabel(ylabel, color=FG)
        full_title = f"{title}\n{note}" if note else title
        ax.set_title(full_title, fontsize=9, color=FG)
        _legend(ax)

    plt.tight_layout()
    path = os.path.join(out_dir, "plot_metrics_admm_vs_baseline.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")


def run_metrics_comparison(out_dir: str):
    """
    Back-to-back ADMM and PI-only runs for Occ(2,2,2) C=[1,1,1] and C=[1,10,1].
    The asymmetric-cost scenario shows ADMM load-shifting; PI-only cannot do that.
    """
    print("\n" + "=" * 60)
    print("  METRICS COMPARISON: ADMM vs PI-only  (Priority 1)")
    print("=" * 60)

    sc_defs = [
        ("Occ222\nC=[1,1,1]",  {1: 2, 2: 2, 3: 2}, {1: 1, 2: 1,  3: 1}),
        ("Occ222\nC=[1,10,1]", {1: 2, 2: 2, 3: 2}, {1: 1, 2: 10, 3: 1}),
    ]

    admm_results, baseline_results, sc_labels = [], [], []

    for label, occs, costs in sc_defs:
        flat = label.replace('\n', ' ')
        print(f"\n  {flat}")
        print("    ADMM run…")
        admm_results.append(run_single_admm(occs, costs, label))
        print("    PI-only baseline…")
        baseline_results.append(run_baseline(occs, label, costs))
        sc_labels.append(label)

    plot_metrics_comparison(admm_results, baseline_results, sc_labels, out_dir)


# ── Priority 4: Occupancy-change transient (OFF → HIGH, ADMM fires) ───────────

def run_occupancy_transient(out_dir: str):
    """
    Capture lux and duty when node 2 switches from OFF to HIGH occupancy.

    All three nodes start OFF so there is no LED-spillover illuminating node 2
    before the switch.  This makes the 0→20 lux step clearly visible.

    Timeline
    --------
    0 – T_PRE s   : steady state (all nodes OFF — LEDs dark, lux ≈ background)
    ~T_PRE s      : send 'o 2 2' — ADMM fires on all nodes  [red marker]
    ADMM duration : shaded orange band while ADMM is running
    after ADMM    : PI settles to new set-point               [orange marker]
    """
    print("\n" + "=" * 60)
    print("  OCCUPANCY CHANGE TRANSIENT  (Node 2: OFF → HIGH, all start OFF)")
    print("=" * 60)

    T_PRE        = 5.0   # s of pre-event steady state to capture
    T_POST       = 10.0  # s to collect after ADMM completes (PI settle)
    TRANSIENT_MAXITER = 50  # firmware currently uses a fixed ADMM iteration budget

    # ── 1. Establish initial state: ALL nodes OFF ─────────────────────────────
    # Starting all-OFF avoids spillover from neighbours pre-illuminating node 2,
    # which would make the lux transition invisible.
    print("  Setting initial state (ALL nodes OFF)…")
    for nid in range(1, N_NODES + 1):
        send(nid, f"C {nid} 1")
    for nid in range(1, N_NODES + 1):
        send(nid, f"o {nid} 0")

    # This transient uses the fixed firmware ADMM iteration budget.

    # Drain stale ADMM then re-trigger with correct occupancies through C.
    for n in nodes:
        n.admm_done = False
    trigger_admm_via_cost({1: 1, 2: 1, 3: 1}, target_nid=1)
    wait_admm(ADMM_WAIT_S)
    time.sleep(3)

    # ── 2. Start streaming ────────────────────────────────────────────────────
    clear_stream_buffers()
    start_streams()

    # ── 3. Pre-event steady state ─────────────────────────────────────────────
    print(f"  Pre-event: {T_PRE:.0f}s steady state…")
    time.sleep(T_PRE)

    # ── 4. Trigger occupancy change ───────────────────────────────────────────
    print("  Switching Node 2: OFF → HIGH — ADMM fires")
    for n in nodes:
        n.admm_done = False
    t_trigger_rel = T_PRE   # wall-clock seconds since streaming started

    t_cmd = time.time()
    send(2, "o 2 2")

    # Measure ADMM completion time
    t_admm_done_rel = t_trigger_rel + ADMM_WAIT_S   # fallback if timeout
    deadline = time.time() + ADMM_WAIT_S
    while time.time() < deadline:
        if any(n.admm_done for n in nodes):
            t_admm_done_rel = t_trigger_rel + (time.time() - t_cmd)
            print(f"  ADMM done at ~t={t_admm_done_rel:.2f}s")
            break
        time.sleep(0.05)

    # ── 5. Post-event PI settle ───────────────────────────────────────────────
    print(f"  Collecting {T_POST:.0f}s post-ADMM settle…")
    time.sleep(T_POST)

    # ── 6. Stop streaming ─────────────────────────────────────────────────────
    stop_streams()
    time.sleep(0.3)

    # ── 7. Read buffers ───────────────────────────────────────────────────────
    stream = stream_snapshot()

    L_vals = {nid: (query_node(nid, 'L') or 0.0) for nid in range(1, N_NODES + 1)}

    # ── 8. Plot ───────────────────────────────────────────────────────────────
    fig, axes = plt.subplots(2, 1, figsize=(13, 9), sharex=True)
    fig.patch.set_facecolor(DARK)
    fig.suptitle(
        f"Occupancy Change Transient — All OFF → Node 2 HIGH  (ADMM k={TRANSIENT_MAXITER} iters + PI settle)\n"
        "Initial state: all LEDs off.  Node 2 switches to occ=2 at t≈5s.",
        color=FG, fontsize=11)

    for ax in axes:
        _style(ax)

    t0_ref = None   # first sample time used to re-zero all traces

    for i, nid in enumerate(range(1, N_NODES + 1)):
        d   = stream.get(nid, {})
        col = PAL[i]
        occ_label = "OFF→HIGH" if nid == 2 else "OFF (unchanged)"

        if d.get('t_lux') and d.get('lux'):
            if t0_ref is None:
                t0_ref = d['t_lux'][0]
            t = [x - t0_ref for x in d['t_lux']]
            axes[0].plot(t, d['lux'], color=col, lw=1.2,
                         label=f"Node {nid} ({occ_label})")
            L = L_vals.get(nid, 0.0)
            if L > 0:
                axes[0].axhline(L, color=col, lw=0.8, ls='--', alpha=0.5,
                                label=f"L{nid}={L:.0f} lux")

        if d.get('t_duty') and d.get('duty'):
            if t0_ref is None:
                t0_ref = d['t_duty'][0]
            t = [x - t0_ref for x in d['t_duty']]
            axes[1].plot(t, d['duty'], color=col, lw=1.2,
                         label=f"Node {nid} ({occ_label})")

    # Markers: t_trigger_rel and t_admm_done_rel are seconds from stream start.
    # After re-zeroing to the first sample (which arrives ~10 ms after stream
    # start), these values align to within one control period — acceptable.
    if t0_ref is not None:
        for ax in axes:
            ax.axvline(t_trigger_rel, color=RED, lw=1.4, ls='--', alpha=0.9,
                       label=f"ADMM trigger (t≈{t_trigger_rel:.1f}s)")
            ax.axvline(t_admm_done_rel, color=ORG, lw=1.2, ls=':', alpha=0.9,
                       label=f"ADMM done (t≈{t_admm_done_rel:.1f}s)")
            ax.axvspan(t_trigger_rel, t_admm_done_rel,
                       alpha=0.08, color=ORG, label="_admm band")

    axes[0].set_ylabel("Illuminance [lux]", color=FG)
    axes[0].set_title(
        "LUX — dashed horizontal = lower bound L,  orange band = ADMM running",
        fontsize=9, color=FG)
    axes[0].set_ylim(bottom=0)
    _legend(axes[0])

    axes[1].set_ylabel("Duty Cycle", color=FG)
    axes[1].set_xlabel("Time [s]", color=FG)
    axes[1].set_title("Duty Cycles", fontsize=9, color=FG)
    axes[1].set_ylim(-0.05, 1.1)
    _legend(axes[1])

    plt.tight_layout()
    path = os.path.join(out_dir, "plot_occupancy_transient.png")
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK)
    plt.close(fig)
    print(f"  Saved: {path}")


# ── Entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    out_dir = os.path.dirname(os.path.abspath(__file__))

    # ── 1. Resolve ports ───────────────────────────────────────────────────
    args = sys.argv[1:]
    REBOOT_SYSTEM = "--reboot" in args
    ports_arg = [a for a in args if a != "--reboot"]

    if ports_arg:
        ports = ports_arg
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
    if REBOOT_SYSTEM:
        send_reboot_to_ports(ports)

    print("Starting serial readers…")

    for p in ports:
        n = Node(p)
        nodes.append(n)
        threading.Thread(target=reader_thread, args=(n,), daemon=True).start()

    # ── 2. Wait for at least one serial hub ─────────────────────────────────
    print(f"\nWaiting for serial hub / node IDs (up to {ID_TIMEOUT_S}s)…")
    t0_id = time.time()
    deadline = t0_id + ID_TIMEOUT_S
    # In normal one-hub mode the connected node may already be past its boot
    # print. With --reboot we wait longer because the Pico is restarting and the
    # COM port can briefly disappear/reappear on Windows.
    while time.time() < deadline:
        if any(n.node_id for n in nodes):
            break
        min_wait = 15.0 if REBOOT_SYSTEM else 5.0
        if any(n.connected for n in nodes) and time.time() - t0_id >= min_wait:
            break
        time.sleep(0.5)

    for n in nodes:
        if n.node_id:
            node_by_id[n.node_id] = n
            print(f"  {n.port} → Node {n.node_id}")
        else:
            print(f"  {n.port} → UNKNOWN (continuing anyway)")

    if not node_by_id:
        if hub_node() is None:
            print("ERROR: No serial hub detected. Exiting.")
            sys.exit(1)
        print("  No [BOOT] node ID detected; continuing with connected Pico as CAN hub.")

    # ── 3. Calibration state ───────────────────────────────────────────────
    print("\nCalibration check…")
    cal_initiator = node_by_id.get(1)
    if cal_initiator is None and node_by_id:
        cal_initiator = node_by_id.get(min(node_by_id))
    if cal_initiator is None:
        cal_initiator = hub_node()
    if cal_initiator is None:
        print("  ERROR: no node available.")
        sys.exit(1)

    if REBOOT_SYSTEM:
        print("  Waiting for calibration after reboot...")
        deadline = time.time() + CAL_TIMEOUT_S
        while time.time() < deadline:
            if any(n.cal_done for n in nodes):
                print("\n  Calibration complete.")
                break
            remaining = int(deadline - time.time())
            print(f"  Calibrating… {CAL_TIMEOUT_S - remaining}s elapsed", end="\r")
            time.sleep(1)
        else:
            print("\n  WARNING: Calibration timed out.")
    elif not node_by_id:
        # In one-hub mode the nodes may already be past wakeup/calibration before
        # Python opens the port. Do not force a fresh `c` calibration here:
        # repeated `c` commands reset the firmware's calibration barrier and can
        # prevent the already-calibrated network from progressing.
        print("  One-hub mode without boot ID: assuming existing firmware calibration.")
    else:
        cal_initiator.write("c ?")
        time.sleep(1.0)
        print("  Using existing calibration state.")

    # ── 3b. Dump coupling-gain matrix for diagnostics when direct IDs exist
    if node_by_id:
        dump_coupling_gains()
    else:
        print("\n  Coupling-gain matrix: skipped in one-hub mode without boot ID.")

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

    # ── 8. Coordinated vs non-coordinated comparison (back-to-back, same conditions)
    # Both runs happen immediately after each other so physical conditions are
    # identical.  Using a stale result from run_tests() would be unfair since
    # ambient light and sensor temperature may have drifted during the full suite.
    # Asymmetric costs: node 2 is cheapest (c=1), nodes 1 and 3 are expensive (c=5).
    # ADMM will load-shift work to node 2, reducing total energy.
    # PI-only cannot do this — each node controls its own LED independently and
    # reaches the same 20 lux target without awareness of peer costs.
    print("\nFresh ADMM run for coordinated vs baseline comparison…")
    admm_fresh = run_single_admm(
        {1: 2, 2: 2, 3: 2}, {1: 5, 2: 1, 3: 5}, "Occ(2,2,2) C=[5,1,5]")

    print("\nRunning non-coordinated baseline (PI only, same scenario)…")
    baseline_all_high = run_baseline({1: 2, 2: 2, 3: 2}, "PI-only Occ(2,2,2)")

    plot_comparison(admm_fresh, baseline_all_high, out_dir)

    # ── 9. Disturbance rejection test ──────────────────────────────────────
    print("\nRunning disturbance rejection test…")
    # Re-establish coordinated steady state before probing disturbance rejection.
    establish_coordinated_state(
        {1: 2, 2: 2, 3: 2}, {1: 1, 2: 1, 3: 1}, settle_s=3)
    run_disturbance_test(disturber_nid=2, out_dir=out_dir)

    # ── 10. Convergence study ───────────────────────────────────────────────
    run_convergence_study(out_dir, results)

    # ── 11. New Plots (Phase 2) ────────────────────────────────────────────
    print("\n" + "━"*62)
    print("  Generating Additional Phase 2 Plots")
    print("━"*62)
    
    # Plot 1: Model Fit (collects data via sweep)
    model_data = collect_model_fit_data()
    plot_model_fit(model_data, out_dir)
    
    # Plot 2: Constraint Satisfaction Heatmap
    plot_constraint_satisfaction_heatmap(results, out_dir)
    
    # Plot 3: Duty Allocation Heatmap
    plot_duty_allocation_heatmap(results, out_dir)
    
    # Plot 4: Improved Disturbance Rejection (runs fresh test)
    establish_coordinated_state(
        {1: 2, 2: 2, 3: 2}, {1: 1, 2: 1, 3: 1}, settle_s=3)
    run_disturbance_test_improved(disturber_nid=2, out_dir=out_dir)

    # Plot 5: ADMM Residual Convergence Study
    run_residual_study(out_dir)

    # ── 12. Priority 1: E/V/F bar chart — ADMM vs PI-only ────────────────────
    run_metrics_comparison(out_dir)

    # ── 13. Priority 4: Occupancy change transient ────────────────────────────
    run_occupancy_transient(out_dir)

    print(f"\n✓ Done. All plots saved to: {out_dir}")
    print("Files: plot_steady_state_occupancy.png  plot_cost_comparison.png"
          "  plot_metrics.png  plot_ts_*.png  model_fit.png"
          "  constraint_satisfaction_heatmap.png  duty_allocation_heatmap.png"
          "  disturbance_rejection_improved.png  residuals_convergence.png"
          "  plot_metrics_admm_vs_baseline.png  plot_occupancy_transient.png")
