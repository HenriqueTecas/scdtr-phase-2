# Distributed Illuminance Control with Consensus ADMM
## SCDTR Phase 2 — Raspberry Pi Pico (RP2040)

This repository contains the implementation of a distributed lighting control system for a 3-node network. The system uses **Consensus ADMM (Alternating Direction Method of Multipliers)** to optimize illuminance across multiple desks while minimizing energy consumption and satisfying individual user requirements.

### Quick Start: Hardware Setup
1. Connect three Raspberry Pi Picos via a CAN bus using MCP2515 controllers.
2. Connect an LED (Pin 15) and an LDR (Pin A0) to each node.
3. Plug all three nodes into your PC via USB.

### How to Flash
We use the `arduino-cli` for automated deployment. Ensure you have the `rp2040:rp2040` core installed.

```bash
# 1. Compile the firmware
arduino-cli compile --fqbn rp2040:rp2040:rpipico main/main.ino

# 2. Flash all three nodes (replace ACM ports if necessary)
arduino-cli upload -p /dev/ttyACM0 --fqbn rp2040:rp2040:rpipico main/main.ino
arduino-cli upload -p /dev/ttyACM1 --fqbn rp2040:rp2040:rpipico main/main.ino
arduino-cli upload -p /dev/ttyACM2 --fqbn rp2040:rp2040:rpipico main/main.ino
```

### Running the Test Suite
The automated test suite handles identity detection, calibration, and executes 10 optimization scenarios.

```bash
python3 test_suite.py
```

### High-Performance Dashboard
To monitor the system in real-time with optimized parallel processing:

```bash
python3 live_plot.py
```

---
*Refer to [PHASE2_REPORT.md](PHASE2_REPORT.md) for a deep dive into the mathematics and result analysis.*
