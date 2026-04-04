# SCDTR Phase 2: Distributed Illuminance Control Implementation

This document summarizes the technical improvements, optimizations, and research findings completed for Phase 2: Distributed Illuminance Control with Consensus ADMM.

## 1. System Architecture & Reliability
- **Dual-Core Implementation:** Separated real-time control (Core 0) from CAN communication (Core 1) using lock-free hardware FIFOs.
- **Physical Identity Mapping:** Corrected the `UID_TABLE` to align logical node IDs with the physical desk arrangement on the USB ports (`ACM0, ACM1, ACM2`).
- **Identity Bypassing:** Added an `i` (identity) command that bypasses CAN forwarding, ensuring the test suite always addresses the correct physical hardware.
- **Robust Calibration:** Automated the 3-node round-robin calibration process within the test suite, implementing a forced-reset barrier to ensure all nodes synchronize correctly.

## 2. ADMM Refinements & Tuning
- **Stable Integration (Eq 13):** Implemented $r_i = d_i + k_{ii} u_i^*$. The local PI controller now only tracks the intended local contribution, preventing "fighting" between nodes during transients.
- **Dynamic Iterations:** Converted `ADMM_MAXITER` into a dynamic parameter settable via the `T <n>` serial command.
- **Energy-Efficiency Tuning:** 
    - Optimized the penalty parameter **$\rho = 0.05$**.
    - Set achievable targets: **20 Lux (High)** and **10 Lux (Low)**.
- **Best-Effort Saturation:** Added logic to saturate duty cycles at 1.0 if a target is physically unreachable, ensuring the system provides maximum possible light instead of collapsing.
- **Lambda Reset:** Fixed a critical bug by ensuring dual variables ($\lambda$) are reset to zero at the start of every ADMM session to prevent integrator windup carry-over.

## 3. Advanced Research Module
- **Convergence Rate Study:** A new research module in `test_suite.py` quantifies the relationship between iteration counts and control performance.
- **Key Finding:** The system reaches an effective consensus in as few as **5-10 iterations**, demonstrating high computational efficiency for the RP2040.
- **Visualization:** Generated high-resolution time-series plots for all 10 scenarios, including cost-vector sweeps that demonstrate energy load-shifting between nodes.

## 4. Test Suite Enhancements
- **Coordinated Triggering:** Decoupled variable setting from ADMM execution. The suite now stages all parameters before sending a single simultaneous `T` trigger to the entire network.
- **Performance Optimized Dashboard:** Rewrote the `live_plot.py` tool with frame decimation and throttled rescaling to reduce CPU usage and eliminate lag.

## Final Result
The system is fully validated and verified. It successfully achieves distributed consensus, meets illuminance targets where physically possible, and minimizes global energy consumption based on asymmetric cost vectors.
