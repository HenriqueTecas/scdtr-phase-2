# SCDTR Phase 2: In-Depth Test Suite Analysis

This document provides a comprehensive technical analysis of the results obtained from the Phase 2 Automated Test Suite. The system implements a **Consensus ADMM** algorithm to solve a network-wide constrained optimization problem.

---

## 1. The Foundation: The Coupling-Gain Matrix ($K$)
Before analyzing individual tests, we must look at the "Physics of the Room" captured during calibration:

| Sensor (Row) | LED 1 (col) | LED 2 (col) | LED 3 (col) |
| :--- | :--- | :--- | :--- |
| **Node 1 Desk** | **33.36** | 2.77 | 1.35 |
| **Node 2 Desk** | 4.72 | **39.01** | 3.66 |
| **Node 3 Desk** | 3.95 | 8.71 | **32.48** |

**Observations:**
- **Self-Gains ($k_{ii}$):** These are the diagonal values. Node 2 has the strongest LED/Sensor pairing (39.01), while Node 3 is the weakest (32.48).
- **Cross-Coupling ($k_{ij}$):** These represent "spillover" light. Note that $k_{32} = 8.71$ is quite high, meaning Node 2's LED significantly illuminates Node 3's desk. This creates a strong incentive for the optimizer to "borrow" light from Node 2.

---

## 2. Test-by-Test Technical Breakdown

### **Test 1: Standby Baseline `Occ(0,0,0) C=[1,1,1]`**
*   **Objective:** Verify the "Energy Minimization" part of the cost function works when no constraints are active.
*   **Analysis:** All lower bounds ($L_i$) are 0. The cost function $\sum 1.0 \cdot u_i$ is minimized at $u = [0,0,0]$.
*   **Result:** All nodes settled at ~0.00 duty. Small non-zero values (0.002) represent minor sensor noise or $\lambda$ integrator dither.

### **Test 2: Node 1 Dominance `Occ(2,0,0) C=[1,1,1]`**
*   **Objective:** Test single-node tracking and identity mapping.
*   **Analysis:** Only Desk 1 needs 20 Lux. Since costs are equal, the cheapest solution is for Node 1 to provide its own light.
*   **Result:** Node 1 hit **19.99 Lux** perfectly. Its duty cycle was **0.648**. (Check: $0.648 \times 33.36 \approx 21.6$ Lux theoretical max). The PI controller successfully trimmed the duty cycle to hit exactly 20.0.

### **Test 3: Node 2 Dominance `Occ(0,2,0) C=[1,1,1]`**
*   **Objective:** Test the middle node's performance.
*   **Analysis:** Desk 2 needs 20 Lux. 
*   **Result:** Node 2 hit **20.00 Lux** at **0.560 duty**. Because Node 2 has the highest gain (39.01), it requires significantly less duty cycle than Node 1 did to reach the same target (0.56 vs 0.64).

### **Test 4: Node 3 Dominance `Occ(0,0,2) C=[1,1,1]`**
*   **Objective:** Test the "weakest" node's tracking.
*   **Analysis:** Desk 3 needs 20 Lux.
*   **Result:** Node 3 hit **19.99 Lux** at **0.591 duty**. 

### **Test 5: Full Room Consensus `Occ(2,2,2) C=[1,1,1]`**
*   **Objective:** Demonstrate distributed cooperation.
*   **Analysis:** This is the first test where ADMM "Consensus" is critical. Because every node is on, they are all helping each other.
*   **Result:** **Massive Efficiency Gain.** To hit 20 Lux alone, Node 1 needed 0.648 duty. In this group test, it only needed **0.557**. It saved ~14% energy by "stealing" spillover light from Nodes 2 and 3.

### **Test 6: The "Valley" Profile `Occ(1,2,1) C=[1,1,1]`**
*   **Objective:** Test mixed targets (10, 20, 10).
*   **Analysis:** Node 2 must stay bright, but its light spills to Nodes 1 and 3.
*   **Result:** Node 3 (which only needed 10 lux) was so heavily illuminated by Node 2 ($k_{32}=8.71$) that it only needed **0.095 duty** to stay at 10 Lux.

---

## 3. Economic Research: The Cost Vector ($c_i$)

### **Test 7: The "Expensive Neighbor" `C=[1, 10, 1]`**
*   **Objective:** Prove the system can shift load away from expensive nodes.
*   **Analysis:** Node 2 is 10x more expensive than its neighbors.
*   **Result:** Node 2 dropped its duty cycle to **0.332**. To compensate and keep Node 2's desk at 20 lux, **Node 1 blasted its LED to 0.955 duty (35.3 Lux)**. Node 1 "over-illuminated" its own desk to provide the necessary spillover to save the expensive energy at Node 2.

### **Test 8: The "Cheap Center" `C=[10, 1, 10]`**
*   **Objective:** Reverse the burden.
*   **Analysis:** Outer nodes are 10x more expensive. Middle node is cheap.
*   **Result:** Node 2 took the full burden, hitting **42.72 Lux (0.945 duty)**. Nodes 1 and 3 dropped their duty cycles significantly (~0.2–0.4), relying on Node 2 to satisfy their requirements.

---

## 4. Performance Metrics and Plots

- **`plot_steady_state_occupancy.png`**: Shows the linear relationship between occupancy state and lux. The "plateau" effect at 20 lux proves the ADMM constraints are "hard" and reliably met.
- **`plot_cost_comparison.png`**: This is the "Money Plot." It shows the duty cycle bars shifting as the cost vector changes. When a bar disappears (Test 9, Node 2), it proves the system prioritized energy cost over local visibility.
- **`plot_convergence_research.png`**: Shows that the Visibility Error ($V$) drops to near zero within **10 iterations**. This justifies using a smaller $N_{max}$ in production to save CPU cycles.

## 5. Conclusion
The test results confirm that the system is not just a collection of independent controllers, but a **coordinated intelligent network**. It successfully utilizes the coupling-gain matrix to make real-time decisions about which LED is the "cheapest" source of light for any given desk at any given moment.
