# Oral Evaluation Guide

This file is a detailed guide to the current firmware in `main/`. It is written for an oral evaluation: use it to explain what the project does, how the code is structured, what the ADMM implementation is doing, and what tradeoffs were made.

Important: this guide describes the current code state. A few report statements may be older than the code. In the current code, `ADMM_RHO = 5.0`, `ADMM_MAXITER = 50`, there is no `ADMM_MAXITER_HARD`, and the final PI reference after ADMM uses the full coupling dot product `d_i + sum_j k_ij u_bar_j`.

## 1. One-Minute Summary

The project is a distributed lighting controller for a 3-node network. Each node has an LED actuator and an LDR sensor. The goal is to satisfy desk illuminance lower bounds while minimizing weighted energy cost across all nodes.

The system has two layers:

- A fast local PI loop runs at 100 Hz and drives the local LED duty cycle.
- A distributed optimizer, implemented using consensus ADMM, computes a coordinated duty-cycle vector that accounts for cross-coupling between desks.
- CAN is used for all node-to-node communication.
- Calibration measures the gain matrix `k_ij`, where `k_ij` means the illuminance change at sensor/desk `i` caused by LED `j`.
- ADMM runs as a non-blocking state machine so it does not stall the 100 Hz PI loop.
- Each ADMM iteration broadcasts a full local primal vector `x_i`, not the dual variable and not the consensus average.
- Each node locally computes the same consensus average if it receives the same set of `x_i` vectors for that iteration.
- A one-step receive lookahead buffer, `admm_recv_next`, handles the case where a peer is one ADMM iteration ahead.

## 2. Main Architecture

The firmware runs on an RP2040-style dual-core setup.

- Core 0 runs `setup()` and `loop()` in `main.ino`.
- Core 0 handles sampling, PI control, serial commands, ADMM state updates, calibration state updates, and CAN message dispatch from the receive queue.
- Core 1 runs `setup1()` and `loop1()` in `main.ino`.
- Core 1 owns the low-level MCP2515 CAN controller interface, interrupt handling, transmit queue draining, receive buffer reading, and error reporting into a queue.
- The two cores communicate using Pico SDK queues: `can_rx_queue`, `can_tx_queue`, and `core1_ready_queue`.
- CAN messages are not handled directly in the interrupt. The interrupt only sets `can_got_irq = true`; `loop1()` later reads the MCP2515 buffers.

Why this architecture matters:

- The PI loop must stay periodic at 100 Hz.
- CAN SPI operations and network delays should not block control timing.
- The ADMM optimizer must be non-blocking because waiting for peers inside the PI loop would increase jitter.
- Queues decouple CAN interrupt timing from application-level message handling.

## 3. File Map

`main.ino`:

- Defines hardware constants, identity mapping, global state, queues, calibration state, setup/loop, network wakeup, calibration state machine, CAN message dispatch, command forwarding, and ADMM result application.

`admm.h`:

- Declares ADMM constants, variables, state machine enum, and API.

`admm.cpp`:

- Implements consensus ADMM, including initialization, local QP solve, receive windows, iteration tags, consensus average, dual update, residual calculation, fixed iteration stopping, and one-step lookahead promotion.

`can_comms.h`:

- Defines the 11-bit CAN ID layout, message types, subtypes, ADMM wire quantization, and helper frame builders.

`can.h`:

- Defines a minimal Linux-style `can_frame` struct and CAN ID flags/masks.

`commands.ino`:

- Parses serial commands, forwards commands to other nodes over CAN, handles local commands, starts ADMM, changes occupancy/cost/reference, reports metrics, and controls calibration/tuning commands.

`calibration.h`:

- Provides blocking local calibration helpers, a duty sweep, feedforward duty calculation, and integrator seeding.

`lux.h`:

- Reads the LDR voltage, applies mean or median filtering, converts voltage to resistance, and converts resistance to lux.

`pid.h`:

- Implements the PI controller, setpoint weighting, anti-windup modes, bumpless transfer, feedback mode, and occupancy state.

`metrics.h`:

- Tracks energy consumption, visibility error, flicker error, sample count, and reset behavior.

`circular_buffer.h`:

- Implements a fixed-size ring buffer for last-minute lux/duty readout.

`mcp2515.h` and `mcp2515.cpp`:

- Implement the MCP2515 SPI CAN driver: register access, reset, bit-rate configuration, filters, modes, transmit, receive, interrupts, and error flags.

## 4. Data Model and Physical Meaning

Node identity:

- `LUMINAIRE` is the node address and is also used as the CAN source/destination ID.
- It is assigned in `setup()` by reading the flash unique ID and matching the last byte against `UID_TABLE`.
- `UID_TABLE = {33, 54, 39}` maps physical boards to logical node IDs 1, 2, and 3.
- If the board UID is not known, the firmware prints a discovery message forever.

Network topology:

- `N_NODES = 3`.
- `other_nodes[]` stores the two peer node IDs.
- `n_other_nodes` counts how many peers were discovered during wakeup.
- `all_nodes[]` is built and sorted so all nodes agree on the calibration active-node order.

Illuminance model:

- `coupling_gains[i][j]` stores `k_ij`, the lux contribution at desk `i` from LED `j` at full duty.
- `sys_gain` is the local self-gain `k_ii`.
- `sys_background` is the local background lux measured during calibration.
- The local static estimate is often `sys_background + sys_gain * duty`.
- ADMM uses the full model `d_i + sum_j k_ij u_j` for each desk constraint.

Occupancy:

- `pid.get_occupancy()` returns 0, 1, or 2.
- Occupancy 0 means off/unoccupied and maps to lower bound 0 in the current code.
- Occupancy 1 maps to `ref_low`, default 10 lux.
- Occupancy 2 maps to `ref_high`, default 20 lux.

Energy cost:

- `energy_cost` is the local cost coefficient for this node's own LED.
- In ADMM, each node's cost vector `admm_c` is zero except at its own index: `admm_c[LUMINAIRE] = energy_cost`.
- This is why a high-cost node can be helped by lower-cost neighbors if coupling allows it.

## 5. Control Loop

The control loop is in `loop()` in `main.ino`.

Main timing:

- `SAMPLE_PERIOD_US = 10000`, so the sampling/control period is 10 ms or 100 Hz.
- `last_sample_us` stores the previous sample time.
- `jitter_us` is the absolute deviation between actual period and 10 ms.

Per sample:

- The code calls `measureLux()` to read the filtered LDR lux value.
- If calibration is done, it computes the duty command.
- If feedback is enabled, duty is `pid.compute_control(r, lux_value)`.
- If feedback is disabled, duty is `serial_duty_cycle`.
- The duty is constrained to `[0, 1]`.
- The LED PWM output is updated with `analogWrite(LED_PIN, u_raw * PWM_MAX)`.
- The last-minute buffer stores `{lux_value, duty_cycle}`.
- Metrics are updated with `updateMetrics(r, duty_cycle, dt)`.
- Streaming outputs are printed if `stream_y`, `stream_u`, or `stream_j` are enabled.

After the sample block:

- `serial_command()` handles local serial input.
- `handle_buffer_readout()` gradually prints buffered data if requested.
- `admm_tick()` advances the ADMM state machine by at most one logical stage.
- If `admm_tick()` returns true, `admm_apply_result()` updates the PI reference/integrator.
- `process_can_messages()` handles queued received CAN frames.
- `calibration_loop()` advances the calibration state machine.

Important ordering:

- `admm_tick()` runs before `process_can_messages()`.
- The comment explains why: stage transitions must be visible before new ADMM packets are classified into current or next receive windows.
- If CAN messages were processed first, a packet might be classified using an old `admm_iter` or old ADMM stage.

## 6. PID Controller

The PI controller is implemented in `pid.h`.

State:

- `kp`, `ki`, `kt`, and `b` are controller parameters.
- `u_proportional`, `u_integral`, and `u` store controller terms.
- `anti_windup` selects the anti-windup mode.
- `feedback` selects closed-loop or open-loop mode.
- `bumpless_transfer` enables adjustment when `kp` or `b` changes.
- `occupancy` stores 0, 1, or 2.

Initialization:

- `PID::init(int LUMINAIRE)` selects gains by node ID.
- Current code uses the same placeholder-like values for all three nodes: `kp = 0.028`, `ki = 0.155`, `kt = 5.0`, `b = 1.0`.
- The default anti-windup mode is `ANTI_WINDUP_STOP_INT`.
- The default feedback mode is enabled.
- The default occupancy is 0.
- The sample time `h` defaults to 0.01 seconds and is later set from `SAMPLE_PERIOD_US`.

Control law:

- Error is `error = r - y`.
- Setpoint-weighted proportional term is `kp * (b * r - y)`.
- Unsaturated controller command is `v = u_proportional + u_integral`.
- Saturated duty is `u = constrain(v, 0, 1)`.

Anti-windup:

- `ANTI_WINDUP_OFF`: always integrates `ki * h * error`.
- `ANTI_WINDUP_BACK_CALC`: integrates `ki * h * error + kt * h * (u - v)`.
- `ANTI_WINDUP_STOP_INT`: stops integration if saturated in the same direction as the error.
- Stop-integration logic prevents further windup at high saturation when error wants more output, and at low saturation when error wants less output.

Bumpless transfer:

- If `bumpless_transfer` is enabled, the integrator is adjusted when `kp` or `b` changes.
- The adjustment keeps the total controller output from jumping due only to parameter changes.
- `kp_old` and `b_old` store previous values for this compensation.

ADMM integration:

- After ADMM completes, `admm_apply_result()` seeds the PI integrator with `admm_u_avg[LUMINAIRE]`.
- This makes the PI output start near the ADMM optimal duty, reducing transient bumps.

## 7. Lux Measurement

The measurement pipeline is in `lux.h`.

LDR calibration:

- `ldr_m` and `ldr_b` are per-board calibration constants.
- They are loaded from `LDR_M_TABLE` and `LDR_B_TABLE` after identifying `LUMINAIRE`.

Voltage reading:

- `readVoltage()` selects between no filter, mean filter, and median filter using `filter_mode`.
- `filter_mode = 0`: single `analogRead`.
- `filter_mode = 1`: mean filter over `NUM_SAMPLES = 64`, default.
- `filter_mode = 2`: median filter over `MEDIAN_FILTER_SIZE = 9`.

Mean filter:

- `readVoltage_mean()` sums 64 ADC samples.
- It delays `SAMPLE_DELAY_US = 100` microseconds between samples.
- It returns average ADC converted to voltage using `ADC_MAX` and `VCC`.

Median filter:

- `median_push()` shifts the median window and inserts a new ADC value.
- `median_read()` copies the window, insertion-sorts it, and returns the middle sample.
- `readVoltage_median()` pushes 9 samples then converts the median ADC value to voltage.

Lux conversion:

- `voltageToResistance(voltage)` uses the divider formula `R_LDR = R_FIXED * (VCC / voltage - 1)`.
- If voltage is near zero, it returns a very large resistance to avoid divide-by-zero.
- `resistanceToLux(resistance_ohm)` uses the fitted log-log relation: `lux = 10^((log10(R) - ldr_b) / ldr_m)`.
- `measureLux()` composes voltage reading, resistance conversion, and lux conversion.

Potential oral point:

- The LDR conversion is nonlinear and depends on the board-specific calibration constants.
- Filtering reduces noise but also costs time. The mean filter takes about 64 * 100 us = 6.4 ms plus ADC overhead, which is significant inside a 10 ms control period.

## 8. Metrics

Metrics are in `metrics.h`.

Tracked variables:

- `energy_consumption` accumulates energy in joules.
- `visibility_error` accumulates positive illuminance shortfall.
- `flicker_error` accumulates a flicker-like duty oscillation metric.
- `sample_count` counts samples since reset.
- `u_prev1` and `u_prev2` store the two previous duty values.
- `flicker_holdoff` suppresses flicker accumulation immediately after reference changes.

Energy:

- `energy_consumption += duty * MAXIMUM_POWER * dt`.
- `MAXIMUM_POWER = 0.0132` W in `main.ino`.
- Because duty is normalized, energy is proportional to duty and time.

Visibility:

- The code estimates illuminance as `sys_background + sys_gain * duty`.
- It adds only positive shortfall `r_ref - l_estimated`.
- This follows the specified duty-cycle-based estimate, but it is local and does not include neighbor coupling.

Flicker:

- `diff1 = d - u_prev1`.
- `diff2 = u_prev1 - u_prev2`.
- If `diff1 * diff2 < 0`, the duty changed direction, so the code adds `abs(diff1) + abs(diff2)`.
- `FLICKER_EXCLUDE_SAMPLES = 50`, so about 0.5 seconds are excluded after reference steps at 100 Hz.

Reset:

- `resetMetrics()` clears all metric accumulators, previous duty values, holdoff, and sample count.
- In the current command parser, there is no visible serial command that calls `resetMetrics()`, so metrics normally persist until firmware reset or until this function is called from future code.

## 9. Last-Minute Buffer

The ring buffer is in `circular_buffer.h`.

Structure:

- `buffer_data` stores `lux_value` and `duty_cycle`.
- `CircularBuffer<CAPACITY>` stores a fixed array of samples.
- In `main.ino`, `CircularBuffer<6000> last_min_buf` stores about one minute at 100 Hz.

Push behavior:

- `push()` writes to `write_idx`, advances modulo capacity, and increases `count` until full.
- Once full, the oldest data is overwritten.

Pop behavior:

- `pop()` returns `{0, 0}` if empty.
- Otherwise it computes the oldest index from `write_idx - count`, decrements `count`, and returns that sample.

Use:

- `loop()` pushes one sample per control period.
- `g b y <i>` or `g b u <i>` starts a buffered readout.
- `handle_buffer_readout()` prints one value per loop iteration, comma-separated, to avoid dumping a huge buffer in one blocking burst.

## 10. CAN Protocol

CAN protocol helpers are in `can_comms.h`.

11-bit ID layout:

- Bits `[10:8]`: message type, 3 bits.
- Bits `[7:4]`: destination node, 4 bits.
- Bits `[3:0]`: source node, 4 bits.
- Destination `0` means broadcast.
- Source is `LUMINAIRE`.

Macros:

- `MAKE_CAN_ID(type, dest, src)` packs the ID.
- `CAN_ID_TYPE(id)` extracts bits `[10:8]`.
- `CAN_ID_DEST(id)` extracts bits `[7:4]`.
- `CAN_ID_SRC(id)` extracts bits `[3:0]`.

Message types:

- `MSG_WAKEUP = 0x0`: peer discovery.
- `MSG_CAL = 0x1`: distributed calibration.
- `MSG_SET = 0x2`: forwarded remote set commands.
- `MSG_GET = 0x3`: forwarded remote get commands.
- `MSG_REPLY = 0x4`: replies to remote get commands.
- `MSG_ADMM = 0x6`: ADMM vector broadcast.
- `MSG_CTRL = 0x7`: ACK and ADMM trigger control messages.

Subtypes:

- Wakeup: `SUB_SYN`, `SUB_SYN_ACK`.
- Calibration: `SUB_CAL_READY`, `SUB_CAL_ON`, `SUB_CAL_LUX`, `SUB_CAL_DONE`.
- Control: `SUB_ACK`, `SUB_ADMM_TRIGGER`.

Frame builders:

- `can_send_float(dest, msg_type, sub, value)` sends a 5-byte frame: subtype plus 4-byte float.
- `can_send_byte(dest, msg_type, sub, val)` sends a 2-byte frame: subtype plus byte value.
- `can_send_sub(dest, msg_type, sub)` sends a 1-byte subtype-only frame.
- `can_send_admm(dest, iter, values)` sends a 7-byte ADMM frame: one iteration byte plus three signed 16-bit fixed-point components.

ADMM wire format:

- `ADMM_WIRE_SCALE = 2048.0`.
- `ADMM_WIRE_MIN = -16.0`.
- `ADMM_WIRE_MAX = 15.9995`.
- Each vector component is rounded to signed fixed point using scale 2048.
- The resolution is `1 / 2048`, about 0.000488.
- The values are ADMM vector components, normally duty-like values, not lux values.
- The frame is specialized to `ADMM_N = 3`: three components fit into 6 bytes plus 1 iteration byte.

Transmit queue:

- `can_queue_tx()` pushes a frame into `can_tx_queue` using `queue_add_blocking`.
- Core 1 later removes frames from this queue and sends them with `can0.sendMessage()`.
- This decouples normal CAN transmit timing from the control loop, but it is still a blocking software queue call if the 64-frame queue becomes completely full.

## 11. Low-Level CAN Driver

The CAN frame structure is in `can.h`.

`can_frame` fields:

- `can_id`: standard or extended CAN ID plus optional flags.
- `can_dlc`: payload length, max 8 bytes.
- `data[8]`: payload.
- The code uses standard 11-bit CAN IDs in this project.

MCP2515 driver:

- `mcp2515.h` defines bit-rate register constants, enums, register addresses, instruction opcodes, and the `MCP2515` class.
- `mcp2515.cpp` implements the class using SPI.

Constructor:

- Initializes the SPI channel.
- Sets TX, RX, and SCK pins to SPI function.
- Sets 8-bit SPI, CPOL 0, CPHA 0, MSB first.
- Configures chip select GPIO and deasserts it with `endSPI()`.

SPI helpers:

- `startSPI()` pulls chip select low with a few `nop`s.
- `endSPI()` pulls chip select high with a few `nop`s.

Reset:

- Sends MCP2515 reset instruction.
- Sleeps 10 ms for oscillator/controller stabilization.
- Clears transmit buffer control registers.
- Clears receive buffer control registers.
- Enables RX0, RX1, error, and message error interrupts.
- Configures receive buffers to accept standard or extended IDs that meet filter criteria.
- Clears filters and masks initially.

Register access:

- `readRegister()` reads one register.
- `readRegisters()` reads a contiguous block.
- `setRegister()` writes one register.
- `setRegisters()` writes a contiguous block.
- `modifyRegister()` uses MCP2515 bit modify instruction.

Modes:

- `setConfigMode()`, `setListenOnlyMode()`, `setSleepMode()`, `setLoopbackMode()`, and `setNormalMode()` call `setMode()`.
- `setMode()` writes the requested mode and polls `CANSTAT` for up to about 10 ms.

Bitrate:

- `setBitrate(canSpeed, canClock)` enters config mode and writes CNF1, CNF2, CNF3 based on the requested speed and oscillator.
- The project uses `CAN_1000KBPS` with `MCP_16MHZ`.

Filters:

- `prepareId()` converts standard or extended CAN IDs into MCP2515 SID/EID register format.
- `setFilterMask()` writes mask registers.
- `setFilter()` writes filter registers.
- In `setup1()`, masks are set so the hardware filters by destination bits only: accept `dest == LUMINAIRE` or `dest == BROADCAST`.

Transmit:

- `sendMessage(TXBn, frame)` writes a frame into a selected transmit buffer and requests transmission.
- It checks DLC, standard/extended ID, RTR flag, ID packing, data payload, and TX error flags.
- `sendMessage(frame)` scans TXB0, TXB1, TXB2 and uses the first non-busy buffer.
- If all transmit buffers are busy, it returns `ERROR_ALLTXBUSY`.

Receive:

- `readMessage(RXBn, frame)` reads ID, DLC, RTR flag, payload, and clears the RX interrupt flag.
- `readMessage(frame)` checks status and reads RXB0 or RXB1 depending on which has a message.

Errors:

- `checkError()` checks the MCP2515 error flags.
- `getErrorFlags()` reads `EFLG`.
- `clearRXnOVRFlags()` clears RX overflow flags.
- `clearMERR()` clears message error interrupt.
- `clearERRIF()` clears error interrupt.
- `errorCountRX()` and `errorCountTX()` read receive/transmit error counters.

## 12. Core 1 CAN Runtime

`setup1()` and `loop1()` are in `main.ino`.

`setup1()`:

- Waits for Core 0 to send a byte on `core1_ready_queue`.
- Resets the MCP2515.
- Sets the bit rate to 1 Mbps with a 16 MHz MCP2515 oscillator.
- Configures hardware filters so only frames for this node or broadcast are accepted.
- Enters normal CAN mode.
- Enables a GPIO falling-edge interrupt on the MCP2515 interrupt pin.

`loop1()` transmit behavior:

- Maintains one `tx_pending` frame.
- If there is no pending frame, it tries to remove one from `can_tx_queue`.
- It calls `can0.sendMessage(&tx_frm)`.
- On success, it clears `tx_pending`.
- If all MCP2515 TX buffers are busy, it keeps the frame pending and retries later.
- For other transmit errors, it logs a `[CAN TX DROP]` and drops the frame.

`loop1()` receive behavior:

- If no CAN interrupt is pending and the interrupt pin is high, it returns quickly.
- Otherwise, it clears the software interrupt flag and reads MCP2515 interrupt flags.
- If RX0 has a frame, it reads RXB0 and queues the frame into `can_rx_queue`.
- If RX1 has a frame, it reads RXB1 and queues the frame into `can_rx_queue`.
- If error flags are set, it creates a sentinel frame with ID `0xFFFFFFFF` containing interrupt flags, error flags, RX error count, and TX error count.
- It clears RX overflow, ERRIF, and MERR flags where appropriate.
- It loops until no interrupt or error condition is left to handle.

Why sentinel frames:

- Core 1 cannot directly call the application-level error handling in Core 0.
- A sentinel frame uses the same queue mechanism as normal frames.
- `process_can_messages()` recognizes `can_id == 0xFFFFFFFF` and prints the CAN error details.

## 13. Network Wakeup

`network_wakeup()` is in `main.ino`.

Purpose:

- Discover the other two nodes before calibration and normal operation.
- Fill `other_nodes[]` and `n_other_nodes`.

Behavior:

- Repeatedly broadcasts `MSG_WAKEUP` with `SUB_SYN` every 200 ms.
- Allows serial commands while waiting by calling `serial_command()`.
- Reads frames directly from `can_rx_queue`.
- If a frame is not a wakeup frame, it puts the frame back into `can_rx_queue` and continues.
- If a wakeup frame comes from a valid node and has not been seen, it records that peer.
- If it receives `SUB_SYN`, it responds with `SUB_SYN_ACK`.
- It exits once `n_other_nodes == N_NODES - 1`.

Important detail:

- Peer discovery is based on CAN source IDs, not USB port names.
- The node ID is determined earlier from `UID_TABLE`.

## 14. Distributed Calibration

Calibration is a non-blocking state machine in `calibration_loop()`.

State enum:

- `CAL_BARRIER`: wait until all peers are ready.
- `CAL_BACKGROUND`: all LEDs off, wait, then measure background lux.
- `CAL_ACTIVE_ON`: this node is active, LED on, waiting for settling.
- `CAL_ACTIVE_SAMPLE`: this node samples its own response to its own LED.
- `CAL_ACTIVE_WAIT`: this node waits for peer lux responses.
- `CAL_PASSIVE_WAIT`: this node waits for another active node's `CAL_ON`.
- `CAL_PASSIVE_SETTLE`: another node is active; wait, measure, send lux to active node.
- `CAL_DONE`: calibration is complete.

Calibration trigger:

- In `setup()`, if all peers were discovered, `cal_stage` is set to `CAL_BARRIER`.
- Command `c` without a subcommand also triggers `CAL_BARRIER`.
- If calibration is done and a remote calibration ready/on message is seen, `process_can_messages()` resets the local calibration state to `CAL_BARRIER`.

Barrier:

- In `CAL_BARRIER`, each node broadcasts `SUB_CAL_READY` every 500 ms.
- Each node counts unique ready messages from peers using `cal_barrier_peers[]`.
- Once all peers are ready, it builds the sorted node list, turns its LED off, and enters background measurement.

Background:

- In `CAL_BACKGROUND`, each node waits `CAL_SETTLE_MS = 3000` ms.
- It measures background lux with `measureLux()`.
- It sets `sys_background = background_lux`.
- It sets `cal_round = -1`, then calls `cal_advance_round()`.

Round order:

- `cal_build_node_list()` constructs `all_nodes` from self plus peers and insertion-sorts it.
- Sorting ensures all nodes agree which node is active in each round.
- `cal_advance_round()` increments `cal_round`.

Active node behavior:

- If `all_nodes[cal_round] == LUMINAIRE`, this node is active.
- It turns the LED on at `PWM_MAX`.
- It broadcasts `SUB_CAL_ON`.
- It waits for settling in `CAL_ACTIVE_ON`.
- Then in `CAL_ACTIVE_SAMPLE`, it measures own lux.
- It computes `coupling_gains[LUMINAIRE][LUMINAIRE] = my_lux - background_lux`.
- It enters `CAL_ACTIVE_WAIT` and waits for peer lux reports.
- If all reports arrive or a 5 second timeout occurs, it turns its LED off, broadcasts `SUB_CAL_DONE`, and advances to the next round.

Passive node behavior:

- If another node is active, this node enters `CAL_PASSIVE_WAIT`.
- On `SUB_CAL_ON` from the expected active node, it turns its own LED off, stores the active ID, waits for settling, and enters `CAL_PASSIVE_SETTLE`.
- In `CAL_PASSIVE_SETTLE`, it measures lux, computes `coupling_gains[LUMINAIRE][cal_expected_active] = my_lux - background_lux`, and sends the measured lux to the active node using `SUB_CAL_LUX`.
- It then waits for `SUB_CAL_DONE` or times out after about 8 seconds.

Completion:

- After all rounds, `cal_advance_round()` sets `sys_gain = coupling_gains[LUMINAIRE][LUMINAIRE]`.
- It sets `sys_background = background_lux`.
- It sets `calibrated = true` and `cal_stage = CAL_DONE`.
- It prints the full gain matrix row-by-row.
- It starts ADMM locally with `admm_request(true)`.

Why `is_responder=true` after calibration:

- All nodes independently reach calibration completion.
- If every node broadcast an ADMM trigger, each node would receive extra triggers.
- Using responder mode starts ADMM locally without rebroadcasting the trigger.

Blocking local calibration helper:

- `calibrate_system_gain()` in `calibration.h` is a separate blocking helper for command `c g`.
- It turns the local LED off, waits 5 s, measures background, then tests 25%, 50%, 75%, and 100% duty with 5 s waits.
- It sets `sys_gain` from the full-duty response minus background.
- It is not the distributed 3-node gain matrix calibration state machine.

Duty sweep:

- `sweep(steps)` in `calibration.h` prints duty, voltage, resistance, and lux for a local duty sweep.
- It is blocking and intended for manual calibration/analysis.

Feedforward:

- `feedforward(ref_lux)` returns `(ref_lux - sys_background) / sys_gain` constrained to `[0, 1]`.
- If not calibrated or gain is invalid, it returns 0.
- `apply_feedforward(ref_lux)` seeds the PI integrator with that duty.

## 15. Serial Commands

Commands are parsed in `commands.ino`.

Print abstraction:

- `commands(char *buffer, Print &out)` writes to any Arduino `Print` object.
- `PRINTF(out, ...)` formats into `_pbuf[128]` and calls `out.print()`.
- This lets the same command handler support local `Serial` output and other `Print` implementations.

Hub forwarding:

- Commands with a target node ID are inspected before local handling.
- If the target ID is not this node, `hub_forward(buffer, dest)` is called.
- Commands `h`, `c`, `i`, and `T` are not forwarded by this generic logic.

Identity:

- `i` prints `i <LUMINAIRE>`.
- It bypasses forwarding and helps verify which physical board is connected.

ADMM:

- `T` calls `admm_request(false)` and prints `ack`.
- As initiator, it broadcasts `SUB_ADMM_TRIGGER` and then starts ADMM locally.
- Current code does not parse `T <n>` to change `ADMM_MAXITER`.

Open-loop duty:

- `u <i> <val>` sets `serial_duty_cycle`.
- If feedback is off, it immediately writes the PWM duty.

Reference:

- `r <i> <val>` sets local reference `r`.
- It resets `flicker_holdoff`.

Occupancy:

- `o <i> <val>` constrains mode to 0, 1, or 2.
- It updates PID occupancy.
- It sets local `r` to high, low, or 0 based on occupancy.
- It resets flicker holdoff.
- It starts ADMM because the lower bound changed.

Anti-windup:

- `a <i> <val>` sets anti-windup mode.
- Values are 0 off, 1 back-calculation, 2 stop-integration.

Feedback:

- `f <i> <val>` sets feedback mode.
- Before changing mode, it stores current duty in `serial_duty_cycle`.
- If enabling feedback, it seeds the integrator with feedforward.

Streaming:

- `s y <i>` starts lux streaming.
- `s u <i>` starts duty streaming.
- `s j <i>` starts jitter streaming.
- `S y/u/j <i>` stops the selected stream.

Phase 2 parameters:

- `O <i> <val>` sets `ref_high`.
- `U <i> <val>` sets `ref_low`.
- `C <i> <val>` sets `energy_cost`.
- These commands set parameters only. Remote set handling comments say the hub/test should send ADMM trigger separately for coordinated starts.

Calibration and tuning:

- `c` triggers the distributed calibration barrier.
- `c b [value]` reads or sets `ldr_b` and prints current lux.
- `c m <value>` sets `ldr_m`.
- `c g` runs blocking local system gain calibration.
- `c s [steps]` runs blocking local duty sweep.
- `c ?` prints calibration, PID, and phase 2 parameter status.
- `c p <i> <val>` sets `kp`.
- `c i <i> <val>` sets `ki`.
- `c B <i> <val>` sets setpoint weighting `b`.
- `c t <i> <val>` sets anti-windup gain `kt`.
- `c w <i> <val>` sets bumpless transfer on/off.
- `c f <i> <val>` sets measurement filter mode.

Get commands:

- `g y <i>` prints measured lux.
- `g u <i>` prints current duty, using PID duty if feedback is on or serial duty if feedback is off.
- `g r <i>` prints reference.
- `g v <i>` prints raw ADC voltage.
- `g o <i>` prints occupancy.
- `g a <i>` prints anti-windup mode.
- `g f <i>` prints feedback mode.
- `g d <i>` prints estimated external illuminance `max(0, lux_value - sys_gain * duty_cycle)`.
- `g k <i>` prints the local row of the gain matrix.
- `g p <i>` prints instantaneous power estimate `pid.get_u() * MAXIMUM_POWER`.
- `g t <i>` prints local time in seconds.
- `g b y/u <i>` starts last-minute buffer readout for lux or duty, local node only.
- `g E <i>` prints energy consumption.
- `g V <i>` prints average visibility error.
- `g F <i>` prints normalized flicker metric.
- `g O <i>` prints high lower bound.
- `g U <i>` prints low lower bound.
- `g L <i>` prints current lower bound from occupancy.
- `g C <i>` prints energy cost.
- `g K <i>` prints ADMM primal residual.
- `g J <i>` prints ADMM dual residual.

Help:

- `h` prints a command reference.

## 16. Command Forwarding and CAN Dispatch

`hub_forward()` in `main.ino` maps local serial commands to CAN messages for other nodes.

Forwarded get:

- `g <sub> <idx>` becomes `MSG_GET` to the destination.
- The subtype is the requested variable character.

Forwarded set:

- `r`, `u`, `O`, `U`, and `C` use `can_send_float`.
- `o`, `f`, and `a` use `can_send_byte`.
- `s` and `S` build a custom `MSG_SET` frame with stream variable and on/off byte.

Remote receive:

- `process_can_messages()` decodes queued CAN frames.
- `MSG_SET` applies remote set commands and sends `SUB_ACK`.
- `MSG_GET` builds a `MSG_REPLY` frame with a float result.
- `MSG_REPLY` prints the response over local serial.
- `MSG_CAL` advances calibration logic.
- `MSG_ADMM` unpacks and forwards components to `admm_receive()`.
- `MSG_CTRL` handles ACK and ADMM trigger. Remote ACKs are printed as `ack:remote`.

Why snapshots:

- At the start of `process_can_messages()`, `snap_lux` and `snap_duty` are copied from global state.
- Replies use these snapshots so all replies in one dispatch pass are internally consistent.

ADMM message unpacking:

- `MSG_ADMM` expects `can_dlc >= 7`.
- Byte 0 is the iteration index.
- Bytes 1-2, 3-4, and 5-6 are three signed 16-bit fixed-point vector components.
- The frame is decoded into three `admm_receive(src, comp, iter, val)` calls.
- Because of this design, `admm_recv_count[src]` counts received components, not high-level frames.

## 17. ADMM Problem Formulation

The network-wide problem is:

```text
minimize    sum_i c_i u_i
subject to  k_i^T u + d_i >= L_i    for each desk i
            0 <= u_i <= 1           for each actuator i
```

Meaning:

- `u_i` is the physical duty cycle of luminaire `i`.
- `c_i` is the energy cost weight of luminaire `i`.
- `k_i` is row `i` of the coupling matrix.
- `d_i` is background lux at desk `i`.
- `L_i` is the occupancy-dependent lower bound.

Why distributed optimization is useful:

- A node's LED may illuminate neighboring desks.
- If one node has low energy cost, it may be cheaper for it to produce extra light that helps a high-cost neighbor.
- Independent local controllers cannot exploit this cross-coupling tradeoff.

Consensus ADMM:

- Each node `i` keeps a local vector `x_i`, representing its local proposed copy of the full duty vector.
- The network seeks agreement: all `x_i` should converge to a shared vector `u_bar`.
- Each node has a local constraint for its own desk illuminance and own actuator bounds.
- Each node has a local dual vector `lambda_i`.

Augmented Lagrangian idea:

- Pure dual decomposition can struggle with linear costs because the local minimization may not be unique.
- ADMM adds the quadratic penalty `(rho/2) ||x_i - u_bar||^2`.
- This makes the local primal update a strongly convex quadratic projection-like problem.
- Current code uses `ADMM_RHO = 5.0`.

Why consensus is not broadcast:

- General consensus update includes `x_i + lambda_i / rho`.
- If dual variables are initialized so `sum_i lambda_i = 0`, and all nodes perform the same dual update, the sum of lambdas remains zero.
- Therefore the consensus update reduces to a plain average:

```text
u_bar = (1/N) sum_i x_i
```

- Each node broadcasts `x_i`.
- Each node computes the same `u_bar` locally if it receives the same `x_i` vectors for that iteration.
- There is no need to broadcast `u_bar` or `lambda`.

Important caveat:

- This is exact for the synchronous algorithm if every node averages the same iteration data.
- With timeouts or missing packets, local `u_bar` copies can differ temporarily.
- Iteration tags and the lookahead buffer reduce the risk of mixing data from different iterations.

## 18. ADMM State Variables

Defined in `admm.cpp` and declared in `admm.h`.

Core variables:

- `admm_u[j]`: local primal vector `x_i[j]`, this node's proposed duty for node `j`.
- `admm_u_avg[j]`: local consensus average `u_bar[j]`.
- `admm_lambda[j]`: local dual vector `lambda_i[j]`.
- `admm_k[j]`: local coupling row `k_ij`, gain from LED `j` to this desk.
- `admm_c[j]`: local cost vector, nonzero only at `j == LUMINAIRE`.
- `admm_d_bg`: local background illuminance.
- `admm_L`: local lower bound from occupancy.
- `admm_n_sq`: squared norm `sum_j k[j]^2`.
- `admm_m_sq`: `admm_n_sq - k_ii^2`, used for two-constraint boundary candidates.

Receive variables:

- `admm_recv[src][comp]`: current-iteration received vector components.
- `admm_recv_seen[src][comp]`: whether a component was already seen in current window.
- `admm_recv_count[src]`: number of components seen from a source in current window.
- `admm_recv_next[src][comp]`: one-iteration-early received vector components.
- `admm_recv_next_seen[src][comp]`: whether a component was already seen in next window.
- `admm_recv_next_count[src]`: number of components seen from a source in next window.

State-machine variables:

- `admm_stage`: `IDLE`, `PRIMAL_UPDATE`, `WAIT_PEERS`, `DUAL_UPDATE`, or `DONE`.
- `admm_iter`: current local ADMM iteration index.
- `admm_wait_start_ms`: start time for peer wait timeout.
- `admm_running`: true while an ADMM run is active.

Diagnostics:

- `admm_primal_res = ||admm_u - admm_u_avg||`.
- `admm_dual_res = rho * ||admm_u_avg - admm_u_avg_prev||`.
- These residuals are recorded for plotting/status; current stopping is fixed-budget.

Constants:

- `ADMM_N = 3`.
- `ADMM_RHO = 5.0`.
- `ADMM_TIMEOUT = 150` ms.
- `ADMM_MAXITER = 50` in `admm.cpp`.

## 19. ADMM Initialization

`admm_init()` prepares a new ADMM run.

Steps:

- Resets residuals.
- Copies the local coupling row from `coupling_gains[LUMINAIRE][j]` into `admm_k[j]`.
- Builds the cost vector `admm_c[j]`, nonzero only at the local node index.
- Resets `admm_lambda[j]` to zero.
- Sets `admm_d_bg = sys_background`.
- Reads occupancy from `pid.get_occupancy()`.
- Computes raw lower bound `L_raw` from `ref_high`, `ref_low`, or zero.
- Computes maximum achievable local desk lux as `d_bg + sum_j k_j`.
- If occupancy is 0, sets `admm_L = 0`.
- Otherwise clamps `admm_L` to `[d_bg, max_achievable]` so the target is physically feasible.
- Computes `admm_n_sq = sum_j k_j^2`.
- Computes `admm_m_sq = admm_n_sq - k_ii^2`.

Warm start:

- `u_ff = feedforward(L_raw)`.
- The local component `admm_u[LUMINAIRE]` and `admm_u_avg[LUMINAIRE]` are set to `u_ff`.
- Non-own components are set to `0.5`.
- Reason: if an unoccupied node used `u_ff = 0` for all components, it would broadcast zeros for neighbors and drag the consensus down even if those neighbors are occupied.
- Starting non-own components at `0.5` is a neutral prior.

Receive window seeding:

- Both current and next receive windows are seeded with `0.5`.
- The local self component is seeded with `u_ff`.
- This means if timeout occurs, missing values do not default to zero.

## 20. ADMM Receive Window Logic

Helper functions:

- `admm_seed_window(buf, seen, count, seed)` initializes receive buffers and seen flags.
- `admm_store_component(buf, seen, count, src, comp, val)` stores one component and increments `count[src]` only the first time that source/component is seen.
- `admm_promote_next_window()` moves one-iteration-early packets into the current window after a dual update.

Why `recv_next` exists:

- Textbook ADMM is synchronous: every node completes iteration `k` before any node starts `k+1`.
- The firmware is a set of non-blocking local state machines, not a true global lock-step barrier.
- Node 1 may receive all vectors for `k`, update its local average, update lambda, increment to `k+1`, and broadcast `x_1(k+1)` before Node 2 has finished processing `k`.
- Node 2 must not average `x_1(k+1)` into its iteration `k` consensus.
- Dropping the early packet would also be wasteful and could cause later timeout.
- The next window stores the early `k+1` packet until the local node advances.

Receive behavior:

- If `iter == admm_iter`, store in `admm_recv`.
- If `iter == admm_iter + 1`, store in `admm_recv_next`.
- Otherwise, print a drop message.
- Invalid source or component indexes are also dropped.

Promotion behavior:

- After the dual update increments `admm_iter`, `admm_promote_next_window()` runs.
- For each peer, `admm_recv_count[peer]` becomes `admm_recv_next_count[peer]`.
- Components seen in the next window are copied into the current window.
- Components not seen are filled with the current `admm_u_avg[j]`.
- Next-window components are reset to `admm_u_avg[j]`, and next seen flags are cleared.

Interpretation:

- `admm_recv_count` is not an ADMM mathematical variable.
- It is a transport completeness counter.
- With the current packed CAN frame, one valid peer frame normally results in all three component calls and count `3/3`.
- The seen flags avoid duplicate components increasing the count twice.

## 21. ADMM Local Feasibility and Cost

`feasible(u)` checks whether a candidate local vector satisfies this node's constraint set.

Constraints:

- Own duty component must satisfy `0 <= u[LUMINAIRE] <= 1`.
- Local desk illuminance must satisfy `k_i^T u >= L_i - d_i`.

Important modeling detail:

- Only the local component is actuator-bounded in this local subproblem.
- Other components are this node's predictions of other nodes' duties, not variables this node can physically clamp.
- Consensus across nodes eventually aligns each component with the owner node's physically bounded value.

`predicted_lux(u)`:

- Computes `admm_d_bg + sum_j admm_k[j] * u[j]`.
- Used for debug/status after consensus updates.

`qp_cost(u, z_i)`:

- Computes the simplified local QP objective `0.5 * rho * u^T u - u^T z_i`.
- It is used to rank feasible candidates.
- Constants common to all candidates are omitted because they do not affect the argmin.

## 22. ADMM State Machine

`admm_start()`:

- Sets `admm_iter = 0`.
- Sets `admm_running = true`.
- Clears residuals.
- Copies current `admm_u_avg` into `admm_u_avg_prev`.
- Sets stage to `PRIMAL_UPDATE`.

`admm_request(is_responder)`:

- Logs the request.
- If ADMM is already running, it returns without restarting.
- If `is_responder == false`, it broadcasts `SUB_ADMM_TRIGGER`.
- Calls `admm_init()`.
- Calls `admm_start()`.

`admm_tick()`:

- Runs one stage of the ADMM state machine.
- Returns false unless ADMM just completed.
- On completion, returns true so `loop()` can call `admm_apply_result()`.

Stages:

- `IDLE`: nothing to do.
- `DONE`: nothing to do.
- `PRIMAL_UPDATE`: solve local primal update and broadcast `x_i`.
- `WAIT_PEERS`: wait for peer vectors or timeout.
- `DUAL_UPDATE`: compute consensus average, update dual, compute residuals, stop or continue.

## 23. ADMM Primal Update

In `PRIMAL_UPDATE`, the code computes:

```text
z_i[j] = rho * u_bar[j] - c_i[j] - lambda_i[j]
u_unc[j] = z_i[j] / rho
```

If `u_unc` is feasible:

- It becomes the new `admm_u`.

If `u_unc` is infeasible:

- The code evaluates a fixed set of analytical boundary candidates.
- `try_cand()` checks feasibility and keeps the feasible candidate with lowest QP cost.

Candidates:

- Candidate C1: illuminance boundary `k_i^T u = L_i - d_i`.
- Candidate C2: own duty lower bound `u_ii = 0`.
- Candidate C3: own duty upper bound `u_ii = 1`.
- Candidate C12: illuminance boundary plus `u_ii = 0`.
- Candidate C13: illuminance boundary plus `u_ii = 1`.

Candidate formulas:

- C1 uses `admm_n_sq` in the denominator because the active constraint normal is the full `k_i` vector.
- C12 and C13 use `admm_m_sq`, which removes the local self-gain term because `u_ii` is fixed.
- The code checks `admm_m_sq > 1e-6` before evaluating the two-constraint candidates to avoid division by zero or near-zero.

Fallback:

- If no feasible candidate is found, the code performs best-effort saturation.
- It clamps all components of `u_unc` to `[0, 1]`.
- It forces the own duty component to 1.0.
- This is intended for physically unreachable or numerically difficult cases.

Broadcast:

- After choosing `admm_u`, the node broadcasts it with `can_send_admm(BROADCAST, admm_iter, admm_u)`.
- It writes its own vector into `admm_recv[LUMINAIRE][j]`.
- It marks all own components as seen.
- It sets `admm_recv_count[LUMINAIRE] = ADMM_N`.
- It records `admm_wait_start_ms = millis()`.
- It enters `WAIT_PEERS`.

## 24. ADMM Wait Peers

In `WAIT_PEERS`, the node checks whether all peers have sent complete vectors.

Readiness:

- For each peer in `other_nodes`, it checks `admm_recv_count[peer] < ADMM_N`.
- If all peers have at least `ADMM_N` received components, it advances to `DUAL_UPDATE`.

Status logging:

- Every 50 ms it prints `[ADMM WAIT]`, the current iteration, elapsed time, and each peer's receive count.

Timeout:

- If elapsed time reaches `ADMM_TIMEOUT = 150` ms, it prints a timeout message.
- It reports which peers are missing components.
- It forces `DUAL_UPDATE`.

Important:

- Timeout prevents deadlock if a CAN packet is lost or delayed.
- Because receive windows were seeded, missing components are not necessarily zeros.
- This is a practical asynchronous implementation rather than a perfect synchronous ADMM barrier.

## 25. ADMM Consensus and Dual Update

In `DUAL_UPDATE`, the node computes the consensus average:

```text
for each component j:
    sum = 0
    for each source i:
        sum += admm_recv[i][j]
    admm_u_avg[j] = constrain(sum / ADMM_N, 0, 1)
```

Then it updates the dual:

```text
lambda_i[j] += rho * (admm_u[j] - admm_u_avg[j])
```

Then it increments the local iteration:

```text
admm_iter++
```

Residuals:

- Primal residual is `sqrt(sum_j (admm_u[j] - admm_u_avg[j])^2)`.
- Dual residual is `rho * sqrt(sum_j (admm_u_avg[j] - admm_u_avg_prev[j])^2)`.
- `admm_u_avg_prev` is updated to the latest average each iteration.

Stopping:

- Current code stops when `admm_iter >= ADMM_MAXITER`.
- Current `ADMM_MAXITER` is 50.
- The code comment says residuals are recorded for plotting only.
- There is no current soft stop at 100 or hard stop at 150 in the code.

Continuation:

- If not done, it promotes the next receive window.
- It sets stage back to `PRIMAL_UPDATE`.

## 26. Applying the ADMM Result

`admm_apply_result()` in `main.ino` runs when `admm_tick()` returns true.

It computes:

```text
r_local = admm_d_bg + sum_j admm_k[j] * admm_u_avg[j]
```

Then:

- If feedback is enabled, it sets `r = r_local`.
- It seeds the PI integrator with `admm_u_avg[LUMINAIRE]`.
- It sets `flicker_holdoff = FLICKER_EXCLUDE_SAMPLES`.
- It prints the local optimal duty and local predicted reference.
- If feedback is off, it only prints the result and does not change the PI reference.

Important oral correction:

- The current code uses full coupling in the ADMM-to-PI reference conversion.
- Do not say the current code uses only `d_i + k_ii u_i^*`.
- Full coupling is consistent with the ADMM constraint `d_i + k_i^T u >= L_i`.

Why seed the integrator:

- ADMM computes a duty target for the local actuator.
- If the PI integrator starts far away, the duty can jump or slowly wind up.
- Seeding it with `u_bar_i` gives a bumpless transfer into closed-loop regulation.

## 27. Correctness of Not Broadcasting Consensus

If asked: "Is it correct not to share the consensus between nodes?"

Answer:

- Yes, if all nodes receive the same `x_i(k)` vectors for the same iteration.
- The consensus average is a deterministic reduction of the broadcast primal vectors.
- Sending `u_bar` separately would duplicate information and add a coordinator or extra broadcast phase.
- The dual variables are also not broadcast because `sum_i lambda_i = 0` is invariant when initialized at zero and updated consistently.

Equivalent centralized view:

```text
central node collects all x_i
central node computes u_bar = average(x_i)
central node broadcasts u_bar
```

Distributed view used here:

```text
every node receives all x_i
every node computes the same u_bar locally
```

Caveat:

- If a timeout occurs and nodes average different data, the implementation becomes an approximate/asynchronous variant.
- The iteration tag and lookahead buffer are there to reduce the risk of using inconsistent iteration data.

## 28. Why `recv_next` Instead of a Strict Barrier

If asked: "Why use `admm_recv_next` instead of just a barrier?"

Answer:

- A strict barrier is simpler conceptually but couples progress to the slowest or most delayed node.
- The PI loop must keep running, so ADMM cannot block the whole main loop waiting for CAN.
- The current design is a non-blocking local barrier with a one-step lookahead.
- It prevents mixing `k` and `k+1` vectors without discarding early packets.

Example:

```text
Node 1 receives all x(k), computes u_bar(k), updates lambda, broadcasts x_1(k+1).
Node 2 is still waiting or processing iteration k.
Node 2 receives x_1(k+1) before it advances.
Node 2 stores it in admm_recv_next instead of averaging it into k.
After Node 2 finishes k, it promotes the next window.
```

What is gained:

- Fewer avoidable packet drops.
- Better tolerance of CAN jitter and processing delays.
- No central coordinator.
- No blocking global barrier in the 100 Hz controller loop.
- Iteration consistency is preserved because packets carry iteration IDs.

What is lost:

- More implementation complexity.
- More state to explain.
- With timeouts, the algorithm is not perfectly synchronous textbook ADMM.

If asked how hard a barrier would be:

- Simple version: drop all future packets and reset one current buffer each iteration. Low effort but less robust.
- True non-blocking barrier: add ready/ack messages, a barrier state, ready counters, timeouts, and possibly session IDs. Medium effort.
- Blocking barrier: simple but not recommended because it can stall control.

## 29. Complexity

Per-node computation:

- ADMM primal update is effectively `O(N)` here because it evaluates a fixed number of candidates and each candidate uses vector loops of length `N`.
- Consensus averaging is `O(N^2)` in general because it averages an `N` by `N` matrix of local vectors.
- For `N = 3`, all of this is tiny on the RP2040.

Communication:

- General all-to-all vector sharing sends `N` vectors of length `N` per iteration, so scalar communication is `O(N^2)`.
- This implementation packs one full 3-component vector into one CAN frame, so each node sends one ADMM frame per iteration.
- Network-wide, there are `N` ADMM frames per iteration for `N = 3`.

Memory:

- Main ADMM vectors are `O(N)`.
- Receive windows are `O(N^2)`.
- The next receive window doubles the receive-buffer memory but still only tiny arrays for `N = 3`.

Timing:

- Worst-case peer wait is bounded by `ADMM_TIMEOUT = 150` ms per iteration.
- With `ADMM_MAXITER = 50`, the extreme timeout-only upper bound is 7.5 s.
- Normal operation should be faster because iterations advance when all peer vectors arrive.

## 30. Current Code vs Report Statements To Be Careful About

Use these during oral evaluation if challenged on exact implementation.

- Current code uses `ADMM_RHO = 5.0`, not `0.05`.
- Current code uses `ADMM_MAXITER = 50`, not 100/150.
- Current code has no `ADMM_MAXITER_HARD`.
- Current code records primal and dual residuals but does not stop on residual thresholds.
- Current code does not parse `T <n>` to change ADMM iterations.
- Current ADMM result conversion uses full coupling `d_i + sum_j k_ij u_bar_j`, not only local contribution `d_i + k_ii u_i`.
- `MSG_ADMM` fixed-point range is about `[-16, 15.9995]` in ADMM vector units, not a lux range.
- `admm_recv_count` counts received vector components, not CAN frames.
- `admm_recv_next` is a transport synchronization mechanism, not an ADMM mathematical variable.
- Visibility metric uses the local duty-cycle estimate `sys_background + sys_gain * duty`, not the measured lux and not the full coupling model.
- `can_queue_tx()` uses a blocking queue add, so the design avoids normal SPI/CAN blocking but could still block if the software TX queue fills.
- `resetMetrics()` is implemented but not currently wired to a serial command in the visible command parser.

## 31. Likely Oral Questions and Answers

Question: Why ADMM instead of independent PI controllers?

Answer:

- Independent PI controllers only react to local lux and local duty.
- They do not optimize network energy cost.
- ADMM uses the coupling matrix, so a cheap node can intentionally help illuminate a neighbor and reduce total weighted energy.

Question: Why does linear cost cause difficulty?

Answer:

- A purely linear local objective with constraints can have non-unique minimizers or corner solutions.
- ADMM adds a quadratic penalty around the consensus variable.
- This makes the local update a strongly convex QP with a unique minimizer.

Question: What exactly is `x_i`?

Answer:

- `x_i` is node `i`'s local copy/proposal of the whole network duty vector.
- It includes predictions for all nodes, not just its own duty.
- Consensus forces all nodes' copies to agree.

Question: Why is only `u_ii` bounded in each local feasibility check?

Answer:

- In node `i`'s local subproblem, only the local actuator is physically controlled by that node.
- Other components are predictions of other nodes' decisions.
- Those other components are bounded in their owner nodes' local subproblems and aligned through consensus.

Question: Why is the consensus average clipped to `[0, 1]`?

Answer:

- Duty cycles are physical normalized commands.
- Clipping prevents numerical or communication artifacts from generating invalid duty references.
- It is a pragmatic safety measure.

Question: Why does the node not broadcast `lambda`?

Answer:

- With zero-initialized duals and the consensus update, the sum of dual variables remains zero.
- The dual term cancels out of the consensus average.
- Broadcasting `lambda` would consume bandwidth without changing the update.

Question: Can one node be one iteration ahead?

Answer:

- Yes, because each node computes its own local copy of the consensus average and the state machines are non-blocking.
- A node that receives all current vectors quickly can advance before another node finishes processing.
- The `admm_recv_next` buffer handles this by storing one-iteration-early packets.

Question: Why not implement a strict global barrier?

Answer:

- A strict barrier is simpler to reason about but can block progress on CAN jitter or packet loss.
- The controller must preserve the 100 Hz PI loop.
- The current implementation keeps ADMM non-blocking and uses iteration tags to avoid mixing iterations.

Question: What happens if a packet is lost?

Answer:

- `WAIT_PEERS` times out after 150 ms.
- The code logs which peer is missing components.
- It proceeds to `DUAL_UPDATE` using whatever is in the receive buffer, which was seeded or promoted from previous consensus estimates.
- This prevents deadlock at the cost of exact synchronous ADMM behavior for that iteration.

Question: What is the biggest implementation risk?

Answer:

- Iteration consistency over asynchronous CAN.
- If vectors from different iterations are averaged together, the ADMM update is wrong.
- The iteration byte and lookahead buffer address this.

Question: What is the biggest modeling risk?

Answer:

- The coupling model is static and depends on calibration.
- If ambient light or physical layout changes, `k_ij` and `d_i` may become inaccurate.
- PI feedback corrects local tracking, but the optimality of ADMM depends on calibration quality.

Question: What is the role of calibration?

Answer:

- It measures the full coupling matrix and background lux.
- Without `k_ij`, the optimizer cannot know how one LED affects other desks.
- Each node only needs its row of the matrix for its local desk constraint, but the distributed process builds the full matrix across nodes.

Question: Why seed non-own ADMM components to 0.5?

Answer:

- If an unoccupied node seeded all components to its own feedforward value, it could broadcast zeros for occupied neighbors.
- That would drag consensus down.
- `0.5` is a neutral guess that lets each owner correct its own component.

Question: Why set the final reference to full predicted lux?

Answer:

- ADMM computes a coordinated vector whose combined illumination should meet the desk lower bound.
- The PI controller controls local measured lux, so its reference should match the full predicted local illuminance from all LEDs.
- The integrator is seeded with only the local duty component because this node only drives its own LED.

Question: What is the CAN ID format?

Answer:

- 11-bit standard CAN ID.
- Top 3 bits are message type.
- Middle 4 bits are destination.
- Bottom 4 bits are source.
- Destination 0 means broadcast.

Question: Why pack ADMM into fixed point?

Answer:

- A CAN payload is limited to 8 bytes.
- Three 32-bit floats plus an iteration byte would not fit.
- Three int16 fixed-point values plus one iteration byte fit in 7 bytes.
- The scale 2048 gives sub-0.001 resolution, enough for duty-like values.

Question: How do you avoid CAN TX drops?

Answer:

- Application code queues frames in `can_tx_queue`.
- Core 1 maintains `tx_pending` so if the MCP2515 hardware TX buffers are busy, it retries instead of dropping immediately.
- Only non-busy hardware errors lead to a logged drop.

Question: Why hardware filtering?

Answer:

- It reduces software load by accepting only frames addressed to this node or broadcast frames.
- The mask checks destination bits `[7:4]`.

Question: What does `g K` and `g J` return?

Answer:

- `g K` returns current ADMM primal residual.
- `g J` returns current ADMM dual residual.
- These are diagnostics for convergence plotting, not current stopping criteria.

Question: What if the target is unreachable?

Answer:

- `admm_init()` clamps occupied targets to the physically achievable range `d_bg` to `d_bg + sum_j k_j`.
- If the solver still fails to find a feasible candidate, it falls back to saturating own duty at 1.0.

Question: Why is occupancy 0 handled specially?

Answer:

- For occupancy 0, `admm_L = 0`.
- This allows the optimizer to turn off or reduce output when the desk is unoccupied, unless it is useful for other desks through consensus.

Question: Does the code currently support more than 3 nodes?

Answer:

- The ADMM math is written in `N`-style notation, but the current implementation is fixed at `ADMM_N = 3`.
- The ADMM CAN frame packs exactly three components.
- Extending to more nodes would require changing the wire format and array sizes.

## 32. Detailed File Walkthrough

Use this section if you need to walk through "every block of code."

### `main.ino`

Include block:

- Includes controller, buffer, lux, metrics, calibration, CAN helpers, ADMM, flash ID access, and Pico queues.

Hardware constants:

- Defines LED pin, LDR pin, ADC resolution, PWM resolution, PWM range, and PWM frequency.
- Defines voltage divider constants and max LED power.

Board identity block:

- Defines `N_NODES`, UID table, LDR calibration tables, `ldr_m`, `ldr_b`, and `LUMINAIRE`.
- `LUMINAIRE` starts at 0 and is assigned during setup.

CAN globals:

- Instantiates `MCP2515 can0(spi0, 17, 19, 16, 18, 10000000)`.
- Defines RX, TX, and core-start queues.
- Defines CAN interrupt pin and interrupt flag.
- `can_irq_handler()` only sets a boolean flag.

Network topology block:

- Holds `other_nodes[]` and `n_other_nodes`.

Gain matrix block:

- Holds the distributed calibration results and calibration status.

Phase 2 parameter block:

- Holds `ref_high`, `ref_low`, and `energy_cost`.

Sampling block:

- Holds sampling period, last sample time, jitter, and jitter stream flag.

PID/control state block:

- Holds the global `PID` object, reference `r`, current lux, current duty, open-loop duty, stream flags, buffer flags, and circular buffer.

Filter block:

- Holds median filter window and filter mode.

Calibration state block:

- Defines `CalStage` enum and all state variables used by distributed calibration.

Forward declarations:

- Declares functions implemented later or in other `.ino` files.

`cal_build_node_list()`:

- Builds a sorted list of all node IDs so every node runs calibration rounds in the same order.

`admm_apply_result()`:

- Converts ADMM consensus vector into local predicted lux reference using full coupling.
- Updates PI reference and integrator if feedback is enabled.

`cal_advance_round()`:

- Moves calibration to the next active LED.
- If all rounds are done, finalizes calibration and starts ADMM.
- If this node is active, turns LED on and broadcasts `CAL_ON`.
- If passive, records which node to wait for.

`network_wakeup()`:

- Broadcasts wakeup messages until both peers are found.
- Records unique peer source IDs.
- Replies to peer `SYN` messages with `SYN_ACK`.

`calibration_loop()`:

- Non-blocking distributed calibration state machine.
- Handles barrier, background, active measurement, passive measurement, timeouts, and completion.

`setup1()`:

- Core 1 setup: waits for Core 0, resets CAN controller, sets bitrate, applies hardware filters, enters normal mode, enables interrupt.

`loop1()`:

- Core 1 runtime: drains TX queue, retries if all hardware TX buffers are busy, reads received frames on interrupt, queues frames to Core 0, reports CAN errors as sentinel frames.

`process_can_messages()`:

- Core 0 CAN dispatch loop.
- Handles sentinel errors, message type decoding, remote set/get, replies, calibration, ADMM packets, and control messages.

`hub_forward()`:

- Converts local serial commands intended for remote nodes into CAN frames.
- Forwarded set/stream commands print `ack:forwarded` immediately when queued locally.
- A later `MSG_CTRL/SUB_ACK` from the destination node is printed as `ack:remote`.

`serial_command()`:

- Reads bytes from `Serial` into a 128-byte line buffer.
- Calls `commands()` when newline or carriage return is seen.
- Resets buffer on overflow.

`handle_buffer_readout()`:

- Prints last-minute buffer values one at a time to avoid blocking output bursts.

`setup()`:

- Initializes serial, queues, ADC, PWM, LED output, and board identity.
- Starts Core 1 by sending `go` through `core1_ready_queue`.
- Runs network wakeup.
- Arms calibration if full network was found.
- Initializes PID and sample time.

`loop()`:

- Runs 100 Hz sample/control work.
- Handles serial commands, buffer readout, ADMM tick/result, CAN dispatch, and calibration tick.

### `admm.h`

Comment block:

- Documents variable notation and the consensus ADMM steps.

Constants:

- Defines `ADMM_N = 3`.
- Defines `ADMM_RHO = 5.0`.
- Declares `ADMM_MAXITER`.
- Defines `ADMM_TIMEOUT = 150` ms.

Extern state:

- Declares all ADMM state arrays and residuals.
- Exposes receive buffer/count only where needed for diagnostics.

State enum:

- `AdmmStage` defines the non-blocking state machine stages.

API:

- `admm_init()`, `admm_start()`, `admm_tick()`, `admm_result()`, `admm_request()`, and `admm_receive()`.

### `admm.cpp`

Include/external block:

- Includes ADMM header, calibration feedforward, and PID.
- Declares external state from `main.ino`.

State block:

- Defines all ADMM arrays, receive windows, residuals, and flags.

Receive helper block:

- Seeds windows, stores components with duplicate protection, and promotes next-window packets.

Initialization block:

- `admm_init()` sets gain row, cost vector, lambda, background, lower bound, denominator terms, warm start, and receive window seed.

Constraint helper block:

- `feasible()` checks own actuator bound and local illuminance lower bound.
- `predicted_lux()` computes local model lux.
- `qp_cost()` ranks QP candidates.

Run control block:

- `admm_start()` initializes iteration and stage.
- `admm_result()` returns the local component of consensus.
- `admm_receive()` classifies packets by iteration.
- `admm_request()` starts a run and optionally broadcasts a trigger.

Tick block:

- `admm_tick()` implements state machine logic for primal update, waiting, dual update, stopping, and loop continuation.

### `can_comms.h`

ID layout block:

- Defines the 11-bit standard CAN ID format and helper macros.

Message constants:

- Defines message types and subtypes.

Global declarations:

- Exposes `can0`, `LUMINAIRE`, and `can_tx_queue`.

Frame builders:

- Queue helper plus float, byte, subtype-only, and ADMM frame senders.

ADMM wire conversion:

- Encodes/decodes fixed-point signed values with scale 2048.

### `can.h`

CAN frame block:

- Defines `CAN_MAX_DLEN = 8`.
- Defines `can_frame` with ID, DLC, padding, and 8-byte payload.

CAN flag/mask block:

- Defines extended, RTR, and error flags.
- Defines standard, extended, and error ID masks.

### `commands.ino`

External block:

- Declares phase 2 globals, ADMM residuals, ADMM request function, and calibration state externs.

Print helper block:

- Defines `_pbuf[128]` and `PRINTF()` macro.

Command function:

- Parses command character.
- Applies hub forwarding if target node is not local.
- Switches on command and executes local behavior.

Control command blocks:

- Handles identity, ADMM trigger, duty, reference, occupancy, anti-windup, feedback, streaming, and phase 2 parameters.

Calibration/tuning command block:

- Handles distributed calibration trigger, local LDR coefficients, local blocking calibration/sweep, status, PID parameters, and filter mode.

Get command block:

- Reports measured values, states, parameters, metrics, gain row, buffer data, and ADMM residuals.

Help/default block:

- Prints command reference or unknown-command error.

### `calibration.h`

External block:

- Declares calibration-related globals from `main.ino`.

`calibrate_system_gain()`:

- Blocking local gain calibration using off, 25%, 50%, 75%, and 100% duty.

`sweep()`:

- Blocking duty sweep printing duty, voltage, resistance, and lux.

`feedforward()`:

- Computes static inverse-model duty from reference lux.

`apply_feedforward()`:

- Seeds the PI integrator with the feedforward duty.

### `lux.h`

External/constants block:

- Declares LDR calibration and hardware constants.
- Defines mean and median filter parameters.

Median helpers:

- `median_push()` shifts window and inserts an ADC sample.
- `median_read()` sorts a copy and returns the median.

Voltage read block:

- Mean read, median read, and selectable `readVoltage()`.

Conversion block:

- Voltage to resistance, resistance to lux, and final `measureLux()`.

### `pid.h`

Constants:

- Defines anti-windup mode IDs.

Class state:

- Holds tuning constants, controller terms, flags, occupancy, and previous parameters.

`init()`:

- Loads per-node gains, initializes controller state, and sets defaults.

`compute_control()`:

- Implements setpoint-weighted PI, saturation, bumpless parameter transfer, and anti-windup.

Setters/getters:

- Expose tuning, flags, duty, occupancy, and integrator state.

### `metrics.h`

State block:

- Holds cumulative energy, visibility, flicker, previous duties, holdoff, and sample count.

`updateMetrics()`:

- Accumulates energy, visibility shortfall, and flicker direction-change metric.

`resetMetrics()`:

- Clears all metric state.

### `circular_buffer.h`

Data struct:

- Stores one lux/duty sample.

Template class:

- Implements capacity, size, empty/full, push, and pop.

Behavior:

- Push overwrites oldest when full.
- Pop returns oldest sample and reduces count.

### `mcp2515.h`

Configuration constants:

- Defines bit-rate register settings for 8 MHz, 16 MHz, and 20 MHz MCP2515 clocks.

Enums:

- Defines CAN clock, speed, CLKOUT, error codes, masks, filters, RX/TX buffers, interrupt flags, error flags, register addresses, and SPI instructions.

Class interface:

- Declares constructor, reset, modes, bitrate, filters, send/read, receive/error checks, interrupt clearing, and error counters.

Private helpers:

- Declares SPI chip-select helpers, mode setting, register access, bit modification, and ID packing.

### `mcp2515.cpp`

Register table block:

- Defines TX buffer and RX buffer register mappings.

Constructor/SPI block:

- Initializes SPI, configures pins, sets format, and configures chip select.

Reset/register block:

- Implements reset and register read/write/modify primitives.

Mode/bitrate block:

- Implements mode switching and bit-rate configuration.

Filter block:

- Packs IDs and writes filter/mask registers.

Send block:

- Sends using a selected TX buffer or scans for a free buffer.

Receive block:

- Reads from selected RX buffer or whichever buffer has a message.

Status/error block:

- Checks receive status, checks error flags, gets/clears interrupts, clears overflow/error flags, and reads RX/TX error counters.

## 33. If You Need To Explain End-to-End Flow

Boot:

- Board starts, initializes queues and hardware.
- Reads flash UID and maps it to `LUMINAIRE`.
- Starts Core 1 CAN runtime.
- Discovers peers over CAN wakeup.
- Starts distributed calibration if all peers are found.
- Initializes PID.

Calibration:

- All nodes synchronize in a ready barrier.
- All measure background.
- Each LED becomes active in sorted node order.
- Other nodes measure the active LED's effect and send lux readings.
- Each node builds its local coupling row.
- Calibration completes and starts ADMM.

ADMM:

- Each node initializes its local constraint and cost.
- Each node solves a local QP for `x_i`.
- Each node broadcasts full `x_i`.
- Each node waits for peer vectors or timeout.
- Each node averages received vectors into `u_bar`.
- Each node updates its local dual vector.
- Iteration repeats for 50 iterations.
- On completion, each node computes its local predicted lux from the final consensus vector.

Control:

- PI reference becomes the ADMM predicted local lux.
- PI integrator is seeded with the local optimized duty.
- 100 Hz PI loop tracks measured lux using local LED duty.
- Metrics are collected continuously.

## 34. Short Defense Statements

Use these if you need concise answers.

- "The consensus variable is not broadcast because it is a deterministic average of the broadcast primal vectors."
- "The receive lookahead is not part of ADMM theory; it is a transport guard against asynchronous CAN timing."
- "The dual variables are local and do not need to be broadcast because their sum remains zero under the standard consensus ADMM update."
- "The ADMM solver is non-blocking so the 100 Hz PI loop continues to run."
- "The local QP is solved analytically by checking the unconstrained minimizer and a finite set of active-constraint boundary candidates."
- "The final PI reference uses full coupling because desk lux depends on all LEDs, not just the local LED."
- "The integrator is seeded with the optimized local duty for bumpless transfer."
- "The system uses fixed-point ADMM frames because three floats plus an iteration byte do not fit in one CAN payload."
- "Timeouts prevent deadlock but make the implementation a practical asynchronous approximation if messages are missing."
- "The biggest difference between the report and current code is that current stopping is fixed-budget at 50 iterations and current `rho` is 5.0."
