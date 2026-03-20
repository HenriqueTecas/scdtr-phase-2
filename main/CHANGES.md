# Firmware Changes — Three-Way Handshake & Distributed Calibration

## Overview

This update replaces the original `can_discovery()` boot routine and per-node single-gain calibration with:

1. **Three-way SYN/SYN-ACK/ACK_DONE handshake** for robust peer discovery
2. **Distributed calibration** that measures all self- and cross-coupling gains
3. **Heap-allocated FIFO-pointer inter-core messaging** (eliminates the shared single-frame buffer bottleneck)

---

## Files Changed

### `src/can_comms.h`

Added 6 new message type defines:

| Define | Value | Purpose |
|--------|-------|---------|
| `MSG_SYN` | `0x10` | "I am here" — broadcast on boot |
| `MSG_SYN_ACK` | `0x11` | "I see you, here I am" |
| `MSG_ACK_DONE` | `0x12` | "I see you back, handshake complete" |
| `MSG_CAL_ON` | `0x30` | "I am turning my LED on, measure me" |
| `MSG_CAL_LUX` | `0x31` | "Here is my LUX reading while you were on" |
| `MSG_CAL_DONE` | `0x32` | "I am done, next node go" |

> Note: `MSG_SYN (0x10)` and `MSG_SYN_ACK (0x11)` reuse the same values as the old `MSG_HELLO`/`MSG_READY` aliases, which are kept as dead defines for backward compatibility.

---

### `src/main.ino`

#### 1. Globals restructured

**Removed:**
- `volatile bool can_msg_ready` — single-frame flag, replaced by FIFO
- `struct can_frame canMsgRx` — single shared buffer, dropped frames under bursts
- `mutex_t can_mutex` — no longer needed

**Added:**
- `#define N_NODES 3` — total number of nodes in the network
- `const uint8_t UID_TABLE[N_NODES]` — maps flash UID last byte to LUMINAIRE index
- `mutex_t can_rx_mutex` — reserved for future use
- `uint8_t other_nodes[N_NODES - 1]` — raw addresses of confirmed peers
- `int n_other_nodes` — peer count accumulated during handshake
- `bool network_ready` — true once all N_NODES-1 peers have completed handshake

**Added (handshake state):**
- `bool peer_syn_received[256]` — guards duplicate SYN-ACK replies
- `bool peer_syn_ack_received[256]` — guards duplicate ACK_DONE replies
- `bool peer_ack_done_received[256]` — guards duplicate peer registrations

**Added (calibration state):**
- `float coupling_gains[N_NODES+1][N_NODES+1]` — 1-indexed; `[i][j]` = LUX at node `i` when node `j` is at full duty
- `float background_lux` — ambient illuminance measured with all LEDs off
- `bool calibration_done`
- `int cal_lux_count`

---

#### 2. `setup1()` — CAN init on Core 1

**Before:** Immediately read flash UID, call `mutex_init(&can_mutex)`, then acquire `spi_mutex` (race condition: Core 1 could grab `spi_mutex` before Core 0 called `mutex_init(&spi_mutex)`).

**After:**
```cpp
while (!mutex_is_initialized(&spi_mutex)) tight_loop_contents();
```
Core 1 now spins until Core 0 has initialised `spi_mutex` in `setup()`. CAN filters are set wide-open (mask = `0x000`) during boot so all handshake messages pass through regardless of destination.

---

#### 3. `loop1()` — CAN RX on Core 1

**Before:** Wrote received frame into a single shared `canMsgRx` struct behind `can_mutex`. Any new frame arriving before Core 0 read the previous one was silently dropped.

**After:** Heap-allocates a `can_frame`, copies the received frame, pushes the pointer through the hardware FIFO:
```cpp
struct can_frame *p = new can_frame;
*p = frm;
rp2040.fifo.push_nb((uint32_t)p);
```
Core 0 pops the pointer, processes the frame, and `delete`s it. This is a proper queue — no frames are dropped under bursts.

---

#### 4. `can_discovery()` removed → `network_wakeup()` added

**Old approach:** Broadcast full 8-byte UID, wait 2 s, sort UIDs lexicographically, assign rank. Problems: no confirmation that peers actually received our UID; a late-booting node missed already-sorted assignments.

**New three-way handshake:**

```
Node A          Node B          Node C
  |──SYN──────────►|              |
  |──SYN────────────────────────►|
  |◄──SYN──────────|              |  (B also broadcasts SYN)
  |◄──SYN──────────────────────── |  (C also broadcasts SYN)
  |──SYN-ACK──────►|              |
  |──SYN-ACK────────────────────►|
  |◄──SYN-ACK──────|              |
  |◄──SYN-ACK──────────────────── |
  |──ACK-DONE─────►|              |
  |──ACK-DONE───────────────────►|
  |◄──ACK-DONE─────|              |
  |◄──ACK-DONE───────────────────|
```

- SYN messages are broadcast; SYN-ACK and ACK-DONE are unicast to the originating node.
- Guard flags (`peer_syn_received[]` etc.) prevent duplicate replies.
- Works even when nodes boot at different times: late-booting nodes still trigger SYN-ACK from already-waiting nodes.
- 5 s timeout prevents deadlock if a node is missing.
- After handshake, CAN filters are tightened to accept only BROADCAST and own `node_address`.

---

#### 5. `distributed_calibration()` added

**Old approach:** Each node ran `calibrate_system_gain()` independently — only the self-gain of that node was measured. Cross-coupling gains were unknown.

**New distributed approach:**

1. **Background round:** All LEDs off (broadcast `MSG_CAL_ON` with `data[1]=0x00`), 3 s settle, measure ambient.
2. **Active rounds (N_NODES rounds):** Every node independently builds an identical sorted list of all node addresses (`other_nodes[] + node_address`, sorted ascending). The node at position `round` in this list is the active node for that round.
   - **Active node:** Turns LED to full duty, broadcasts `MSG_CAL_ON` with `data[1]=node_address`, waits 3 s, records self-gain, collects `MSG_CAL_LUX` replies from peers, turns off, broadcasts `MSG_CAL_DONE`.
   - **Passive nodes:** Wait for `MSG_CAL_ON` from the active node, wait 3 s (same settle delay), measure LUX, send `MSG_CAL_LUX` unicast to active node, wait for `MSG_CAL_DONE`.
3. Results stored in `coupling_gains[receiver_idx][sender_idx]`.
4. `sys_gain` is set to the self-gain `coupling_gains[LUMINAIRE][LUMINAIRE]` for PID feedforward.

**Why the round order is deterministic without a master:** Because all nodes ran the same handshake and discovered the same peers, sorting `other_nodes[] + self` by raw address gives the same sequence on every node — no negotiation needed.

---

#### 6. `setup()` updated

New boot sequence:

1. `mutex_init(&spi_mutex)` and `mutex_init(&can_rx_mutex)` — **before** Core 1 can touch SPI
2. ADC/PWM/LED init
3. Read flash UID on Core 0 (with `idleOtherCore`), look up `node_address` in `UID_TABLE`, assign `LUMINAIRE`; halt with error message if UID not in table
4. `delay(node_address % 5 * 100)` — deterministic per-board stagger (0–400 ms) to prevent all nodes broadcasting SYN simultaneously and causing a CAN collision
5. `network_wakeup()`
6. `distributed_calibration()` if `network_ready`, else warn and skip
7. `pid.init(LUMINAIRE)`

---

#### 7. `process_can_messages()` updated

**Before:** Checked `can_msg_ready` flag, copied `canMsgRx` behind `can_mutex`.

**After:** Drains the FIFO with `rp2040.fifo.pop_nb()` in a loop, processes each heap-allocated frame, and `delete`s it:

```cpp
uint32_t raw;
while (rp2040.fifo.pop_nb(&raw)) {
    struct can_frame *p = (struct can_frame *)raw;
    // ... process p ...
    delete p;
}
```

---

## Configuring `UID_TABLE` for New Boards

1. Flash any board with this firmware. If the UID is not in `UID_TABLE`, the node prints:
   ```
   ERROR: unknown board 0xXX — add to UID_TABLE
   ```
2. Note the hex value `0xXX` (or its decimal equivalent).
3. Add the decimal value to `UID_TABLE` in `main.ino`:
   ```cpp
   const uint8_t UID_TABLE[N_NODES] = { 33, 38, 39 };  // add new values here
   ```
4. Update `N_NODES` if adding a board to the network.
5. Re-flash all nodes (all must agree on `UID_TABLE` and `N_NODES`).

> The values in `UID_TABLE` were chosen such that their upper nibble encodes the LUMINAIRE index (1–3) when shifted into the CAN ID field — this is why `node_address` is used directly in `MAKE_CAN_ID()` for directed messages.

---

## Verification

| Test | Expected output |
|------|----------------|
| Single board boot (UID in table) | `[BOOT] node_address=0xXX LUMINAIRE=N` |
| Three boards, all powered on within ~10 s | Each prints `[WAKEUP] READY — found 2/2 peers` |
| Calibration completes | Each prints `[CAL] Calibration complete` with nonzero `sys_gain` |
| Serial command `g y 1` | Returns current lux of node 1 |
| Unknown board | Prints `ERROR: unknown board 0xXX — add to UID_TABLE` and halts |
