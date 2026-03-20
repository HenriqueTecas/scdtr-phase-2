#include <cstdio>
#include "pid.h"
#include "circular_buffer.h"
#include "lux.h"
#include "metrics.h"
#include "calibration.h"
#include "can_comms.h"
#include "hardware/flash.h"
#include "pico/mutex.h"

// ─── Pin & hardware constants ─────────────────────────────────────────────────
const int LED_PIN = 15;
const int LDR_PIN = A0;

const int ADC_BITS = 12;
const int ADC_MAX  = (1 << ADC_BITS) - 1;  // 4095
const int PWM_BITS = 12;
const int PWM_MAX  = (1 << PWM_BITS) - 1;  // 4095
const int PWM_FREQ_HZ = 30000;

// ─── Physical constants ────────────────────────────────────────────────────────
const float VCC           = 3.3f;
const float R_FIXED       = 10000.0f;
const float MAXIMUM_POWER = 0.0132f;

// ─── Board identity ──────────────────────────────────────────────────────────
// Add your boards here after reading pico_flash_id[7] on first boot.
// These are decimal values of the last byte of each board's flash UID.
#define N_NODES 3
const uint8_t UID_TABLE[N_NODES] = { 33, 39, 54 };  // DISCOVERY MODE — replace after reading serial output

uint8_t node_address;   // raw last byte of flash UID
int     LUMINAIRE = 0;  // clean index 1/2/3 assigned at boot from UID_TABLE

// ─── CAN-BUS globals ────────────────────────────────────────────────────────
MCP2515 can0(spi0, 17, 19, 16, 18, 10000000);
mutex_t spi_mutex;
mutex_t can_rx_mutex;

// inter-core FIFO carries pointers to heap-allocated can_frame structs
// Core 1 allocates, Core 0 reads and frees

// ─── Network topology ───────────────────────────────────────────────────────
// Populated during three-way handshake
uint8_t other_nodes[N_NODES - 1];   // raw addresses of peers
int     n_other_nodes = 0;          // how many peers confirmed so far
bool    network_ready = false;      // set true when all N_NODES-1 peers confirmed

// Remote state cache
float remote_lux[4]  = {0};
float remote_duty[4] = {0};

// ─── LDR calibration ──────────────────────────────────────────────────────────
float ldr_m = -0.8f;
float ldr_b = 6.3044f;

// ─── System gain calibration ──────────────────────────────────────────────────
float sys_gain       = 0.0f;
float sys_background = 0.0f;
bool  calibrated     = false;

// ─── Phase 2: Table 3 settable parameters ─────────────────────────────────────
float ref_high    = 30.0f;
float ref_low     = 20.0f;
float energy_cost =  1.0f;

// ─── Sampling ─────────────────────────────────────────────────────────────────
const unsigned long SAMPLE_PERIOD_US = 10000;   // 100 Hz
unsigned long last_sample_us         = 0;

float jitter_us = 0.0f;
int   stream_j  = 0;

// ─── PID & state ──────────────────────────────────────────────────────────────
PID   pid;
float r                 = 0.0f;
float lux_value         = 0.0f;
float duty_cycle        = 0.0f;
float serial_duty_cycle = 0.0f;

int stream_y = 0, stream_u = 0;
int buffer_y = 0, buffer_u = 0, buffer_read_size = 0, buffer_read_counter = 0;

CircularBuffer<6000> last_min_buf;

// ─── Forward declarations ─────────────────────────────────────────────────────
void commands(char *buf, Print &out);
void hub_forward(const char* cmd_str, uint8_t dest_node);
void network_wakeup();
void distributed_calibration();

// ─── Wake-up handshake state ─────────────────────────────────────────────────
// Tracks which peers have completed the full three-way handshake with us
bool peer_syn_received[256]      = {false};  // indexed by raw node_address
bool peer_syn_ack_received[256]  = {false};
bool peer_ack_done_received[256] = {false};

// ─── Calibration state ───────────────────────────────────────────────────────
// coupling_gains[i][j] = LUX measured at node i when node j is at full duty
// 1-indexed: coupling_gains[LUMINAIRE][LUMINAIRE] = self-gain
float coupling_gains[N_NODES + 1][N_NODES + 1] = {0};
float background_lux   = 0.0f;
bool  calibration_done = false;
int   cal_lux_count    = 0;

// =============================================================================
// Wake-up — Three-way handshake (SYN / SYN-ACK / ACK-DONE)
// =============================================================================
void network_wakeup() {
    Serial.printf("[WAKEUP] Node %d (addr 0x%02X) starting handshake\n",
                  LUMINAIRE, node_address);

    // Step 1 — broadcast SYN: "I am here"
    struct can_frame syn;
    syn.can_id  = MAKE_CAN_ID(BROADCAST, MSG_SYN);
    syn.can_dlc = 1;
    syn.data[0] = node_address;
    mutex_enter_blocking(&spi_mutex);
    can0.sendMessage(&syn);
    mutex_exit(&spi_mutex);

    unsigned long start        = millis();
    unsigned long last_syn_ms  = millis();

    // Wait until all N_NODES-1 peers complete handshake or timeout (5s)
    while (n_other_nodes < N_NODES - 1 && millis() - start < 5000) {

        // Re-broadcast SYN every 500ms so late-booting nodes can catch it.
        // Without this, a node that boots after the initial SYN is sent will
        // never receive it, so it can never send SYN_ACK back, so the early
        // node never sends ACK_DONE, so the late node never adds the early peer.
        if (millis() - last_syn_ms >= 500) {
            mutex_enter_blocking(&spi_mutex);
            can0.sendMessage(&syn);
            mutex_exit(&spi_mutex);
            last_syn_ms = millis();
        }

        uint32_t raw;
        while (rp2040.fifo.pop_nb(&raw)) {
            struct can_frame *p = (struct can_frame *)raw;
            uint8_t type = CAN_ID_TYPE(p->can_id);
            uint8_t src  = p->data[0];

            if (type == MSG_SYN) {
                // Step 2 — received SYN from peer, reply with SYN-ACK
                if (!peer_syn_received[src]) {
                    peer_syn_received[src] = true;
                    Serial.printf("[WAKEUP] SYN from 0x%02X, sending SYN-ACK\n", src);
                    struct can_frame syn_ack;
                    syn_ack.can_id  = MAKE_CAN_ID(src, MSG_SYN_ACK);
                    syn_ack.can_dlc = 1;
                    syn_ack.data[0] = node_address;
                    mutex_enter_blocking(&spi_mutex);
                    can0.sendMessage(&syn_ack);
                    mutex_exit(&spi_mutex);
                }

            } else if (type == MSG_SYN_ACK) {
                // Step 3 — received SYN-ACK, reply with ACK_DONE
                if (!peer_syn_ack_received[src]) {
                    peer_syn_ack_received[src] = true;
                    Serial.printf("[WAKEUP] SYN-ACK from 0x%02X, sending ACK_DONE\n", src);
                    struct can_frame ack;
                    ack.can_id  = MAKE_CAN_ID(src, MSG_ACK_DONE);
                    ack.can_dlc = 1;
                    ack.data[0] = node_address;
                    mutex_enter_blocking(&spi_mutex);
                    can0.sendMessage(&ack);
                    mutex_exit(&spi_mutex);
                }

            } else if (type == MSG_ACK_DONE) {
                // Handshake complete with this peer
                if (!peer_ack_done_received[src]) {
                    peer_ack_done_received[src] = true;
                    if (n_other_nodes < N_NODES - 1) {
                        other_nodes[n_other_nodes++] = src;
                        Serial.printf("[WAKEUP] Handshake complete with 0x%02X (%d/%d)\n",
                                      src, n_other_nodes, N_NODES - 1);
                    }
                }
            }
            delete p;
        }
    }

    network_ready = (n_other_nodes == N_NODES - 1);
    Serial.printf("[WAKEUP] %s — found %d/%d peers\n",
                  network_ready ? "READY" : "TIMEOUT", n_other_nodes, N_NODES - 1);

    // Tighten CAN filters now that we know our address
    mutex_enter_blocking(&spi_mutex);
    can0.setFilterMask(MCP2515::MASK0, false, 0x700);
    can0.setFilter(MCP2515::RXF0, false, MAKE_CAN_ID(BROADCAST, 0));
    can0.setFilter(MCP2515::RXF1, false, MAKE_CAN_ID(node_address, 0));
    can0.setFilterMask(MCP2515::MASK1, false, 0x700);
    can0.setFilter(MCP2515::RXF2, false, MAKE_CAN_ID(BROADCAST, 0));
    can0.setFilter(MCP2515::RXF3, false, MAKE_CAN_ID(node_address, 0));
    mutex_exit(&spi_mutex);
}

// =============================================================================
// Distributed calibration — each node takes a turn at full duty
// =============================================================================
void distributed_calibration() {
    Serial.println("[CAL] Starting distributed calibration");
    calibration_done = false;

    // ── Round 0: all LEDs off, measure background ────────────────────────────
    analogWrite(LED_PIN, 0);
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(BROADCAST, MSG_CAL_ON);
    msg.can_dlc = 2;
    msg.data[0] = node_address;
    msg.data[1] = 0x00;  // 0x00 = background round, all off
    mutex_enter_blocking(&spi_mutex);
    can0.sendMessage(&msg);
    mutex_exit(&spi_mutex);

    delay(3000);
    background_lux = measureLux();
    Serial.printf("[CAL] Background = %.2f LUX\n", background_lux);
    sys_background = background_lux;

    // ── Build sorted list of all node addresses ──────────────────────────────
    // Every node independently builds the same list from other_nodes[] + itself
    // and sorts by raw address — giving an identical round order on all nodes.
    uint8_t all_nodes[N_NODES];
    all_nodes[0] = node_address;
    for (int i = 0; i < n_other_nodes; i++)
        all_nodes[i + 1] = other_nodes[i];
    // Insertion sort by address value
    for (int i = 1; i < N_NODES; i++) {
        uint8_t key = all_nodes[i];
        int j = i - 1;
        while (j >= 0 && all_nodes[j] > key) {
            all_nodes[j + 1] = all_nodes[j];
            j--;
        }
        all_nodes[j + 1] = key;
    }

    // ── Rounds 1..N_NODES: each node turns on in sorted order ────────────────
    for (int round = 0; round < N_NODES; round++) {
        uint8_t active_addr = all_nodes[round];
        Serial.printf("[CAL] Round %d: node 0x%02X is active\n", round + 1, active_addr);

        if (active_addr == node_address) {
            // ── I am the active node this round ─────────────────────────────
            analogWrite(LED_PIN, PWM_MAX);
            msg.can_id  = MAKE_CAN_ID(BROADCAST, MSG_CAL_ON);
            msg.can_dlc = 2;
            msg.data[0] = node_address;
            msg.data[1] = node_address;  // non-zero = this is the active node
            mutex_enter_blocking(&spi_mutex);
            can0.sendMessage(&msg);
            mutex_exit(&spi_mutex);

            delay(3000);

            // Record my own self-gain
            float my_lux = measureLux();
            int my_idx = LUMINAIRE;
            coupling_gains[my_idx][my_idx] = my_lux - background_lux;
            Serial.printf("[CAL] Self-gain[%d][%d] = %.2f\n",
                          my_idx, my_idx, coupling_gains[my_idx][my_idx]);

            // Collect CAL_LUX replies from all other nodes
            cal_lux_count = 0;
            unsigned long t = millis();
            while (cal_lux_count < n_other_nodes && millis() - t < 3000) {
                uint32_t raw;
                while (rp2040.fifo.pop_nb(&raw)) {
                    struct can_frame *p = (struct can_frame *)raw;
                    if (CAN_ID_TYPE(p->can_id) == MSG_CAL_LUX) {
                        uint8_t peer_addr = p->data[0];
                        float peer_lux;
                        memcpy(&peer_lux, p->data + 1, 4);
                        // Find peer's LUMINAIRE index from UID_TABLE
                        int peer_idx = -1;
                        for (int k = 0; k < N_NODES; k++)
                            if (UID_TABLE[k] == peer_addr) { peer_idx = k + 1; break; }
                        if (peer_idx > 0) {
                            coupling_gains[peer_idx][my_idx] = peer_lux - background_lux;
                            Serial.printf("[CAL] Cross-gain[%d][%d] = %.2f\n",
                                          peer_idx, my_idx, coupling_gains[peer_idx][my_idx]);
                        }
                        cal_lux_count++;
                    }
                    delete p;
                }
            }

            analogWrite(LED_PIN, 0);
            msg.can_id  = MAKE_CAN_ID(BROADCAST, MSG_CAL_DONE);
            msg.can_dlc = 1;
            msg.data[0] = node_address;
            mutex_enter_blocking(&spi_mutex);
            can0.sendMessage(&msg);
            mutex_exit(&spi_mutex);

        } else {
            // ── I am a passive node this round ──────────────────────────────
            analogWrite(LED_PIN, 0);

            // Wait for MSG_CAL_ON from the active node
            unsigned long t = millis();
            bool got_cal_on = false;
            while (!got_cal_on && millis() - t < 5000) {
                uint32_t raw;
                while (rp2040.fifo.pop_nb(&raw)) {
                    struct can_frame *p = (struct can_frame *)raw;
                    if (CAN_ID_TYPE(p->can_id) == MSG_CAL_ON &&
                        p->data[0] == active_addr && p->data[1] == active_addr)
                        got_cal_on = true;
                    delete p;
                }
            }
            if (!got_cal_on) {
                Serial.printf("[CAL] Timeout waiting for CAL_ON from 0x%02X\n", active_addr);
                continue;
            }

            delay(3000);

            // Measure and send CAL_LUX to active node
            float my_lux = measureLux();
            struct can_frame lux_msg;
            lux_msg.can_id  = MAKE_CAN_ID(active_addr, MSG_CAL_LUX);
            lux_msg.can_dlc = 5;
            lux_msg.data[0] = node_address;
            memcpy(lux_msg.data + 1, &my_lux, 4);
            mutex_enter_blocking(&spi_mutex);
            can0.sendMessage(&lux_msg);
            mutex_exit(&spi_mutex);

            // Wait for MSG_CAL_DONE before next round
            t = millis();
            bool got_done = false;
            while (!got_done && millis() - t < 5000) {
                uint32_t raw;
                while (rp2040.fifo.pop_nb(&raw)) {
                    struct can_frame *p = (struct can_frame *)raw;
                    if (CAN_ID_TYPE(p->can_id) == MSG_CAL_DONE &&
                        p->data[0] == active_addr)
                        got_done = true;
                    delete p;
                }
            }
        }
    }

    // Update sys_gain for PID feedforward (self-gain of this node)
    sys_gain = coupling_gains[LUMINAIRE][LUMINAIRE];
    calibrated = true;
    calibration_done = true;
    Serial.println("[CAL] Calibration complete");
    Serial.printf("[CAL] sys_gain=%.2f  background=%.2f\n", sys_gain, sys_background);
}

// =============================================================================
// Core 1 — High-priority CAN receiver
// =============================================================================
void setup1() {
    // Wait for Core 0 to initialise spi_mutex before touching it
    while (!mutex_is_initialized(&spi_mutex)) tight_loop_contents();

    uint8_t pico_flash_id[8];
    rp2040.idleOtherCore();
    flash_get_unique_id(pico_flash_id);
    rp2040.resumeOtherCore();
    node_address = pico_flash_id[7];

    mutex_enter_blocking(&spi_mutex);
    can0.reset();
    can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    // Accept everything during handshake — filters tightened after wake-up
    can0.setFilterMask(MCP2515::MASK0, false, 0x000);
    can0.setFilterMask(MCP2515::MASK1, false, 0x000);
    can0.setNormalMode();
    mutex_exit(&spi_mutex);
}

void loop1() {
    struct can_frame frm;

    mutex_enter_blocking(&spi_mutex);
    MCP2515::ERROR err = can0.readMessage(&frm);
    mutex_exit(&spi_mutex);

    if (err == MCP2515::ERROR_OK) {
        // Heap-allocate a copy and push pointer to Core 0 via FIFO
        struct can_frame *p = new can_frame;
        *p = frm;
        rp2040.fifo.push_nb((uint32_t)p);
    }
}

// =============================================================================
// Core 0 — Control & Communication
// =============================================================================

void process_can_messages() {
    uint32_t raw;
    while (rp2040.fifo.pop_nb(&raw)) {
        struct can_frame *p = (struct can_frame *)raw;

        uint8_t src  = p->data[0]; // convention: first byte = sender index
        uint8_t type = CAN_ID_TYPE(p->can_id);

        switch (type) {
            case MSG_LUX: {
                float lux;
                memcpy(&lux, p->data + 1, 4);
                if (src < 4) remote_lux[src] = lux;
                break;
            }
            case MSG_DUTY: {
                float duty;
                memcpy(&duty, p->data + 1, 4);
                if (src < 4) remote_duty[src] = duty;
                break;
            }
            case MSG_SET_R: {
                float ref;
                memcpy(&ref, p->data + 1, 4);
                r = ref;
                flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
                can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
                break;
            }
            case MSG_SET_OCC: {
                int mode = constrain((int)p->data[1], 0, 2);
                pid.set_occupancy(mode);
                r = (mode == 2) ? ref_high : (mode == 1) ? ref_low : 0.0f;
                flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
                can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
                break;
            }
            case MSG_GET: {
                char var = (char)p->data[1];
                struct can_frame reply;
                reply.can_id  = MAKE_CAN_ID(src, MSG_REPLY);
                reply.can_dlc = 6;
                reply.data[0] = (uint8_t)LUMINAIRE;
                reply.data[1] = (uint8_t)var;
                float val = 0;
                if      (var == 'y') val = lux_value;
                else if (var == 'u') val = pid.get_duty_cycle();
                else if (var == 'r') val = r;
                else if (var == 'p') val = pid.get_u() * MAXIMUM_POWER;
                memcpy(reply.data + 2, &val, 4);
                mutex_enter_blocking(&spi_mutex);
                can0.sendMessage(&reply);
                mutex_exit(&spi_mutex);
                break;
            }
            case MSG_REPLY: {
                char var = (char)p->data[1];
                float val;
                memcpy(&val, p->data + 2, 4);
                Serial.printf("%c %d %.4f\n", var, src, val);
                break;
            }
            case MSG_ACK:
                break;
            default:
                break;
        }
        delete p;
    }
}

void serial_command() {
    const int   BUFFER_SIZE = 128;
    static char buf[BUFFER_SIZE];
    static int  buf_pos = 0;
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (buf_pos > 0) {
                buf[buf_pos] = '\0';
                commands(buf, Serial);
                buf_pos = 0;
            }
        } else if (buf_pos < BUFFER_SIZE - 1) {
            buf[buf_pos++] = c;
        } else {
            buf_pos = 0;
        }
    }
}

void handle_buffer_readout() {
    if ((buffer_y || buffer_u) && buffer_read_counter < buffer_read_size) {
        buffer_data d = last_min_buf.pop();
        if (buffer_y) Serial.printf("%.4f", d.lux_value);
        else          Serial.printf("%.4f", d.duty_cycle);
        buffer_read_counter++;
        if (buffer_read_counter < buffer_read_size) Serial.print(",");
        else { Serial.print("\n"); buffer_y = buffer_u = 0; }
    }
}

void setup() {
    Serial.begin(115200);
    mutex_init(&spi_mutex);
    mutex_init(&can_rx_mutex);

    analogReadResolution(ADC_BITS);
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_MAX);
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 0);

    // ── Identity: map flash UID to clean index ───────────────────────────────
    uint8_t pico_flash_id[8];
    rp2040.idleOtherCore();
    flash_get_unique_id(pico_flash_id);
    rp2040.resumeOtherCore();
    node_address = pico_flash_id[7];

    LUMINAIRE = 0;
    for (int i = 0; i < N_NODES; i++) {
        if (UID_TABLE[i] == node_address) {
            LUMINAIRE = i + 1;
            break;
        }
    }
    if (LUMINAIRE == 0) {
        // UID not in table — print it repeatedly so you can read it in the serial monitor,
        // then update UID_TABLE in main.ino with the decimal value.
        while (true) {
            Serial.printf("=== DISCOVERY: this board's UID last byte = 0x%02X (%d decimal) ===\n",
                          node_address, node_address);
            delay(2000);
        }
    }

    Serial.printf("[BOOT] node_address=0x%02X  LUMINAIRE=%d\n", node_address, LUMINAIRE);

    // ── Wake-up handshake ────────────────────────────────────────────────────
    // Deterministic per-board stagger to avoid CAN collision on first SYN
    delay(node_address % 5 * 100);
    network_wakeup();

    // ── Distributed calibration ──────────────────────────────────────────────
    if (network_ready)
        distributed_calibration();
    else
        Serial.println("[BOOT] Running without full network — calibration skipped");

    // ── PID init ─────────────────────────────────────────────────────────────
    pid.init(LUMINAIRE);
    pid.h = SAMPLE_PERIOD_US * 1e-6f;

    Serial.printf("=== Ready — LUMINAIRE=%d ===\n", LUMINAIRE);
}

void loop() {
    unsigned long now = micros();
    if (now - last_sample_us >= SAMPLE_PERIOD_US) {
        unsigned long actual_dt_us = now - last_sample_us;
        float dt = actual_dt_us * 1e-6f;
        jitter_us      = fabsf((float)actual_dt_us - (float)SAMPLE_PERIOD_US);
        last_sample_us = now;

        lux_value = measureLux();
        float u_raw = pid.get_feedback() ? pid.compute_control(r, lux_value) : serial_duty_cycle;
        u_raw      = constrain(u_raw, 0.0f, 1.0f);
        duty_cycle = u_raw;
        analogWrite(LED_PIN, (int)(u_raw * PWM_MAX));

        last_min_buf.push({lux_value, duty_cycle});
        updateMetrics(r, duty_cycle, dt);

        unsigned long ts_ms = now / 1000UL;
        if (stream_y) Serial.printf("s y %d %.4f %lu\n", LUMINAIRE, lux_value, ts_ms);
        if (stream_u) Serial.printf("s u %d %.4f %lu\n", LUMINAIRE, duty_cycle, ts_ms);
        if (stream_j) Serial.printf("s j %d %.2f %lu\n", LUMINAIRE, jitter_us,  ts_ms);

        // Periodic broadcast
        static int bc_ctr = 0;
        if (++bc_ctr >= 10) {
            can_broadcast_lux(lux_value);
            bc_ctr = 0;
        }
    }
    serial_command();
    handle_buffer_readout();
    process_can_messages();
}

// =============================================================================
// Hub Routing — Forward serial commands to remote nodes over CAN
// =============================================================================
void hub_forward(const char* cmd_str, uint8_t dest_node) {
    char command = cmd_str[0];

    if (command == 'g') {
        char sub;
        int  idx;
        sscanf(cmd_str, "%c %c %d", &command, &sub, &idx);
        struct can_frame msg;
        msg.can_id  = MAKE_CAN_ID(dest_node, MSG_GET);
        msg.can_dlc = 2;
        msg.data[0] = (uint8_t)LUMINAIRE;
        msg.data[1] = (uint8_t)sub;
        mutex_enter_blocking(&spi_mutex);
        can0.sendMessage(&msg);
        mutex_exit(&spi_mutex);

    } else if (command == 'r') {
        float val;
        int   idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET_R, val);
        Serial.println("ack");

    } else if (command == 'o') {
        float val;
        int   idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_byte(dest_node, MSG_SET_OCC, (uint8_t)val);
        Serial.println("ack");

    } else if (command == 'u') {
        float val;
        int   idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET_R, val); // reuse SET_R for now as per instructions
        Serial.println("ack");

    } else if (command == 's') {
        char  var;
        int   idx;
        sscanf(cmd_str, "%c %c %d", &command, &var, &idx);
        can_send_byte(dest_node, MSG_GET, (uint8_t)var);
        Serial.println("ack");

    } else {
        Serial.println("err -> hub cannot forward this command yet");
    }
}
