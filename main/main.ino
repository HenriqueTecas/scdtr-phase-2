#include <cstdio>
#include "pid.h"
#include "circular_buffer.h"
#include "lux.h"
#include "metrics.h"
#include "calibration.h"
#include "can_comms.h"
#include "admm.h"
#include "hardware/flash.h"
#include "pico/mutex.h"
#include "pico/util/queue.h"

// ─── Pin & hardware constants ─────────────────────────────────────────────────
const int LED_PIN     = 15;
const int LDR_PIN     = A0;
const int ADC_BITS    = 12;
const int ADC_MAX     = (1 << ADC_BITS) - 1;   // 4095
const int PWM_BITS    = 12;
const int PWM_MAX     = (1 << PWM_BITS) - 1;   // 4095
const int PWM_FREQ_HZ = 30000;

// ─── Physical constants ───────────────────────────────────────────────────────
const float VCC           = 3.3f;
const float R_FIXED       = 10000.0f;
const float MAXIMUM_POWER = 0.0132f;

// ─── Board identity and per-board LDR parameters ─────────────────────────────
#define N_NODES 3
const uint8_t UID_TABLE[N_NODES]   = { 33,      39,      54     };
const float   LDR_B_TABLE[N_NODES] = { 6.3044f, 6.3044f, 6.3044f };
const float   LDR_M_TABLE[N_NODES] = { -0.8f,   -0.8f,   -0.8f   };

uint8_t node_address;
int     LUMINAIRE = 0;

// ─── CAN-BUS globals ──────────────────────────────────────────────────────────
MCP2515 can0(spi0, 17, 19, 16, 18, 10000000);
mutex_t spi_mutex;
mutex_t can_rx_mutex;
queue_t can_rx_queue;

// ─── Network topology ─────────────────────────────────────────────────────────
uint8_t other_nodes[N_NODES - 1];
int     n_other_nodes = 0;

// ─── Wakeup state ─────────────────────────────────────────────────────────────
enum class WakeupStage : uint8_t {
    WAKEUP_HANDSHAKE,  // exchanging SYN/SYN-ACK/ACK_DONE
    WAKEUP_READY       // all peers confirmed, calibration can start
};
WakeupStage wakeup_stage = WakeupStage::WAKEUP_HANDSHAKE;

// Keep network_ready for backward compat with calibration.h extern checks
bool network_ready = false;

// ─── Handshake state ──────────────────────────────────────────────────────────
bool peer_syn_received[256]      = {false};
bool peer_syn_ack_received[256]  = {false};
bool peer_ack_done_received[256] = {false};

// ─── Remote state cache ───────────────────────────────────────────────────────
float remote_lux[256]  = {0};
float remote_duty[256] = {0};

// ─── LDR calibration ──────────────────────────────────────────────────────────
float ldr_m = -0.8f;
float ldr_b = 6.3044f;

// ─── Gain matrix (Module 16 notation) ────────────────────────────────────────
// coupling_gains[i][j] = k_ij
//   row i = sensor that measures, col j = LED that is on
// coupling_gains[LUMINAIRE][LUMINAIRE] = k_ii = direct gain = sys_gain
// coupling_gains[LUMINAIRE][j]         = k_ij = coupling gain from node j to me
float coupling_gains[N_NODES + 1][N_NODES + 1] = {0};

float sys_gain       = 0.0f;
float sys_background = 0.0f;
bool  calibrated     = false;

// ─── Phase 2 parameters ───────────────────────────────────────────────────────
float ref_high    = 30.0f;
float ref_low     = 20.0f;
float energy_cost =  1.0f;

// ─── Sampling ─────────────────────────────────────────────────────────────────
const unsigned long SAMPLE_PERIOD_US = 10000;
unsigned long last_sample_us         = 0;
float jitter_us = 0.0f;
int   stream_j  = 0;

// ─── PID & control state ──────────────────────────────────────────────────────
PID   pid;
float r                 = 0.0f;
float lux_value         = 0.0f;
float duty_cycle        = 0.0f;
float serial_duty_cycle = 0.0f;

int stream_y = 0, stream_u = 0;
int buffer_y = 0, buffer_u = 0, buffer_read_size = 0, buffer_read_counter = 0;

CircularBuffer<6000> last_min_buf;

// ─── Calibration state machine ────────────────────────────────────────────────
enum class CalStage : uint8_t {
    CAL_BARRIER,        // waiting for all nodes to broadcast MSG_CAL_READY
    CAL_BACKGROUND,     // all LEDs off, waiting CAL_SETTLE_MS, then sample background
    CAL_ACTIVE_ON,      // I am active: LED on, waiting for LDR to settle
    CAL_ACTIVE_SAMPLE,  // I am active: take measurement, store k_ii
    CAL_ACTIVE_WAIT,    // I am active: waiting for CAL_LUX replies from all peers
    CAL_PASSIVE_WAIT,   // I am passive: waiting for MSG_CAL_ON from active node
    CAL_PASSIVE_SETTLE, // I am passive: CAL_ON received, waiting for LDR to settle
    CAL_DONE
};
CalStage cal_stage = CalStage::CAL_DONE;

uint8_t all_nodes[N_NODES] = {0};
int     cal_round           = 0;

bool          cal_barrier_peers[256]     = {false};
int           cal_barrier_count          = 0;
unsigned long cal_barrier_last_broadcast = 0;

unsigned long       cal_settle_start = 0;
const unsigned long CAL_SETTLE_MS    = 3000;

int     cal_lux_received    = 0;
uint8_t cal_expected_active = 0;

float background_lux   = 0.0f;
bool  calibration_done = false;

// ─── Forward declarations ─────────────────────────────────────────────────────
void commands(char *buf, Print &out);
void hub_forward(const char* cmd_str, uint8_t dest_node);
void network_wakeup();
void calibration_loop();
void process_can_messages();

// ─── ADMM event flag ──────────────────────────────────────────────────────────
// Set to true whenever occupancy, cost, or calibration changes.
// loop() detects this flag and calls admm_trigger() once, then clears it.
// This makes ADMM event-driven — it only re-runs when something meaningful
// changes, not every control cycle.
static volatile bool admm_pending = false;

// ─────────────────────────────────────────────────────────────────────────────
// admm_trigger
// Called whenever something changes that affects the optimal duty cycle:
//   - calibration just completed (gain matrix is now valid)
//   - occupancy state changed (lower bound L_i changes)
//   - energy cost changed (cost vector c_i changes)
//
// Runs admm_init() to reload state from the latest calibration data and
// occupancy/cost settings, then runs admm_run() to compute the new optimal
// duty u*.
//
// The result is converted from duty [0,1] to lux and stored in r so the PI
// controller tracks the ADMM-optimal illuminance setpoint.
//
// IMPORTANT — when to apply the result to r:
//   Currently we only update r when feedback is enabled AND calibration is done.
//   This is conservative: if the PI is in open-loop mode, writing to r would
//   have no effect anyway (loop() ignores r when feedback=0). Enabling it
//   unconditionally would mean ADMM silently pre-sets r even in open-loop mode,
//   which is fine from a correctness standpoint — the PI just won't act on it
//   until feedback is re-enabled. Ask the teacher whether ADMM should set r
//   regardless of feedback state, or only when feedback=1.
//   TODO: confirm with teacher — should ADMM update r even when feedback=0?
// ─────────────────────────────────────────────────────────────────────────────
static void admm_trigger() {
    if (!calibration_done) return;   // gain matrix not ready yet

    admm_init();
    float u_opt = admm_run();        // optimal duty ∈ [0,1]

    // Convert to lux: r = background + self_gain × u*
    // This is the illuminance the PI must track to deliver u_opt at steady state.
    // Note: u_opt is already guaranteed ≥ (L_i - d_bg)/k_ii by the ADMM
    // feasibility constraint, so r ≥ L_i is ensured.
    float r_admm = sys_background + sys_gain * u_opt;

    if (pid.get_feedback()) {
        r = r_admm;
        flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
        Serial.printf("[ADMM] u*=%.4f  r_admm=%.2f LUX\n", u_opt, r_admm);
    } else {
        // PI is in open-loop — store the result but don't apply yet.
        // When feedback is re-enabled ('f <i> 1'), r will still be the old value.
        // TODO: ask teacher — should we apply r_admm here even in open-loop?
        Serial.printf("[ADMM] u*=%.4f  r_admm=%.2f LUX (feedback off, r not updated)\n",
                      u_opt, r_admm);
    }
}

// =============================================================================
// Helper: build sorted all_nodes[]
// =============================================================================
void cal_build_node_list() {
    all_nodes[0] = node_address;
    for (int i = 0; i < n_other_nodes; i++)
        all_nodes[i + 1] = other_nodes[i];
    for (int i = 1; i < N_NODES; i++) {
        uint8_t key = all_nodes[i];
        int j = i - 1;
        while (j >= 0 && all_nodes[j] > key) {
            all_nodes[j + 1] = all_nodes[j];
            j--;
        }
        all_nodes[j + 1] = key;
    }
}

// =============================================================================
// Helper: send a 2-byte CAN frame
// =============================================================================
static inline void cal_send(uint8_t dest, uint8_t msg_type, uint8_t b0 = 0) {
    struct can_frame f;
    f.can_id  = MAKE_CAN_ID(dest, msg_type);
    f.can_dlc = 2;
    f.data[0] = node_address;
    f.data[1] = b0;
    mutex_enter_blocking(&spi_mutex);
    can0.sendMessage(&f);
    mutex_exit(&spi_mutex);
}

// =============================================================================
// Helper: advance calibration to next round or finish
// =============================================================================
void cal_advance_round() {
    cal_round++;
    if (cal_round >= N_NODES) {
        sys_gain         = coupling_gains[LUMINAIRE][LUMINAIRE];
        sys_background   = background_lux;
        calibrated       = true;
        calibration_done = true;
        cal_stage        = CalStage::CAL_DONE;
        Serial.println("[CAL] Calibration complete");
        Serial.printf("[CAL] sys_gain=%.2f  background=%.2f\n", sys_gain, sys_background);
        Serial.println("[CAL] Gain matrix (row=sensor, col=LED on):");
        for (int i = 1; i <= N_NODES; i++) {
            for (int j = 1; j <= N_NODES; j++)
                Serial.printf("  k[%d][%d]=%.2f", i, j, coupling_gains[i][j]);
            Serial.println();
        }
        // Calibration gives us a valid gain matrix — trigger ADMM immediately
        // so the PI gets an energy-optimal setpoint as soon as we go live.
        admm_pending = true;
    } else {
        if (all_nodes[cal_round] == node_address) {
            analogWrite(LED_PIN, PWM_MAX);
            cal_send(BROADCAST, MSG_CAL_ON, node_address);
            cal_settle_start = millis();
            cal_lux_received = 0;
            cal_stage = CalStage::CAL_ACTIVE_ON;
            Serial.printf("[CAL] Round %d — active\n", cal_round + 1);
        } else {
            cal_expected_active = all_nodes[cal_round];
            cal_settle_start    = millis();
            cal_stage = CalStage::CAL_PASSIVE_WAIT;
            Serial.printf("[CAL] Round %d — passive, waiting 0x%02X\n",
                          cal_round + 1, cal_expected_active);
        }
    }
}

// =============================================================================
// Wake-up — Three-way handshake (SYN / SYN-ACK / ACK-DONE)
// =============================================================================
void network_wakeup() {
    Serial.printf("[WAKEUP] Node %d (0x%02X) starting\n", LUMINAIRE, node_address);

    unsigned long last_syn_ms = 0;

    while (n_other_nodes < N_NODES - 1) {

        if (millis() - last_syn_ms >= 500) {
            struct can_frame syn;
            syn.can_id  = MAKE_CAN_ID(BROADCAST, MSG_SYN);
            syn.can_dlc = 1;
            syn.data[0] = node_address;
            mutex_enter_blocking(&spi_mutex);
            can0.sendMessage(&syn);
            mutex_exit(&spi_mutex);
            last_syn_ms = millis();
        }

        struct can_frame frm;
        while (queue_try_remove(&can_rx_queue, &frm)) {
            uint8_t type = CAN_ID_TYPE(frm.can_id);
            uint8_t src  = frm.data[0];

            if (type == MSG_SYN) {
                // Always reply — peer may not have received our SYN-ACK
                if (!peer_syn_received[src]) {
                    peer_syn_received[src] = true;
                    Serial.printf("[WAKEUP] +peer 0x%02X\n", src);
                }
                struct can_frame syn_ack;
                syn_ack.can_id  = MAKE_CAN_ID(src, MSG_SYN_ACK);
                syn_ack.can_dlc = 1;
                syn_ack.data[0] = node_address;
                mutex_enter_blocking(&spi_mutex);
                can0.sendMessage(&syn_ack);
                mutex_exit(&spi_mutex);

            } else if (type == MSG_SYN_ACK) {
                if (!peer_syn_ack_received[src]) {
                    peer_syn_ack_received[src] = true;
                    struct can_frame ack;
                    ack.can_id  = MAKE_CAN_ID(src, MSG_ACK_DONE);
                    ack.can_dlc = 1;
                    ack.data[0] = node_address;
                    mutex_enter_blocking(&spi_mutex);
                    can0.sendMessage(&ack);
                    mutex_exit(&spi_mutex);
                }

            } else if (type == MSG_ACK_DONE) {
                if (!peer_ack_done_received[src]) {
                    peer_ack_done_received[src] = true;
                    if (n_other_nodes < N_NODES - 1) {
                        other_nodes[n_other_nodes++] = src;
                        Serial.printf("[WAKEUP] confirmed 0x%02X (%d/%d)\n",
                                      src, n_other_nodes, N_NODES - 1);
                    }
                }
            }
            // All other types discarded during handshake
        }
    }

    wakeup_stage  = WakeupStage::WAKEUP_READY;
    network_ready = true;
    Serial.printf("[WAKEUP] READY — all %d peers found\n", N_NODES - 1);

    // Leave filters fully open — 0x700 mask was blocking broadcast frames
    mutex_enter_blocking(&spi_mutex);
    can0.setFilterMask(MCP2515::MASK0, false, 0x000);
    can0.setFilterMask(MCP2515::MASK1, false, 0x000);
    mutex_exit(&spi_mutex);

    // Flush only handshake frames, preserve any early CAL_READY frames
    {
        struct can_frame buf[64];
        int keep = 0, flush = 0;
        struct can_frame tmp;
        while (queue_try_remove(&can_rx_queue, &tmp)) {
            uint8_t t = CAN_ID_TYPE(tmp.can_id);
            if (t == MSG_SYN || t == MSG_SYN_ACK || t == MSG_ACK_DONE)
                flush++;
            else if (keep < 64)
                buf[keep++] = tmp;
        }
        for (int i = 0; i < keep; i++)
            queue_try_add(&can_rx_queue, &buf[i]);
        Serial.printf("[WAKEUP] flushed %d handshake frames, kept %d\n", flush, keep);
    }
}

// =============================================================================
// Non-blocking calibration state machine
// =============================================================================
void calibration_loop() {
    if (cal_stage == CalStage::CAL_DONE) return;

    switch (cal_stage) {

    case CalStage::CAL_BARRIER: {
        if (millis() - cal_barrier_last_broadcast >= 500) {
            cal_send(BROADCAST, MSG_CAL_READY);
            cal_barrier_last_broadcast = millis();
            static int last_printed = -1;
            if (cal_barrier_count != last_printed) {
                Serial.printf("[CAL] BARRIER — %d/%d peers ready\n",
                              cal_barrier_count, N_NODES - 1);
                last_printed = cal_barrier_count;
            }
        }
        if (cal_barrier_count >= N_NODES - 1) {
            Serial.println("[CAL] All peers ready — background round starting");
            cal_build_node_list();
            analogWrite(LED_PIN, 0);
            cal_settle_start = millis();
            cal_stage = CalStage::CAL_BACKGROUND;
        }
        break;
    }

    case CalStage::CAL_BACKGROUND: {
        if (millis() - cal_settle_start >= CAL_SETTLE_MS) {
            background_lux = measureLux();
            sys_background = background_lux;
            Serial.printf("[CAL] Background = %.2f LUX\n", background_lux);
            cal_round = -1;
            cal_advance_round();
        }
        break;
    }

    case CalStage::CAL_ACTIVE_ON: {
        if (millis() - cal_settle_start >= CAL_SETTLE_MS)
            cal_stage = CalStage::CAL_ACTIVE_SAMPLE;
        break;
    }

    case CalStage::CAL_ACTIVE_SAMPLE: {
        float my_lux = measureLux();
        coupling_gains[LUMINAIRE][LUMINAIRE] = my_lux - background_lux;
        sys_gain = coupling_gains[LUMINAIRE][LUMINAIRE];
        Serial.printf("[CAL] k[%d][%d] (self) = %.2f\n",
                      LUMINAIRE, LUMINAIRE, coupling_gains[LUMINAIRE][LUMINAIRE]);
        cal_lux_received = 0;
        cal_settle_start = millis();
        cal_stage = CalStage::CAL_ACTIVE_WAIT;
        break;
    }

    case CalStage::CAL_ACTIVE_WAIT: {
        bool all_rx  = (cal_lux_received >= n_other_nodes);
        bool timeout = (millis() - cal_settle_start >= 5000);
        if (timeout && !all_rx)
            Serial.printf("[CAL] Timeout — got %d/%d replies\n",
                          cal_lux_received, n_other_nodes);
        if (all_rx || timeout) {
            analogWrite(LED_PIN, 0);
            cal_send(BROADCAST, MSG_CAL_DONE, node_address);
            cal_advance_round();
        }
        break;
    }

    case CalStage::CAL_PASSIVE_WAIT: {
        if (millis() - cal_settle_start >= 10000) {
            Serial.printf("[CAL] Timeout waiting CAL_ON from 0x%02X\n",
                          cal_expected_active);
            cal_advance_round();
        }
        break;
    }

    case CalStage::CAL_PASSIVE_SETTLE: {
        if (millis() - cal_settle_start >= CAL_SETTLE_MS) {
            float my_lux = measureLux();

            int active_luminaire = -1;
            for (int k = 0; k < N_NODES; k++)
                if (UID_TABLE[k] == cal_expected_active) {
                    active_luminaire = k + 1;
                    break;
                }

            if (active_luminaire > 0) {
                coupling_gains[LUMINAIRE][active_luminaire] = my_lux - background_lux;
                Serial.printf("[CAL] k[%d][%d] (coupling) = %.2f\n",
                              LUMINAIRE, active_luminaire,
                              coupling_gains[LUMINAIRE][active_luminaire]);
            }

            struct can_frame lux_msg;
            lux_msg.can_id  = MAKE_CAN_ID(cal_expected_active, MSG_CAL_LUX);
            lux_msg.can_dlc = 5;
            lux_msg.data[0] = node_address;
            memcpy(lux_msg.data + 1, &my_lux, 4);
            mutex_enter_blocking(&spi_mutex);
            can0.sendMessage(&lux_msg);
            mutex_exit(&spi_mutex);
            Serial.printf("[CAL] CAL_LUX %.2f → 0x%02X\n", my_lux, cal_expected_active);

            cal_settle_start = millis(); // reuse as CAL_DONE timeout
        }
        if (millis() - cal_settle_start >= 8000) {
            Serial.printf("[CAL] Timeout waiting CAL_DONE from 0x%02X\n",
                          cal_expected_active);
            cal_advance_round();
        }
        break;
    }

    default: break;
    }
}

// =============================================================================
// Core 1 — CAN receiver
// =============================================================================
void setup1() {
    while (!mutex_is_initialized(&spi_mutex)) tight_loop_contents();

    uint8_t pico_flash_id[8];
    rp2040.idleOtherCore();
    flash_get_unique_id(pico_flash_id);
    rp2040.resumeOtherCore();
    node_address = pico_flash_id[7];

    mutex_enter_blocking(&spi_mutex);
    can0.reset();
    can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    can0.setFilterMask(MCP2515::MASK0, false, 0x000);
    can0.setFilter(MCP2515::RXF0,  false, 0x000);
    can0.setFilter(MCP2515::RXF1,  false, 0x000);
    can0.setFilterMask(MCP2515::MASK1, false, 0x000);
    can0.setFilter(MCP2515::RXF2,  false, 0x000);
    can0.setFilter(MCP2515::RXF3,  false, 0x000);
    can0.setFilter(MCP2515::RXF4,  false, 0x000);
    can0.setFilter(MCP2515::RXF5,  false, 0x000);
    can0.setNormalMode();
    mutex_exit(&spi_mutex);
}

void loop1() {
    struct can_frame frm;
    mutex_enter_blocking(&spi_mutex);
    MCP2515::ERROR err = can0.readMessage(&frm);
    mutex_exit(&spi_mutex);
    if (err == MCP2515::ERROR_OK)
        queue_try_add(&can_rx_queue, &frm);
}

// =============================================================================
// process_can_messages
// =============================================================================
void process_can_messages() {
    struct can_frame frm;
    while (queue_try_remove(&can_rx_queue, &frm)) {
        uint8_t type = CAN_ID_TYPE(frm.can_id);
        uint8_t src  = frm.data[0];

        // Drop handshake messages once wakeup is done
        if (wakeup_stage == WakeupStage::WAKEUP_READY &&
            (type == MSG_SYN || type == MSG_SYN_ACK || type == MSG_ACK_DONE))
            continue;

        switch (type) {

        case MSG_LUX: {
            float lux;
            memcpy(&lux, frm.data + 1, 4);
            remote_lux[src] = lux;
            break;
        }
        case MSG_DUTY: {
            float duty;
            memcpy(&duty, frm.data + 1, 4);
            remote_duty[src] = duty;
            break;
        }
        case MSG_SET_R: {
            float ref;
            memcpy(&ref, frm.data + 1, 4);
            r = ref;
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_SET_OCC: {
            int mode = constrain((int)frm.data[1], 0, 2);
            pid.set_occupancy(mode);
            r = (mode == 2) ? ref_high : (mode == 1) ? ref_low : 0.0f;
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            // Occupancy change → lower bound L_i changes → re-optimise
            admm_pending = true;
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_SET_DUTY: {
            float val; memcpy(&val, frm.data + 1, 4);
            serial_duty_cycle = constrain(val, 0.0f, 1.0f);
            if (!pid.get_feedback()) analogWrite(LED_PIN, (int)(serial_duty_cycle * PWM_MAX));
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_SET_FBCK: {
            int mode = (int)frm.data[1];
            serial_duty_cycle = pid.get_duty_cycle();
            if (mode == 1) apply_feedforward(r);
            pid.set_feedback(mode);
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_SET_AW: {
            pid.set_anti_windup((int)frm.data[1]);
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_SET_OCC_HI: {
            float val; memcpy(&val, frm.data + 1, 4);
            ref_high = val;
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_SET_OCC_LO: {
            float val; memcpy(&val, frm.data + 1, 4);
            ref_low = val;
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_SET_COST: {
            float val; memcpy(&val, frm.data + 1, 4);
            energy_cost = val;
            // Cost change → cost vector c_i changes → re-optimise
            admm_pending = true;
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_STREAM_ON: {
            char var = (char)frm.data[1];
            if      (var == 'y') stream_y = 1;
            else if (var == 'u') stream_u = 1;
            else if (var == 'j') stream_j = 1;
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_STREAM_OFF: {
            char var = (char)frm.data[1];
            if      (var == 'y') stream_y = 0;
            else if (var == 'u') stream_u = 0;
            else if (var == 'j') stream_j = 0;
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }

        case MSG_GET: {
            char var  = (char)frm.data[1];
            char xvar = (frm.can_dlc >= 3) ? (char)frm.data[2] : 0;
            if (var == 'b') {
                // Buffer readout — too large for CAN; stream locally via Serial
                buffer_y = buffer_u = 0;
                buffer_read_size    = last_min_buf.size();
                buffer_read_counter = 0;
                if      (xvar == 'y') { buffer_y = 1; Serial.printf("b y %d ", LUMINAIRE); }
                else if (xvar == 'u') { buffer_u = 1; Serial.printf("b u %d ", LUMINAIRE); }
                can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
                break;
            }
            struct can_frame reply;
            reply.can_id  = MAKE_CAN_ID(src, MSG_REPLY);
            reply.can_dlc = 6;
            reply.data[0] = (uint8_t)LUMINAIRE;
            reply.data[1] = (uint8_t)var;
            float val = 0;
            if      (var == 'y') val = lux_value;
            else if (var == 'u') val = pid.get_feedback() ? pid.get_duty_cycle() : serial_duty_cycle;
            else if (var == 'r') val = r;
            else if (var == 'p') val = pid.get_u() * MAXIMUM_POWER;
            else if (var == 'd') val = max(0.0f, lux_value - sys_gain * duty_cycle);
            else if (var == 't') val = micros() * 1e-6f;
            else if (var == 'E') val = (float)energy_consumption;
            else if (var == 'V') val = sample_count ? (float)(visibility_error/(double)sample_count) : 0.0f;
            else if (var == 'F') val = sample_count ? (float)((flicker_error/(double)sample_count)/pid.h) : 0.0f;
            else if (var == 'O') val = ref_high;
            else if (var == 'U') val = ref_low;
            else if (var == 'L') val = (pid.get_occupancy()==2)?ref_high:(pid.get_occupancy()==1)?ref_low:0.0f;
            else if (var == 'C') val = energy_cost;
            else if (var == 'o') val = (float)pid.get_occupancy();
            else if (var == 'a') val = (float)pid.get_anti_windup();
            else if (var == 'f') val = (float)pid.get_feedback();
            else if (var == 'v') val = (analogRead(LDR_PIN) / (float)ADC_MAX) * VCC;
            memcpy(reply.data + 2, &val, 4);
            mutex_enter_blocking(&spi_mutex);
            can0.sendMessage(&reply);
            mutex_exit(&spi_mutex);
            break;
        }
        case MSG_U_OPT: {
            // Peer node j broadcast its converged duty u_jj during ADMM iteration.
            // Store it in admm_recv[] so admm_run() can compute the consensus ū.
            float val;
            memcpy(&val, frm.data + 1, 4);
            admm_recv[src]     = val;
            admm_recv_new[src] = true;
            break;
        }

        case MSG_REPLY: {
            char var = (char)frm.data[1];
            float val;
            memcpy(&val, frm.data + 2, 4);
            // Map raw CAN address → luminaire index for human-readable output
            int lum_idx = 0;
            for (int k = 0; k < N_NODES; k++)
                if (UID_TABLE[k] == src) { lum_idx = k + 1; break; }
            Serial.printf("%c %d %.4f\n", var, lum_idx ? lum_idx : (int)src, val);
            break;
        }
        case MSG_ACK:
            Serial.println("ack");
            break;

        case MSG_CAL_READY: {
            if (!cal_barrier_peers[src]) {
                cal_barrier_peers[src] = true;
                cal_barrier_count++;
                Serial.printf("[CAL] Peer 0x%02X ready (%d/%d)\n",
                              src, cal_barrier_count, N_NODES - 1);
            }
            break;
        }

        case MSG_CAL_ON: {
            uint8_t active = frm.data[1];
            if (active == 0x00) {
                analogWrite(LED_PIN, 0);
            } else if (cal_stage == CalStage::CAL_PASSIVE_WAIT &&
                       active == cal_expected_active) {
                analogWrite(LED_PIN, 0);
                cal_settle_start = millis();
                cal_stage = CalStage::CAL_PASSIVE_SETTLE;
                Serial.printf("[CAL] CAL_ON from 0x%02X — settling\n", active);
            }
            break;
        }

        case MSG_CAL_LUX: {
            if (cal_stage == CalStage::CAL_ACTIVE_WAIT) {
                float peer_lux;
                memcpy(&peer_lux, frm.data + 1, 4);
                int peer_idx = -1;
                for (int k = 0; k < N_NODES; k++)
                    if (UID_TABLE[k] == src) { peer_idx = k + 1; break; }
                if (peer_idx > 0) {
                    coupling_gains[peer_idx][LUMINAIRE] = peer_lux - background_lux;
                    Serial.printf("[CAL] k[%d][%d] = %.2f (0x%02X)\n",
                                  peer_idx, LUMINAIRE,
                                  coupling_gains[peer_idx][LUMINAIRE], src);
                }
                cal_lux_received++;
            }
            break;
        }

        case MSG_CAL_DONE: {
            if ((cal_stage == CalStage::CAL_PASSIVE_SETTLE ||
                 cal_stage == CalStage::CAL_PASSIVE_WAIT) &&
                frm.data[0] == cal_expected_active) {
                Serial.printf("[CAL] CAL_DONE from 0x%02X\n", frm.data[0]);
                cal_advance_round();
            }
            break;
        }

        case MSG_REBOOT:{
            delay(3000);
            rp2040.reboot();
            break;
        }

        default: break;
        }
    }
}

// =============================================================================
// serial_command
// =============================================================================
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

// =============================================================================
// handle_buffer_readout
// =============================================================================
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

// =============================================================================
// setup
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(3000);
    Serial.println("=== FIRMWARE v2 ===");
    mutex_init(&spi_mutex);
    mutex_init(&can_rx_mutex);
    queue_init(&can_rx_queue, sizeof(struct can_frame), 64);

    analogReadResolution(ADC_BITS);
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_MAX);
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 0);

    uint8_t pico_flash_id[8];
    rp2040.idleOtherCore();
    flash_get_unique_id(pico_flash_id);
    rp2040.resumeOtherCore();
    node_address = pico_flash_id[7];

    LUMINAIRE = 0;
    for (int i = 0; i < N_NODES; i++) {
        if (UID_TABLE[i] == node_address) { LUMINAIRE = i + 1; break; }
    }
    if (LUMINAIRE == 0) {
        while (true) {
            Serial.printf("=== DISCOVERY: 0x%02X (%d decimal) ===\n",
                          node_address, node_address);
            delay(2000);
        }
    }

    ldr_b = LDR_B_TABLE[LUMINAIRE - 1];
    ldr_m = LDR_M_TABLE[LUMINAIRE - 1];

    Serial.printf("[BOOT] 0x%02X  LUMINAIRE=%d  ldr_b=%.4f\n",
                  node_address, LUMINAIRE, ldr_b);

    delay(node_address % 5 * 100);
    network_wakeup();

    if (wakeup_stage == WakeupStage::WAKEUP_READY) {
        cal_stage = CalStage::CAL_BARRIER;
        cal_barrier_last_broadcast = 0;
        Serial.println("[CAL] Calibration armed");
    } else {
        cal_stage = CalStage::CAL_DONE;
        Serial.println("[BOOT] No full network — calibration skipped");
    }

    pid.init(LUMINAIRE);
    pid.h = SAMPLE_PERIOD_US * 1e-6f;

    Serial.printf("=== Ready — LUMINAIRE=%d ===\n", LUMINAIRE);
}

// =============================================================================
// loop
// =============================================================================
void loop() {
    unsigned long now = micros();
    if (now - last_sample_us >= SAMPLE_PERIOD_US) {
        unsigned long actual_dt_us = now - last_sample_us;
        float dt       = actual_dt_us * 1e-6f;
        jitter_us      = fabsf((float)actual_dt_us - (float)SAMPLE_PERIOD_US);
        last_sample_us = now;

        lux_value = measureLux();

        if (cal_stage == CalStage::CAL_DONE) {
            float u_raw = pid.get_feedback() ?
                          pid.compute_control(r, lux_value) : serial_duty_cycle;
            u_raw      = constrain(u_raw, 0.0f, 1.0f);
            duty_cycle = u_raw;
            analogWrite(LED_PIN, (int)(u_raw * PWM_MAX));
        }

        last_min_buf.push({lux_value, duty_cycle});
        updateMetrics(r, duty_cycle, dt);

        unsigned long ts_ms = now / 1000UL;
        if (stream_y) Serial.printf("s y %d %.4f %lu\n", LUMINAIRE, lux_value, ts_ms);
        if (stream_u) Serial.printf("s u %d %.4f %lu\n", LUMINAIRE, duty_cycle, ts_ms);
        if (stream_j) Serial.printf("s j %d %.2f %lu\n",  LUMINAIRE, jitter_us,  ts_ms);

        static int bc_ctr = 0;
        if (++bc_ctr >= 10 && cal_stage == CalStage::CAL_DONE) {
            can_broadcast_lux(lux_value);
            bc_ctr = 0;
        }
    }

    serial_command();
    handle_buffer_readout();
    process_can_messages();
    calibration_loop();

    // ── ADMM event trigger ────────────────────────────────────────────────────
    // admm_pending is set by: calibration complete, occupancy change, cost change.
    // We run ADMM here (not inside the handlers) so it executes on Core 0
    // with no risk of blocking the CAN receive queue on Core 1.
    // admm_run() will block for up to ADMM_MAXITER × ADMM_TIMEOUT ms while
    // exchanging CAN frames with peers — the PI misses samples during this
    // window, which is acceptable since ADMM only fires on discrete events.
    if (admm_pending) {
        admm_pending = false;
        admm_trigger();
    }
}

// =============================================================================
// hub_forward
// =============================================================================
void hub_forward(const char* cmd_str, uint8_t dest_node) {
    char command = cmd_str[0];

    if (command == 'g') {
        char sub = 0, xvar = 0; int idx = 0;
        int n = sscanf(cmd_str, "%c %c %c %d", &command, &sub, &xvar, &idx);
        struct can_frame msg;
        msg.can_id  = MAKE_CAN_ID(dest_node, MSG_GET);
        msg.data[0] = (uint8_t)LUMINAIRE;
        msg.data[1] = (uint8_t)sub;
        if (sub == 'b' && n == 4) { msg.can_dlc = 3; msg.data[2] = (uint8_t)xvar; }
        else                       { msg.can_dlc = 2; }
        mutex_enter_blocking(&spi_mutex);
        can0.sendMessage(&msg);
        mutex_exit(&spi_mutex);
        return;
    }
    if (command == 'r') {
        float val; int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET_R, val);
        Serial.println("ack"); return;
    }
    if (command == 'u') {
        float val; int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET_DUTY, val);
        Serial.println("ack"); return;
    }
    if (command == 'o') {
        float val; int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_byte(dest_node, MSG_SET_OCC, (uint8_t)val);
        Serial.println("ack"); return;
    }
    if (command == 'f') {
        float val; int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_byte(dest_node, MSG_SET_FBCK, (uint8_t)val);
        Serial.println("ack"); return;
    }
    if (command == 'a') {
        float val; int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_byte(dest_node, MSG_SET_AW, (uint8_t)val);
        Serial.println("ack"); return;
    }
    if (command == 'O') {
        float val; int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET_OCC_HI, val);
        Serial.println("ack"); return;
    }
    if (command == 'U') {
        float val; int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET_OCC_LO, val);
        Serial.println("ack"); return;
    }
    if (command == 'C') {
        float val; int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET_COST, val);
        Serial.println("ack"); return;
    }
    if (command == 's') {
        char var; int idx;
        sscanf(cmd_str, "%c %c %d", &command, &var, &idx);
        can_send_byte(dest_node, MSG_STREAM_ON, (uint8_t)var);
        Serial.println("ack"); return;
    }
    if (command == 'S') {
        char var; int idx;
        sscanf(cmd_str, "%c %c %d", &command, &var, &idx);
        can_send_byte(dest_node, MSG_STREAM_OFF, (uint8_t)var);
        Serial.println("ack"); return;
    }
    if (command == 'R') {
        can_send_byte(BROADCAST, MSG_REBOOT, 0);
        delay(500);
        rp2040.reboot();
        return;
    }
    Serial.println("err -> hub cannot forward this command");
}
