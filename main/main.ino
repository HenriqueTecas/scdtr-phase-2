#include <cstdio>
#include "pid.h"
#include "circular_buffer.h"
#include "lux.h"
#include "metrics.h"
#include "calibration.h"
#include "can_comms.h"
#include "admm.h"
#include "hardware/flash.h"
#include "pico/util/queue.h"

// ─── Pin & hardware constants ─────────────────────────────────────────────────
const int LED_PIN = 15;
const int LDR_PIN = A0;
const int ADC_BITS = 12;
const int ADC_MAX = (1 << ADC_BITS) - 1;
const int PWM_BITS = 12;
const int PWM_MAX = (1 << PWM_BITS) - 1;
const int PWM_FREQ_HZ = 30000;

// ─── Physical constants ───────────────────────────────────────────────────────
const float VCC = 3.3f;
const float R_FIXED = 10000.0f;
const float MAXIMUM_POWER = 0.0132f;

// ─── Board identity ───────────────────────────────────────────────────────────
#define N_NODES 3
const uint8_t UID_TABLE[N_NODES] = {33, 54, 39};
const float LDR_B_TABLE[N_NODES] = {5.952f, 5.92f, 6.34f};
const float LDR_M_TABLE[N_NODES] = {-0.8f, -0.8f, -0.8f};

float ldr_m;
float ldr_b;
int LUMINAIRE = 0; // 1/2/3 — this IS the CAN src/dest address

// ─── CAN globals ──────────────────────────────────────────────────────────────
MCP2515 can0(spi0, 17, 19, 16, 18, 10000000);
queue_t can_rx_queue;
queue_t can_tx_queue;
queue_t core1_ready_queue;

const uint8_t CAN_INT_PIN = 20;
volatile bool can_got_irq = false;
void can_irq_handler(uint gpio, uint32_t events) { can_got_irq = true; }

// ─── Network topology ─────────────────────────────────────────────────────────
uint8_t other_nodes[N_NODES - 1]; // luminaire indices of peers
int n_other_nodes = 0;

// ─── Gain matrix ──────────────────────────────────────────────────────────────
float coupling_gains[N_NODES + 1][N_NODES + 1] = {0};
float sys_gain = 0.0f;
float sys_background = 0.0f;
bool calibrated = false;

// ─── Phase 2 parameters ───────────────────────────────────────────────────────
float ref_high = 20.0f;
float ref_low = 10.0f;
float energy_cost = 1.0f;

// ─── Sampling ─────────────────────────────────────────────────────────────────
const unsigned long SAMPLE_PERIOD_US = 10000;
unsigned long last_sample_us = 0;
float jitter_us = 0.0f;
int stream_j = 0;

// ─── PID & control state ──────────────────────────────────────────────────────
PID pid;
float r = 0.0f;
float lux_value = 0.0f;
float duty_cycle = 0.0f;
float serial_duty_cycle = 0.0f;

int stream_y = 0, stream_u = 0;
uint8_t can_stream_y_dest = 0, can_stream_u_dest = 0, can_stream_j_dest = 0;
int buffer_y = 0, buffer_u = 0, buffer_read_size = 0, buffer_read_counter = 0;
CircularBuffer<6000> last_min_buf;

int median_window[MEDIAN_FILTER_SIZE] = {0};
int filter_mode = 1; // 0=none, 1=mean (default), 2=median

// ─── Calibration state machine ────────────────────────────────────────────────
enum class CalStage : uint8_t
{
    CAL_BARRIER,
    CAL_BACKGROUND,
    CAL_ACTIVE_ON,
    CAL_ACTIVE_SAMPLE,
    CAL_ACTIVE_WAIT,
    CAL_PASSIVE_WAIT,
    CAL_PASSIVE_SETTLE,
    CAL_DONE
};
CalStage cal_stage = CalStage::CAL_DONE;

uint8_t all_nodes[N_NODES] = {0};
int cal_round = 0;

bool cal_barrier_peers[N_NODES + 1] = {false};
int cal_barrier_count = 0;
unsigned long cal_barrier_last_broadcast = 0;

unsigned long cal_settle_start = 0;
const unsigned long CAL_SETTLE_MS = 3000;

int cal_lux_received = 0;
uint8_t cal_expected_active = 0;

float background_lux = 0.0f;

bool cal_lux_sent = false;
unsigned long cal_done_deadline = 0;

// ─── Forward declarations ─────────────────────────────────────────────────────
void commands(char *buf, Print &out);
void hub_forward(const char *cmd_str, uint8_t dest_node);
void network_wakeup();
void calibration_loop();
void process_can_messages();

// Populate all_nodes[] with every luminaire index in sorted order so every
// node agrees on the same per-round active node sequence.
static void cal_build_node_list()
{
    all_nodes[0] = (uint8_t)LUMINAIRE;
    for (int i = 0; i < n_other_nodes; i++)
        all_nodes[i + 1] = other_nodes[i];
    // Insertion sort — N_NODES is small (3)
    for (int i = 1; i < N_NODES; i++)
    {
        uint8_t key = all_nodes[i];
        int j = i - 1;
        while (j >= 0 && all_nodes[j] > key)
        {
            all_nodes[j + 1] = all_nodes[j];
            j--;
        }
        all_nodes[j + 1] = key;
    }
}

static void admm_apply_result()
{
    // r_local = d_i + Σⱼ k_ij * ū_j : full ADMM-predicted illuminance at this desk.
    // Using the sum over all nodes (not just own LED) ensures the PI setpoint equals
    // the ADMM constraint target (~L), so actual lux converges to the occupancy target
    // rather than only own-LED contribution (which under-shoots when peers contribute).
    float r_local = admm_d_bg;
    for (int j = 1; j <= ADMM_N; j++)
        r_local += admm_k[j] * admm_u_avg[j];

    if (pid.get_feedback())
    {
        r = r_local;
        pid.set_integrator(admm_u_avg[LUMINAIRE]); // seed from optimal duty to avoid wind-up transient
        flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
        Serial.printf("[ADMM] done  u*=%.4f  r_local=%.2f LUX\n", admm_u_avg[LUMINAIRE], r_local);
    }
    else
    {
        Serial.printf("[ADMM] done  u*=%.4f  r_local=%.2f LUX (feedback off)\n",
                      admm_u_avg[LUMINAIRE], r_local);
    }
}

// =============================================================================
// cal_advance_round
// =============================================================================
void cal_advance_round()
{
    cal_round++;
    if (cal_round >= N_NODES)
    {
        sys_gain = coupling_gains[LUMINAIRE][LUMINAIRE];
        sys_background = background_lux;
        calibrated = true;
        cal_stage = CalStage::CAL_DONE;
        Serial.println("[CAL] Calibration complete");
        Serial.printf("[CAL] sys_gain=%.2f  background=%.2f\n", sys_gain, sys_background);
        Serial.println("[CAL] Gain matrix (row=sensor, col=LED on):");
        for (int i = 1; i <= N_NODES; i++)
        {
            for (int j = 1; j <= N_NODES; j++)
                Serial.printf("  k[%d][%d]=%.2f", i, j, coupling_gains[i][j]);
            Serial.println();
        }
        // All nodes reach this point at calibration completion.
        // Use is_responder=true so each node starts ADMM locally
        // without broadcasting ADMM_TRIGGER — if every node broadcast,
        // each would receive N-1 extra triggers and restart N-1 times.
        admm_request(true);
    }
    else
    {
        if (all_nodes[cal_round] == (uint8_t)LUMINAIRE)
        {
            analogWrite(LED_PIN, PWM_MAX);
            can_send_sub(BROADCAST, MSG_CAL, SUB_CAL_ON);
            cal_settle_start = millis();
            cal_lux_received = 0;
            cal_stage = CalStage::CAL_ACTIVE_ON;
            Serial.printf("[CAL] Round %d — active\n", cal_round + 1);
        }
        else
        {
            cal_expected_active = all_nodes[cal_round];
            cal_settle_start = millis();
            cal_stage = CalStage::CAL_PASSIVE_WAIT;
            Serial.printf("[CAL] Round %d — passive, waiting %d\n",
                          cal_round + 1, cal_expected_active);
        }
    }
}

// =============================================================================
// network_wakeup
// =============================================================================
void network_wakeup()
{
    Serial.printf("[WAKEUP] Node %d starting\n", LUMINAIRE);
    bool peer_seen[N_NODES + 1] = {false};
    unsigned long last_syn_ms = 0;

    while (n_other_nodes < N_NODES - 1)
    {
        serial_command(); // Allow commands while waiting for peers

        if (millis() - last_syn_ms >= 200)
        {
            can_send_sub(BROADCAST, MSG_WAKEUP, SUB_SYN);
            last_syn_ms = millis();
        }
        struct can_frame frm;
        if (!queue_try_remove(&can_rx_queue, &frm))
            continue;
        if (frm.can_id == 0xFFFFFFFF)
            continue;

        uint8_t type = CAN_ID_TYPE(frm.can_id);
        uint8_t src = CAN_ID_SRC(frm.can_id);
        uint8_t sub = frm.data[0];

        if (type != MSG_WAKEUP)
        {
            queue_try_add(&can_rx_queue, &frm);
            continue;
        }

        Serial.printf("[RX] WAKEUP sub=0x%02X src=%d\n", sub, src);
        if (src >= 1 && src <= N_NODES && !peer_seen[src])
        {
            peer_seen[src] = true;
            other_nodes[n_other_nodes++] = src;
            Serial.printf("[WAKEUP] confirmed %d (%d/%d)\n",
                          src, n_other_nodes, N_NODES - 1);
        }
        if (sub == SUB_SYN)
            can_send_sub(BROADCAST, MSG_WAKEUP, SUB_SYN_ACK);
    }
    Serial.printf("[WAKEUP] READY — all %d peers found\n", N_NODES - 1);
}

// =============================================================================
// calibration_loop
// =============================================================================
void calibration_loop()
{
    if (cal_stage == CalStage::CAL_DONE)
        return;

    switch (cal_stage)
    {

    case CalStage::CAL_BARRIER:
    {
        if (millis() - cal_barrier_last_broadcast >= 500)
        {
            can_send_sub(BROADCAST, MSG_CAL, SUB_CAL_READY);
            cal_barrier_last_broadcast = millis();
            static int last_printed = -1;
            if (cal_barrier_count != last_printed)
            {
                Serial.printf("[CAL] BARRIER — %d/%d peers ready\n",
                              cal_barrier_count, N_NODES - 1);
                last_printed = cal_barrier_count;
            }
        }
        if (cal_barrier_count >= N_NODES - 1)
        {
            Serial.println("[CAL] All peers ready — background round starting");
            cal_build_node_list();
            analogWrite(LED_PIN, 0);
            cal_settle_start = millis();
            cal_stage = CalStage::CAL_BACKGROUND;
        }
        break;
    }

    case CalStage::CAL_BACKGROUND:
    {
        if (millis() - cal_settle_start >= CAL_SETTLE_MS)
        {
            background_lux = measureLux();
            sys_background = background_lux;
            Serial.printf("[CAL] Background = %.2f LUX\n", background_lux);
            cal_round = -1;
            cal_advance_round();
        }
        break;
    }

    case CalStage::CAL_ACTIVE_ON:
        if (millis() - cal_settle_start >= CAL_SETTLE_MS)
            cal_stage = CalStage::CAL_ACTIVE_SAMPLE;
        break;

    case CalStage::CAL_ACTIVE_SAMPLE:
    {
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

    case CalStage::CAL_ACTIVE_WAIT:
    {
        bool all_rx = (cal_lux_received >= n_other_nodes);
        bool timeout = (millis() - cal_settle_start >= 5000);
        if (timeout && !all_rx)
            Serial.printf("[CAL] Timeout — got %d/%d replies\n",
                          cal_lux_received, n_other_nodes);
        if (all_rx || timeout)
        {
            analogWrite(LED_PIN, 0);
            can_send_sub(BROADCAST, MSG_CAL, SUB_CAL_DONE);
            cal_advance_round();
        }
        break;
    }

    case CalStage::CAL_PASSIVE_WAIT:
        if (millis() - cal_settle_start >= 10000)
        {
            Serial.printf("[CAL] Timeout waiting CAL_ON from %d\n", cal_expected_active);
            cal_advance_round();
        }
        break;

    case CalStage::CAL_PASSIVE_SETTLE:
    {
        if (!cal_lux_sent && millis() - cal_settle_start >= CAL_SETTLE_MS)
        {
            float my_lux = measureLux();
            // cal_expected_active IS the luminaire index — no UID lookup needed
            coupling_gains[LUMINAIRE][cal_expected_active] = my_lux - background_lux;
            Serial.printf("[CAL] k[%d][%d] (coupling) = %.2f\n",
                          LUMINAIRE, cal_expected_active,
                          coupling_gains[LUMINAIRE][cal_expected_active]);
            can_send_float(cal_expected_active, MSG_CAL, SUB_CAL_LUX, my_lux);
            Serial.printf("[CAL] CAL_LUX %.2f -> %d\n", my_lux, cal_expected_active);
            cal_lux_sent = true;
            cal_done_deadline = millis() + 8000;
        }
        if (cal_lux_sent && millis() >= cal_done_deadline)
        {
            Serial.printf("[CAL] Timeout waiting CAL_DONE from %d\n", cal_expected_active);
            cal_lux_sent = false;
            cal_advance_round();
        }
        break;
    }

    default:
        break;
    }
}

// =============================================================================
// setup1 — Core 1
// =============================================================================
void setup1()
{
    uint8_t go;
    queue_remove_blocking(&core1_ready_queue, &go);

    can0.reset();
    can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);

    // Hardware filter: accept only frames where dest==LUMINAIRE or dest==BROADCAST
    // ID layout [10:8]=type [7:4]=dest [3:0]=src → mask checks bits [7:4] only
    uint32_t mask = 0x0F0;
    uint32_t filter_self = ((uint32_t)LUMINAIRE & 0xF) << 4;
    uint32_t filter_bcast = 0x000;

    can0.setFilterMask(MCP2515::MASK0, false, mask);
    can0.setFilter(MCP2515::RXF0, false, filter_self);
    can0.setFilter(MCP2515::RXF1, false, filter_bcast);
    can0.setFilterMask(MCP2515::MASK1, false, mask);
    can0.setFilter(MCP2515::RXF2, false, filter_self);
    can0.setFilter(MCP2515::RXF3, false, filter_bcast);
    can0.setFilter(MCP2515::RXF4, false, filter_self);
    can0.setFilter(MCP2515::RXF5, false, filter_bcast);

    can0.setNormalMode();
    gpio_set_irq_enabled_with_callback(
        CAN_INT_PIN, GPIO_IRQ_EDGE_FALL, true, &can_irq_handler);
}

// =============================================================================
// loop1 — Core 1
// =============================================================================
void loop1()
{
    // Keep one pending software-TX frame so a temporary "all TX buffers busy"
    // condition does not silently drop a CAN frame.
    static bool tx_pending = false;
    static struct can_frame tx_frm;

    if (!tx_pending)
        tx_pending = queue_try_remove(&can_tx_queue, &tx_frm);

    if (tx_pending)
    {
        MCP2515::ERROR tx_rc = can0.sendMessage(&tx_frm);
        if (tx_rc == MCP2515::ERROR_OK)
        {
            tx_pending = false;
        }
        else if (tx_rc != MCP2515::ERROR_ALLTXBUSY)
        {
            Serial.printf("[CAN TX DROP] rc=%d id=0x%03lX dlc=%d\n",
                          (int)tx_rc,
                          (unsigned long)(tx_frm.can_id & CAN_SFF_MASK),
                          tx_frm.can_dlc);
            tx_pending = false;
        }
    }

    if (!can_got_irq && gpio_get(CAN_INT_PIN))
        return;
    can_got_irq = false;

    struct can_frame frm;
    while (true)
    {
        uint8_t irq = can0.getInterrupts();
        uint8_t eflg = can0.getErrorFlags();
        bool handled = false;

        if (irq & MCP2515::CANINTF_RX0IF)
        {
            if (can0.readMessage(MCP2515::RXB0, &frm) == MCP2515::ERROR_OK)
                queue_try_add(&can_rx_queue, &frm);
            handled = true;
        }

        if (irq & MCP2515::CANINTF_RX1IF)
        {
            if (can0.readMessage(MCP2515::RXB1, &frm) == MCP2515::ERROR_OK)
                queue_try_add(&can_rx_queue, &frm);
            handled = true;
        }

        bool error_irq = (irq & (MCP2515::CANINTF_ERRIF | MCP2515::CANINTF_MERRF)) != 0;
        bool error_flag = (eflg & (MCP2515::EFLG_RX1OVR |
                                   MCP2515::EFLG_RX0OVR |
                                   MCP2515::EFLG_TXBO |
                                   MCP2515::EFLG_TXEP |
                                   MCP2515::EFLG_RXEP)) != 0;
        if (error_irq || error_flag)
        {
            struct can_frame sentinel;
            sentinel.can_id = 0xFFFFFFFF;
            sentinel.can_dlc = 4;
            sentinel.data[0] = irq;
            sentinel.data[1] = eflg;
            sentinel.data[2] = can0.errorCountRX();
            sentinel.data[3] = can0.errorCountTX();
            queue_try_add(&can_rx_queue, &sentinel);
            if (eflg & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR))
                can0.clearRXnOVRFlags();
            if (irq & MCP2515::CANINTF_ERRIF)
                can0.clearERRIF();
            if (irq & MCP2515::CANINTF_MERRF)
                can0.clearMERR();
            handled = true;
        }

        if (!handled)
            break;
    }
}

// =============================================================================
// process_can_messages
// =============================================================================
void process_can_messages()
{
    const float snap_lux = lux_value;
    const float snap_duty = duty_cycle;

    struct can_frame frm;
    while (queue_try_remove(&can_rx_queue, &frm))
    {

        if (frm.can_id == 0xFFFFFFFF)
        {
            if (frm.can_dlc >= 4)
            {
                Serial.printf("[CAN ERR] CANINTF=0x%02X  EFLG=0x%02X  REC=%u  TEC=%u\n",
                              frm.data[0], frm.data[1], frm.data[2], frm.data[3]);
            }
            else
            {
                Serial.printf("[CAN ERR] CANINTF=0x%02X  EFLG=0x%02X\n",
                              frm.data[0], frm.data[1]);
            }
            continue;
        }

        uint8_t type = CAN_ID_TYPE(frm.can_id);
        uint8_t src = CAN_ID_SRC(frm.can_id); // luminaire index 1/2/3
        uint8_t sub = frm.data[0];            // sub-type always data[0]

        switch (type)
        {

        case MSG_WAKEUP:
            // handled entirely inside network_wakeup() — drop here
            break;

        case MSG_SET:
        {
            switch ((char)sub)
            {
            case 'r':
            {
                float val;
                memcpy(&val, frm.data + 1, 4);
                r = val;
                flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
                can_send_byte(src, MSG_CTRL, SUB_ACK, (uint8_t)LUMINAIRE);
                break;
            }
            case 'o':
            {
                int mode = constrain((int)frm.data[1], 0, 2);
                pid.set_occupancy(mode);
                r = (mode == 2) ? ref_high : (mode == 1) ? ref_low
                                                         : 0.0f;
                flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
                can_send_byte(src, MSG_CTRL, SUB_ACK, (uint8_t)LUMINAIRE);
                admm_request(false);
                break;
            }
            case 'u':
            {
                float val;
                memcpy(&val, frm.data + 1, 4);
                serial_duty_cycle = constrain(val, 0.0f, 1.0f);
                if (!pid.get_feedback())
                    analogWrite(LED_PIN, (int)(serial_duty_cycle * PWM_MAX));
                can_send_byte(src, MSG_CTRL, SUB_ACK, (uint8_t)LUMINAIRE);
                break;
            }
            case 'f':
            {
                int mode = (int)frm.data[1];
                serial_duty_cycle = pid.get_duty_cycle();
                if (mode == 1)
                    apply_feedforward(r);
                pid.set_feedback(mode);
                can_send_byte(src, MSG_CTRL, SUB_ACK, (uint8_t)LUMINAIRE);
                break;
            }
            case 'a':
                pid.set_anti_windup((int)frm.data[1]);
                can_send_byte(src, MSG_CTRL, SUB_ACK, (uint8_t)LUMINAIRE);
                break;
            case 'O':
            {
                float val;
                memcpy(&val, frm.data + 1, 4);
                ref_high = val;
                can_send_byte(src, MSG_CTRL, SUB_ACK, (uint8_t)LUMINAIRE);
                break;
            }
            case 'U':
            {
                float val;
                memcpy(&val, frm.data + 1, 4);
                ref_low = val;
                can_send_byte(src, MSG_CTRL, SUB_ACK, (uint8_t)LUMINAIRE);
                break;
            }
            case 'C':
            {
                float val;
                memcpy(&val, frm.data + 1, 4);
                energy_cost = val;
                can_send_byte(src, MSG_CTRL, SUB_ACK, (uint8_t)LUMINAIRE);
                admm_request(false);
                break;
            }
            case 's':
            {
                char var = (char)frm.data[1];
                int on_off = (int)frm.data[2];
                if (var == 'y')
                {
                    if (on_off)
                        can_stream_y_dest = src;
                    else if (can_stream_y_dest == src)
                        can_stream_y_dest = 0;
                }
                else if (var == 'u')
                {
                    if (on_off)
                        can_stream_u_dest = src;
                    else if (can_stream_u_dest == src)
                        can_stream_u_dest = 0;
                }
                else if (var == 'j')
                {
                    if (on_off)
                        can_stream_j_dest = src;
                    else if (can_stream_j_dest == src)
                        can_stream_j_dest = 0;
                }
                can_send_byte(src, MSG_CTRL, SUB_ACK, (uint8_t)LUMINAIRE);
                break;
            }
            }
            break;
        }

        case MSG_GET:
        {
            struct can_frame reply;
            reply.can_id = MAKE_CAN_ID(MSG_REPLY, src, LUMINAIRE);
            reply.can_dlc = 5;
            reply.data[0] = sub;
            float val = 0;
            switch ((char)sub)
            {
            case 'y':
                val = snap_lux;
                break;
            case 'u':
                val = pid.get_feedback() ? pid.get_duty_cycle() : serial_duty_cycle;
                break;
            case 'r':
                val = r;
                break;
            case 'p':
                val = pid.get_u() * MAXIMUM_POWER;
                break;
            case 'd':
                val = max(0.0f, snap_lux - sys_gain * snap_duty);
                break;
            case 't':
                val = micros() * 1e-6f;
                break;
            case 'E':
                val = (float)energy_consumption;
                break;
            case 'V':
                val = sample_count ? (float)(visibility_error / (double)sample_count) : 0.0f;
                break;
            case 'F':
                val = sample_count ? (float)((flicker_error / (double)sample_count) / pid.h) : 0.0f;
                break;
            case 'O':
                val = ref_high;
                break;
            case 'U':
                val = ref_low;
                break;
            case 'L':
                val = (pid.get_occupancy() == 2) ? ref_high : (pid.get_occupancy() == 1) ? ref_low
                                                                                         : 0.0f;
                break;
            case 'C':
                val = energy_cost;
                break;
            case 'K':
                extern float admm_primal_res;
                val = admm_primal_res;
                break;
            case 'J':
                extern float admm_dual_res;
                val = admm_dual_res;
                break;
            case 'o':
                val = (float)pid.get_occupancy();
                break;
            case 'a':
                val = (float)pid.get_anti_windup();
                break;
            case 'f':
                val = (float)pid.get_feedback();
                break;
            case 'v':
                val = (analogRead(LDR_PIN) / (float)ADC_MAX) * VCC;
                break;
            }
            memcpy(reply.data + 1, &val, 4);
            can_queue_tx(reply);
            break;
        }

        case MSG_REPLY:
        {
            float val;
            memcpy(&val, frm.data + 1, 4);
            Serial.printf("%c %d %.4f\n", (char)sub, src, val);
            break;
        }

        case MSG_STREAM:
        {
            if (frm.can_dlc < 5)
            {
                Serial.printf("[STREAM RX DROP] frame too short src=%d dlc=%d\n",
                              src, frm.can_dlc);
                break;
            }
            char var = (char)frm.data[0];
            float val;
            memcpy(&val, frm.data + 1, 4);
            unsigned long ts_ms = millis();
            if (var == 'j')
                Serial.printf("s %c %d %.2f %lu\n", var, src, val, ts_ms);
            else
                Serial.printf("s %c %d %.4f %lu\n", var, src, val, ts_ms);
            break;
        }

        case MSG_CAL:
        {
            // Robust Barrier: any calibration message triggers a reset if finished
            if (cal_stage == CalStage::CAL_DONE &&
                (sub == SUB_CAL_READY || sub == SUB_CAL_ON))
            {
                cal_stage = CalStage::CAL_BARRIER;
                cal_barrier_count = 0;
                memset(cal_barrier_peers, 0, sizeof(cal_barrier_peers));
                Serial.println("[CAL] Resetting state to BARRIER (remote trigger)");
            }

            switch (sub)
            {
            case SUB_CAL_READY:
                if (!cal_barrier_peers[src])
                {
                    cal_barrier_peers[src] = true;
                    cal_barrier_count++;
                    Serial.printf("[CAL] Peer %d ready (%d/%d)\n",
                                  src, cal_barrier_count, N_NODES - 1);
                }
                break;
            case SUB_CAL_ON:
                if (src != (uint8_t)LUMINAIRE)
                {
                    analogWrite(LED_PIN, 0);
                    cal_expected_active = src;
                    cal_settle_start = millis();
                    cal_lux_sent = false;
                    cal_stage = CalStage::CAL_PASSIVE_SETTLE;
                    Serial.printf("[CAL] CAL_ON from %d — settling\n", src);
                }
                break;
            case SUB_CAL_LUX:
                if (cal_stage == CalStage::CAL_ACTIVE_WAIT ||
                    cal_stage == CalStage::CAL_ACTIVE_ON ||
                    cal_stage == CalStage::CAL_ACTIVE_SAMPLE)
                {
                    float peer_lux;
                    memcpy(&peer_lux, frm.data + 1, 4);
                    coupling_gains[src][LUMINAIRE] = peer_lux - background_lux;
                    Serial.printf("[CAL] k[%d][%d] = %.2f\n",
                                  src, LUMINAIRE, coupling_gains[src][LUMINAIRE]);
                    cal_lux_received++;
                }
                break;
            case SUB_CAL_DONE:
                if ((cal_stage == CalStage::CAL_PASSIVE_SETTLE ||
                     cal_stage == CalStage::CAL_PASSIVE_WAIT) &&
                    src == cal_expected_active)
                {
                    Serial.printf("[CAL] CAL_DONE from %d\n", src);
                    cal_lux_sent = false;
                    cal_advance_round();
                }
                break;
            }
            break;
        }

        case MSG_ADMM:
        {
            if (frm.can_dlc < 7)
            {
                Serial.printf("[ADMM RX DROP] packed frame too short src=%d dlc=%d\n",
                              src, frm.can_dlc);
                break;
            }
            uint8_t iter = frm.data[0];
            int16_t q1, q2, q3;
            memcpy(&q1, frm.data + 1, sizeof(q1));
            memcpy(&q2, frm.data + 3, sizeof(q2));
            memcpy(&q3, frm.data + 5, sizeof(q3));
            admm_receive(src, 1, iter, admm_wire_decode(q1));
            admm_receive(src, 2, iter, admm_wire_decode(q2));
            admm_receive(src, 3, iter, admm_wire_decode(q3));
            break;
        }

        case MSG_CTRL:
            if (sub == SUB_ACK)
                Serial.println("ack:remote");
            else if (sub == SUB_ADMM_TRIGGER)
            {
                admm_request(true);
            }
            else if (sub == SUB_RESTART)
            {
                Serial.println("[CTRL] restart");
                delay(50);
                rp2040.reboot();
            }
            break;

        default:
            break;
        }
    }
}

// =============================================================================
// hub_forward
// =============================================================================
void hub_forward(const char *cmd_str, uint8_t dest_node)
{
    char command = cmd_str[0];

    if (command == 'g')
    {
        char sub = 0;
        int idx = 0;
        sscanf(cmd_str, "%c %c %d", &command, &sub, &idx);
        can_send_sub(dest_node, MSG_GET, (uint8_t)sub);
        return;
    }
    if (command == 'r')
    {
        float val;
        int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET, 'r', val);
        Serial.println("ack:forwarded");
        return;
    }
    if (command == 'u')
    {
        float val;
        int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET, 'u', val);
        Serial.println("ack:forwarded");
        return;
    }
    if (command == 'o')
    {
        float val;
        int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_byte(dest_node, MSG_SET, 'o', (uint8_t)val);
        Serial.println("ack:forwarded");
        return;
    }
    if (command == 'f')
    {
        float val;
        int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_byte(dest_node, MSG_SET, 'f', (uint8_t)val);
        Serial.println("ack:forwarded");
        return;
    }
    if (command == 'a')
    {
        float val;
        int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_byte(dest_node, MSG_SET, 'a', (uint8_t)val);
        Serial.println("ack:forwarded");
        return;
    }
    if (command == 'O')
    {
        float val;
        int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET, 'O', val);
        Serial.println("ack:forwarded");
        return;
    }
    if (command == 'U')
    {
        float val;
        int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET, 'U', val);
        Serial.println("ack:forwarded");
        return;
    }
    if (command == 'C')
    {
        float val;
        int idx;
        sscanf(cmd_str, "%c %d %f", &command, &idx, &val);
        can_send_float(dest_node, MSG_SET, 'C', val);
        Serial.println("ack:forwarded");
        return;
    }
    if (command == 's' || command == 'S')
    {
        char var;
        int idx;
        sscanf(cmd_str, "%c %c %d", &command, &var, &idx);
        struct can_frame msg;
        msg.can_id = MAKE_CAN_ID(MSG_SET, dest_node, LUMINAIRE);
        msg.can_dlc = 3;
        msg.data[0] = 's';
        msg.data[1] = (uint8_t)var;
        msg.data[2] = (command == 's') ? 1 : 0;
        can_queue_tx(msg);
        Serial.println("ack:forwarded");
        return;
    }
    Serial.println("err -> hub cannot forward this command");
}

// =============================================================================
// serial_command
// =============================================================================
void serial_command()
{
    const int BUFFER_SIZE = 128;
    static char buf[BUFFER_SIZE];
    static int buf_pos = 0;
    while (Serial.available() > 0)
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (buf_pos > 0)
            {
                buf[buf_pos] = '\0';
                commands(buf, Serial);
                buf_pos = 0;
            }
        }
        else if (buf_pos < BUFFER_SIZE - 1)
        {
            buf[buf_pos++] = c;
        }
        else
        {
            buf_pos = 0;
        }
    }
}

// =============================================================================
// handle_buffer_readout
// =============================================================================
void handle_buffer_readout()
{
    if ((buffer_y || buffer_u) && buffer_read_counter < buffer_read_size)
    {
        buffer_data d = last_min_buf.pop();
        if (buffer_y)
            Serial.printf("%.4f", d.lux_value);
        else
            Serial.printf("%.4f", d.duty_cycle);
        buffer_read_counter++;
        if (buffer_read_counter < buffer_read_size)
            Serial.print(",");
        else
        {
            Serial.print("\n");
            buffer_y = buffer_u = 0;
        }
    }
}

// =============================================================================
// setup
// =============================================================================
void setup()
{
    Serial.begin(115200);
    delay(3000);
    Serial.println("=== FIRMWARE v11 ===");

    queue_init(&can_rx_queue, sizeof(struct can_frame), 256);
    queue_init(&can_tx_queue, sizeof(struct can_frame), 64);
    queue_init(&core1_ready_queue, sizeof(uint8_t), 1);

    analogReadResolution(ADC_BITS);
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_MAX);
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 0);

    uint8_t pico_flash_id[8];
    rp2040.idleOtherCore();
    flash_get_unique_id(pico_flash_id);
    rp2040.resumeOtherCore();
    uint8_t uid = pico_flash_id[7];

    LUMINAIRE = 0;
    for (int i = 0; i < N_NODES; i++)
    {
        if (UID_TABLE[i] == uid)
        {
            LUMINAIRE = i + 1;
            break;
        }
    }
    if (LUMINAIRE == 0)
    {
        while (true)
        {
            Serial.printf("=== DISCOVERY: 0x%02X (%d decimal) ===\n", uid, uid);
            delay(2000);
        }
    }

    ldr_b = LDR_B_TABLE[LUMINAIRE - 1];
    ldr_m = LDR_M_TABLE[LUMINAIRE - 1];
    Serial.printf("[BOOT] LUMINAIRE=%d  ldr_b=%.4f\n", LUMINAIRE, ldr_b);

    uint8_t go = 1;
    queue_try_add(&core1_ready_queue, &go);

    delay(LUMINAIRE * 100);
    network_wakeup();

    if (n_other_nodes == N_NODES - 1)
    {
        cal_stage = CalStage::CAL_BARRIER;
        cal_barrier_last_broadcast = 0;
        Serial.println("[CAL] Calibration armed");
    }
    else
    {
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
void loop()
{
    unsigned long now = micros();
    if (now - last_sample_us >= SAMPLE_PERIOD_US)
    {
        unsigned long actual_dt_us = now - last_sample_us;
        float dt = actual_dt_us * 1e-6f;
        jitter_us = fabsf((float)actual_dt_us - (float)SAMPLE_PERIOD_US);
        last_sample_us = now;

        lux_value = measureLux();

        if (cal_stage == CalStage::CAL_DONE)
        {
            float u_raw = pid.get_feedback() ? pid.compute_control(r, lux_value) : serial_duty_cycle;
            u_raw = constrain(u_raw, 0.0f, 1.0f);
            duty_cycle = u_raw;
            analogWrite(LED_PIN, (int)(u_raw * PWM_MAX));
        }

        last_min_buf.push({lux_value, duty_cycle});
        updateMetrics(r, duty_cycle, dt);

        unsigned long ts_ms = now / 1000UL;
        if (stream_y)
            Serial.printf("s y %d %.4f %lu\n", LUMINAIRE, lux_value, ts_ms);
        if (stream_u)
            Serial.printf("s u %d %.4f %lu\n", LUMINAIRE, duty_cycle, ts_ms);
        if (stream_j)
            Serial.printf("s j %d %.2f %lu\n", LUMINAIRE, jitter_us, ts_ms);
        if (can_stream_y_dest)
            can_send_stream(can_stream_y_dest, 'y', lux_value);
        if (can_stream_u_dest)
            can_send_stream(can_stream_u_dest, 'u', duty_cycle);
        if (can_stream_j_dest)
            can_send_stream(can_stream_j_dest, 'j', jitter_us);
    }

    serial_command();
    handle_buffer_readout();
    // admm_tick must run BEFORE process_can_messages so stage transitions are
    // visible before newly arrived ADMM packets are classified into the current
    // or next receive window.
    if (admm_tick())
        admm_apply_result();
    process_can_messages();
    calibration_loop();
}
