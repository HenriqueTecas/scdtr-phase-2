#include <Arduino.h>
#include <cstdio>
#include "pid.h"
#include "circular_buffer.h"
#include "lux.h"
#include "metrics.h"
#include "calibration.h"
#include "hub.h"            // ← Phase 2: pulls in can_comms.h + hub routing

// ─── CAN-BUS global instances (declared extern in can_comms.h) ───────────────
MCP2515 can0{spi0, 17, 19, 16, 18, 10000000};
uint8_t node_address = 0;

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
// MAXIMUM_POWER = VF * IF  where  IF = (VCC - VF) / R2 = (3.3 - 3.1) / 47 = 4.26 mA
const float MAXIMUM_POWER = 0.0132f;

// ─── Node identity ─────────────────────────────────────────────────────────────
// Assigned dynamically at boot via flash unique ID negotiation in can_init().
// Smallest flash signature → node 1, next → node 2, largest → node 3.
int LUMINAIRE = 0;

// ─── LDR calibration ──────────────────────────────────────────────────────────
float ldr_m = -0.8f;
float ldr_b = 6.3044f;

// ─── Filter mode (declared extern in lux.h) ──────────────────────────────────
int filter_mode = 1; // 0=none, 1=mean (default), 2=median

// ─── System gain calibration ──────────────────────────────────────────────────
float sys_gain       = 0.0f;
float sys_background = 0.0f;
bool  calibrated     = false;

// ─── Phase 2: Table 3 settable parameters ─────────────────────────────────────
float ref_high    = 30.0f;  // HIGH-occupancy lower bound [LUX]  – 'O' command
float ref_low     = 20.0f;  // LOW-occupancy  lower bound [LUX]  – 'U' command
float energy_cost =  1.0f;  // energy cost coefficient           – 'C' command

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

// 60 s × 100 Hz = 6000 samples
CircularBuffer<6000> last_min_buf;

// ─── CAN debugging counters ───────────────────────────────────────────────────
volatile uint32_t can_tx_count = 0;
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_rx_hello = 0;
volatile uint32_t can_rx_lux = 0;
volatile uint32_t can_rx_cmd = 0;
volatile uint32_t can_error_count = 0;

// ─── Phase 2: CAN LUX broadcast throttle ──────────────────────────────────────
// Broadcast every 10 control cycles = 100 ms, keeping bus load low.
static int _can_bc_ctr = 0;
const  int CAN_BC_EVERY = 10;

// ─── Forward declaration ──────────────────────────────────────────────────────
void commands(char *buf, Print &out);

// =============================================================================
// Non-blocking serial reader
// =============================================================================
void serial_command()
{
    const int   BUFFER_SIZE = 128;
    static char buf[BUFFER_SIZE];
    static int  buf_pos = 0;

    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (buf_pos > 0) {
                buf[buf_pos] = '\0';
                commands(buf, Serial);   // ← pass Serial as Print& (Phase 2 signature)
                buf_pos = 0;
            }
        } else if (buf_pos < BUFFER_SIZE - 1) {
            buf[buf_pos++] = c;
        } else {
            buf_pos = 0;  // overflow: discard
        }
    }
}

// =============================================================================
// Non-blocking buffer readout (one sample per loop tick)
// =============================================================================
void handle_buffer_readout()
{
    if ((buffer_y || buffer_u) && buffer_read_counter < buffer_read_size) {
        buffer_data d = last_min_buf.pop();
        if (buffer_y) Serial.printf("%.4f", d.lux_value);
        else          Serial.printf("%.4f", d.duty_cycle);
        buffer_read_counter++;
        if (buffer_read_counter < buffer_read_size)
            Serial.print(",");
        else {
            Serial.print("\n");
            buffer_y = buffer_u = 0;
        }
    }
}

// =============================================================================
// Setup
// =============================================================================
void setup()
{
    Serial.begin(115200);
    // Do NOT wait for USB host — externally-powered Picos have no USB host
    // and would block here forever.

    analogReadResolution(ADC_BITS);
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_MAX);
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 0);

    pid.init(LUMINAIRE);
    pid.h = SAMPLE_PERIOD_US * 1e-6f;  // 0.01 s

    // ── Phase 2: initialise CAN-BUS ──────────────────────────────────────────
    // Resets MCP2515, sets 1 Mbit/s, enters normal mode, sends hello broadcast.
    can_init();

    Serial.printf("=== SCDTR Luminaire Ready (Phase 2) — THIS NODE = %d ===\n", LUMINAIRE);
}

// =============================================================================
// Loop
// =============================================================================
void loop()
{
    unsigned long now = micros();

    // ── 100 Hz time-triggered control loop ────────────────────────────────────
    if (now - last_sample_us >= SAMPLE_PERIOD_US)
    {
        unsigned long actual_dt_us = now - last_sample_us;
        float dt = actual_dt_us * 1e-6f;
        jitter_us      = fabsf((float)actual_dt_us - (float)SAMPLE_PERIOD_US);
        last_sample_us = now;

        // 1. Measure
        lux_value = measureLux();

        // 2. Control
        float u_raw;
        if (pid.get_feedback())
            u_raw = pid.compute_control(r, lux_value);
        else
            u_raw = serial_duty_cycle;

        // 3. Actuate
        u_raw      = constrain(u_raw, 0.0f, 1.0f);
        duty_cycle = u_raw;
        analogWrite(LED_PIN, (int)(u_raw * PWM_MAX));

        // 4. Store in last-minute buffer
        last_min_buf.push({lux_value, duty_cycle});

        // 5. Update metrics
        updateMetrics(r, duty_cycle, dt);

        // 6. Stream if active
        unsigned long ts_ms = now / 1000UL;
        if (stream_y) Serial.printf("s y %d %.4f %lu\n", LUMINAIRE, lux_value, ts_ms);
        if (stream_u) Serial.printf("s u %d %.4f %lu\n", LUMINAIRE, duty_cycle, ts_ms);
        if (stream_j) Serial.printf("s j %d %.2f %lu\n", LUMINAIRE, jitter_us,  ts_ms);

        // 7. ── Phase 2: periodic CAN broadcast ───────────────────────────────
        // Every CAN_BC_EVERY control cycles (= 100 ms) broadcast our state.
        // All nodes receive this; hub caches it for fast "g y/u/r" replies.
        if (++_can_bc_ctr >= CAN_BC_EVERY) {
            MCP2515::ERROR ce = can_broadcast_lux(lux_value, duty_cycle, r);
            if (ce == MCP2515::ERROR_ALLTXBUSY) {
                // TX buffers jammed by unACKed frames (no peers on bus).
                // Reset MCP2515 to clear stuck buffers.
                can0.reset();
                can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
                can0.setNormalMode();
            }
            _can_bc_ctr = 0;
        }
    }

    // ── Non-blocking background tasks ─────────────────────────────────────────
    serial_command();
    handle_buffer_readout();

    // ── Phase 2: drain CAN RX buffer ─────────────────────────────────────────
    // can_poll() is non-blocking; returns valid=false when buffer is empty.
    // hub_dispatch_msg() handles HELLO, LUX_BROADCAST, CMD_FORWARD, CMD_RESPONSE.
    {
        CanRxMsg msg = can_poll();
        while (msg.valid) {
            hub_dispatch_msg(msg);
            msg = can_poll();
        }
    }
}
