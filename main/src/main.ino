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

// ─── Node identity ─────────────────────────────────────────────────────────────
uint8_t node_address;       // last byte of flash unique ID
int     LUMINAIRE = 0;      // integer index 1/2/3 assigned at boot

// ─── CAN-BUS globals ─────────────────────────────────────────────────────────
MCP2515 can0(spi0, 17, 19, 16, 18, 10000000);
volatile bool can_msg_ready = false;
struct can_frame canMsgRx;  // filled by core 1, read by core 0
mutex_t can_mutex;          // protects canMsgRx
mutex_t spi_mutex;          // protects can0 SPI access

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
void can_discovery();

// =============================================================================
// Discovery — Boot-time address negotiation (Wake-up Handshake)
// =============================================================================
void can_discovery() {
    uint8_t own_uid[8];
    // Need a tiny delay for Core 1 to initialize CAN
    delay(500);
    
    rp2040.idleOtherCore();
    flash_get_unique_id(own_uid);
    rp2040.resumeOtherCore();

    Serial.printf("[CAN] Discovery: Own UID %02X%02X%02X%02X%02X%02X%02X%02X\n",
                  own_uid[0], own_uid[1], own_uid[2], own_uid[3],
                  own_uid[4], own_uid[5], own_uid[6], own_uid[7]);

    // 1. Broadcast MSG_HELLO
    struct can_frame hello;
    hello.can_id  = MAKE_CAN_ID(BROADCAST, MSG_HELLO);
    hello.can_dlc = 8;
    memcpy(hello.data, own_uid, 8);
    
    mutex_enter_blocking(&spi_mutex);
    can0.sendMessage(&hello);
    mutex_exit(&spi_mutex);

    // 2. Listen for MSG_HELLO from peers
    uint8_t peers_uid[3][8];
    int n_peers = 0;
    
    // Own UID is always in the list
    memcpy(peers_uid[0], own_uid, 8);
    n_peers = 1;

    unsigned long start_discovery = millis();
    while (millis() - start_discovery < 2000) { // Wait 2 seconds
        if (can_msg_ready) {
            struct can_frame frame;
            mutex_enter_blocking(&can_mutex);
            frame = canMsgRx;
            can_msg_ready = false;
            mutex_exit(&can_mutex);

            if (CAN_ID_TYPE(frame.can_id) == MSG_HELLO && frame.can_dlc == 8) {
                bool duplicate = false;
                for (int i = 0; i < n_peers; i++) {
                    if (memcmp(peers_uid[i], frame.data, 8) == 0) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate && n_peers < 3) {
                    memcpy(peers_uid[n_peers], frame.data, 8);
                    n_peers++;
                    Serial.printf("[CAN] Found peer %d\n", n_peers - 1);
                }
            }
        }
    }

    // 3. Sort UIDs to determine rank
    for (int i = 0; i < n_peers - 1; i++) {
        for (int j = i + 1; j < n_peers; j++) {
            if (memcmp(peers_uid[i], peers_uid[j], 8) > 0) {
                uint8_t temp[8];
                memcpy(temp, peers_uid[i], 8);
                memcpy(peers_uid[i], peers_uid[j], 8);
                memcpy(peers_uid[j], temp, 8);
            }
        }
    }

    // 4. Assign LUMINAIRE index (rank + 1)
    for (int i = 0; i < n_peers; i++) {
        if (memcmp(peers_uid[i], own_uid, 8) == 0) {
            LUMINAIRE = i + 1;
            break;
        }
    }

    Serial.printf("[CAN] Assigned index LUMINAIRE = %d\n", LUMINAIRE);

    // 5. Broadcast MSG_READY
    struct can_frame ready;
    ready.can_id  = MAKE_CAN_ID(BROADCAST, MSG_READY);
    ready.can_dlc = 1;
    ready.data[0] = (uint8_t)LUMINAIRE;
    
    mutex_enter_blocking(&spi_mutex);
    can0.sendMessage(&ready);
    
    // 6. Set filters: only BROADCAST or ME
    // Standard ID layout: (dest << 8) | type
    // Mask: 0x0700 (bits 8-10) - filters by destination index
    can0.setFilterMask(MCP2515::MASK0, 0, 0x0700);
    can0.setFilter(MCP2515::RXF0, 0, MAKE_CAN_ID(BROADCAST, 0));
    can0.setFilter(MCP2515::RXF1, 0, MAKE_CAN_ID(LUMINAIRE, 0));
    
    can0.setFilterMask(MCP2515::MASK1, 0, 0x0700);
    can0.setFilter(MCP2515::RXF2, 0, MAKE_CAN_ID(BROADCAST, 0));
    can0.setFilter(MCP2515::RXF3, 0, MAKE_CAN_ID(LUMINAIRE, 0));
    mutex_exit(&spi_mutex);
}

// =============================================================================
// Core 1 — High-priority CAN receiver
// =============================================================================
void setup1() {
    uint8_t pico_flash_id[8];
    rp2040.idleOtherCore();
    flash_get_unique_id(pico_flash_id);
    rp2040.resumeOtherCore();
    node_address = pico_flash_id[7];

    mutex_init(&can_mutex);
    // spi_mutex initialized in setup() on Core 0

    mutex_enter_blocking(&spi_mutex);
    can0.reset();
    can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
    can0.setNormalMode();
    mutex_exit(&spi_mutex);
}

void loop1() {
    struct can_frame frame;
    mutex_enter_blocking(&spi_mutex);
    MCP2515::ERROR err = can0.readMessage(&frame);
    mutex_exit(&spi_mutex);

    if (err == MCP2515::ERROR_OK) {
        mutex_enter_blocking(&can_mutex);
        canMsgRx   = frame;
        can_msg_ready = true;
        mutex_exit(&can_mutex);
    }
}

// =============================================================================
// Core 0 — Control & Communication
// =============================================================================

void process_can_messages() {
    if (!can_msg_ready) return;

    struct can_frame frame;
    mutex_enter_blocking(&can_mutex);
    frame = canMsgRx;
    can_msg_ready = false;
    mutex_exit(&can_mutex);

    uint8_t dest = CAN_ID_DEST(frame.can_id);
    uint8_t type = CAN_ID_TYPE(frame.can_id);
    uint8_t src  = frame.data[0]; // convention: first byte = sender index

    switch (type) {
        case MSG_LUX: {
            float lux;
            memcpy(&lux, frame.data + 1, 4);
            if (src < 4) remote_lux[src] = lux;
            break;
        }
        case MSG_DUTY: {
            float duty;
            memcpy(&duty, frame.data + 1, 4);
            if (src < 4) remote_duty[src] = duty;
            break;
        }
        case MSG_SET_R: {
            float ref;
            memcpy(&ref, frame.data + 1, 4);
            r = ref;
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_SET_OCC: {
            int mode = constrain((int)frame.data[1], 0, 2);
            pid.set_occupancy(mode);
            r = (mode == 2) ? ref_high : (mode == 1) ? ref_low : 0.0f;
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            can_send_byte(src, MSG_ACK, (uint8_t)LUMINAIRE);
            break;
        }
        case MSG_GET: {
            char var = (char)frame.data[1];
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
            char var = (char)frame.data[1];
            float val;
            memcpy(&val, frame.data + 2, 4);
            Serial.printf("%c %d %.4f\n", var, src, val);
            break;
        }
        case MSG_ACK:
            break;
        default:
            break;
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

    analogReadResolution(ADC_BITS);
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_MAX);
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 0);

    // ── Phase 2: initialise CAN-BUS handshake ────────────────────────────────
    can_discovery();

    pid.init(LUMINAIRE);
    pid.h = SAMPLE_PERIOD_US * 1e-6f;

    Serial.printf("=== SCDTR Luminaire Ready (Session 4) — THIS NODE = %d ===\n", LUMINAIRE);
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
