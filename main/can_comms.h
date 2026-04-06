#pragma once
#include "mcp2515.h"
#include "pico/util/queue.h"

// ── 11-bit CAN ID layout ──────────────────────────────────────────────────────
// [10:8] type (3 bits)   0–7
// [7:4]  dest (4 bits)   0=broadcast, 1/2/3 = luminaire index
// [3:0]  src  (4 bits)   1/2/3 = luminaire index
#define MAKE_CAN_ID(type, dest, src) \
    ((((uint32_t)(type) & 0x7) << 8) | \
     (((uint32_t)(dest) & 0xF) << 4) | \
      ((uint32_t)(src)  & 0xF))

#define CAN_ID_TYPE(id)  (((id) >> 8) & 0x07)
#define CAN_ID_DEST(id)  (((id) >> 4) & 0x0F)
#define CAN_ID_SRC(id)   (((id) >> 0) & 0x0F)
#define BROADCAST        0x0

// ── Message types (3 bits = 8 types) ─────────────────────────────────────────
#define MSG_WAKEUP  0x0
#define MSG_CAL     0x1
#define MSG_SET     0x2
#define MSG_GET     0x3
#define MSG_REPLY   0x4
#define MSG_ADMM    0x6
#define MSG_CTRL    0x7

// ── Sub-types (data[0]) — only non-obvious numeric constants need names ───────
// MSG_WAKEUP
#define SUB_SYN         0x01
#define SUB_SYN_ACK     0x02
// MSG_CAL
#define SUB_CAL_READY   0x01
#define SUB_CAL_ON      0x02
#define SUB_CAL_LUX     0x03
#define SUB_CAL_DONE    0x04
// MSG_CTRL
#define SUB_ACK         0x01
#define SUB_REBOOT      0x02
#define SUB_ADMM_TRIGGER 0x05

// ── Globals from main.ino ─────────────────────────────────────────────────────
extern MCP2515  can0;
extern int      LUMINAIRE;
extern queue_t  can_tx_queue;

// ── Frame builders ────────────────────────────────────────────────────────────
inline void can_queue_tx(const struct can_frame &msg) {
    queue_add_blocking(&can_tx_queue, &msg);
}

// ADMM transport uses one packed CAN frame per iteration.
// Each component is quantized as signed Q11 fixed-point with scale 2048,
// which keeps the full vector inside a 7-byte CAN payload.
static constexpr float ADMM_WIRE_SCALE = 2048.0f;
static constexpr float ADMM_WIRE_MIN = -16.0f;
static constexpr float ADMM_WIRE_MAX = 15.9995f;

inline int16_t admm_wire_encode(float value) {
    if (value < ADMM_WIRE_MIN)
        value = ADMM_WIRE_MIN;
    else if (value > ADMM_WIRE_MAX)
        value = ADMM_WIRE_MAX;

    float scaled = value * ADMM_WIRE_SCALE;
    scaled += (scaled >= 0.0f) ? 0.5f : -0.5f;
    return (int16_t)scaled;
}

inline float admm_wire_decode(int16_t raw) {
    return (float)raw / ADMM_WIRE_SCALE;
}

inline void can_send_float(uint8_t dest, uint8_t msg_type,
                           uint8_t sub, float value) {
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(msg_type, dest, LUMINAIRE);
    msg.can_dlc = 5;
    msg.data[0] = sub;
    memcpy(msg.data + 1, &value, 4);
    can_queue_tx(msg);
}

inline void can_send_byte(uint8_t dest, uint8_t msg_type,
                          uint8_t sub, uint8_t val) {
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(msg_type, dest, LUMINAIRE);
    msg.can_dlc = 2;
    msg.data[0] = sub;
    msg.data[1] = val;
    can_queue_tx(msg);
}

inline void can_send_sub(uint8_t dest, uint8_t msg_type, uint8_t sub) {
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(msg_type, dest, LUMINAIRE);
    msg.can_dlc = 1;
    msg.data[0] = sub;
    can_queue_tx(msg);
}

inline void can_send_admm(uint8_t dest, uint8_t iter, const float values[4]) {
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(MSG_ADMM, dest, LUMINAIRE);
    msg.can_dlc = 7;
    msg.data[0] = iter;

    int16_t q1 = admm_wire_encode(values[1]);
    int16_t q2 = admm_wire_encode(values[2]);
    int16_t q3 = admm_wire_encode(values[3]);
    memcpy(msg.data + 1, &q1, sizeof(q1));
    memcpy(msg.data + 3, &q2, sizeof(q2));
    memcpy(msg.data + 5, &q3, sizeof(q3));
    can_queue_tx(msg);
}
