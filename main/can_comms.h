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
// MSG_ADMM
#define SUB_ADMM_U_OPT  0x01

// ── Globals from main.ino ─────────────────────────────────────────────────────
extern MCP2515  can0;
extern int      LUMINAIRE;
extern queue_t  can_tx_queue;

// ── Frame builders ────────────────────────────────────────────────────────────
inline void can_send_float(uint8_t dest, uint8_t msg_type,
                           uint8_t sub, float value) {
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(msg_type, dest, LUMINAIRE);
    msg.can_dlc = 5;
    msg.data[0] = sub;
    memcpy(msg.data + 1, &value, 4);
    queue_try_add(&can_tx_queue, &msg);
}

inline void can_send_byte(uint8_t dest, uint8_t msg_type,
                          uint8_t sub, uint8_t val) {
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(msg_type, dest, LUMINAIRE);
    msg.can_dlc = 2;
    msg.data[0] = sub;
    msg.data[1] = val;
    queue_try_add(&can_tx_queue, &msg);
}

inline void can_send_sub(uint8_t dest, uint8_t msg_type, uint8_t sub) {
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(msg_type, dest, LUMINAIRE);
    msg.can_dlc = 1;
    msg.data[0] = sub;
    queue_try_add(&can_tx_queue, &msg);
}