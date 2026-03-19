#pragma once

#include "mcp2515.h"

// ── Message types ────────────────────────────────────────────────────────────
#define MSG_LUX     0x01
#define MSG_DUTY    0x02
#define MSG_SET_R   0x03
#define MSG_SET_OCC 0x04
#define MSG_GET     0x05
#define MSG_REPLY   0x06
#define MSG_HELLO   0x10
#define MSG_READY   0x11
#define MSG_LAMBDA  0x20
#define MSG_U_OPT   0x21
#define MSG_ACK     0xFF

// ── Broadcast address ────────────────────────────────────────────────────────
#define BROADCAST   0x00

// ── CAN ID macros ───────────────────────────────────────────────────────────
// Bits [10:8]  = destination node index (0=broadcast, 1,2,3 = specific node)
// Bits [7:0]   = message type
#define MAKE_CAN_ID(dest, type)  (((uint32_t)(dest) << 8) | (type))
#define CAN_ID_DEST(id)          (((id) >> 8) & 0x07)
#define CAN_ID_TYPE(id)          ((id) & 0xFF)

// ── Global CAN instances (defined in main.ino) ──────────────────────────────
extern MCP2515 can0;
extern int LUMINAIRE;
extern uint8_t node_address;
#include "pico/mutex.h"
extern mutex_t spi_mutex;

// ── Helper functions for sending messages ────────────────────────────────────

// Send a float value to a destination node
// Convention: data[0] = source index, data[1..4] = float value
inline void can_send_float(uint8_t dest, uint8_t msg_type, float value) {
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(dest, msg_type);
    msg.can_dlc = 5;
    msg.data[0] = (uint8_t)LUMINAIRE;
    memcpy(msg.data + 1, &value, 4);
    mutex_enter_blocking(&spi_mutex);
    can0.sendMessage(&msg);
    mutex_exit(&spi_mutex);
}

// Send a single byte to a destination node
// Convention: data[0] = source index, data[1] = byte value
inline void can_send_byte(uint8_t dest, uint8_t msg_type, uint8_t byte_val) {
    struct can_frame msg;
    msg.can_id  = MAKE_CAN_ID(dest, msg_type);
    msg.can_dlc = 2;
    msg.data[0] = (uint8_t)LUMINAIRE;
    msg.data[1] = byte_val;
    mutex_enter_blocking(&spi_mutex);
    can0.sendMessage(&msg);
    mutex_exit(&spi_mutex);
}

// Broadcast this node's lux reading to all nodes
inline void can_broadcast_lux(float lux) {
    can_send_float(BROADCAST, MSG_LUX, lux);
}

// Broadcast this node's duty cycle to all nodes
inline void can_broadcast_duty(float duty) {
    can_send_float(BROADCAST, MSG_DUTY, duty);
}
