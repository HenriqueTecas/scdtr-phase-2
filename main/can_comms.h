#pragma once

#include "mcp2515.h"

// ── Message types ────────────────────────────────────────────────────────────
#define MSG_LUX     0x01
#define MSG_DUTY    0x02
#define MSG_SET_R   0x03
#define MSG_SET_OCC 0x04
#define MSG_GET     0x05
#define MSG_REPLY   0x06
#define MSG_LAMBDA  0x20
#define MSG_U_OPT   0x21
#define MSG_ACK     0xFF

//Reboot
#define MSG_REBOOT 0x07
// ── New set-command message types (Part 2 hub forwarding) ────────────────────
#define MSG_SET_DUTY    0x08   // set open-loop duty     data[1..4]=float
#define MSG_SET_FBCK    0x09   // set feedback on/off    data[1]=uint8
#define MSG_SET_AW      0x0A   // set anti-windup mode   data[1]=uint8
#define MSG_SET_OCC_HI  0x0B   // set HIGH occ bound     data[1..4]=float
#define MSG_SET_OCC_LO  0x0C   // set LOW  occ bound     data[1..4]=float
#define MSG_SET_COST    0x0D   // set energy cost        data[1..4]=float
#define MSG_STREAM_ON   0x0E   // start stream  data[1]=variable char
#define MSG_STREAM_OFF  0x0F   // stop  stream  data[1]=variable char


// Wake-up handshake
#define MSG_SYN      0x10   // "I am here" — broadcast on boot
#define MSG_SYN_ACK  0x11   // "I see you, here I am"
#define MSG_ACK_DONE 0x12   // "I see you back, handshake complete"
#define MSG_CAL_READY 0x13

// Distributed calibration
#define MSG_CAL_ON   0x30   // "I am turning my LED on, measure me"
#define MSG_CAL_LUX  0x31   // "Here is my LUX reading while you were on"
#define MSG_CAL_DONE 0x32   // "I am done, next node go"

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
