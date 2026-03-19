#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// hub.h  –  Hub routing + CanPrint + per-node state cache
// SCDTR Phase 2 – Session 4
//
// What this file provides:
//
//  CanPrint  – Arduino Print subclass that writes over CAN instead of Serial.
//              Passed to commands() when executing a remotely-forwarded cmd.
//
//  hub_forward_command()  – called from commands.ino when a command targets a
//              different node; wraps it in a MSG_CMD_FORWARD CAN frame.
//
//  hub_dispatch_msg()     – called in the main loop for every CAN message;
//              handles HELLO, LUX_BROADCAST, CMD_FORWARD, CMD_RESPONSE.
//
//  NodeState cache        – filled by LUX_BROADCAST frames; lets hub answer
//              "g y/u/r <i>" queries from cache (no CAN round-trip needed).
// ─────────────────────────────────────────────────────────────────────────────

#include "can_comms.h"

// ── CAN debugging counters (defined in main.cpp) ───────────────────────────
extern volatile uint32_t can_tx_count;
extern volatile uint32_t can_rx_count;
extern volatile uint32_t can_rx_hello;
extern volatile uint32_t can_rx_lux;
extern volatile uint32_t can_rx_cmd;

// ── CanPrint ──────────────────────────────────────────────────────────────
// A Print subclass that buffers output and sends it as MSG_CMD_RESPONSE
// CAN frames back to the requesting hub node.
//
// Usage in hub_dispatch_msg():
//   CanPrint cp(msg.src);     // reply to whoever sent the command
//   commands(_cmd_rx, cp);    // all out.print* calls go over CAN
//   cp.flush();               // send any trailing bytes
class CanPrint : public Print
{
    uint8_t _dest;
    char    _buf[32];
    uint8_t _len;

public:
    explicit CanPrint(uint8_t dest) : _dest(dest), _len(0) { _buf[0] = '\0'; }

    // Required base-class method – everything else in Print calls this.
    size_t write(uint8_t c) override
    {
        if (_len < 30) { _buf[_len++] = (char)c; _buf[_len] = '\0'; }
        if (c == '\n' || _len >= 28) flush();   // send on newline or when near-full
        return 1;
    }

    size_t write(const uint8_t *buf, size_t n) override
    {
        for (size_t i = 0; i < n; i++) write(buf[i]);
        return n;
    }

    // printf-style helper (Print base class lacks this).
    int printf(const char *fmt, ...) __attribute__((format(printf, 2, 3)))
    {
        char tmp[128];
        va_list a;
        va_start(a, fmt);
        vsnprintf(tmp, sizeof(tmp), fmt, a);
        va_end(a);
        print(tmp);
        return (int)strlen(tmp);
    }

    // Flush any remaining buffered bytes to CAN.
    void flush()
    {
        if (_len > 0) {
            can_send_resp(_dest, _buf);
            _len    = 0;
            _buf[0] = '\0';
        }
    }
};

// ── Per-node state cache ─────────────────────────────────────────────────
// Populated by incoming MSG_LUX_BROADCAST frames.
// Lets the hub node answer "g y/u/r <i>" from its own memory instead of
// making a CAN round-trip and blocking waiting for a reply.
#define MAX_NODES 4  // support up to 4 luminaires (indices 1–4)

static struct {
    float lux, duty, ref;
    bool  alive;
} _ns[MAX_NODES + 1] = {};   // 1-indexed; slot 0 unused

inline float hub_cached_lux (uint8_t n){ return (n >= 1 && n <= MAX_NODES) ? _ns[n].lux  : 0.f; }
inline float hub_cached_duty(uint8_t n){ return (n >= 1 && n <= MAX_NODES) ? _ns[n].duty : 0.f; }
inline float hub_cached_ref (uint8_t n){ return (n >= 1 && n <= MAX_NODES) ? _ns[n].ref  : 0.f; }
inline bool  hub_node_alive (uint8_t n){ return (n >= 1 && n <= MAX_NODES) && _ns[n].alive; }

// ── Multi-frame reassembly buffers ────────────────────────────────────────
static char    _cmd_rx[16]   = {};
static uint8_t _cmd_rx_len   = 0;
static uint8_t _cmd_rx_from  = 0;  // hub node that sent the command – we reply here

static char    _resp_rx[16]  = {};
static uint8_t _resp_rx_len  = 0;

// ── Forward declaration of commands() ─────────────────────────────────────
// Defined in commands.ino; signature changed for Phase 2.
void commands(char *buf, Print &out);

// ── Forward a serial command to a remote node via CAN ─────────────────────
// Called from commands.ino for any command whose luminaire_index != LUMINAIRE.
// The response will arrive asynchronously as MSG_CMD_RESPONSE and
// hub_dispatch_msg() will print it to Serial.
inline bool hub_forward_command(const char *cmd, uint8_t dest_node)
{
    MCP2515::ERROR err = can_send_cmd(dest_node, cmd);
    if (err == MCP2515::ERROR_ALLTXBUSY) {
        // TX buffers jammed by unACKed broadcasts. Reset and retry once.
        can0.reset();
        can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
        can0.setNormalMode();
        err = can_send_cmd(dest_node, cmd);
    }
    if (err != MCP2515::ERROR_OK) {
        Serial.printf("[CAN] forward to node %d failed (error %d)\n", (int)dest_node, (int)err);
        return false;
    }
    return true;
}

// ── Dispatch one received CAN message ────────────────────────────────────
// Call this in the main loop for every message returned by can_poll().
inline void hub_dispatch_msg(const CanRxMsg &msg)
{
    if (!msg.valid) return;
    
    can_rx_count++;  // Increment total received counter

    switch (msg.type) {

    // ── Another node announced itself at boot ────────────────────────────
    case MSG_HELLO:
        can_rx_hello++;
        if (msg.src >= 1 && msg.src <= MAX_NODES)
            _ns[msg.src].alive = true;
        Serial.printf("[CAN] node %d online\n", (int)msg.src);
        break;

    // ── Periodic state broadcast ─────────────────────────────────────────
    // Decode and store in the node state cache.
    case MSG_LUX_BROADCAST:
        can_rx_lux++;
        if (msg.src >= 1 && msg.src <= MAX_NODES && msg.payload_len >= 6) {
            uint16_t L = ((uint16_t)msg.payload[0] << 8) | msg.payload[1];
            uint16_t D = ((uint16_t)msg.payload[2] << 8) | msg.payload[3];
            uint16_t R = ((uint16_t)msg.payload[4] << 8) | msg.payload[5];
            _ns[msg.src].lux   = L / 10.0f;
            _ns[msg.src].duty  = D / 65535.0f;
            _ns[msg.src].ref   = R / 10.0f;
            _ns[msg.src].alive = true;
        }
        break;

    // ── Remote command forwarded by the hub – execute locally ─────────────
    // Reassemble across frames (if needed), then call commands().
    // Output is routed back to the originating hub via CanPrint.
    case MSG_CMD_FORWARD:
        can_rx_cmd++;
        if (!can_is_for_me(msg)) break;
        {
            bool more = (msg.dest & 0x80) != 0;   // continuation flag
            _cmd_rx_from = msg.src;
            for (int i = 0; i < msg.payload_len && _cmd_rx_len < 14; i++)
                _cmd_rx[_cmd_rx_len++] = (char)msg.payload[i];

            if (!more) {
                // Trim null padding that CAN fixed-length frames may add
                while (_cmd_rx_len > 0 && _cmd_rx[_cmd_rx_len - 1] == '\0')
                    _cmd_rx_len--;
                _cmd_rx[_cmd_rx_len] = '\0';

                CanPrint cp(_cmd_rx_from);
                commands(_cmd_rx, cp);
                cp.flush();
                _cmd_rx_len = 0;
            }
        }
        break;

    // ── Response from a remote node – print to PC Serial ──────────────────
    case MSG_CMD_RESPONSE:
        if (!can_is_for_me(msg)) break;
        {
            bool more = (msg.dest & 0x80) != 0;
            for (int i = 0; i < msg.payload_len && _resp_rx_len < 14; i++)
                _resp_rx[_resp_rx_len++] = (char)msg.payload[i];
            if (!more) {
                _resp_rx[_resp_rx_len] = '\0';
                Serial.print(_resp_rx);
                _resp_rx_len = 0;
            }
        }
        break;

    default:
        break;
    }
}
