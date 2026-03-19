#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// can_comms.h  –  Low-level CAN-BUS layer for Joy-It MCP2515 on Pico SPI0
// SCDTR Phase 2 – Session 4
//
// CAN-ID layout (11-bit standard frame):
//   bits[10:4] = msg_type  (7 bits – lower value = higher bus priority)
//   bits[ 3:0] = src_node  (4 bits – luminaire index 1/2/3)
//
// Frame layout:
//   data[0]            = destination node (0x00=broadcast; bit7=continuation)
//   data[1 .. dlc-1]   = payload (up to 7 bytes per frame)
//
// Two-frame strings: supports commands/responses up to 14 characters.
// Session 5: swap compile-time LUMINAIRE assignment for boot-negotiated index.
// ─────────────────────────────────────────────────────────────────────────────

#include "mcp2515/mcp2515.h"        // autowp arduino-pico MCP2515 library
#include "pico/unique_id.h" // pico_get_unique_board_id()

// ── MCP2515 object ─────────────────────────────────────────────────────────
// Joy-It module wired to SPI0:  CS=17  MOSI=19  MISO=16  SCK=18
// If your MCP2515 module has a 16 MHz crystal change setBitrate() below:
//   can0.setBitrate(CAN_1000KBPS, MCP_16MHZ);
MCP2515 can0{spi0, 17, 19, 16, 18, 10000000};

// ── Node address (assigned dynamically in can_init via flash ID) ───────────
extern int LUMINAIRE;
uint8_t node_address = 0;

// ── Message type constants ─────────────────────────────────────────────────
// Lower value → higher bus priority (CAN arbitration).
#define MSG_ID_ANNOUNCE    0x00  // boot: flash-signature broadcast for address negotiation
#define MSG_HELLO          0x01  // boot: "I am alive"
#define MSG_LUX_BROADCAST  0x02  // periodic: lux + duty + ref (10 Hz)
#define MSG_CMD_FORWARD    0x03  // hub → remote: serial command string
#define MSG_CMD_RESPONSE   0x04  // remote → hub: response string

#define CAN_BROADCAST_ADDR 0x00  // dest=0x00 → accepted by all nodes

// ── CAN-ID encode / decode ─────────────────────────────────────────────────
inline uint32_t make_can_id(uint8_t type, uint8_t src){
    return ((uint32_t)(type & 0x7F) << 4) | (src & 0x0F);
}
inline uint8_t can_id_type(uint32_t id){ return (id >> 4) & 0x7F; }
inline uint8_t can_id_src (uint32_t id){ return  id       & 0x0F; }

// ── Internal: build and send one CAN data frame ───────────────────────────
static MCP2515::ERROR _send_frame(uint8_t type, uint8_t dest,
                                   const uint8_t *pay, uint8_t plen)
{
    struct can_frame f;
    f.can_id  = make_can_id(type, node_address);
    f.can_dlc = 1 + min((int)plen, 7);
    f.data[0] = dest;
    for (int i = 0; i < f.can_dlc - 1; i++)
        f.data[i + 1] = (i < plen) ? pay[i] : 0;
    return can0.sendMessage(&f);
}

// ── Internal: send a string, splitting across two frames when > 7 chars ───
// Sets bit7 of dest in the first frame to signal "continuation follows".
static MCP2515::ERROR _send_string(uint8_t type, uint8_t dest, const char *s)
{
    size_t len = strlen(s);
    if (len <= 7)
        return _send_frame(type, dest, (const uint8_t *)s, (uint8_t)len);
    // First frame: mark continuation bit in dest byte
    MCP2515::ERROR e = _send_frame(type, dest | 0x80,
                                   (const uint8_t *)s, 7);
    if (e != MCP2515::ERROR_OK) return e;
    // Second frame: remainder (up to 7 more bytes)
    return _send_frame(type, dest,
                       (const uint8_t *)(s + 7), (uint8_t)min((int)len - 7, 7));
}

// ── Public send helpers ────────────────────────────────────────────────────

// Broadcast once at boot to announce this node.
inline MCP2515::ERROR can_send_hello(){
    uint8_t p = node_address;
    return _send_frame(MSG_HELLO, CAN_BROADCAST_ADDR, &p, 1);
}

// Hub → remote: forward a serial command string (max 14 chars).
inline MCP2515::ERROR can_send_cmd(uint8_t dest, const char *cmd){
    return _send_string(MSG_CMD_FORWARD, dest, cmd);
}

// Remote → hub: send a response string back (max 14 chars).
inline MCP2515::ERROR can_send_resp(uint8_t hub_addr, const char *resp){
    return _send_string(MSG_CMD_RESPONSE, hub_addr, resp);
}

// Periodic state broadcast: lux (×10→uint16), duty (×65535→uint16),
// ref (×10→uint16).  All nodes receive this; hub uses it for caching.
MCP2515::ERROR can_broadcast_lux(float lux, float duty, float ref)
{
    uint8_t d[6];
    uint16_t L = (uint16_t)constrain(lux  * 10.0f, 0.f, 65535.f);
    uint16_t D = (uint16_t)(duty * 65535.0f);
    uint16_t R = (uint16_t)constrain(ref  * 10.0f, 0.f, 65535.f);
    d[0]=L>>8; d[1]=L&0xFF;
    d[2]=D>>8; d[3]=D&0xFF;
    d[4]=R>>8; d[5]=R&0xFF;
    return _send_frame(MSG_LUX_BROADCAST, CAN_BROADCAST_ADDR, d, 6);
}

// ── Received frame wrapper ─────────────────────────────────────────────────
struct CanRxMsg {
    uint8_t type;
    uint8_t src;
    uint8_t dest;         // raw data[0]; bit7 = continuation flag
    uint8_t payload[7];
    uint8_t payload_len;
    bool    valid;
};

// Non-blocking poll – takes one frame from the MCP2515 RX FIFO.
// Returns valid=false immediately when the RX buffer is empty.
CanRxMsg can_poll()
{
    CanRxMsg m{};
    struct can_frame f;
    if (can0.readMessage(&f) != MCP2515::ERROR_OK) return m;
    m.type        = can_id_type(f.can_id);
    m.src         = can_id_src(f.can_id);
    m.dest        = (f.can_dlc > 0) ? f.data[0] : 0;
    m.payload_len = (f.can_dlc > 1) ? (uint8_t)(f.can_dlc - 1) : 0;
    for (int i = 0; i < m.payload_len; i++) m.payload[i] = f.data[i + 1];
    m.valid = true;
    return m;
}

// True when a message is addressed to this node or to all nodes.
inline bool can_is_for_me(const CanRxMsg &m){
    uint8_t d = m.dest & 0x7F;   // strip continuation flag
    return (d == node_address) || (d == CAN_BROADCAST_ADDR);
}

// ── Dynamic node-address assignment via flash unique ID ────────────────────
// Broadcasts this Pico's 4-byte flash signature once per second for up to
// 60 seconds, collecting peers' signatures.  Exits early once all 3 nodes
// are seen.  Smallest signature → node 1, next → node 2, largest → node 3.
#define CAN_DISCOVER_NODES   3    // total nodes in the network
#define CAN_DISCOVER_TIMEOUT 60   // absolute max seconds before giving up
#define CAN_DISCOVER_IDLE    15   // exit early if no new peer found for this many seconds

static void can_assign_id()
{
    // Read this Pico's 8-byte flash unique ID (last 4 bytes are most unique).
    pico_unique_board_id_t board_id;
    pico_get_unique_board_id(&board_id);
    uint32_t own_sig;
    memcpy(&own_sig, &board_id.id[4], 4);

    uint32_t sigs[CAN_DISCOVER_NODES] = {};
    int      n_sigs   = 0;
    int      idle_sec = 0;   // seconds since last new peer was found
    sigs[n_sigs++]    = own_sig;

    Serial.printf("[CAN] discovery started  sig=0x%08X  waiting for %d nodes...\n",
                  own_sig, CAN_DISCOVER_NODES);

    // One broadcast per second, then listen for the rest of that second.
    // Sending once per second keeps TX error counter well below bus-off.
    for (int sec = 0; sec < CAN_DISCOVER_TIMEOUT; sec++) {
        int prev_n = n_sigs;

        _send_frame(MSG_ID_ANNOUNCE, CAN_BROADCAST_ADDR,
                    (const uint8_t *)&own_sig, 4);

        unsigned long until = millis() + 1000;
        while ((long)(millis() - until) < 0) {
            CanRxMsg m = can_poll();
            if (!m.valid || m.type != MSG_ID_ANNOUNCE || m.payload_len < 4) continue;
            uint32_t their_sig;
            memcpy(&their_sig, m.payload, 4);
            bool seen = false;
            for (int i = 0; i < n_sigs; i++) if (sigs[i] == their_sig) { seen = true; break; }
            if (!seen && n_sigs < CAN_DISCOVER_NODES) {
                sigs[n_sigs++] = their_sig;
                Serial.printf("[CAN] found peer 0x%08X  (%d/%d)\n",
                              their_sig, n_sigs, CAN_DISCOVER_NODES);
            }
        }

        if (n_sigs == CAN_DISCOVER_NODES) break;  // all nodes seen

        if (n_sigs > prev_n) {
            idle_sec = 0;   // new peer found this second – reset idle counter
        } else {
            idle_sec++;
            if (idle_sec >= CAN_DISCOVER_IDLE) {
                Serial.printf("[CAN] no new peers for %ds, stopping discovery\n", CAN_DISCOVER_IDLE);
                break;
            }
        }
        Serial.printf("[CAN] %ds  seen %d/%d nodes\n", sec + 1, n_sigs, CAN_DISCOVER_NODES);
    }

    // Sort ascending so the smallest signature always maps to node 1.
    for (int i = 0; i < n_sigs - 1; i++)
        for (int j = i + 1; j < n_sigs; j++)
            if (sigs[j] < sigs[i]) { uint32_t t = sigs[i]; sigs[i] = sigs[j]; sigs[j] = t; }

    // Own rank (1-indexed) becomes the permanent node address.
    for (int i = 0; i < n_sigs; i++) {
        if (sigs[i] == own_sig) { node_address = (uint8_t)(i + 1); break; }
    }

    LUMINAIRE = (int)node_address;
    Serial.printf("[CAN] assigned node=%d  (found %d/%d peers)\n",
                  node_address, n_sigs - 1, CAN_DISCOVER_NODES - 1);
}

// ── Initialisation ─────────────────────────────────────────────────────────
// Call once in setup(), after Serial.begin().
void can_init()
{
    if (can0.reset()                         != MCP2515::ERROR_OK){ Serial.println("[CAN] reset failed");   return; }
    if (can0.setBitrate(CAN_1000KBPS, MCP_16MHZ) != MCP2515::ERROR_OK){ Serial.println("[CAN] bitrate failed"); return; }
    if (can0.setNormalMode()                 != MCP2515::ERROR_OK){ Serial.println("[CAN] mode failed");    return; }

    can_assign_id();   // sets node_address and LUMINAIRE

    // Safety fallback: if no peers found, default to node 1
    if (node_address == 0) {
        node_address = 1;
        LUMINAIRE    = 1;
        Serial.println("[CAN] no peers found, defaulting to node 1");
    }

    // Re-init MCP2515 to flush TX buffers jammed by unACKed discovery frames.
    // During single-node discovery every TX frame is never ACKed, so the
    // MCP2515 TX error counter rises and all three TX buffers end up stuck
    // (ERROR_ALLTXBUSY / error 2).  A full reset clears them cleanly.
    if (can0.reset()                             != MCP2515::ERROR_OK) { Serial.println("[CAN] post-discovery reset failed");   return; }
    if (can0.setBitrate(CAN_1000KBPS, MCP_16MHZ) != MCP2515::ERROR_OK) { Serial.println("[CAN] post-discovery bitrate failed"); return; }
    if (can0.setNormalMode()                     != MCP2515::ERROR_OK) { Serial.println("[CAN] post-discovery mode failed");    return; }

    Serial.printf("[CAN] ready – node=%d\n", node_address);
    can_send_hello();  // announce presence on the bus
}