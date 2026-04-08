#pragma once

#include "Arduino.h"
#include "can_comms.h"

#define ADMM_N 3 // number of nodes in the network

// ── Tuning constants ──────────────────────────────────────────────────────────
static constexpr float ADMM_RHO = 5.0f;            // ρ  — penalty parameter
extern int ADMM_MAXITER;                           // fixed ADMM iteration budget
static constexpr unsigned long ADMM_TIMEOUT = 150; // ms to wait for CAN peers (N msgs each)

// ── ADMM state (1-indexed, matching LUMINAIRE convention) ─────────────────────
extern float admm_u[ADMM_N + 1];      // x_i : primal vector (proposed duties)
extern float admm_u_avg[ADMM_N + 1];  // ū   : consensus average
extern float admm_lambda[ADMM_N + 1]; // λ_i : dual variable vector
extern float admm_k[ADMM_N + 1];      // k_ij: coupling gains to this desk
extern float admm_c[ADMM_N + 1];      // c_i : cost vector
extern float admm_d_bg;               // d_i : background lux
extern float admm_L;                  // L_i : illuminance lower bound
extern float admm_n_sq;               // ‖kᵢ‖²
extern float admm_m_sq;               // ‖kᵢ‖² − k_ii²

extern float admm_primal_res; // primal residual ‖u_i − ū‖
extern float admm_dual_res;   // dual residual ρ‖ū^k − ū^{k-1}‖

extern float admm_recv[ADMM_N + 1][ADMM_N + 1]; // [src][component]
extern int admm_recv_count[ADMM_N + 1];         // msgs received per node
extern bool admm_running;                       // true while consensus is active

// ── Consensus state machine ───────────────────────────────────────────────────
enum class AdmmStage : uint8_t
{
    IDLE,
    PRIMAL_UPDATE,
    WAIT_PEERS,
    DUAL_UPDATE,
    DONE
};
extern AdmmStage admm_stage;
extern int admm_iter;

// ── API ───────────────────────────────────────────────────────────────────────
void admm_init();
void admm_start();
bool admm_tick();
float admm_result();
void admm_request(bool is_responder);
void admm_receive(uint8_t src, uint8_t comp, uint8_t iter, float val);
