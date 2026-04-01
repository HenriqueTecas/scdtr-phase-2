#pragma once
// ─── admm.h ───────────────────────────────────────────────────────────────────
// Consensus ADMM — notation follows Module 19 slides exactly.
//
// ── Variables (slide 5 / slide 10) ───────────────────────────────────────────
//
//   u_i[j]    x_i  : this node's proposed duty for node j  ∈ [0,1]
//                    the full primal vector (one entry per node)
//   u_avg[j]  ū    : consensus average for node j's duty   (slide 5 z-update)
//   lambda[j] λ_i  : dual variable vector, local to node i (slide 5 λ-update)
//   k[j]      k_ij : coupling gain — how node j's LED illuminates this desk
//   c[j]      c_i  : cost vector — nonzero only at j == LUMINAIRE (slide 9)
//   d_bg      d_i  : background illuminance at this desk (lux)
//   L         L_i  : illuminance lower bound from occupancy state
//   n_sq      ‖kᵢ‖²           = Σⱼ k[j]²          (slide 14 denominator)
//   m_sq      ‖kᵢ‖² − k_ii²   = n_sq − k[i]²       (slide 17/18 denominator)
//
// ── Slide 6 key result ────────────────────────────────────────────────────────
//   Σᵢ λᵢ^(k) = 0  always  →  ū^(k+1) = (1/N)·Σᵢ xᵢ^(k+1)  (plain average)
//   Consequence: no need to broadcast λ. Full xᵢ vector is broadcast.
//
// ── The three updates per iteration ──────────────────────────────────────────
//   Step 1  x-update  (slide 10): compute zᵢ = ρ·ū − cᵢ − λᵢ,
//                                 then solve min (1/2)ρu^Tu − u^Tzᵢ  s.t. u∈Cᵢ
//           6 candidates (slides 12–18): unconstrained + 5 boundary points
//   Step 2  broadcast full xᵢ to all peers via MSG_ADMM, collect their vectors
//   Step 3  ū-update  (Eq. 4):   ū[j] = (1/N)·Σᵢ x_i[j]
//           λ-update  (slide 5):  λᵢ += ρ·(xᵢ − ū)
// ─────────────────────────────────────────────────────────────────────────────

#include "Arduino.h"
#include "can_comms.h"

#define ADMM_N 3 // number of nodes in the network

// ── Tuning constants ──────────────────────────────────────────────────────────
static constexpr float ADMM_RHO = 0.07f;           // ρ  — penalty parameter
static constexpr float ADMM_EPS = 1e-4f;           // convergence threshold
static constexpr int ADMM_MAXITER = 20;            // max iterations
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

extern float admm_recv[ADMM_N + 1][ADMM_N + 1]; // [src][component]
extern int admm_recv_count[ADMM_N + 1];         // msgs received per node
extern bool admm_running;                        // true while consensus is active

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