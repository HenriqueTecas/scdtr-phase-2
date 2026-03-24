// ─── admm.cpp ─────────────────────────────────────────────────────────────────
// Consensus ADMM implementation.
// Every formula is derived directly from Module 19 slides.
// See admm.h for variable naming convention.
// ─────────────────────────────────────────────────────────────────────────────
#include "admm.h"
#include "calibration.h"  // feedforward(), sys_background
#include "pid.h"

void process_can_messages();  // defined in main.ino

// ── Externals from main.ino ───────────────────────────────────────────────────
extern float   coupling_gains[4][4]; // [sensor][led], 1-indexed
extern float   energy_cost;
extern float   ref_high, ref_low;
extern int     LUMINAIRE;
extern int     n_other_nodes;
extern uint8_t other_nodes[];
extern float   sys_background;
extern PID     pid;

// ── State ─────────────────────────────────────────────────────────────────────
float admm_u[ADMM_N + 1]      = {0};
float admm_u_avg[ADMM_N + 1]  = {0};
float admm_lambda[ADMM_N + 1] = {0};
float admm_k[ADMM_N + 1]      = {0};
float admm_c[ADMM_N + 1]      = {0};
float admm_d_bg                = 0;
float admm_L                   = 0;
float admm_n_sq                = 0;
float admm_m_sq                = 0;
float admm_recv      [ADMM_N + 1][ADMM_N + 1] = {{0}};
int   admm_recv_count[ADMM_N + 1]             = {0};
bool  admm_running                             = false;

// ─────────────────────────────────────────────────────────────────────────────
// admm_init
// Sets up all constants that depend on calibration data and occupancy state.
// Must be called after calibration completes and whenever occupancy or
// energy_cost changes (since L and c_i depend on them).
// ─────────────────────────────────────────────────────────────────────────────
void admm_init() {

    // k[j] = k_ij — coupling gain from node j's LED to THIS desk (slide 7).
    // Filled from the gain matrix measured during distributed calibration.
    for (int j = 1; j <= ADMM_N; j++)
        admm_k[j] = coupling_gains[LUMINAIRE][j];

    // c_i[j] — cost vector (slide 9): c_i = [0 … c_i … 0]^T.
    // Only this node's own duty cycle has a cost; others are zero because
    // node i cannot control node j's LED.
    for (int j = 1; j <= ADMM_N; j++)
        admm_c[j] = (j == LUMINAIRE) ? energy_cost : 0.0f;

    // λ_i = 0 at start — no constraint violation history yet (slide 5).
    for (int j = 1; j <= ADMM_N; j++)
        admm_lambda[j] = 0.0f;

    // d_i — background illuminance, measured during calibration (slide 7).
    admm_d_bg = sys_background;

    // L_i — illuminance lower bound from occupancy state.
    // Clamped to [d_bg, d_bg + Σk_j] so it is always physically achievable.
    int   occ   = pid.get_occupancy();
    float L_raw = (occ == 2) ? ref_high : (occ == 1) ? ref_low : 0.0f;
    float max_achievable = admm_d_bg;
    for (int j = 1; j <= ADMM_N; j++) max_achievable += admm_k[j];
    admm_L = constrain(L_raw, admm_d_bg, max_achievable);

    // n_sq = ‖k_i‖² = Σⱼ k_ij²  — denominator in the slide 14 solution.
    admm_n_sq = 0.0f;
    for (int j = 1; j <= ADMM_N; j++)
        admm_n_sq += admm_k[j] * admm_k[j];

    // m_sq = ‖k_i‖² − k_ii²  — denominator in the slide 17/18 solutions.
    // It equals (A·A^T)^{-1} determinant when two constraints are active.
    admm_m_sq = admm_n_sq - admm_k[LUMINAIRE] * admm_k[LUMINAIRE];

    // Warm-start: initialise u and ū from the feedforward (open-loop) estimate.
    // This avoids unnecessary iterations when the system is near steady state.
    // With u = ū, the initial λ = 0 is consistent (no residual).
    float u_ff = feedforward(L_raw);
    for (int j = 1; j <= ADMM_N; j++) {
        admm_u[j]     = u_ff;
        admm_u_avg[j] = u_ff;
    }

    // Seed receive buffers with the same estimate.
    for (int i = 1; i <= ADMM_N; i++) {
        for (int j = 1; j <= ADMM_N; j++)
            admm_recv[i][j] = u_ff;
        admm_recv_count[i] = 0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Feasibility check — slide 8 constraint set Cᵢ
//
//   C1: k_i^T · u ≥ L_i − d_i   (illuminance lower bound at this desk)
//   C2: u[LUMINAIRE] ≥ 0         (LED duty cannot be negative)
//   C3: u[LUMINAIRE] ≤ 1         (LED duty cannot exceed 100%)
//
// Only the own-index component is bounded by C2/C3 because node i only
// controls its own LED. The other components represent predictions about
// what the other nodes will do, not actual constraints on them here.
// ─────────────────────────────────────────────────────────────────────────────
static bool feasible(const float u[]) {
    const float tol = 1e-4f;
    if (u[LUMINAIRE] < -tol || u[LUMINAIRE] > 1.0f + tol)
        return false;
    float k_dot_u = 0.0f;
    for (int j = 1; j <= ADMM_N; j++)
        k_dot_u += admm_k[j] * u[j];
    return k_dot_u >= admm_L - admm_d_bg - tol;
}

// ─────────────────────────────────────────────────────────────────────────────
// QP objective — slide 10 simplified form
//
//   g(u) = (1/2)ρ·u^T·u − u^T·z_i
//
// Used only to rank feasible candidates against each other.
// Constants that are the same for all candidates (terms in ū and λ) are
// dropped — they cancel when comparing, so this is sufficient.
// ─────────────────────────────────────────────────────────────────────────────
static double qp_cost(const float u[], const float z_i[]) {
    double val = 0.0;
    for (int j = 1; j <= ADMM_N; j++)
        val += 0.5 * ADMM_RHO * u[j] * u[j] - (double)u[j] * z_i[j];
    return val;
}

// ─────────────────────────────────────────────────────────────────────────────
// admm_run
// Runs the full ADMM loop to convergence and returns the optimal duty
// for this node's LED, u_i[LUMINAIRE] ∈ [0,1].
// ─────────────────────────────────────────────────────────────────────────────
float admm_run() {

    admm_running = true;

    for (int iter = 0; iter < ADMM_MAXITER; iter++) {

        // ══════════════════════════════════════════════════════════════════════
        // STEP 1 — PRIMAL UPDATE  (slides 10–18)
        //
        // Build the intermediate vector z_i (slide 10):
        //   z_i[j] = ρ·ū[j] − c_i[j] − λ_i[j]
        //
        // This rewrites the x-update objective into the standard QP form
        //   min (1/2)ρ·u^T·u − u^T·z_i   s.t. u ∈ Cᵢ
        // ══════════════════════════════════════════════════════════════════════

        float z_i[ADMM_N + 1];
        float k_dot_z = 0.0f;     // k_i^T · z_i  (precomputed, used in slides 14–18)
        float z_ii    = 0.0f;     // z_i[LUMINAIRE] (precomputed, used in slides 17–18)

        for (int j = 1; j <= ADMM_N; j++) {
            z_i[j]  = ADMM_RHO * admm_u_avg[j] - admm_c[j] - admm_lambda[j];
            k_dot_z += admm_k[j] * z_i[j];
        }
        z_ii = z_i[LUMINAIRE];

        // ── Candidate 1: unconstrained minimum (slide 12) ─────────────────────
        // ∇g = ρ·u − z_i = 0  →  u* = ρ⁻¹·z_i
        // Expanded per component (slide 12):
        //   u_ij = ū[j] − ρ⁻¹·λ_i[j]            for j ≠ i
        //   u_ii = ū[i] − ρ⁻¹·λ_i[i] − ρ⁻¹·c_i  for j = i
        // (both follow from u = ρ⁻¹·z_i)
        float u_unc[ADMM_N + 1];
        for (int j = 1; j <= ADMM_N; j++)
            u_unc[j] = (1.0f / ADMM_RHO) * z_i[j];

        // If feasible, this is the global minimiser — strictly convex QP has
        // a unique minimum, and if it lies inside the feasible set it wins.
        if (feasible(u_unc)) {
            for (int j = 1; j <= ADMM_N; j++) admm_u[j] = u_unc[j];
            goto step2;
        }

        {
            // Track the best feasible candidate found so far.
            float  best[ADMM_N + 1];
            double best_cost = 1e30;
            bool   found     = false;

            // Inline helper: test candidate, keep if feasible and cheapest.
            auto try_cand = [&](float cand[]) {
                if (!feasible(cand)) return;
                double c = qp_cost(cand, z_i);
                if (!found || c < best_cost) {
                    best_cost = c; found = true;
                    for (int j = 1; j <= ADMM_N; j++) best[j] = cand[j];
                }
            };

            // ── Candidate 2: on C1 boundary (slide 14) ────────────────────────
            // Active constraint: k_i^T·u = L_i − d_i  (illuminance tight)
            // A = −k_i^T,  b = d_i − L_i
            // (A·A^T)⁻¹ = 1/‖k_i‖² = 1/n_sq
            //
            // Slide 14 solution:
            //   u* = ρ⁻¹·z_i − (k_i/‖k_i‖²)·(d_i − L_i + ρ⁻¹·k_i^T·z_i)
            {
                float f = (admm_d_bg - admm_L + (1.0f/ADMM_RHO) * k_dot_z)
                          / admm_n_sq;
                float u_c1[ADMM_N + 1];
                for (int j = 1; j <= ADMM_N; j++)
                    u_c1[j] = (1.0f/ADMM_RHO) * z_i[j] - admm_k[j] * f;
                try_cand(u_c1);
            }

            // ── Candidate 3: on C2 boundary (slide 15) ────────────────────────
            // Active constraint: u_ii = 0  (LED lower bound tight)
            // A = −e_i^T,  b = 0
            // (A·A^T)⁻¹ = 1  (scalar)
            //
            // Slide 15 solution:
            //   u_ij = ρ⁻¹·z_i[j]  for j ≠ i
            //   u_ii = 0
            {
                float u_c2[ADMM_N + 1];
                for (int j = 1; j <= ADMM_N; j++)
                    u_c2[j] = (1.0f/ADMM_RHO) * z_i[j];
                u_c2[LUMINAIRE] = 0.0f;
                try_cand(u_c2);
            }

            // ── Candidate 4: on C3 boundary (slide 16) ────────────────────────
            // Active constraint: u_ii = 1  (LED upper bound tight)
            // A = e_i^T,  b = 1
            // (A·A^T)⁻¹ = 1  (scalar)
            //
            // Slide 16 solution:
            //   u_ij = ρ⁻¹·z_i[j]  for j ≠ i
            //   u_ii = 1
            {
                float u_c3[ADMM_N + 1];
                for (int j = 1; j <= ADMM_N; j++)
                    u_c3[j] = (1.0f/ADMM_RHO) * z_i[j];
                u_c3[LUMINAIRE] = 1.0f;
                try_cand(u_c3);
            }

            if (admm_m_sq > 1e-6f) {

                // ── Candidate 5: C1 ∩ C2 boundary (slide 17) ─────────────────
                // Active: k_i^T·u = L_i−d_i  AND  u_ii = 0
                // A = [−k_i^T; −e_i^T],  b = [d_i−L_i; 0]
                // det(A·A^T) = ‖k_i‖²·1 − k_ii² = m_sq
                //
                // Slide 17 solution for j ≠ i:
                //   u_ij = ρ⁻¹·z_i[j]
                //          − k_ij/m_sq · (d_i − L_i)
                //          + ρ⁻¹·k_ij/m_sq · (−k_dot_z + k_ii·z_ii)
                // Factor common to all j:
                //   f17 = [ (d_i−L_i) + ρ⁻¹·(k_dot_z − k_ii·z_ii) ] / m_sq
                {
                    float f17 = (admm_d_bg - admm_L
                                + (1.0f/ADMM_RHO) * (k_dot_z - admm_k[LUMINAIRE] * z_ii))
                                / admm_m_sq;
                    float u_c12[ADMM_N + 1];
                    for (int j = 1; j <= ADMM_N; j++)
                        u_c12[j] = (1.0f/ADMM_RHO) * z_i[j] - admm_k[j] * f17;
                    u_c12[LUMINAIRE] = 0.0f;
                    try_cand(u_c12);
                }

                // ── Candidate 6: C1 ∩ C3 boundary (slide 18) ─────────────────
                // Active: k_i^T·u = L_i−d_i  AND  u_ii = 1
                // A = [−k_i^T; e_i^T],  b = [d_i−L_i; 1]
                // det(A·A^T) same = m_sq  (sign flip on row 2 doesn't change det)
                //
                // Slide 18 solution for j ≠ i:
                //   u_ij = ρ⁻¹·z_i[j]
                //          − k_ij·(d_i−L_i + k_ii)/m_sq
                //          + ρ⁻¹·k_ij/m_sq · (−k_dot_z + k_ii·z_ii)
                // Factor (same structure as f17, but b adds k_ii from the u_ii=1 term):
                //   f18 = [ (d_i−L_i + k_ii) + ρ⁻¹·(k_dot_z − k_ii·z_ii) ] / m_sq
                {
                    float f18 = (admm_d_bg - admm_L + admm_k[LUMINAIRE]
                                + (1.0f/ADMM_RHO) * (k_dot_z - admm_k[LUMINAIRE] * z_ii))
                                / admm_m_sq;
                    float u_c13[ADMM_N + 1];
                    for (int j = 1; j <= ADMM_N; j++)
                        u_c13[j] = (1.0f/ADMM_RHO) * z_i[j] - admm_k[j] * f18;
                    u_c13[LUMINAIRE] = 1.0f;
                    try_cand(u_c13);
                }
            }

            if (found)
                for (int j = 1; j <= ADMM_N; j++) admm_u[j] = best[j];
            // If no candidate is feasible (only possible if L was not clamped
            // correctly) keep the previous admm_u[] — already feasible.
        }

        step2:

        // ══════════════════════════════════════════════════════════════════════
        // STEP 2 — BROADCAST FULL x_i VECTOR, COLLECT PEERS' VECTORS
        //
        // Each node broadcasts ALL N components of its primal vector x_i.
        // This is required so that the consensus average ū_j = (1/N)·Σᵢ x_ij
        // can be computed correctly. Broadcasting only x_ii would make the
        // own-component residual always zero, preventing dual variable buildup
        // and causing premature convergence to u=0.
        // ══════════════════════════════════════════════════════════════════════

        // Broadcast all components: sub-type byte = component index j.
        for (int j = 1; j <= ADMM_N; j++)
            can_send_float(BROADCAST, MSG_ADMM, (uint8_t)j, admm_u[j]);

        // Store own values directly.
        for (int j = 1; j <= ADMM_N; j++)
            admm_recv[LUMINAIRE][j] = admm_u[j];
        admm_recv_count[LUMINAIRE] = ADMM_N;

        // Wait for all peers to send their full vectors (N messages each).
        // Flush any stale ADMM messages that arrived outside the receive
        // window (e.g. from the main loop's process_can_messages()), then
        // reset both counters AND data to avoid mixing iterations.
        {
            process_can_messages();           // drain queue first
            for (int p = 0; p < n_other_nodes; p++) {
                int nd = other_nodes[p];
                admm_recv_count[nd] = 0;
                for (int j = 1; j <= ADMM_N; j++)
                    admm_recv[nd][j] = 0.0f;  // clear stale data
            }

            unsigned long t0 = millis();
            int heard = 0;
            while (heard < n_other_nodes && millis() - t0 < ADMM_TIMEOUT) {
                process_can_messages();
                heard = 0;
                for (int p = 0; p < n_other_nodes; p++)
                    if (admm_recv_count[other_nodes[p]] >= ADMM_N) heard++;
            }
        }

        // ══════════════════════════════════════════════════════════════════════
        // STEP 3 — ū UPDATE  (Eq. 4)
        //
        //   ū_j^(k+1) = (1/N) · Σᵢ x_ij^(k+1)
        //
        // Because Σλ = 0, the slide 5 formula reduces to a plain average
        // of all nodes' primal vectors.
        // ══════════════════════════════════════════════════════════════════════
        for (int j = 1; j <= ADMM_N; j++) {
            float sum = 0.0f;
            for (int i = 1; i <= ADMM_N; i++)
                sum += admm_recv[i][j];
            admm_u_avg[j] = sum / ADMM_N;
        }

        // ══════════════════════════════════════════════════════════════════════
        // STEP 4 — λ UPDATE  (slide 5)
        //
        //   λ_i^(k+1) = λ_i^(k) + ρ·( x_i^(k+1) − ū^(k+1) )
        //
        // The residual x_i[j] − ū[j] measures how much this node's proposal
        // for node j disagrees with node j's own broadcast. λ accumulates
        // this disagreement and drives future x-updates to close the gap.
        // ══════════════════════════════════════════════════════════════════════
        for (int j = 1; j <= ADMM_N; j++)
            admm_lambda[j] += ADMM_RHO * (admm_u[j] - admm_u_avg[j]);

        // ── Convergence: full primal residual ‖x_i − ū‖ ─────────────────────
        float res_sq = 0.0f;
        for (int j = 1; j <= ADMM_N; j++) {
            float diff = admm_u[j] - admm_u_avg[j];
            res_sq += diff * diff;
        }
        float residual = sqrtf(res_sq);
        if (residual < ADMM_EPS) {
            Serial.printf("[ADMM] converged k=%d  u_ii=%.4f  lambda_ii=%.4f\n",
                          iter + 1, admm_u[LUMINAIRE], admm_lambda[LUMINAIRE]);
            break;
        }
    }

    admm_running = false;
    return admm_u[LUMINAIRE];
}
