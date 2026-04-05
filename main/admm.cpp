// ─── admm.cpp ─────────────────────────────────────────────────────────────────
// Consensus ADMM implementation.
// Every formula is derived directly from Module 19 slides.
// See admm.h for variable naming convention.
// ─────────────────────────────────────────────────────────────────────────────
#include "admm.h"
#include "calibration.h" // feedforward(), sys_background
#include "pid.h"

void process_can_messages(); // defined in main.ino

// ── Externals from main.ino ───────────────────────────────────────────────────
extern float coupling_gains[4][4]; // [sensor][led], 1-indexed
extern float energy_cost;
extern float ref_high, ref_low;
extern int LUMINAIRE;
extern int n_other_nodes;
extern uint8_t other_nodes[];
extern float sys_background;
extern PID pid;

// ── State ─────────────────────────────────────────────────────────────────────
int ADMM_MAXITER = 100;
float admm_u[ADMM_N + 1] = {0};
float admm_u_avg[ADMM_N + 1] = {0};
float admm_lambda[ADMM_N + 1] = {0};
float admm_k[ADMM_N + 1] = {0};
float admm_c[ADMM_N + 1] = {0};
float admm_d_bg = 0;
float admm_L = 0;
float admm_n_sq = 0;
float admm_m_sq = 0;
float admm_recv[ADMM_N + 1][ADMM_N + 1] = {{0}};
static bool admm_recv_seen[ADMM_N + 1][ADMM_N + 1] = {{false}};
int admm_recv_count[ADMM_N + 1] = {0};
static float admm_recv_next[ADMM_N + 1][ADMM_N + 1] = {{0}};
static bool admm_recv_next_seen[ADMM_N + 1][ADMM_N + 1] = {{false}};
static int admm_recv_next_count[ADMM_N + 1] = {0};
bool admm_running = false;

static void admm_seed_window(float buf[][ADMM_N + 1],
                             bool seen[][ADMM_N + 1],
                             int count[],
                             float seed)
{
    for (int i = 1; i <= ADMM_N; i++)
    {
        count[i] = 0;
        for (int j = 1; j <= ADMM_N; j++)
        {
            buf[i][j] = seed;
            seen[i][j] = false;
        }
    }
}

static void admm_store_component(float buf[][ADMM_N + 1],
                                 bool seen[][ADMM_N + 1],
                                 int count[],
                                 uint8_t src, uint8_t comp, float val)
{
    if (!seen[src][comp])
    {
        seen[src][comp] = true;
        count[src]++;
    }
    buf[src][comp] = val;
}

static void admm_promote_next_window()
{
    for (int p = 0; p < n_other_nodes; p++)
    {
        const int nd = other_nodes[p];
        admm_recv_count[nd] = admm_recv_next_count[nd];
        admm_recv_next_count[nd] = 0;

        for (int j = 1; j <= ADMM_N; j++)
        {
            if (admm_recv_next_seen[nd][j])
                admm_recv[nd][j] = admm_recv_next[nd][j];
            else
                admm_recv[nd][j] = admm_u_avg[j];

            admm_recv_seen[nd][j] = admm_recv_next_seen[nd][j];
            admm_recv_next[nd][j] = admm_u_avg[j];
            admm_recv_next_seen[nd][j] = false;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// admm_init
// Sets up all constants that depend on calibration data and occupancy state.
// Must be called after calibration completes and whenever occupancy or
// energy_cost changes (since L and c_i depend on them).
// ─────────────────────────────────────────────────────────────────────────────
void admm_init()
{

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
    int occ = pid.get_occupancy();
    float L_raw = (occ == 2) ? ref_high : (occ == 1) ? ref_low
                                                     : 0.0f;
    float max_achievable = admm_d_bg;
    for (int j = 1; j <= ADMM_N; j++)
        max_achievable += admm_k[j];
    if (occ == 0)
    {
        admm_L = 0.0f;
    }
    else
    {
        admm_L = constrain(L_raw, admm_d_bg, max_achievable);
    }

    // n_sq = ‖k_i‖² = Σⱼ k_ij²  — denominator in the slide 14 solution.
    admm_n_sq = 0.0f;
    for (int j = 1; j <= ADMM_N; j++)
        admm_n_sq += admm_k[j] * admm_k[j];

    // m_sq = ‖k_i‖² − k_ii²  — denominator in the slide 17/18 solutions.
    // It equals (A·A^T)^{-1} determinant when two constraints are active.
    admm_m_sq = admm_n_sq - admm_k[LUMINAIRE] * admm_k[LUMINAIRE];

    // Warm-start: own component uses feedforward; non-own components start at 0.5
    // (neutral prior). If non-own were initialised to u_ff, an unoccupied node
    // (L=0 → u_ff=0) would broadcast x[j]=0 for every peer component, dragging
    // the consensus ū[j] = (x_1[j]+x_2[j]+x_3[j])/3 to ~0 even when node j is
    // occupied and needs duty ≈ 0.5. Starting at 0.5 gives a neutral prior that
    // the occupied node can correct within a few iterations.
    float u_ff = feedforward(L_raw);
    for (int j = 1; j <= ADMM_N; j++)
        admm_lambda[j] = 0.0f; // Clear integrators to prevent windup carry-over
    admm_u[LUMINAIRE]     = u_ff;
    admm_u_avg[LUMINAIRE] = u_ff;
    for (int j = 1; j <= ADMM_N; j++)
    {
        if (j != LUMINAIRE)
        {
            admm_u[j]     = 0.5f;
            admm_u_avg[j] = 0.5f;
        }
    }

    // Seed receive windows: own row uses u_ff; peer rows use 0.5 so a timeout
    // falls back to a neutral estimate rather than zero.
    admm_seed_window(admm_recv, admm_recv_seen, admm_recv_count, 0.5f);
    admm_seed_window(admm_recv_next, admm_recv_next_seen, admm_recv_next_count, 0.5f);
    admm_recv[LUMINAIRE][LUMINAIRE]      = u_ff;
    admm_recv_next[LUMINAIRE][LUMINAIRE] = u_ff;

    Serial.printf("[ADMM INIT] occ=%d L_raw=%.2f L=%.2f d_bg=%.2f u_ff=%.4f\n",
                  occ, L_raw, admm_L, admm_d_bg, u_ff);
    for (int j = 1; j <= ADMM_N; j++)
        Serial.printf("[ADMM INIT] k[%d]=%.4f\n", j, admm_k[j]);
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
static bool feasible(const float u[])
{
    const float tol = 1e-4f;
    if (u[LUMINAIRE] < -tol || u[LUMINAIRE] > 1.0f + tol)
        return false;
    float k_dot_u = 0.0f;
    for (int j = 1; j <= ADMM_N; j++)
        k_dot_u += admm_k[j] * u[j];
    return k_dot_u >= admm_L - admm_d_bg - tol;
}

static float predicted_lux(const float u[])
{
    float lux = admm_d_bg;
    for (int j = 1; j <= ADMM_N; j++)
        lux += admm_k[j] * u[j];
    return lux;
}

static bool avg_hits_local_target(const float u[])
{
    if (admm_L <= 0.0f)
        return true;
    return predicted_lux(u) + ADMM_LUX_TOL >= admm_L;
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
static float qp_cost(const float u[], const float z_i[])
{
    float val = 0.0f;
    for (int j = 1; j <= ADMM_N; j++)
        val += 0.5f * ADMM_RHO * u[j] * u[j] - u[j] * z_i[j];
    return val;
}

// ── State machine state ───────────────────────────────────────────────────────
AdmmStage admm_stage = AdmmStage::IDLE;
int admm_iter = 0;
static unsigned long admm_wait_start_ms = 0;

void admm_start()
{
    admm_iter = 0;
    admm_running = true;
    admm_stage = AdmmStage::PRIMAL_UPDATE;
}

float admm_result()
{
    return admm_u_avg[LUMINAIRE];
}

void admm_receive(uint8_t src, uint8_t comp, uint8_t iter, float val)
{
    if (src < 1 || src > ADMM_N || comp < 1 || comp > ADMM_N)
    {
        Serial.printf("[ADMM RX DROP] src=%d iter=%d comp=%d val=%.4f (invalid)\n",
                      src, iter, comp, val);
        return;
    }

    if (iter == (uint8_t)admm_iter)
    {
        admm_store_component(admm_recv, admm_recv_seen, admm_recv_count,
                             src, comp, val);
        return;
    }

    if (iter == (uint8_t)(admm_iter + 1))
    {
        admm_store_component(admm_recv_next, admm_recv_next_seen,
                             admm_recv_next_count, src, comp, val);
        return;
    }

    Serial.printf("[ADMM RX DROP] src=%d iter=%d comp=%d val=%.4f (expect %d or %d)\n",
                  src, iter, comp, val, admm_iter, admm_iter + 1);
}

void admm_request(bool is_responder)
{
    Serial.printf("[ADMM REQ] stage=%d responder=%d\n", (int)admm_stage, is_responder);
    if (admm_stage != AdmmStage::IDLE && admm_stage != AdmmStage::DONE)
        return;
    if (!is_responder)
        can_send_sub(BROADCAST, MSG_CTRL, SUB_ADMM_TRIGGER);
    admm_init();
    admm_start();
}

bool admm_tick()
{
    switch (admm_stage)
    {

    case AdmmStage::IDLE:
    case AdmmStage::DONE:
        return false;

    case AdmmStage::PRIMAL_UPDATE:
    {
        float z_i[ADMM_N + 1];
        float k_dot_z = 0.0f;
        float z_ii = 0.0f;

        for (int j = 1; j <= ADMM_N; j++)
        {
            z_i[j] = ADMM_RHO * admm_u_avg[j] - admm_c[j] - admm_lambda[j];
            k_dot_z += admm_k[j] * z_i[j];
        }
        z_ii = z_i[LUMINAIRE];

        float u_unc[ADMM_N + 1];
        for (int j = 1; j <= ADMM_N; j++)
            u_unc[j] = (1.0f / ADMM_RHO) * z_i[j];

        // Clamp non-own duty estimates to [0,1].
        // Each u[j] for j≠LUMINAIRE is an estimate of node j's physical duty cycle,
        // which cannot leave [0,1]. Without clamping, after many iterations the dual
        // variables λ drive z[j]/ρ to large negative values, making predicted_lux()
        // artificially negative and causing ALL candidates (including C3 at own duty=1)
        // to appear infeasible. The fix is to clamp peers before feasibility evaluation.
        auto clamp_peers = [](float u[])
        {
            for (int j = 1; j <= ADMM_N; j++)
                if (j != LUMINAIRE)
                    u[j] = constrain(u[j], 0.0f, 1.0f);
        };
        clamp_peers(u_unc);

        if (feasible(u_unc))
        {
            for (int j = 1; j <= ADMM_N; j++)
                admm_u[j] = u_unc[j];
        }
        else
        {
            float best[ADMM_N + 1];
            float best_cost = 1e30f;
            bool found = false;

            auto try_cand = [&](float cand[])
            {
                clamp_peers(cand);
                if (!feasible(cand))
                    return;
                float c = qp_cost(cand, z_i);
                if (!found || c < best_cost)
                {
                    best_cost = c;
                    found = true;
                    for (int j = 1; j <= ADMM_N; j++)
                        best[j] = cand[j];
                }
            };

            {
                float f = (admm_d_bg - admm_L + (1.0f / ADMM_RHO) * k_dot_z) / admm_n_sq;
                float u_c1[ADMM_N + 1];
                for (int j = 1; j <= ADMM_N; j++)
                    u_c1[j] = (1.0f / ADMM_RHO) * z_i[j] - admm_k[j] * f;
                try_cand(u_c1);

                // C1b: peers clamped to [0,1], own duty set to minimum that still
                // meets the illuminance constraint.  Needed when the C1 formula
                // places a peer component above 1 (physically impossible); after
                // clamping, illuminance drops below L and C1 is declared infeasible,
                // forcing a wasteful fallback to C3 (own duty = 1).  C1b avoids this
                // by fixing peers at their physical maximum and solving the 1-D
                // problem for the minimum own duty.
                {
                    float cross = 0.0f;
                    for (int j = 1; j <= ADMM_N; j++)
                        if (j != LUMINAIRE)
                            cross += admm_k[j] * constrain(u_c1[j], 0.0f, 1.0f);
                    float min_own = (admm_k[LUMINAIRE] > 1e-6f)
                                    ? (admm_L - admm_d_bg - cross) / admm_k[LUMINAIRE]
                                    : 1.0f;
                    float u_c1b[ADMM_N + 1];
                    for (int j = 1; j <= ADMM_N; j++)
                        u_c1b[j] = constrain(u_c1[j], 0.0f, 1.0f);
                    u_c1b[LUMINAIRE] = constrain(min_own, 0.0f, 1.0f);
                    try_cand(u_c1b);
                }
            }
            {
                float u_c2[ADMM_N + 1];
                for (int j = 1; j <= ADMM_N; j++)
                    u_c2[j] = (1.0f / ADMM_RHO) * z_i[j];
                u_c2[LUMINAIRE] = 0.0f;
                try_cand(u_c2);
            }
            {
                float u_c3[ADMM_N + 1];
                for (int j = 1; j <= ADMM_N; j++)
                    u_c3[j] = (1.0f / ADMM_RHO) * z_i[j];
                u_c3[LUMINAIRE] = 1.0f;
                try_cand(u_c3);
            }
            if (admm_m_sq > 1e-6f)
            {
                {
                    float f17 = (admm_d_bg - admm_L + (1.0f / ADMM_RHO) * (k_dot_z - admm_k[LUMINAIRE] * z_ii)) / admm_m_sq;
                    float u_c12[ADMM_N + 1];
                    for (int j = 1; j <= ADMM_N; j++)
                        u_c12[j] = (1.0f / ADMM_RHO) * z_i[j] - admm_k[j] * f17;
                    u_c12[LUMINAIRE] = 0.0f;
                    try_cand(u_c12);
                }
                {
                    float f18 = (admm_d_bg - admm_L + admm_k[LUMINAIRE] + (1.0f / ADMM_RHO) * (k_dot_z - admm_k[LUMINAIRE] * z_ii)) / admm_m_sq;
                    float u_c13[ADMM_N + 1];
                    for (int j = 1; j <= ADMM_N; j++)
                        u_c13[j] = (1.0f / ADMM_RHO) * z_i[j] - admm_k[j] * f18;
                    u_c13[LUMINAIRE] = 1.0f;
                    try_cand(u_c13);
                }
            }
            if (found)
            {
                for (int j = 1; j <= ADMM_N; j++)
                    admm_u[j] = best[j];
            }
            else
            {
                // Best effort: if target is unreachable, saturate own duty at 1.0
                // and constrain neighbor predictions to [0,1].
                for (int j = 1; j <= ADMM_N; j++)
                {
                    admm_u[j] = constrain(u_unc[j], 0.0f, 1.0f);
                }
                admm_u[LUMINAIRE] = 1.0f;
            }
        }

        // Serial.printf("[ADMM TX] iter=%d  x_i=[%.4f, %.4f, %.4f]\n",
        //               admm_iter, admm_u[1], admm_u[2], admm_u[3]);
        can_send_admm(BROADCAST, (uint8_t)admm_iter, admm_u);

        for (int j = 1; j <= ADMM_N; j++)
        {
            admm_recv[LUMINAIRE][j] = admm_u[j];
            admm_recv_seen[LUMINAIRE][j] = true;
        }
        admm_recv_count[LUMINAIRE] = ADMM_N;

        admm_wait_start_ms = millis();
        admm_stage = AdmmStage::WAIT_PEERS;
        return false;
    }

    case AdmmStage::WAIT_PEERS:
    {
        bool all_ready = true;
        for (int p = 0; p < n_other_nodes; p++)
            if (admm_recv_count[other_nodes[p]] < ADMM_N)
                all_ready = false;

        if (all_ready)
        {
            admm_stage = AdmmStage::DUAL_UPDATE;
            return false;
        }

        unsigned long elapsed = millis() - admm_wait_start_ms;

        // Periodic status print every 50 ms
        static unsigned long last_status_ms = 0;
        if (millis() - last_status_ms >= 50)
        {
            last_status_ms = millis();
            Serial.printf("[ADMM WAIT] iter=%d  elapsed=%lums\n", admm_iter, elapsed);
            for (int p = 0; p < n_other_nodes; p++)
            {
                uint8_t nd = other_nodes[p];
                Serial.printf("  peer %d: recv_count=%d/%d\n",
                              nd, admm_recv_count[nd], ADMM_N);
            }
        }

        // Timeout: force DUAL_UPDATE with whatever we have so the loop doesn't
        // hang indefinitely, and print exactly which peer(s) failed to reply.
        if (elapsed >= ADMM_TIMEOUT)
        {
            Serial.printf("[ADMM TIMEOUT] iter=%d  after %lums — forcing DUAL_UPDATE\n",
                          admm_iter, elapsed);
            for (int p = 0; p < n_other_nodes; p++)
            {
                uint8_t nd = other_nodes[p];
                if (admm_recv_count[nd] < ADMM_N)
                    Serial.printf("  MISSING peer %d: only %d/%d components received\n",
                                  nd, admm_recv_count[nd], ADMM_N);
            }
            admm_stage = AdmmStage::DUAL_UPDATE;
        }
        return false;
    }

    case AdmmStage::DUAL_UPDATE:
    {
        for (int j = 1; j <= ADMM_N; j++)
        {
            float sum = 0.0f;
            for (int i = 1; i <= ADMM_N; i++)
                sum += admm_recv[i][j];
            admm_u_avg[j] = sum / ADMM_N;
        }

        for (int j = 1; j <= ADMM_N; j++)
            admm_lambda[j] += ADMM_RHO * (admm_u[j] - admm_u_avg[j]);

        admm_iter++;

        float res_sq = 0.0f;
        for (int j = 1; j <= ADMM_N; j++)
        {
            float diff = admm_u[j] - admm_u_avg[j];
            res_sq += diff * diff;
        }
        float avg_lux = predicted_lux(admm_u_avg);
        bool avg_feasible = avg_hits_local_target(admm_u_avg);

        // Two-tier stopping (matches the intent documented in admm.h):
        //   Soft stop — ADMM_MAXITER reached AND solution is feasible.
        //               Allows extra iterations if consensus hasn't yet met the
        //               illuminance target at iter = ADMM_MAXITER.
        //   Hard stop — ADMM_MAXITER_HARD reached unconditionally.
        //               Safety cap when feasibility is never achieved.
        //
        // NOTE: primal-residual-based early stopping (converged flag) is
        // intentionally omitted.  The residual u_i − ū can reach ~0 within
        // a few iterations because each node's dual variable absorbs its cost
        // offset in one step (λ → −c, making u_i = ū exactly), yet ū itself
        // has not yet reached the LP optimum — unoccupied peer components are
        // still elevated and cost-differentiated duties have not propagated.
        // Using only the MAXITER / MAXITER_HARD thresholds avoids this false
        // convergence while preserving the correct steady-state result.
        bool do_stop = (avg_feasible && admm_iter >= ADMM_MAXITER) ||
                       (admm_iter >= ADMM_MAXITER_HARD);
        if (do_stop)
        {
            Serial.printf("[ADMM] done k=%d  u_ii=%.4f  avg_lux=%.2f%s\n",
                          admm_iter, admm_u_avg[LUMINAIRE], avg_lux,
                          admm_iter >= ADMM_MAXITER_HARD ? " (hard-cap)" : "");
            admm_running = false;
            admm_stage = AdmmStage::DONE;
            return true;
        }

        // Promote packets that arrived one iteration early so we do not mix
        // iteration k and k+1 vectors or drop the early packets outright.
        admm_promote_next_window();

        admm_stage = AdmmStage::PRIMAL_UPDATE;
        return false;
    }

    default:
        return false;
    }
}
