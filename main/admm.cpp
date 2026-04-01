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
int admm_recv_count[ADMM_N + 1] = {0};
bool admm_running = false;

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

    // Warm-start: initialise u and ū from the feedforward (open-loop) estimate.
    // This avoids unnecessary iterations when the system is near steady state.
    // With u = ū, the initial λ = 0 is consistent (no residual).
    float u_ff = feedforward(L_raw);
    for (int j = 1; j <= ADMM_N; j++)
    {
        admm_u[j] = u_ff;
        admm_u_avg[j] = u_ff;
    }

    // Seed receive buffers with the same estimate.
    for (int i = 1; i <= ADMM_N; i++)
    {
        for (int j = 1; j <= ADMM_N; j++)
            admm_recv[i][j] = u_ff;
        admm_recv_count[i] = 0;
    }

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

// ─────────────────────────────────────────────────────────────────────────────
// QP objective — slide 10 simplified form
//
//   g(u) = (1/2)ρ·u^T·u − u^T·z_i
//
// Used only to rank feasible candidates against each other.
// Constants that are the same for all candidates (terms in ū and λ) are
// dropped — they cancel when comparing, so this is sufficient.
// ─────────────────────────────────────────────────────────────────────────────
static double qp_cost(const float u[], const float z_i[])
{
    double val = 0.0;
    for (int j = 1; j <= ADMM_N; j++)
        val += 0.5 * ADMM_RHO * u[j] * u[j] - (double)u[j] * z_i[j];
    return val;
}

// ── State machine state ───────────────────────────────────────────────────────
AdmmStage admm_stage = AdmmStage::IDLE;
int admm_iter = 0;

void admm_start()
{
    admm_iter = 0;
    admm_running = true;
    admm_stage = AdmmStage::PRIMAL_UPDATE;
}

float admm_result()
{
    return admm_u[LUMINAIRE];
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
        // Reset peer receive buffers at the start of each iteration so that
        // only messages from the current iteration are counted.  This must
        // happen before the broadcast below; because loop() now calls
        // admm_tick() before process_can_messages(), any peer messages that
        // arrive after this reset will be accumulated correctly.
        for (int p = 0; p < n_other_nodes; p++)
        {
            int nd = other_nodes[p];
            admm_recv_count[nd] = 0;
            for (int j = 1; j <= ADMM_N; j++)
                admm_recv[nd][j] = 0.0f;
        }

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

        if (feasible(u_unc))
        {
            for (int j = 1; j <= ADMM_N; j++)
                admm_u[j] = u_unc[j];
        }
        else
        {
            float best[ADMM_N + 1];
            double best_cost = 1e30;
            bool found = false;

            auto try_cand = [&](float cand[])
            {
                if (!feasible(cand))
                    return;
                double c = qp_cost(cand, z_i);
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
                for (int j = 1; j <= ADMM_N; j++)
                    admm_u[j] = best[j];
        }

        for (int j = 1; j <= ADMM_N; j++)
            can_send_float(BROADCAST, MSG_ADMM, (uint8_t)j, admm_u[j]);

        for (int j = 1; j <= ADMM_N; j++)
            admm_recv[LUMINAIRE][j] = admm_u[j];
        admm_recv_count[LUMINAIRE] = ADMM_N;

        admm_stage = AdmmStage::WAIT_PEERS;
        return false;
    }

    case AdmmStage::WAIT_PEERS:
    {
        for (int p = 0; p < n_other_nodes; p++)
            if (admm_recv_count[other_nodes[p]] < ADMM_N)
                return false;
        admm_stage = AdmmStage::DUAL_UPDATE;
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
        bool converged = (sqrtf(res_sq) < ADMM_EPS);

        if (converged || admm_iter >= ADMM_MAXITER)
        {
            Serial.printf("[ADMM] %s k=%d  u_ii=%.4f\n",
                          converged ? "converged" : "maxiter",
                          admm_iter, admm_u[LUMINAIRE]);
            admm_running = false;
            admm_stage = AdmmStage::DONE;
            return true;
        }

        admm_stage = AdmmStage::PRIMAL_UPDATE;
        return false;
    }

    default:
        return false;
    }
}