#ifndef METRICS_H
#define METRICS_H

#include "Arduino.h"

// ─── Externals (defined in main) ──────────────────────────────────────────────
extern float sys_gain;
extern float sys_background;
extern const float MAXIMUM_POWER;

// ─── Performance metrics ──────────────────────────────────────────────────────
static double energy_consumption = 0.0;
static double visibility_error = 0.0;
static double flicker_error = 0.0;
static float u_prev1 = 0.0f;
static float u_prev2 = 0.0f;

// Suppress flicker accumulation for N samples after a reference step
const int FLICKER_EXCLUDE_SAMPLES = 50; // 50 × 10 ms = 0.5 s
static int flicker_holdoff = 0;

static unsigned long sample_count = 0;

static void updateMetrics(float r_ref, float d, float dt)
{
    sample_count++;
    // Energy [J]
    energy_consumption += (double)d * MAXIMUM_POWER * dt;

    // Visibility error: use duty-cycle estimate per spec
    float l_estimated = sys_background + sys_gain * d;
    float vis = r_ref - l_estimated;
    if (vis > 0.0f)
        visibility_error += vis;

    // Flicker: skip transients after reference changes
    if (flicker_holdoff > 0)
    {
        flicker_holdoff--;
    }
    else
    {
        float diff1 = d - u_prev1;
        float diff2 = u_prev1 - u_prev2;
        if (diff1 * diff2 < 0.0f)
            flicker_error += fabsf(diff1) + fabsf(diff2);
    }

    u_prev2 = u_prev1;
    u_prev1 = d;
}

static void resetMetrics()
{
    energy_consumption = 0.0;
    visibility_error = 0.0;
    flicker_error = 0.0;
    u_prev1 = u_prev2 = 0.0f;
    flicker_holdoff = 0;
    sample_count = 0;
}

#endif