#ifndef PID_H
#define PID_H

#include "Arduino.h"

#define ANTI_WINDUP_OFF 0
#define ANTI_WINDUP_BACK_CALC 1
#define ANTI_WINDUP_STOP_INT 2

class PID
{
public:
    float h;

    void init(int LUMINAIRE)
    {
        // ── Per-luminaire tuning ───────────────────────────────────────────────
        // Each node has a different static gain k_ii (measured during distributed
        // calibration) so their open-loop gains differ. The PI gains are tuned
        // individually in Part 1 — load the correct set here.
        //
        // TODO: replace placeholder values with your actual Part 1 results.
        // The switch uses LUMINAIRE index (1/2/3) set during boot from UID_TABLE.
        // If a new board is added, add a case here and to UID_TABLE in main.ino.
        switch (LUMINAIRE)
        {
        case 1:
            kp = 0.028f; // TODO: replace with node 1 tuned kp
            ki = 0.155f; // TODO: replace with node 1 tuned ki
            kt = 5.0f;
            b = 1.0f;
            break;
        case 2:
            kp = 0.028f; // TODO: replace with node 2 tuned kp
            ki = 0.155f; // TODO: replace with node 2 tuned ki
            kt = 5.0f;
            b = 1.0f;
            break;
        case 3:
            kp = 0.028f; // TODO: replace with node 3 tuned kp
            ki = 0.155f; // TODO: replace with node 3 tuned ki
            kt = 5.0f;
            b = 1.0f;
            break;
        default:
            // Unknown luminaire index — fall back to safe conservative gains.
            // This should never happen if UID_TABLE is filled correctly.
            kp = 0.020f;
            ki = 0.100f;
            kt = 5.0f;
            b = 1.0f;
            break;
        }

        u_proportional = 0;
        u_integral = 0;
        u = 0;
        anti_windup = ANTI_WINDUP_STOP_INT;
        bumpless_transfer = 1;
        feedback = 1;
        occupancy = 0;
        kp_old = kp;
        b_old = b;
        h = 0.01f;
    }

    float compute_control(float r, float y)
    {
        float error = r - y;
        float v;
        if (bumpless_transfer)
        {
            u_integral += kp_old * (b_old * r - y) - kp * (b * r - y);
            kp_old = kp;
            b_old = b;
        }
        u_proportional = kp * (b * r - y);
        v = u_proportional + u_integral;
        u = constrain(v, 0.0f, 1.0f);
        switch (anti_windup)
        {
        case ANTI_WINDUP_OFF:
            u_integral += ki * h * error;
            break;
        case ANTI_WINDUP_BACK_CALC:
            u_integral += ki * h * error + kt * h * (u - v);
            break;
        case ANTI_WINDUP_STOP_INT:
        default:
        {
            bool sat_high = (v >= 1.0f) && (error > 0.0f);
            bool sat_low = (v <= 0.0f) && (error < 0.0f);
            if (!sat_high && !sat_low)
                u_integral += ki * h * error;
        }
        break;
        }
        return u;
    }

    void set_anti_windup(int v) { anti_windup = v; }
    int get_anti_windup() { return anti_windup; }
    void set_occupancy(int v) { occupancy = v; }
    int get_occupancy() { return occupancy; }
    void set_feedback(int v) { feedback = v; }
    int get_feedback() { return feedback; }
    void set_bumpless(int v) { bumpless_transfer = v; }
    int get_bumpless() { return bumpless_transfer; }
    float get_u() { return u; }
    float get_duty_cycle() { return u; }
    void set_integrator(float v) { u_integral = v; }
    float get_integrator() { return u_integral; }
    void set_kp(float v) { kp = v; }
    float get_kp() { return kp; }
    void set_ki(float v) { ki = v; }
    float get_ki() { return ki; }
    void set_b(float v) { b = v; }
    float get_b() { return b; }
    void set_kt(float v) { kt = v; }
    float get_kt() { return kt; }

private:
    float kp, ki, kt, b;
    float u, u_proportional, u_integral;
    int occupancy, anti_windup, feedback, bumpless_transfer;
    float kp_old, b_old;
};

#endif