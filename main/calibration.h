#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "Arduino.h"
#include "lux.h"
#include "pid.h"

// ─── System gain calibration ────────────────────────────────
extern float sys_gain;
extern float sys_background;
extern bool calibrated;
extern const int LED_PIN;
extern const int PWM_MAX;
extern PID pid;
extern float r;

// ─── Calibration─────────────────────
// Command: 'c g'
static void calibrate_system_gain()
{
    Serial.println("--- System Gain Calibration ---");

    analogWrite(LED_PIN, 0);
    delay(5000);
    sys_background = measureLux();
    Serial.printf("Background = %.2f LUX\n", sys_background);

    float duties[] = {0.25f, 0.50f, 0.75f, 1.00f};
    float lux_at_full = 0.0f;
    Serial.println("Duty\tLUX");

    for (int i = 0; i < 4; i++)
    {
        analogWrite(LED_PIN, (int)(duties[i] * PWM_MAX));
        delay(5000);
        float lux = measureLux();
        Serial.printf("%.0f%%\t%.2f\n", duties[i] * 100, lux);
        if (i == 3)
            lux_at_full = lux;
    }

    sys_gain = lux_at_full - sys_background;
    calibrated = true;
    analogWrite(LED_PIN, 0);

    Serial.printf("Gain G = %.2f LUX/duty\n", sys_gain);
    Serial.println("--- Calibration Complete ---");
}

// ─── Duty-cycle sweep ─────────────────────────
// Command: 'c s [steps]'
static void sweep(int steps)
{
    steps = constrain(steps, 2, 50);
    Serial.println("duty,voltage,resistance_ohm,lux");
    for (int i = 0; i <= steps; i++)
    {
        float d = (float)i / steps;
        analogWrite(LED_PIN, (int)(d * PWM_MAX));
        delay(5000);
        float v = readVoltage();
        float res = voltageToResistance(v);
        float lux = resistanceToLux(res);
        Serial.printf("%.4f,%.4f,%.1f,%.4f\n", d, v, res, lux);
    }
    analogWrite(LED_PIN, 0);
    Serial.println("END");
}

// ─── Feedforward integrator seeding ──────────────────────────────────────────
// Inverts static model: duty_ff = (r - background) / G
// Returns normalized duty [0,1]
static float feedforward(float ref_lux)
{
    if (!calibrated || sys_gain <= 0.0f)
        return 0.0f;
    return constrain((ref_lux - sys_background) / sys_gain, 0.0f, 1.0f);
}

// Seeds the integrator at feedback enable only — not on reference changes.
static void apply_feedforward(float ref_lux)
{
    pid.set_integrator(feedforward(ref_lux));
}

#endif