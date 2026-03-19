#ifndef LUX_H
#define LUX_H
#include "Arduino.h"

// ─── LDR calibration parameters ───────────────────────────────────────────────
extern float ldr_m;
extern float ldr_b;

// ─── Hardware constants (defined in main) ─────────────────────────────────────
extern const int LDR_PIN;
extern const float VCC;
extern const float R_FIXED;
extern const int ADC_MAX;

// ─── Mean Filter ─────────────────────────────────────
const int NUM_SAMPLES = 64;
const float SAMPLE_DELAY_US = 100.f;

// ─── Median filter ────────────────────────────────────────────────────────────
#define MEDIAN_FILTER_SIZE 9
static int median_window[MEDIAN_FILTER_SIZE] = {0};
static int filter_mode = 1; // 0=none, 1=mean (default), 2=median

void median_push(int adc_val)
{
    memmove(median_window, median_window + 1, sizeof(int) * (MEDIAN_FILTER_SIZE - 1));
    median_window[MEDIAN_FILTER_SIZE - 1] = adc_val;
}

int median_read()
{
    int buf[MEDIAN_FILTER_SIZE];
    memcpy(buf, median_window, sizeof(median_window));
    // insertion sort
    for (int i = 1; i < MEDIAN_FILTER_SIZE; i++)
    {
        int key = buf[i], j = i - 1;
        while (j >= 0 && buf[j] > key)
        {
            buf[j + 1] = buf[j];
            j--;
        }
        buf[j + 1] = key;
    }
    return buf[MEDIAN_FILTER_SIZE / 2];
}

// ─── ADC reading with selectable filter ───────────────────────────────────────
static float readVoltage_mean()
{
    long sum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        sum += analogRead(LDR_PIN);
        delayMicroseconds((int)SAMPLE_DELAY_US);
    }
    return ((float)sum / NUM_SAMPLES / ADC_MAX) * VCC;
}

static float readVoltage_median()
{
    for (int i = 0; i < MEDIAN_FILTER_SIZE; i++)
    {
        median_push(analogRead(LDR_PIN));
        delayMicroseconds((int)SAMPLE_DELAY_US);
    }
    return ((float)median_read() / ADC_MAX) * VCC;
}

static float readVoltage()
{
    if (filter_mode == 2)
        return readVoltage_median();
    if (filter_mode == 1)
        return readVoltage_mean();
    return ((float)analogRead(LDR_PIN) / ADC_MAX) * VCC;
}

// ─── LUX conversion pipeline ──────────────────────────────────────────────────
static float voltageToResistance(float voltage)
{
    if (voltage <= 0.001f)
        return 1e9f;
    return R_FIXED * (VCC / voltage - 1.0f);
}

static float resistanceToLux(float resistance_ohm)
{
    if (resistance_ohm <= 0.0f)
        return 0.0f;
    return pow(10.0f, (log10(resistance_ohm) - ldr_b) / ldr_m);
}

static float measureLux()
{
    return resistanceToLux(voltageToResistance(readVoltage()));
}

#endif