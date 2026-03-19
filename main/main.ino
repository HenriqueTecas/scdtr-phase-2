#include <cstdio>
#include "pid.h"
#include "circular_buffer.h"
#include "lux.h"
#include "metrics.h"
#include "calibration.h"

// ─── Pin & hardware constants ──────────────────────────────────────────────────
const int LED_PIN = 15;
const int LDR_PIN = A0;

const int ADC_BITS = 12;
const int ADC_MAX = (1 << ADC_BITS) - 1; // 4095
const int PWM_BITS = 12;
const int PWM_MAX = (1 << PWM_BITS) - 1; // 4095
const int PWM_FREQ_HZ = 30000;

// ─── Physical constants ────────────────────────────────────────────────────────
const float VCC = 3.3f;
const float R_FIXED = 10000.0f;
// MAXIMUM_POWER = VF * IF where IF = (VCC - VF) / R2 = (3.3 - 3.1) / 47 = 4.26 mA
// VF = 3.1V (typical from datasheet), R2 = 47 Ohm
const float MAXIMUM_POWER = 0.0132f;

const int LUMINAIRE = 1;
float ldr_m = -0.8f;
float ldr_b = 6.3044f;

float sys_gain = 0.0f;
float sys_background = 0.0f;
bool calibrated = false;

// ─── Sampling ──────────────────────────────────────────────────────────────────
const unsigned long SAMPLE_PERIOD_US = 10000;
unsigned long last_sample_us = 0;

float jitter_us = 0.0f;
int stream_j = 0;

PID pid;
float r = 0.0f;                 // illuminance reference [LUX]
float lux_value = 0.0f;         // latest LUX measurement
float duty_cycle = 0.0f;        // current duty cycle [0,1]
float serial_duty_cycle = 0.0f; // duty cycle set manually via serial

int stream_y = 0;
int stream_u = 0;
int buffer_y = 0, buffer_u = 0, buffer_read_size = 0, buffer_read_counter = 0;

// For storing the last minute of data at 100 Hz: 60 s * 100 samples/s = 6000 samples
CircularBuffer<6000> last_min_buf;

void commands(char *buf);

// =============================================================================
// Non-blocking serial reader
// =============================================================================

void serial_command()
{
    const int BUFFER_SIZE = 128;
    static char buf[BUFFER_SIZE];
    static int buf_pos = 0;

    while (Serial.available() > 0)
    {
        char c = Serial.read();
        if (c == '\n' || c == '\r')
        {
            if (buf_pos > 0)
            {
                buf[buf_pos] = '\0';
                commands(buf);
                buf_pos = 0;
            }
        }
        else if (buf_pos < BUFFER_SIZE - 1)
        {
            buf[buf_pos++] = c;
        }
        else
        {
            buf_pos = 0; // overflow: discard
        }
    }
}

// =============================================================================
// Non-blocking buffer readout (one value per loop tick)
// =============================================================================

void handle_buffer_readout()
{
    if ((buffer_y || buffer_u) && buffer_read_counter < buffer_read_size)
    {
        buffer_data d = last_min_buf.pop();

        if (buffer_y)
            Serial.printf("%.4f", d.lux_value);
        else
            Serial.printf("%.4f", d.duty_cycle);

        buffer_read_counter++;
        if (buffer_read_counter < buffer_read_size)
            Serial.print(",");
        else
        {
            Serial.print("\n");
            buffer_y = 0;
            buffer_u = 0;
        }
    }
}

// =============================================================================
// Setup
// =============================================================================

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    analogReadResolution(ADC_BITS);
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_MAX);
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 0);

    pid.init(LUMINAIRE);
    pid.h = SAMPLE_PERIOD_US * 1e-6f; // 0.01 s

    Serial.println("=== SCDTR Luminaire Ready ===");
}

// =============================================================================
// Loop
// =============================================================================

void loop()
{
    unsigned long now = micros();

    // ── 100 Hz time-triggered control loop ────────────────────────────────────
    if (now - last_sample_us >= SAMPLE_PERIOD_US)
    {
        unsigned long actual_dt_us = now - last_sample_us;
        float dt = actual_dt_us * 1e-6f;
        jitter_us = fabsf((float)actual_dt_us - (float)SAMPLE_PERIOD_US);
        last_sample_us = now;

        // 1. Measure
        lux_value = measureLux();

        // 2. Control
        float u_raw;
        if (pid.get_feedback())
            u_raw = pid.compute_control(r, lux_value);
        else
            u_raw = serial_duty_cycle;

        // 3. Actuate
        u_raw = constrain(u_raw, 0.0f, 1.0f);
        duty_cycle = u_raw;
        analogWrite(LED_PIN, (int)(u_raw * PWM_MAX));

        // 4. Store in last-minute buffer
        buffer_data bd = {lux_value, duty_cycle};
        last_min_buf.push(bd);

        // 5. Update metrics
        updateMetrics(r, duty_cycle, dt);

        // 6. Stream if active
        unsigned long ts_ms = now / 1000UL;
        if (stream_y)
            Serial.printf("s y %d %.4f %lu\n", LUMINAIRE, lux_value, ts_ms);
        if (stream_u)
            Serial.printf("s u %d %.4f %lu\n", LUMINAIRE, duty_cycle, ts_ms);
        if (stream_j)
            Serial.printf("s j %d %.2f %lu\n", LUMINAIRE, jitter_us, ts_ms);
    }

    // ── Non-blocking background tasks ─────────────────────────────────────────
    serial_command();
    handle_buffer_readout();
}
