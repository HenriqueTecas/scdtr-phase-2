// All serial commands for the SCDTR luminaire controller
const float OCC_REF[3] = {10.0f, 20.0f, 30.0f};
void commands(char *buffer)
{
    char command, sub_command, x_variable;
    int luminaire_index;
    float value;
    command = buffer[0];

    switch (command)
    {

    // ── u <i> <val> : Set duty cycle directly (open-loop) ─────────────────────
    case 'u':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            serial_duty_cycle = constrain(value, 0.0f, 1.0f);
            if (!pid.get_feedback())
                analogWrite(LED_PIN, (int)(serial_duty_cycle * PWM_MAX));
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;

    // ── r <i> <val> : Set illuminance reference [LUX] ─────────────────────────
    case 'r':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            r = value;
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;

    // ── o <i> <val> : Set occupancy state ─────────────────────────────────────
    case 'o':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            int mode = constrain((int)value, 0, 2);
            pid.set_occupancy(mode);
            r = OCC_REF[mode];
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            Serial.printf("ack: occupancy = %d  r = %.1f LUX\n", mode, r);
        }
        else
            Serial.println("err");
        break;

    // ── a <i> <val> : Set anti-windup mode ────────────────────────────────────
    //   0 = off
    //   1 = back-calculation
    //   2 = stop-integrating
    case 'a':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            pid.set_anti_windup((int)value);
            Serial.printf("ack: anti_windup = %d\n", pid.get_anti_windup());
        }
        else
            Serial.println("err");
        break;

    // ── f <i> <val> : Set feedback on(1)/off(0) ───────────────────────────────
    case 'f':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            serial_duty_cycle = pid.get_duty_cycle(); // bumpless hand-off open→closed
            if ((int)value == 1)
                apply_feedforward(r); // seed integrator before closing loop
            pid.set_feedback((int)value);
            Serial.println("ack");
        }
        else
            Serial.println("err");
        break;

    // ── s <x> <i> : Start streaming variable x for luminaire i ───────────────
    case 's':
        std::sscanf(buffer, "%c %c %d", &command, &x_variable, &luminaire_index);
        if (LUMINAIRE == luminaire_index)
        {
            if (x_variable == 'y')
                stream_y = 1;
            else if (x_variable == 'u')
                stream_u = 1;
            else if (x_variable == 'j')
                stream_j = 1;
        }
        break;

    // ── S <x> <i> : Stop streaming variable x ─────────────────────────────────
    case 'S':
        std::sscanf(buffer, "%c %c %d", &command, &x_variable, &luminaire_index);
        if (LUMINAIRE == luminaire_index)
        {
            if (x_variable == 'y')
                stream_y = 0;
            else if (x_variable == 'u')
                stream_u = 0;
            else if (x_variable == 'j')
                stream_j = 0;
        }
        Serial.println("ack");
        break;

    // ── R : Reset all performance metrics ────────────────────────────────────
    case 'R':
        resetMetrics();
        Serial.println("ack: metrics reset");
        break;


    // ── c : Calibration + gain-tuning commands ────────────────────────────────
    //   c b <val>    set ldr_b manually while watching live LUX
    //   c m <val>    set ldr_m slope
    //   c g          calibrate system gain (background + K)
    //   c s [steps]  duty-cycle sweep 0→1
    //   c ?          print all calibration + gain parameters
    //   --- PID gain tuning (your commands) ---
    //   c p <i> <val>   set kp
    //   c i <i> <val>   set ki
    //   c B <i> <val>   set setpoint weight b
    //   c t <i> <val>   set back-calculation gain kt
    case 'c':
        std::sscanf(buffer, "%c %c", &command, &sub_command);

        switch (sub_command)
        {
        // ── c b <val> : set b and immediately print measured LUX ──────────────
        case 'b':
        {
            float new_b;
            if (std::sscanf(buffer + 3, "%f", &new_b) == 1)
            {
                ldr_b = new_b;
                float lux_now = measureLux();
                Serial.printf("ack: ldr_b = %.4f  →  current LUX = %.2f\n",
                              ldr_b, lux_now);
            }
            else
            {
                // No value given: just print current LUX with the stored b
                float lux_now = measureLux();
                Serial.printf("ldr_b = %.4f  →  current LUX = %.2f\n",
                              ldr_b, lux_now);
            }
            break;
        }

        // ── c m <val> : set ldr_m slope ───────────────────────────────────────
        case 'm':
        {
            float new_m;
            if (std::sscanf(buffer + 3, "%f", &new_m) == 1)
            {
                ldr_m = new_m;
                Serial.printf("ack: ldr_m = %.4f\n", ldr_m);
            }
            else
            {
                Serial.println("err: usage  c m <value>");
            }
            break;
        }

        // ── c g : calibrate system gain inside the box ────────────────────────
        case 'g':
            calibrate_system_gain();
            break;

        // ── c s [steps] : duty-cycle sweep ────
        case 's':
        {
            int steps = 10;
            if (strlen(buffer) > 3)
                std::sscanf(buffer + 3, "%d", &steps);
            sweep(constrain(steps, 2, 50));
            break;
        }

        // ── c ? : print all calibration parameters ────────────────────────────
        case '?':
            Serial.println("--- Calibration Parameters ---");
            Serial.printf("  ldr_m      = %.4f\n", ldr_m);
            Serial.printf("  ldr_b      = %.4f\n", ldr_b);
            Serial.printf("  background = %.2f LUX\n", sys_background);
            Serial.printf("  gain K     = %.2f LUX/duty\n", sys_gain);
            Serial.printf("  calibrated = %s\n", calibrated ? "yes" : "no");
            Serial.printf("  anti_windup= %d  (0=off, 1=back-calc, 2=stop-int)\n",
                          pid.get_anti_windup());
            Serial.printf("  kp=%.3f  ki=%.3f  b=%.3f  kt=%.3f\n",
                          pid.get_kp(), pid.get_ki(), pid.get_b(), pid.get_kt());
            break;

        // ── c p <i> <val> : set kp ────────────────────────────────────────────
        case 'p':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_kp(val);
                Serial.printf("ack: kp = %.4f\n", pid.get_kp());
            }
            else
                Serial.println("err");
            break;
        }

        // ── c i <i> <val> : set ki ────────────────────────────────────────────
        case 'i':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_ki(val);
                Serial.printf("ack: ki = %.4f\n", pid.get_ki());
            }
            else
                Serial.println("err");
            break;
        }

        // ── c B <i> <val> : set setpoint weight b (capital B) ─────────────────
        case 'B':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_b(val);
                Serial.printf("ack: b = %.4f\n", pid.get_b());
            }
            else
                Serial.println("err");
            break;
        }

        // ── c t <i> <val> : set back-calculation gain kt ──────────────────────
        case 't':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_kt(val);
                Serial.printf("ack: kt = %.4f\n", pid.get_kt());
            }
            else
                Serial.println("err");
            break;
        }

        // ── c w <i> <val> : set bumpless transfer on(1)/off(0) ───────────────
        case 'w':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_bumpless((int)val);
                Serial.printf("ack: bumpless = %d\n", pid.get_bumpless());
            }
            else
                Serial.println("err");
            break;
        }

        // ── c f <i> <val> : set filter mode  0=none  1=mean  2=median ────────
        case 'f':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                filter_mode = (int)val;
                Serial.printf("ack: filter_mode = %d\n", filter_mode);
            }
            else
                Serial.println("err");
            break;
        }

        default:
            Serial.println("err -> unknown calibration sub-command");
            break;
        }
        break;

    // ── g : Get variable ───────────────────────────────────────────────────────
    case 'g':
        std::sscanf(buffer, "%c %c %d", &command, &sub_command, &luminaire_index);
        switch (sub_command)
        {
        case 'u': // duty cycle
            if (LUMINAIRE == luminaire_index)
                Serial.printf("u %d %.4f\n", LUMINAIRE,
                              pid.get_feedback() ? pid.get_duty_cycle() : serial_duty_cycle);
            break;

        case 'r': // illuminance reference
            if (LUMINAIRE == luminaire_index)
                Serial.printf("r %d %.2f\n", LUMINAIRE, r);
            break;

        case 'y': // measured illuminance
            if (LUMINAIRE == luminaire_index)
                Serial.printf("y %d %.4f\n", LUMINAIRE, lux_value);
            break;

        case 'v': // ADC voltage
            if (LUMINAIRE == luminaire_index)
            {
                float v_adc = (analogRead(LDR_PIN) / (float)ADC_MAX) * VCC;
                Serial.printf("v %d %.4f\n", LUMINAIRE, v_adc);
            }
            break;

        case 'o': // occupancy
            if (LUMINAIRE == luminaire_index)
                Serial.printf("o %d %d\n", LUMINAIRE, pid.get_occupancy());
            break;

        case 'a': // anti-windup mode
            if (LUMINAIRE == luminaire_index)
                Serial.printf("a %d %d\n", LUMINAIRE, pid.get_anti_windup());
            break;

        case 'f': // feedback state
            if (LUMINAIRE == luminaire_index)
                Serial.printf("f %d %d\n", LUMINAIRE, pid.get_feedback());
            break;

        case 'd': // external (disturbance) illuminance estimate
            if (LUMINAIRE == luminaire_index)
            {
                float d_ext = lux_value - sys_gain * duty_cycle;
                if (d_ext < 0.0f)
                    d_ext = 0.0f;
                Serial.printf("d %d %.4f\n", LUMINAIRE, d_ext);
            }
            break;

        case 'p': // instantaneous power [W]
            if (LUMINAIRE == luminaire_index)
                Serial.printf("p %d %.4f\n", LUMINAIRE, pid.get_u() * MAXIMUM_POWER);
            break;

        case 't': // elapsed time [s]
            if (LUMINAIRE == luminaire_index)
                Serial.printf("t %d %.3f\n", LUMINAIRE, micros() * 1e-6f);
            break;

        case 'b':
        { // last-minute buffer
            std::sscanf(buffer, "%c %c %c %d",
                        &command, &sub_command, &x_variable, &luminaire_index);
            if (LUMINAIRE == luminaire_index)
            {
                buffer_y = 0;
                buffer_u = 0;
                buffer_read_size = last_min_buf.size();
                buffer_read_counter = 0;
                if (x_variable == 'y')
                {
                    buffer_y = 1;
                    Serial.printf("b y %d ", LUMINAIRE);
                }
                else if (x_variable == 'u')
                {
                    buffer_u = 1;
                    Serial.printf("b u %d ", LUMINAIRE);
                }
                else
                {
                    Serial.println("err -> use y or u");
                }
            }
            break;
        }

        case 'E': // cumulative energy [J]
            if (LUMINAIRE == luminaire_index)
                Serial.printf("E %d %.4f\n", LUMINAIRE, energy_consumption);
            break;

        case 'V': // average visibility error
            if (LUMINAIRE == luminaire_index)
                Serial.printf("V %d %.6f\n", LUMINAIRE,
                              sample_count ? visibility_error / (double)sample_count : 0.0);
            break;

        case 'F': // average flicker error [s^-1]
            if (LUMINAIRE == luminaire_index)
                Serial.printf("F %d %.6f\n", LUMINAIRE,
                              sample_count ? (flicker_error / (double)sample_count) / pid.h : 0.0);
            break;

        default:
            Serial.println("err -> unknown get sub-command");
            break;
        }
        break;

    // ── h : Help ───────────────────────────────────────────────────────────────
    case 'h':
        Serial.println("=== SCDTR Command Reference ===");
        Serial.println("Control:");
        Serial.println("  r <i> <lux>      set illuminance reference");
        Serial.println("  u <i> <duty>     set duty cycle [0-1] (open-loop)");
        Serial.println("  f <i> <0|1>      feedback off/on");
        Serial.println("  o <i> <0|1|2>    occupancy: 0=off 1=low 2=high");
        Serial.println("  a <i> <0|1|2>    anti-windup: 0=off 1=back-calc 2=stop-int");
        Serial.println("Calibration & Tuning (all under 'c'):");
        Serial.println("  c b [val]        set/view ldr_b + show live LUX");
        Serial.println("  c m <val>        set ldr_m slope");
        Serial.println("  c g              calibrate system gain (run inside box)");
        Serial.println("  c s [steps]      duty sweep 0->1, CSV for plot_sweep.py");
        Serial.println("  c p <i> <val>    set kp");
        Serial.println("  c i <i> <val>    set ki");
        Serial.println("  c B <i> <val>    set setpoint weight b");
        Serial.println("  c t <i> <val>    set back-calc gain kt");
        Serial.println("  c w <i> <0|1>    bumpless transfer off/on");
        Serial.println("  c f <i> <0|1|2>  filter: 0=none 1=mean 2=median");
        Serial.println("  c ?              show all calibration + gain parameters");
        Serial.println("Get:");
        Serial.println("  g y <i>          measured LUX");
        Serial.println("  g u <i>          duty cycle");
        Serial.println("  g r <i>          reference LUX");
        Serial.println("  g v <i>          ADC voltage");
        Serial.println("  g d <i>          estimated disturbance LUX");
        Serial.println("  g p <i>          power [W]");
        Serial.println("  g t <i>          elapsed time [s]");
        Serial.println("  g b y <i>        last-minute LUX buffer");
        Serial.println("  g b u <i>        last-minute duty buffer");
        Serial.println("  g E/V/F <i>      energy / visibility / flicker metrics");
        Serial.println("Stream:");
        Serial.println("  s y <i>          start LUX stream");
        Serial.println("  S y <i>          stop  LUX stream");
        Serial.println("  s u <i>          start duty stream");
        Serial.println("  S u <i>          stop  duty stream");
        Serial.println("  s j <i>          start jitter stream [us]");
        Serial.println("  S j <i>          stop  jitter stream");
        break;

    default:
        Serial.println("err -> unknown command (type 'h' for help)");
        break;
    }
}
