// ─────────────────────────────────────────────────────────────────────────────
// commands.ino  –  Serial command processor
// Phase 2 changes vs your Phase 1 commnands.ino:
//
//  1. Signature: void commands(char *buffer, Print &out)
//       • Called from serial_command()  →  out = Serial   (unchanged behaviour)
//       • Called from hub_dispatch_msg() →  out = CanPrint (routes reply over CAN)
//
//  2. All Serial.print / Serial.printf / Serial.println
//     → out.print / PRINTF(out,...) / out.println
//     (PRINTF macro below handles printf-style formatting for any Print object)
//
//  3. Commands for another luminaire:
//     Serial.println("err")  →  hub_forward_command(buffer, luminaire_index)
//
//  4. Table 3 commands added: O, U, C, g O, g U, g L, g C
//
//  5. 'o' command uses ref_high / ref_low (settable via O/U) instead of
//     the old hardcoded OCC_REF[] array.
//
//  Everything else is identical to your Phase 1 code.
// ─────────────────────────────────────────────────────────────────────────────

// ── Phase 2 globals (defined in main.ino) ────────────────────────────────
extern float ref_high;     // Table 3: HIGH occupancy lower bound [LUX]
extern float ref_low;      // Table 3: LOW  occupancy lower bound [LUX]
extern float energy_cost;  // Table 3: energy cost coefficient

// ── printf helper that works with any Print& ─────────────────────────────
// Arduino's Print base class has no printf(), so we snprintf into a temp
// buffer and call the universal print(const char*) method.
static char _pbuf[128];
#define PRINTF(out, ...) \
    do { snprintf(_pbuf, sizeof(_pbuf), __VA_ARGS__); (out).print(_pbuf); } while(0)

// =============================================================================
void commands(char *buffer, Print &out)
// =============================================================================
{
    char    command, sub_command, x_variable;
    int     luminaire_index;
    float   value;
    command = buffer[0];

    switch (command)
    {

    // ── u <i> <val> : Set duty cycle (open-loop) ─────────────────────────────
    case 'u':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index) {
            serial_duty_cycle = constrain(value, 0.0f, 1.0f);
            if (!pid.get_feedback())
                analogWrite(LED_PIN, (int)(serial_duty_cycle * PWM_MAX));
            out.println("ack");
        } else {
            hub_forward_command(buffer, (uint8_t)luminaire_index);
        }
        break;

    // ── r <i> <val> : Set illuminance reference [LUX] ────────────────────────
    case 'r':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index) {
            r = value;
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            out.println("ack");
        } else {
            hub_forward_command(buffer, (uint8_t)luminaire_index);
        }
        break;

    // ── o <i> <val> : Set occupancy state (0=off 1=low 2=high) ──────────────
    // Phase 2: uses ref_high / ref_low instead of the old hardcoded OCC_REF[].
    case 'o':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index) {
            int mode = constrain((int)value, 0, 2);
            pid.set_occupancy(mode);
            r = (mode == 2) ? ref_high : (mode == 1) ? ref_low : 0.0f;
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            PRINTF(out, "ack: occupancy=%d  r=%.1f LUX\n", mode, r);
        } else {
            hub_forward_command(buffer, (uint8_t)luminaire_index);
        }
        break;

    // ── a <i> <val> : Set anti-windup mode (0=off 1=back-calc 2=stop-int) ───
    case 'a':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index) {
            pid.set_anti_windup((int)value);
            PRINTF(out, "ack: anti_windup=%d\n", pid.get_anti_windup());
        } else {
            hub_forward_command(buffer, (uint8_t)luminaire_index);
        }
        break;

    // ── f <i> <val> : Set feedback on(1)/off(0) ──────────────────────────────
    case 'f':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index) {
            serial_duty_cycle = pid.get_duty_cycle();
            if ((int)value == 1)
                apply_feedforward(r);
            pid.set_feedback((int)value);
            out.println("ack");
        } else {
            hub_forward_command(buffer, (uint8_t)luminaire_index);
        }
        break;

    // ── s <x> <i> : Start streaming ──────────────────────────────────────────
    // Streaming is always local; we just set the flag on this node.
    case 's':
        std::sscanf(buffer, "%c %c %d", &command, &x_variable, &luminaire_index);
        if (LUMINAIRE == luminaire_index) {
            if      (x_variable == 'y') { stream_y = 1; out.println("ack"); }
            else if (x_variable == 'u') { stream_u = 1; out.println("ack"); }
            else if (x_variable == 'j') { stream_j = 1; out.println("ack"); }
            else out.println("err: use y, u or j");
        } else {
            PRINTF(out, "err: this node is %d, not %d\n", LUMINAIRE, luminaire_index);
        }
        break;

    // ── S <x> <i> : Stop streaming ───────────────────────────────────────────
    case 'S':
        std::sscanf(buffer, "%c %c %d", &command, &x_variable, &luminaire_index);
        if (LUMINAIRE == luminaire_index) {
            if      (x_variable == 'y') stream_y = 0;
            else if (x_variable == 'u') stream_u = 0;
            else if (x_variable == 'j') stream_j = 0;
        }
        out.println("ack");
        break;

    // ── R : Restart / reset metrics (Table 3) ────────────────────────────────
    case 'R':
        resetMetrics();
        r           = 0.0f;
        energy_cost = 1.0f;
        pid.set_occupancy(0);
        pid.set_feedback(0);
        analogWrite(LED_PIN, 0);
        out.println("ack");
        break;

    // ── O <i> <val> : Set HIGH-occupancy lower bound (Table 3) ───────────────
    case 'O':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index) {
            ref_high = value;
            out.println("ack");
        } else {
            hub_forward_command(buffer, (uint8_t)luminaire_index);
        }
        break;

    // ── U <i> <val> : Set LOW-occupancy lower bound (Table 3) ────────────────
    case 'U':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index) {
            ref_low = value;
            out.println("ack");
        } else {
            hub_forward_command(buffer, (uint8_t)luminaire_index);
        }
        break;

    // ── C <i> <val> : Set energy cost coefficient (Table 3) ──────────────────
    case 'C':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index) {
            energy_cost = value;
            out.println("ack");
        } else {
            hub_forward_command(buffer, (uint8_t)luminaire_index);
        }
        break;

    // ── c : Calibration + PID tuning commands ─────────────────────────────────
    case 'c':
        std::sscanf(buffer, "%c %c", &command, &sub_command);
        switch (sub_command)
        {
        case 'b': {
            float new_b;
            if (std::sscanf(buffer + 3, "%f", &new_b) == 1) {
                ldr_b = new_b;
                PRINTF(out, "ack: ldr_b=%.4f  →  LUX=%.2f\n", ldr_b, measureLux());
            } else {
                PRINTF(out, "ldr_b=%.4f  →  LUX=%.2f\n", ldr_b, measureLux());
            }
            break;
        }
        case 'm': {
            float new_m;
            if (std::sscanf(buffer + 3, "%f", &new_m) == 1) {
                ldr_m = new_m;
                PRINTF(out, "ack: ldr_m=%.4f\n", ldr_m);
            } else {
                out.println("err: usage  c m <value>");
            }
            break;
        }
        case 'g':
            calibrate_system_gain();   // long blocking procedure, prints to Serial directly
            break;
        case 's': {
            int steps = 10;
            if (strlen(buffer) > 3) std::sscanf(buffer + 3, "%d", &steps);
            sweep(constrain(steps, 2, 50));   // also prints to Serial directly
            break;
        }
        case '?':
            out.println("--- Calibration Parameters ---");
            PRINTF(out, "  ldr_m      = %.4f\n",   ldr_m);
            PRINTF(out, "  ldr_b      = %.4f\n",   ldr_b);
            PRINTF(out, "  background = %.2f LUX\n", sys_background);
            PRINTF(out, "  gain K     = %.2f LUX/duty\n", sys_gain);
            PRINTF(out, "  calibrated = %s\n",     calibrated ? "yes" : "no");
            PRINTF(out, "  anti_windup= %d  (0=off 1=back-calc 2=stop-int)\n", pid.get_anti_windup());
            PRINTF(out, "  kp=%.3f  ki=%.3f  b=%.3f  kt=%.3f\n",
                   pid.get_kp(), pid.get_ki(), pid.get_b(), pid.get_kt());
            PRINTF(out, "  ref_high=%.1f  ref_low=%.1f  cost=%.3f\n",
                   ref_high, ref_low, energy_cost);
            break;
        case 'p': { float val; int idx; std::sscanf(buffer+3,"%d %f",&idx,&val);
            if (LUMINAIRE==idx){ pid.set_kp(val); PRINTF(out,"ack: kp=%.4f\n",val); }
            else out.println("err"); break; }
        case 'i': { float val; int idx; std::sscanf(buffer+3,"%d %f",&idx,&val);
            if (LUMINAIRE==idx){ pid.set_ki(val); PRINTF(out,"ack: ki=%.4f\n",val); }
            else out.println("err"); break; }
        case 'B': { float val; int idx; std::sscanf(buffer+3,"%d %f",&idx,&val);
            if (LUMINAIRE==idx){ pid.set_b(val);  PRINTF(out,"ack: b=%.4f\n", val); }
            else out.println("err"); break; }
        case 't': { float val; int idx; std::sscanf(buffer+3,"%d %f",&idx,&val);
            if (LUMINAIRE==idx){ pid.set_kt(val); PRINTF(out,"ack: kt=%.4f\n",val); }
            else out.println("err"); break; }
        case 'w': { float val; int idx; std::sscanf(buffer+3,"%d %f",&idx,&val);
            if (LUMINAIRE==idx){ pid.set_bumpless((int)val); PRINTF(out,"ack: bumpless=%d\n",(int)val); }
            else out.println("err"); break; }
        case 'f': { float val; int idx; std::sscanf(buffer+3,"%d %f",&idx,&val);
            if (LUMINAIRE==idx){ filter_mode=(int)val; PRINTF(out,"ack: filter=%d\n",(int)val); }
            else out.println("err"); break; }
        default:
            out.println("err: unknown calibration sub-command");
            break;
        }
        break;

    // ── g : Get variable ───────────────────────────────────────────────────────
    case 'g':
        std::sscanf(buffer, "%c %c %d", &command, &sub_command, &luminaire_index);
        switch (sub_command)
        {
        // ── Measurements: serve from hub cache for remote nodes ───────────────
        case 'y':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "y %d %.4f\n", LUMINAIRE, lux_value);
            else if (hub_node_alive((uint8_t)luminaire_index))
                PRINTF(out, "y %d %.4f\n", luminaire_index, hub_cached_lux((uint8_t)luminaire_index));
            else
                hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'u':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "u %d %.4f\n", LUMINAIRE,
                       pid.get_feedback() ? pid.get_duty_cycle() : serial_duty_cycle);
            else if (hub_node_alive((uint8_t)luminaire_index))
                PRINTF(out, "u %d %.4f\n", luminaire_index, hub_cached_duty((uint8_t)luminaire_index));
            else
                hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'r':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "r %d %.2f\n", LUMINAIRE, r);
            else if (hub_node_alive((uint8_t)luminaire_index))
                PRINTF(out, "r %d %.2f\n", luminaire_index, hub_cached_ref((uint8_t)luminaire_index));
            else
                hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        // ── Other gets: local only or forward over CAN ────────────────────────
        case 'v':
            if (LUMINAIRE == luminaire_index) {
                float v = (analogRead(LDR_PIN) / (float)ADC_MAX) * VCC;
                PRINTF(out, "v %d %.4f\n", LUMINAIRE, v);
            } else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'o':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "o %d %d\n", LUMINAIRE, pid.get_occupancy());
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'a':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "a %d %d\n", LUMINAIRE, pid.get_anti_windup());
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'f':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "f %d %d\n", LUMINAIRE, pid.get_feedback());
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'd':
            if (LUMINAIRE == luminaire_index) {
                float d_ext = max(0.0f, lux_value - sys_gain * duty_cycle);
                PRINTF(out, "d %d %.4f\n", LUMINAIRE, d_ext);
            } else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'p':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "p %d %.4f\n", LUMINAIRE, pid.get_u() * MAXIMUM_POWER);
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 't':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "t %d %.3f\n", LUMINAIRE, micros() * 1e-6f);
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'b': {   // last-minute buffer – too large for CAN, hub/local only
            std::sscanf(buffer, "%c %c %c %d",
                        &command, &sub_command, &x_variable, &luminaire_index);
            if (LUMINAIRE == luminaire_index) {
                buffer_y = buffer_u = 0;
                buffer_read_size    = last_min_buf.size();
                buffer_read_counter = 0;
                if (x_variable == 'y') {
                    buffer_y = 1;
                    Serial.printf("b y %d ", LUMINAIRE);   // always Serial for bulk dump
                } else if (x_variable == 'u') {
                    buffer_u = 1;
                    Serial.printf("b u %d ", LUMINAIRE);
                } else {
                    out.println("err: use y or u");
                }
            } else {
                out.println("err: g b only on hub node");
            }
            break;
        }

        case 'E':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "E %d %.4f\n", LUMINAIRE, energy_consumption);
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'V':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "V %d %.6f\n", LUMINAIRE,
                       sample_count ? visibility_error / (double)sample_count : 0.0);
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'F':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "F %d %.6f\n", LUMINAIRE,
                       sample_count ? (flicker_error / (double)sample_count) / pid.h : 0.0);
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        // ── Table 3 getters ───────────────────────────────────────────────────
        case 'O':   // HIGH-occupancy lower bound
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "O %d %.2f\n", LUMINAIRE, ref_high);
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'U':   // LOW-occupancy lower bound
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "U %d %.2f\n", LUMINAIRE, ref_low);
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'L':   // current active lower bound
            if (LUMINAIRE == luminaire_index) {
                float lb = (pid.get_occupancy() == 2) ? ref_high :
                           (pid.get_occupancy() == 1) ? ref_low  : 0.0f;
                PRINTF(out, "L %d %.2f\n", LUMINAIRE, lb);
            } else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        case 'C':   // energy cost
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "C %d %.4f\n", LUMINAIRE, energy_cost);
            else hub_forward_command(buffer, (uint8_t)luminaire_index);
            break;

        default:
            out.println("err: unknown get sub-command");
            break;
        }
        break;

    // ── h : Help ──────────────────────────────────────────────────────────────
    case 'h':
        out.println("=== SCDTR Command Reference (Phase 2) ===");
        out.println("Control:  r u f o a <i> <val>");
        out.println("Phase 2:  O <i> <lux>  HIGH bound | U <i> <lux>  LOW bound");
        out.println("          C <i> <val>  energy cost | R  restart");
        out.println("Get:      g y/u/r/v/o/a/f/d/p/t/E/V/F/O/U/L/C <i>");
        out.println("          g b y/u <i>  (hub/local only)");
        out.println("Calib:    c g | c s [n] | c b [v] | c m <v> | c ?");
        out.println("Tuning:   c p/i/B/t/w/f <i> <val>");
        out.println("Stream:   s/S y/u/j <i>");
        out.println("(Commands for other nodes are auto-forwarded over CAN)");
        break;

    default:
        out.println("err: unknown command (type h for help)");
        break;
    }
}
