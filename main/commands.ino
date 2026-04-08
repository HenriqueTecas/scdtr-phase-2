extern float ref_high;
extern float ref_low;
extern float energy_cost;
extern float admm_primal_res;
extern float admm_dual_res;
extern volatile unsigned long ping_sent_us;
void admm_request(bool is_responder);
void process_can_messages();

enum class CalStage : uint8_t;
extern CalStage cal_stage;
extern int cal_barrier_count;
extern bool cal_barrier_peers[];

// ── printf helper that works with any Print& ─────────────────────────────
// Arduino's Print base class has no printf(), so we snprintf into a temp
// buffer and call the universal print(const char*) method.
static char _pbuf[128];
#define PRINTF(out, fmt, ...)                               \
    do                                                      \
    {                                                       \
        snprintf(_pbuf, sizeof(_pbuf), fmt, ##__VA_ARGS__); \
        (out).print(_pbuf);                                 \
    } while (0)

// =============================================================================
void commands(char *buffer, Print &out)
// =============================================================================
{
    char command, sub_command, x_variable;
    int luminaire_index;
    float value;
    command = buffer[0];

    // ── Hub forwarding ──────────────────────────────────────────────────────
    // Parse the luminaire index from any command that has one
    // If it's not ours, forward the whole buffer over CAN and return
    if (command != 'h' && command != 'c' && command != 'R' && command != 'P')
    {
        int idx = -1;
        char *p = buffer + 1;
        while (*p && !isdigit(*p) && *p != '-')
            p++;
        if (*p && sscanf(p, "%d", &idx) == 1)
        {
            if (idx > 0 && idx != LUMINAIRE)
            {
                hub_forward(buffer, (uint8_t)idx);
                return;
            }
        }
    }

    switch (command)
    {

    // ── R : Restart all nodes and re-run startup calibration ───────────────────
    case 'R':
        can_send_byte(BROADCAST, MSG_CTRL, SUB_RESTART, (uint8_t)LUMINAIRE);
        out.println("ack");
        delay(100); // let Core 1 put the broadcast restart frame on the CAN bus
        rp2040.reboot();
        break;

    // ── P <dest> : RTT ping sweep to a peer node ──────────────────────────────
    case 'P':
    {
        int dest_node = 0;
        if (std::sscanf(buffer, "%c %d", &command, &dest_node) != 2 ||
            dest_node < 1 || dest_node > N_NODES)
        {
            out.println("err: usage P <dest_node>");
            break;
        }

        out.println("ack: ping sweep");
        for (int i = 0; i < 500; i++)
        {
            ping_sent_us = micros();
            can_send_sub((uint8_t)dest_node, MSG_CTRL, SUB_PING);

            unsigned long wait_start_ms = millis();
            while (millis() - wait_start_ms < 20)
            {
                process_can_messages();
            }
        }
        break;
    }

    // ── u <i> <val> : Set duty cycle (open-loop) ─────────────────────────────
    case 'u':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            serial_duty_cycle = constrain(value, 0.0f, 1.0f);
            if (!pid.get_feedback())
                analogWrite(LED_PIN, (int)(serial_duty_cycle * PWM_MAX));
            out.println("ack");
        }
        break;

    // ── r <i> <val> : Set illuminance reference [LUX] ────────────────────────
    case 'r':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            r = value;
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            out.println("ack");
        }
        break;

    // ── o <i> <val> : Set occupancy state (0=off 1=low 2=high) ──────────────
    // Phase 2: uses ref_high / ref_low instead of the old hardcoded OCC_REF[].
    case 'o':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            int mode = constrain((int)value, 0, 2);
            pid.set_occupancy(mode);
            r = (mode == 2) ? ref_high : (mode == 1) ? ref_low
                                                     : 0.0f;
            flicker_holdoff = FLICKER_EXCLUDE_SAMPLES;
            // Occupancy change → lower bound L_i changes → re-optimise.
            // This node is the initiator: broadcast ADMM_TRIGGER so all peers join.
            admm_request(false);
            PRINTF(out, "ack: occupancy=%d  r=%.1f LUX\n", mode, r);
        }
        break;

    // ── a <i> <val> : Set anti-windup mode (0=off 1=back-calc 2=stop-int) ───
    case 'a':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            pid.set_anti_windup((int)value);
            PRINTF(out, "ack: anti_windup=%d\n", pid.get_anti_windup());
        }
        break;

    // ── f <i> <val> : Set feedback on(1)/off(0) ──────────────────────────────
    case 'f':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            serial_duty_cycle = pid.get_duty_cycle();
            if ((int)value == 1)
                apply_feedforward(r);
            pid.set_feedback((int)value);
            out.println("ack");
        }
        break;

    // ── s <x> <i> : Start streaming ──────────────────────────────────────────
    // Streaming is always local; we just set the flag on this node.
    case 's':
        std::sscanf(buffer, "%c %c %d", &command, &x_variable, &luminaire_index);
        if (LUMINAIRE == luminaire_index)
        {
            if (x_variable == 'y')
            {
                stream_y = 1;
                out.println("ack");
            }
            else if (x_variable == 'u')
            {
                stream_u = 1;
                out.println("ack");
            }
            else if (x_variable == 'j')
            {
                stream_j = 1;
                out.println("ack");
            }
            else
                out.println("err: use y, u or j");
        }
        break;

    // ── S <x> <i> : Stop streaming ───────────────────────────────────────────
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
        out.println("ack");
        break;

    // ── O <i> <val> : Set HIGH-occupancy lower bound (Table 3) ───────────────
    case 'O':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            ref_high = value;
            out.println("ack");
        }
        break;

    // ── U <i> <val> : Set LOW-occupancy lower bound (Table 3) ────────────────
    case 'U':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            ref_low = value;
            out.println("ack");
        }
        break;

    // ── C <i> <val> : Set energy cost coefficient (Table 3) ──────────────────
    case 'C':
        std::sscanf(buffer, "%c %d %f", &command, &luminaire_index, &value);
        if (LUMINAIRE == luminaire_index)
        {
            energy_cost = value;
            admm_request(false);
            out.println("ack");
        }
        break;

    // ── c : Calibration + PID tuning commands ─────────────────────────────────
    case 'c':
        sub_command = '\0';
        std::sscanf(buffer, "%c %c", &command, &sub_command);
        if (sub_command == '\0')
        {
            cal_stage = CalStage::CAL_BARRIER;
            cal_barrier_count = 0;
            memset(cal_barrier_peers, 0, sizeof(cal_barrier_peers));
            out.println("ack: calibration triggered");
            break;
        }
        switch (sub_command)
        {
        case 'b':
        {
            float new_b;
            if (std::sscanf(buffer + 3, "%f", &new_b) == 1)
            {
                ldr_b = new_b;
                PRINTF(out, "ack: ldr_b=%.4f  →  LUX=%.2f\n", ldr_b, measureLux());
            }
            else
            {
                PRINTF(out, "ldr_b=%.4f  →  LUX=%.2f\n", ldr_b, measureLux());
            }
            break;
        }
        case 'm':
        {
            float new_m;
            if (std::sscanf(buffer + 3, "%f", &new_m) == 1)
            {
                ldr_m = new_m;
                PRINTF(out, "ack: ldr_m=%.4f\n", ldr_m);
            }
            else
            {
                out.println("err: usage  c m <value>");
            }
            break;
        }
        case 'g':
            calibrate_system_gain(); // long blocking procedure, prints to Serial directly
            break;
        case 's':
        {
            int steps = 10;
            if (strlen(buffer) > 3)
                std::sscanf(buffer + 3, "%d", &steps);
            sweep(constrain(steps, 2, 50)); // also prints to Serial directly
            break;
        }
        case '?':
            out.println("--- Calibration Parameters ---");
            PRINTF(out, "  ldr_m      = %.4f\n", ldr_m);
            PRINTF(out, "  ldr_b      = %.4f\n", ldr_b);
            PRINTF(out, "  background = %.2f LUX\n", sys_background);
            PRINTF(out, "  gain K     = %.2f LUX/duty\n", sys_gain);
            PRINTF(out, "  calibrated = %s\n", calibrated ? "yes" : "no");
            PRINTF(out, "  anti_windup= %d  (0=off 1=back-calc 2=stop-int)\n", pid.get_anti_windup());
            PRINTF(out, "  kp=%.3f  ki=%.3f  b=%.3f  kt=%.3f\n",
                   pid.get_kp(), pid.get_ki(), pid.get_b(), pid.get_kt());
            PRINTF(out, "  ref_high=%.1f  ref_low=%.1f  cost=%.3f\n",
                   ref_high, ref_low, energy_cost);
            break;
        case 'p':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_kp(val);
                PRINTF(out, "ack: kp=%.4f\n", val);
            }
            break;
        }
        case 'i':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_ki(val);
                PRINTF(out, "ack: ki=%.4f\n", val);
            }
            break;
        }
        case 'B':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_b(val);
                PRINTF(out, "ack: b=%.4f\n", val);
            }
            break;
        }
        case 't':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_kt(val);
                PRINTF(out, "ack: kt=%.4f\n", val);
            }
            break;
        }
        case 'w':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                pid.set_bumpless((int)val);
                PRINTF(out, "ack: bumpless=%d\n", (int)val);
            }
            break;
        }
        case 'f':
        {
            float val;
            int idx;
            std::sscanf(buffer + 3, "%d %f", &idx, &val);
            if (LUMINAIRE == idx)
            {
                filter_mode = (int)val;
                PRINTF(out, "ack: filter=%d\n", (int)val);
            }
            break;
        }
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
        case 'y':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "y %d %.4f\n", LUMINAIRE, lux_value);
            break;

        case 'u':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "u %d %.4f\n", LUMINAIRE,
                       pid.get_feedback() ? pid.get_duty_cycle() : serial_duty_cycle);
            break;

        case 'r':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "r %d %.2f\n", LUMINAIRE, r);
            break;

        case 'v':
            if (LUMINAIRE == luminaire_index)
            {
                float v = (analogRead(LDR_PIN) / (float)ADC_MAX) * VCC;
                PRINTF(out, "v %d %.4f\n", LUMINAIRE, v);
            }
            break;

        case 'o':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "o %d %d\n", LUMINAIRE, pid.get_occupancy());
            break;

        case 'a':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "a %d %d\n", LUMINAIRE, pid.get_anti_windup());
            break;

        case 'f':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "f %d %d\n", LUMINAIRE, pid.get_feedback());
            break;

        case 'd':
            if (LUMINAIRE == luminaire_index)
            {
                float d_ext = max(0.0f, lux_value - sys_gain * duty_cycle);
                PRINTF(out, "d %d %.4f\n", LUMINAIRE, d_ext);
            }
            break;

        case 'k':
            if (LUMINAIRE == luminaire_index)
            {
                PRINTF(out, "k %d %.4f %.4f %.4f\n", LUMINAIRE,
                       coupling_gains[LUMINAIRE][1],
                       coupling_gains[LUMINAIRE][2],
                       coupling_gains[LUMINAIRE][3]);
            }
            break;

        case 'p':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "p %d %.4f\n", LUMINAIRE, pid.get_u() * MAXIMUM_POWER);
            break;

        case 't':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "t %d %.3f\n", LUMINAIRE, micros() * 1e-6f);
            break;

        case 'b':
        { // last-minute buffer – too large for CAN, hub/local only
            std::sscanf(buffer, "%c %c %c %d",
                        &command, &sub_command, &x_variable, &luminaire_index);
            if (LUMINAIRE == luminaire_index)
            {
                buffer_y = buffer_u = 0;
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
                    out.println("err: use y or u");
                }
            }
            else
            {
                out.println("err: g b only on local node");
            }
            break;
        }

        case 'E':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "E %d %.4f\n", LUMINAIRE, energy_consumption);
            break;

        case 'V':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "V %d %.6f\n", LUMINAIRE,
                       sample_count ? visibility_error / (double)sample_count : 0.0);
            break;

        case 'F':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "F %d %.6f\n", LUMINAIRE,
                       sample_count ? (flicker_error / (double)sample_count) / pid.h : 0.0);
            break;

        case 'O':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "O %d %.2f\n", LUMINAIRE, ref_high);
            break;

        case 'U':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "U %d %.2f\n", LUMINAIRE, ref_low);
            break;

        case 'L':
            if (LUMINAIRE == luminaire_index)
            {
                float lb = (pid.get_occupancy() == 2) ? ref_high : (pid.get_occupancy() == 1) ? ref_low
                                                                                              : 0.0f;
                PRINTF(out, "L %d %.2f\n", LUMINAIRE, lb);
            }
            break;

        case 'C':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "C %d %.4f\n", LUMINAIRE, energy_cost);
            break;

        case 'K':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "K %d %.6f\n", LUMINAIRE, admm_primal_res);
            break;

        case 'J':
            if (LUMINAIRE == luminaire_index)
                PRINTF(out, "J %d %.6f\n", LUMINAIRE, admm_dual_res);
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
        out.println("          C <i> <val>  energy cost");
        out.println("ADMM:     automatic on o/C updates");
        out.println("System:   R  restart all nodes and recalibrate");
        out.println("Ping:     P <dest>  500 RTT pings at 20 ms spacing");
        out.println("Get:      g y/u/r/v/o/a/f/d/p/t/E/V/F/O/U/L/C/K/J <i>");
        out.println("          (K=primal res, J=dual res)");
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
