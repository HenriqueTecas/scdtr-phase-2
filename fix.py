import os

file_path = "live_plot.py"
with open(file_path, "r") as f:
    content = f.read()

# 1. Increase update interval to allow the GUI event loop time to breathe
content = content.replace("UPDATE_MS  = 150", "UPDATE_MS  = 333")

# 2. Reduce the maximum number of points plotted per line (decimation)
content = content.replace("MAX_PLOT_PTS = 300", "MAX_PLOT_PTS = 150")

# 3. Rescale axes slightly less often to save CPU
content = content.replace("RESCALE_EVERY = 6", "RESCALE_EVERY = 3")

# 4. Avoid redundantly updating text and title artists if they haven't changed
old_text_block = """        # ── Always update: text readouts (cheap) ──────────────────────────
        status_txts[col].set_text(status)
        vals_txts[col].set_text(
            f"y={last_y:.2f} lux   u={last_u:.3f}   r={r_val:.1f}"
        )

        if not _graphs_on:
            # Text-only mode: update log, skip all plot work
            log_texts[col].set_text("\\n".join(log[-4:]) if log else "waiting…")
            continue

        # ── Graphs-on mode ────────────────────────────────────────────────
        short_port = ns.port.replace("/dev/tty", "")
        dot = "●" if conn else "○"
        ax_lux[col].set_title(
            f"{dot} Node {nid}  ({short_port})",
            color=WHITE if conn else "#f38ba8", fontsize=9, pad=4
        )
        log_texts[col].set_text("\\n".join(log[-4:]) if log else "waiting…")"""

new_text_block = """        # ── Always update: text readouts (cheap) ──────────────────────────
        if status_txts[col].get_text() != status:
            status_txts[col].set_text(status)
        
        new_vals = f"y={last_y:.2f} lux   u={last_u:.3f}   r={r_val:.1f}"
        if vals_txts[col].get_text() != new_vals:
            vals_txts[col].set_text(new_vals)

        new_log = "\\n".join(log[-4:]) if log else "waiting…"

        if not _graphs_on:
            # Text-only mode: update log, skip all plot work
            if log_texts[col].get_text() != new_log:
                log_texts[col].set_text(new_log)
            continue

        # ── Graphs-on mode ────────────────────────────────────────────────
        short_port = ns.port.replace("/dev/tty", "")
        dot = "●" if conn else "○"
        new_title = f"{dot} Node {nid}  ({short_port})"
        if ax_lux[col].get_title() != new_title:
            ax_lux[col].set_title(
                new_title, color=WHITE if conn else "#f38ba8", fontsize=9, pad=4
            )
        if log_texts[col].get_text() != new_log:
            log_texts[col].set_text(new_log)"""

content = content.replace(old_text_block, new_text_block)

# 5. Remove redundant draw_idle() because FuncAnimation handles it internally
old_draw_block = """                hi = max(max(yp), max(rp)) + 2
                ax_lux[col].set_ylim(max(0, lo), hi)

    fig.canvas.draw_idle()"""

new_draw_block = """                hi = max(max(yp), max(rp)) + 2
                ax_lux[col].set_ylim(max(0, lo), hi)

    # FuncAnimation automatically draws the canvas"""

content = content.replace(old_draw_block, new_draw_block)

with open(file_path, "w") as f:
    f.write(content)

print("Optimizations applied successfully.")
