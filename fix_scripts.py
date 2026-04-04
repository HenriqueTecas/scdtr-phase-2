import sys
import os

with open("test_suite.py", "r") as f:
    ts = f.read()

ts = ts.replace("occ: 0=off, 1=low(20lux), 2=high(30lux)", "occ: 0=off, 1=low(15lux), 2=high(25lux)")
ts = ts.replace("axes[0].axhline(30, color=RED, lw=1.2, ls='--', alpha=0.7, label='ref_high=30')", "axes[0].axhline(25, color=RED, lw=1.2, ls='--', alpha=0.7, label='ref_high=25')")
ts = ts.replace("axes[0].axhline(20, color=ORG, lw=1.2, ls='--', alpha=0.7, label='ref_low=20')", "axes[0].axhline(15, color=ORG, lw=1.2, ls='--', alpha=0.7, label='ref_low=15')")
ts = ts.replace("fig.suptitle(\"Effect of Cost Vector  (all nodes HIGH occupancy, L=30 lux)\",", "fig.suptitle(\"Effect of Cost Vector  (all nodes HIGH occupancy, L=25 lux)\",")
ts = ts.replace("axes[1].axhline(30, color=RED, lw=1, ls='--', alpha=0.6)", "axes[1].axhline(25, color=RED, lw=1, ls='--', alpha=0.6)")
ts = ts.replace("axes[0].axhline(30, color=RED, lw=0.9, ls='--', alpha=0.6, label='ref_high=30')", "axes[0].axhline(25, color=RED, lw=0.9, ls='--', alpha=0.6, label='ref_high=25')")
ts = ts.replace("axes[0].axhline(20, color=ORG, lw=0.9, ls='--', alpha=0.6, label='ref_low=20')", "axes[0].axhline(15, color=ORG, lw=0.9, ls='--', alpha=0.6, label='ref_low=15')")
ts = ts.replace("axes[0].set_title('LUX (dashed = lower bounds 20 / 30)', fontsize=9)", "axes[0].set_title('LUX (dashed = lower bounds 15 / 25)', fontsize=9)")

with open("test_suite.py", "w") as f:
    f.write(ts)

with open("live_plot.py", "r") as f:
    lp = f.read()

lp = lp.replace("OCC_REF = [10.0, 20.0, 30.0]", "OCC_REF = [0.0, 15.0, 25.0]")

# Add the CLI parameter logic
import_block = "import sys\nimport time"
new_import_block = "import sys\nimport time"
# Assuming it's already there, we'll insert before 'def toggle_graphs' or after '_graphs_on = True'

target_graphs_on = "_graphs_on = True\n_frame_skip = 0"
new_graphs_on = """_graphs_on = True
if "--no-graphs" in sys.argv:
    _graphs_on = False
    sys.argv.remove("--no-graphs")

_frame_skip = 0"""

lp = lp.replace(target_graphs_on, new_graphs_on)

# Also update the initial button state if starting with --no-graphs
button_init = """for b, c in [(btn, WHITE), (btn_save, "#a6e3a1"), (btn_all, "#cba6f7"),
             (btn_tog, "#a6e3a1")]:
    b.label.set_color(c)"""

new_button_init = """for b, c in [(btn, WHITE), (btn_save, "#a6e3a1"), (btn_all, "#cba6f7"),
             (btn_tog, "#a6e3a1" if _graphs_on else "#3a1a1a")]:
    b.label.set_color(c)
if not _graphs_on:
    btn_tog.label.set_text("📊 OFF")
    btn_tog.color = "#3a1a1a"
"""
lp = lp.replace(button_init, new_button_init)

with open("live_plot.py", "w") as f:
    f.write(lp)
