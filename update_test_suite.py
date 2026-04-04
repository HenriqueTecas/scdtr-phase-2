import sys

file_path = "test_suite.py"
with open(file_path, "r") as f:
    lines = f.readlines()

# Add Convergence study steps constant
header_end = 0
for i, line in enumerate(lines):
    if "SCENARIOS = [" in line:
        header_end = i
        break

lines.insert(header_end, "CONVERGENCE_STEPS = [5, 10, 20, 40, 80, 150]\n\n")

# Find the end of run_tests to insert the new study
run_tests_end = 0
for i, line in enumerate(lines):
    if "plot_timeseries(results, out_dir)" in line:
        run_tests_end = i + 1
        break

study_func = """
def run_convergence_study(out_dir):
    print(\"\\n\" + \"=\"*60)
    print(\"  RESEARCH: ADMM Convergence Study (Occ 2,2,2)\")
    print(\"=\"*60)
    
    # Set all nodes to High Occupancy
    for n in nodes:
        n.write(f\"o {n.node_id} 2\")
    time.sleep(1.0)
    
    convergence_results = []
    
    for iters in CONVERGENCE_STEPS:
        print(f\"  Testing {iters} iterations...\")
        for n in nodes: n.admm_done = False
        
        # Trigger ADMM with specific iteration count
        node_by_id[1].write(f\"T {iters}\")
        
        if wait_admm(ADMM_WAIT_S):
            time.sleep(COLLECT_S)
            snap = get_snapshot()
            
            total_v = sum(snap[nid]['V'] for nid in node_by_id)
            total_e = sum(snap[nid]['E'] for nid in node_by_id)
            
            convergence_results.append({
                'iters': iters,
                'visibility_error': total_v,
                'energy': total_e
            })
            print(f\"    Total V={total_v:.2f}  Total E={total_e:.4f}\")
        else:
            print(\"    Timeout!\")

    # Plot results
    if not convergence_results: return
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    x = [r['iters'] for r in convergence_results]
    y_v = [r['visibility_error'] for r in convergence_results]
    y_e = [r['energy'] for r in convergence_results]
    
    color = 'tab:red'
    ax1.set_xlabel('ADMM Iterations')
    ax1.set_ylabel('Total Visibility Error [lux]', color=color)
    ax1.plot(x, y_v, 'o-', color=color, lw=2, label='Visibility Error')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.grid(True, alpha=0.3)
    
    ax2 = ax1.twinx()
    color = 'tab:blue'
    ax2.set_ylabel('Total Energy [J]', color=color)
    ax2.plot(x, y_e, 's--', color=color, lw=1.5, alpha=0.7, label='Energy')
    ax2.tick_params(axis='y', labelcolor=color)
    
    plt.title(\"ADMM Convergence Study: Iterations vs. Performance\")
    fname = os.path.join(out_dir, \"plot_convergence_research.png\")
    plt.savefig(fname, dpi=150)
    print(f\"  Saved: {fname}\")
"""

lines.insert(run_tests_end, study_func)

# Call the study at the end of main
for i, line in enumerate(lines):
    if "print(f\"\\n✓ Done. Plots saved to: {out_dir}\")" in line:
        lines.insert(i, "    run_convergence_study(out_dir)\n")
        break

with open(file_path, "w") as f:
    f.writelines(lines)
