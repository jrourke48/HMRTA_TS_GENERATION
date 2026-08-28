#!/usr/bin/env python3
"""
Plot automaton states vs computation time from individual CSV files
No pandas required - pure Python parsing
"""

import matplotlib.pyplot as plt
import os
import glob

# Create Plots directory if it doesn't exist
os.makedirs('Plots', exist_ok=True)

# Get all CSV files from raw_csv folder
output_dir = "raw_csv"
csv_files = glob.glob(os.path.join(output_dir, "automaton_test_*.csv"))

if not csv_files:
    print(f"Error: No CSV files found in {output_dir}/")
    exit(1)

# Parse data from CSV files
data_3robots = {}
data_6robots = {}

for csv_file in sorted(csv_files):
    filename = os.path.basename(csv_file)
    # Extract robot count and automaton ID from filename
    # Format: automaton_test_Nrobots_A.csv where N is robots, A is automaton ID
    parts = filename.replace("automaton_test_", "").replace(".csv", "").split("robots_")
    robots = int(parts[0])
    automaton_id = int(parts[1])
    
    # Read CSV file
    metrics = {}
    with open(csv_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line and line != "Metric,Value,Unit":
                parts = line.split(',')
                if len(parts) >= 2:
                    metric_name = parts[0]
                    try:
                        value = float(parts[1])
                        metrics[metric_name] = value
                    except ValueError:
                        metrics[metric_name] = parts[1]
    
    # Store in appropriate dict
    if robots == 3:
        data_3robots[automaton_id] = metrics
    else:
        data_6robots[automaton_id] = metrics

# Extract data for plotting
automaton_ids_3 = sorted(data_3robots.keys())
automaton_ids_6 = sorted(data_6robots.keys())

times_3robots = [data_3robots[aid]["Total Computation Time"] for aid in automaton_ids_3]
states_3robots = [data_3robots[aid]["Automaton States"] for aid in automaton_ids_3]

times_6robots = [data_6robots[aid]["Total Computation Time"] for aid in automaton_ids_6]
states_6robots = [data_6robots[aid]["Automaton States"] for aid in automaton_ids_6]

# Create separate figures for each robot count
# ============================================================================
# FIGURE 1: 3-Robot Environment
# ============================================================================
fig1, ax1 = plt.subplots(figsize=(10, 6))
fig1.suptitle('Automaton Complexity vs Computation Time (3-Robot Team)', fontsize=16, fontweight='bold')

ax1.plot(states_3robots, times_3robots, marker='o', markersize=8, 
         linewidth=2.5, color='#1f77b4', label='3-Robot Team')
ax1.set_xlabel('Number of Automaton States', fontsize=12, fontweight='bold')
ax1.set_ylabel('Computation Time (ms)', fontsize=12, fontweight='bold')
ax1.grid(True, alpha=0.3)

# Add value labels on points
for i, (s, t) in enumerate(zip(states_3robots, times_3robots)):
    ax1.text(s, t, f'{t:.2f}ms', ha='center', va='bottom', fontweight='bold')

plt.tight_layout()
plt.savefig('Plots/automaton_computation_time_3robots.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_computation_time_3robots.png")
plt.close(fig1)

# ============================================================================
# FIGURE 2: 6-Robot Environment
# ============================================================================
fig2, ax2 = plt.subplots(figsize=(10, 6))
fig2.suptitle('Automaton Complexity vs Computation Time (6-Robot Team)', fontsize=16, fontweight='bold')

ax2.plot(states_6robots, times_6robots, marker='s', markersize=8, 
         linewidth=2.5, color='#ff7f0e', label='6-Robot Team')
ax2.set_xlabel('Number of Automaton States', fontsize=12, fontweight='bold')
ax2.set_ylabel('Computation Time (ms)', fontsize=12, fontweight='bold')
ax2.grid(True, alpha=0.3)

# Add value labels on points
for i, (s, t) in enumerate(zip(states_6robots, times_6robots)):
    ax2.text(s, t, f'{t:.2f}ms', ha='center', va='bottom', fontweight='bold')

plt.tight_layout()
plt.savefig('Plots/automaton_computation_time_6robots.png', dpi=300, bbox_inches='tight')
print("✓ Plot saved as Plots/automaton_computation_time_6robots.png")
plt.close(fig2)
plt.close()

# Print summary
print("\n" + "="*80)
print("SUMMARY STATISTICS")
print("="*80 + "\n")

print("3-Robot Environment:")
for aid, t in zip(automaton_ids_3, times_3robots):
    states = states_3robots[automaton_ids_3.index(aid)]
    print(f"  Automaton {aid}: {states} states, {t:.2f} ms")

print("\n6-Robot Environment:")
for aid, t in zip(automaton_ids_6, times_6robots):
    states = states_6robots[automaton_ids_6.index(aid)]
    print(f"  Automaton {aid}: {states} states, {t:.2f} ms")

print("\n" + "="*80 + "\n")
