#!/usr/bin/env python3
"""
Plot results from automaton_states_results.csv
Generates graphs showing computation time and memory usage scaling
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

# Read the CSV file
try:
    df = pd.read_csv('automaton_states_results.csv')
except FileNotFoundError:
    print("Error: automaton_states_results.csv not found!")
    print("Run the test executable first.")
    sys.exit(1)

# Filter only successful tests
df_success = df[df['Status'] == 'SUCCESS']

if len(df_success) == 0:
    print("Warning: No successful tests found in results")

# Create figure with subplots
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Automaton States Scaling Analysis', fontsize=16, fontweight='bold')

# Color palette for 4 automata
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
automaton_names = ['Automaton 1: G(F(p0)) & G(F(p2))',
                   'Automaton 2: Nested Temporal',
                   'Automaton 3: Liveness & Until',
                   'Automaton 4: Complex Nesting']

# ============================================================================
# PLOT 1: Computation Time vs Automaton States (All Automata)
# ============================================================================
ax1 = axes[0, 0]
for aut_id in range(1, 5):
    data = df_success[df_success['AutomatonID'] == aut_id]
    if len(data) > 0:
        ax1.plot(data['NumStates'], data['Computation_Time_ms'], 
                marker='o', label=f'Automaton {aut_id}', color=colors[aut_id-1], linewidth=2)

ax1.set_xlabel('Number of Automaton States', fontsize=11, fontweight='bold')
ax1.set_ylabel('Computation Time (ms)', fontsize=11, fontweight='bold')
ax1.set_title('Computation Time vs Automaton States', fontsize=12, fontweight='bold')
ax1.legend()
ax1.grid(True, alpha=0.3)

# ============================================================================
# PLOT 2: Memory Usage vs Automaton States (All Automata)
# ============================================================================
ax2 = axes[0, 1]
for aut_id in range(1, 5):
    data = df_success[df_success['AutomatonID'] == aut_id]
    if len(data) > 0:
        ax2.plot(data['NumStates'], data['Memory_MB'], 
                marker='s', label=f'Automaton {aut_id}', color=colors[aut_id-1], linewidth=2)

ax2.set_xlabel('Number of Automaton States', fontsize=11, fontweight='bold')
ax2.set_ylabel('Memory Usage (MB)', fontsize=11, fontweight='bold')
ax2.set_title('Memory Usage vs Automaton States', fontsize=12, fontweight='bold')
ax2.legend()
ax2.grid(True, alpha=0.3)

# ============================================================================
# PLOT 3: Average Time per Automaton
# ============================================================================
ax3 = axes[1, 0]
avg_times = []
for aut_id in range(1, 5):
    data = df_success[df_success['AutomatonID'] == aut_id]
    if len(data) > 0:
        avg_times.append(data['Computation_Time_ms'].mean())
    else:
        avg_times.append(0)

bars = ax3.bar(range(1, 5), avg_times, color=colors, alpha=0.7, edgecolor='black', linewidth=1.5)
ax3.set_xlabel('Automaton ID', fontsize=11, fontweight='bold')
ax3.set_ylabel('Average Time (ms)', fontsize=11, fontweight='bold')
ax3.set_title('Average Computation Time by Automaton', fontsize=12, fontweight='bold')
ax3.set_xticks(range(1, 5))
ax3.grid(True, alpha=0.3, axis='y')

# Add value labels on bars
for bar in bars:
    height = bar.get_height()
    ax3.text(bar.get_x() + bar.get_width()/2., height,
            f'{height:.2f}ms', ha='center', va='bottom', fontweight='bold')

# ============================================================================
# PLOT 4: Success/Failure Rate
# ============================================================================
ax4 = axes[1, 1]
success_count = len(df[df['Status'] == 'SUCCESS'])
failed_count = len(df[df['Status'] == 'FAILED'])
error_count = len(df[df['Status'] == 'ERROR'])

statuses = ['Success', 'Failed', 'Error']
counts = [success_count, failed_count, error_count]
colors_pie = ['#2ca02c', '#d62728', '#ff9999']

if sum(counts) > 0:
    wedges, texts, autotexts = ax4.pie(counts, labels=statuses, autopct='%1.1f%%',
                                        colors=colors_pie, startangle=90,
                                        textprops={'fontsize': 11, 'fontweight': 'bold'})
    ax4.set_title('Test Results Distribution', fontsize=12, fontweight='bold')
else:
    ax4.text(0.5, 0.5, 'No data available', ha='center', va='center', fontsize=14)

plt.tight_layout()

# Save the figure
output_file = 'automaton_states_analysis.png'
plt.savefig(output_file, dpi=300, bbox_inches='tight')
print(f"✓ Plot saved to: {output_file}")

# Also create individual per-automaton plots
for aut_id in range(1, 5):
    fig_ind, (ax_time, ax_mem) = plt.subplots(1, 2, figsize=(12, 5))
    fig_ind.suptitle(f'Automaton {aut_id} Scaling Analysis', fontsize=14, fontweight='bold')
    
    data = df_success[df_success['AutomatonID'] == aut_id]
    
    if len(data) > 0:
        # Time plot
        ax_time.plot(data['NumStates'], data['Computation_Time_ms'], 
                    marker='o', color=colors[aut_id-1], linewidth=2.5, markersize=8)
        ax_time.set_xlabel('Number of Automaton States', fontsize=11, fontweight='bold')
        ax_time.set_ylabel('Computation Time (ms)', fontsize=11, fontweight='bold')
        ax_time.set_title(f'Computation Time - {automaton_names[aut_id-1]}', fontsize=11, fontweight='bold')
        ax_time.grid(True, alpha=0.3)
        
        # Memory plot
        ax_mem.plot(data['NumStates'], data['Memory_MB'], 
                   marker='s', color=colors[aut_id-1], linewidth=2.5, markersize=8)
        ax_mem.set_xlabel('Number of Automaton States', fontsize=11, fontweight='bold')
        ax_mem.set_ylabel('Memory Usage (MB)', fontsize=11, fontweight='bold')
        ax_mem.set_title(f'Memory Usage - {automaton_names[aut_id-1]}', fontsize=11, fontweight='bold')
        ax_mem.grid(True, alpha=0.3)
    
    plt.tight_layout()
    ind_output = f'automaton_{aut_id}_analysis.png'
    plt.savefig(ind_output, dpi=300, bbox_inches='tight')
    print(f"✓ Plot saved to: {ind_output}")
    plt.close(fig_ind)

plt.show()

# ============================================================================
# Print Summary Statistics
# ============================================================================
print("\n" + "="*80)
print("SUMMARY STATISTICS")
print("="*80 + "\n")

print(f"Total tests: {len(df)}")
print(f"Successful: {success_count}")
print(f"Failed: {failed_count}")
print(f"Error: {error_count}")

print("\n" + "-"*80)
print("Per-Automaton Statistics:")
print("-"*80 + "\n")

for aut_id in range(1, 5):
    data = df_success[df_success['AutomatonID'] == aut_id]
    if len(data) > 0:
        print(f"Automaton {aut_id}: {automaton_names[aut_id-1]}")
        print(f"  Tests: {len(data)}")
        print(f"  Avg Time: {data['Computation_Time_ms'].mean():.2f} ms")
        print(f"  Min Time: {data['Computation_Time_ms'].min():.2f} ms")
        print(f"  Max Time: {data['Computation_Time_ms'].max():.2f} ms")
        print(f"  Avg Memory: {data['Memory_MB'].mean():.2f} MB")
        print(f"  Min Memory: {data['Memory_MB'].min():.2f} MB")
        print(f"  Max Memory: {data['Memory_MB'].max():.2f} MB")
        print()

print("="*80 + "\n")
