#!/usr/bin/env python3
"""
Plot results from HMRTA test suite
Generates comparison plots for each independent variable test
"""

import pandas as pd
import matplotlib.pyplot as plt
import os
import sys

def plot_results():
    """Generate plots from test result CSV files"""
    
    # Create plots directory if it doesn't exist
    os.makedirs('plots', exist_ok=True)
    
    # Define test result files and their corresponding plot configurations
    test_configs = [
        {
            'file': 'automaton_states_results.csv',
            'x_col': 'States',
            'title': 'Automaton States Impact',
            'xlabel': 'Number of Automaton States',
            'metrics': ['Computation_Time_ms', 'Quality_Score']
        },
        {
            'file': 'number_robots_results.csv',
            'x_col': 'Robots',
            'title': 'Robot Fleet Size Scalability',
            'xlabel': 'Number of Robots',
            'metrics': ['Computation_Time_ms', 'Scalability_Score', 'Load_Balance']
        },
        {
            'file': 'transition_system_regions_results.csv',
            'x_col': 'Regions',
            'title': 'Transition System Complexity',
            'xlabel': 'Number of Regions',
            'metrics': ['Computation_Time_ms', 'EnvironmentComplexity_Score', 'PathPlanning_Efficiency']
        },
        {
            'file': 'average_capabilities_results.csv',
            'x_col': 'AvgCapabilities',
            'title': 'Average Capabilities per Robot',
            'xlabel': 'Average Capabilities per Robot',
            'metrics': ['Computation_Time_ms', 'TaskCoverage_Score', 'Redundancy_Score']
        },
        {
            'file': 'robot_homogeneity_results.csv',
            'x_col': 'Homogeneity',
            'title': 'Robot Fleet Homogeneity',
            'xlabel': 'Homogeneity Score (Independent Capabilities / Robots)',
            'metrics': ['Computation_Time_ms', 'AllocationEfficiency_Score', 'RobustnessTolerance', 'FleetScalability']
        }
    ]
    
    # Generate plots for each test configuration
    for config in test_configs:
        if not os.path.exists(config['file']):
            print(f"Warning: {config['file']} not found. Skipping...")
            continue
        
        try:
            # Read CSV file
            df = pd.read_csv(config['file'])
            
            # Create figure with subplots for each metric
            fig, axes = plt.subplots(len(config['metrics']), 1, figsize=(12, 4*len(config['metrics'])))
            if len(config['metrics']) == 1:
                axes = [axes]
            
            fig.suptitle(config['title'], fontsize=16, fontweight='bold')
            
            # Plot each metric
            for idx, metric in enumerate(config['metrics']):
                if metric in df.columns:
                    ax = axes[idx]
                    ax.plot(df[config['x_col']], df[metric], marker='o', linewidth=2, markersize=8)
                    ax.set_xlabel(config['xlabel'])
                    ax.set_ylabel(metric)
                    ax.grid(True, alpha=0.3)
                    ax.set_title(f'{config["title"]} - {metric}')
            
            # Save figure
            plot_filename = f"plots/{config['file'].replace('_results.csv', '')}_plot.png"
            plt.tight_layout()
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"✓ Generated: {plot_filename}")
            plt.close()
            
        except Exception as e:
            print(f"✗ Error processing {config['file']}: {str(e)}")
    
    print("\n" + "="*50)
    print("Plot generation complete!")
    print("Results saved in ./plots/ directory")
    print("="*50)

if __name__ == '__main__':
    print("Plotting HMRTA Test Results...")
    print("-"*50)
    plot_results()
