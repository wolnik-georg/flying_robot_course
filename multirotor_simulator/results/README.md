# Results Directory

This directory contains all outputs from the multirotor simulator assignments.

## Directory Structure

```
results/
├── data/                    # CSV data files from Rust binaries
│   ├── trajectory_modular_*.csv    # Assignment 1: Integration method comparison
│   └── assignment2_*.csv           # Assignment 2: Geometric control scenarios
└── images/                  # PNG visualization files from Python scripts
    ├── assignment1_*.png           # Assignment 1: Integration comparison plots
    └── assignment2_*.png           # Assignment 2: Geometric control plots
```

## Assignment 1: Integration Method Comparison

**Data Files:**
- `trajectory_modular_Euler.csv` - Euler integration results
- `trajectory_modular_RK4.csv` - 4th-order Runge-Kutta results
- `trajectory_modular_ExpEuler.csv` - Exponential Euler results
- `trajectory_modular_ExpRK4.csv` - Exponential RK4 results

**Visualization Files:**
- `assignment1_position_comparison.png` - Position/velocity comparison across methods
- `assignment1_accuracy_analysis.png` - Error analysis relative to RK4

## Assignment 2: Geometric Control Performance

**Data Files:**
- `assignment2_hover.csv` - Stationary hover control data
- `assignment2_figure8.csv` - Figure-8 trajectory tracking data
- `assignment2_circle.csv` - Circular trajectory tracking data

**Visualization Files:**
- `assignment2_hover_performance.png` - Hover control analysis
- `assignment2_figure8_tracking.png` - Figure-8 trajectory tracking
- `assignment2_circle_tracking.png` - Circular trajectory tracking
- `assignment2_motor_speeds_*.png` - Motor speed analysis for each scenario

## Usage

### Generate Data
```bash
# Assignment 1: Integration comparison
cargo run --bin assignment1

# Assignment 2: Geometric control
cargo run --bin assignment2
```

### Generate Visualizations
```bash
# Assignment 1 plots
python3 plot_assignment1.py

# Assignment 2 plots
python3 plot_assignment2.py
```

## File Formats

- **CSV Files**: Time-series data with headers, suitable for analysis in Python, MATLAB, or Excel
- **PNG Files**: High-resolution plots (150 DPI) for reports and presentations

## Notes

- All data is generated using the Crazyflie 2.1 parameters
- Simulations use 10ms time steps (100 Hz control rate)
- Geometric controller gains may need tuning for better trajectory tracking performance