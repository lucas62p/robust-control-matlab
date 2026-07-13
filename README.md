# Discrete-Time State-Feedback Control & Observer Design

## Description
This repository provides a comprehensive framework for the design and analysis of a digital control strategy for a second-order linear dynamic system. The project demonstrates the implementation of **Full State Feedback with Integral Action** (for robust disturbance rejection) coupled with a **Luenberger Observer** to estimate unmeasured states.

## Project Scope & Objectives
The project aimed to control a levitation system (Airpong) through a complete engineering lifecycle:
1. **System Identification:** Extracting the transfer function using experimental input-output data and MATLAB System Identification Toolbox.
2. **Controller Synthesis:** Designing robust state-feedback laws and Luenberger observers to ensure stability and tracking performance.
3. **Sensor Fusion & Real-time Control:** Implementing a closed-loop system where the ball's height is modulated by real-time distance measurements from an infrared sensor (human-machine interaction).

## Key Features
* **System Modeling:** State-space representation and identification for discrete-time systems.
* **Control Synthesis:** Comparative implementation of **Pole Placement (Acker)** and **Linear Quadratic Regulator (LQR)** for optimal control performance.
* **State Estimation:** Design of a Luenberger observer to ensure observability and state estimation in noisy environments.
* **Validation:** Full system simulation (Simulink/MATLAB) to verify transient response, steady-state error, and stability.

## Project Structure
- `src/`
    - `control_synthesis.m`: Core script for parameter initialization, controller design, and matrix computation.
    - `trace_results.m`: Utility for generating performance plots and analyzing frequency response.
- `docs/`
    - `Airpong_System_Report.pdf`: Original assignment documentation.
    - `perf_plots/`: Generated simulation results.
- `sim/`
    - `Airpong.slx`: Dynamic simulation model for closed-loop testing.

## Getting Started
1. **Requirements:** MATLAB with Control System Toolbox.
2. **Execution:** Run `src/control_synthesis.m` to generate the closed-loop state matrices.
3. **Simulation:** Open `sim/Airpong.slx` to observe the system's dynamic response to setpoint changes and perturbations.

## Credits
This project was developed as an advanced control engineering assignment at Télécom Physique Strasbourg.
