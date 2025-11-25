# Inverted-pendulum
# Modern Control Project: Double Inverted Pendulum (DIP) State-Space Control

## Description

This repository contains the analysis and implementation details for a **Modern Control Systems** project. The core focus is on the **Double Inverted Pendulum (DIP)** system, covering its dynamic modeling, linearization, and the design of various state-space controllers and observers.

The project was conducted as part of the Modern Control course at **Amirkabir University of Technology (Tehran Polytechnic)** in Fall 2023 (پاییز 1402).

## Key Features

The project report and accompanying files cover the following topics in detail:

* [cite_start]**System Modeling:** Derivation of the non-linear dynamic equations for the Double Inverted Pendulum system using the Lagrangian method[cite: 656, 691, 734].
* [cite_start]**Linearization and State-Space Model:** Linearizing the non-linear model around unstable equilibrium points (0 and $\pi$) to obtain the state-space representation[cite: 625, 733, 744, 750].
* [cite_start]**Fundamental Analysis:** Comprehensive analysis of the system's stability, **Controllability**, and **Observability**[cite: 625, 771, 774].
* [cite_start]**State Feedback Controller Design:** Design of a state feedback controller (pole placement) to stabilize the system[cite: 625].
* **Tracking Control (Pre-compensator Design):**
    * [cite_start]Design of a **Static Pre-compensator** for steady-state error elimination[cite: 573].
    * [cite_start]Design of a **Dynamic (Integral) Pre-compensator** for robust tracking and disturbance rejection[cite: 581].
* [cite_start]**Observer Design:** Design and validation of both a **Full-Order Observer** and a **Reduced-Order Observer** for state estimation[cite: 625].
* [cite_start]**Simulation:** All designs and analyses are validated through extensive simulations using MATLAB and Simulink[cite: 625, 769].

## Implementation

The project relies heavily on **MATLAB** and **Simulink** for all computations, analysis, and simulation.

**Files (Expected):**
* **Report:** `MC_Project_7_2.pdf` (The main project report).
* **MATLAB Scripts (`.m` files):** Code used for deriving models, calculating gains, and performing analytical checks (e.g., `ctrb`, `obsv`, `eig`, `jacobian`).
* **Simulink Models (`.slx` files):** Models for non-linear and linear simulations, incorporating the designed controllers and observers.

## Usage

To run the project code and reproduce the results, you will need MATLAB with the Control System Toolbox.

1.  Clone the repository.
2.  Open MATLAB.
3.  Execute the relevant `.m` files to perform the analysis and design.
4.  Run the `.slx` files in Simulink to view the time responses of the system under various control strategies.

## Contributors

* Amir Hossein Dezjdar
* Mahshid Jafari
* Mohammad Amin Mohtashami
* Zaynab Sadat Mirhashemi
* Negar Talavi

**Instructor:** Dr. Hajar Atrianfar
