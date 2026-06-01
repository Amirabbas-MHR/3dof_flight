# 3DoF Flight Dynamics Engine

A nonlinear longitudinal aircraft flight dynamics simulator written in Python.

This project implements a physics-based three-degree-of-freedom (3DoF) aircraft model with aerodynamic force and moments, flight control systems, trimming utilities, disturbance injection, ground-gear interaction, and visualization tools.

The primary goal of this project is educational: it helped me understand flight dynamics and control systems and provides a transparent and extensible flight simulation framework that bridges the gap between textbook flight dynamics and practical numerical simulation.

---

## Features

### Aircraft Dynamics

* Nonlinear longitudinal aircraft dynamics
* Aerodynamic lift, drag, and pitching moment modeling
* Angle-of-attack dependent aerodynamics

### Numerical Simulation

* Fixed-step simulation
* Fourth-order Runge-Kutta (RK4) integration
* State logging and data collection

### Flight Control and guidance

* Modular flight control architecture
* Flight modes
* Implemnted conventional controllers (PD, PID, ...)
* Actuator saturation support
* Guidance laws

### Environment and Disturbances

* External disturbance injection
* Ground interaction model
* Spring-damper landing gear

### Analysis and Visualization

* Standard avionics-style plots
* Custom plotting utilities
* Time-history visualization

![scheme Placeholder](docs/images/scheme.png)

### * for more details, read the [technical_reference](/docs/techincal_reference.md)

---

## Project Structure

```text
3dof-flight-dynamics/

├── README.md
├── LICENSE
├── requirements.txt

├── src/
│   └── flight3dof/
│       ├── dynamics.py
│       ├── simulator.py
│       ├── controls.py
│       ├── plotting.py
│       └── utils.py

├── examples/
│   ├── t0.py
│   ├── t1.py
│   ├── t2.py
│   ├── t3.py
│   ├── t4.py
│   ├── t5.py
│   └── t6.py

├── docs/
│   ├── technical_reference.md
│   └── images/
│
└── results/
```

---

## State Definition

The aircraft state vector is

```math
X =
\begin{bmatrix}
x_I \\
z_I \\
\theta \\
u \\
w \\
q
\end{bmatrix}
```

where

| Variable | Description                  |
| -------- | ---------------------------- |
| x_I      | Inertial horizontal position |
| z_I      | Inertial vertical position   |
| θ        | Pitch attitude               |
| u        | x body velocity              |
| w        | z body velocity              |
| q        | Pitch rate                   |


![coordinate_system Placeholder](docs/images/coordinate_system.png)

---

## Control Inputs

```math
U =
\begin{bmatrix}
\delta_t \\
\delta_e
\end{bmatrix}
```

| Variable | Description         |
| -------- | ------------------- |
| δ_t      | Throttle command    |
| δ_e      | Elevator deflection |


---
## Example Scenarios

### T0 – Ballistic Free Fall

No aerodynamic forces, free falling to test bare-minimum physics and integration stability.

![T0 Placeholder](docs/images/t0.png)

---

### T1 – Aerodynamic Free Fall

Lift and drag effects enabled.

![T1 Placeholder](docs/images/t1.png)

Note that due to native angle-of-attack damping (given by aircraft model aerodynamic coefficients), AoA tends to zero as time goes on.

---

### T2 – Trimmed Level Flight

Steady-state flight without active control.

![T2 Placeholder](docs/images/t2.png)
![T2 Placeholder](docs/images/t2'.png)

retsulting trim values:

X_trim: [ 0.          0.          0.22490592 29.24445279  6.69043953  0.        ]
U_trim: [ 0.22362218 -0.08161739]


to validate the trim solution (only Vx is non-zero and is equal to the desired cruising speed)

X_dot trim:  [ 3.00000000e+01  0.00000000e+00  0.00000000e+00 -1.82458848e-11
 -5.24475278e-13  4.60408008e-13]
---

### T3 – Trimmed Climb

Steady climbing flight condition.

![T3 Placeholder](docs/images/t3.png)
![T3 Placeholder](docs/images/t3'.png)

resulting trim values:

X_trim: [ 0.          0.          0.36921033 29.78277377  6.16330969  0.        ]
U_trim: [ 0.700899   -0.06705723]

to validate the trim solution (Vx and Vz are non-zero and equal to the desired cruising speeds)

X_dot trim:  [ 3.00000000e+01 -5.00000000e+00  0.00000000e+00 -1.08191974e-12
 -2.57690166e-14  2.37321274e-14]
---

### T4 – Disturbance Rejection

Comparison between uncontrolled aircraft and PD-controlled pitch stabilization.

![T4 Placeholder](docs/images/t4.png)

---

### T5 – Ground Interaction

Aircraft standing on runway with spring-damper landing gear model.

![T5 Placeholder](docs/images/t5.png)

---

### T6 – Takeoff

Ground roll, rotation, and initial climb.

![T6 Placeholder](docs/images/t6.png)

---


## Future Work

* Wind model
* Atmospheric density variation
* Sensor models
* State estimation
* Autopilot modes
* Navigation systems
* 6DoF extension
* Aircraft configuration files
* Linearization tools
* Frequency-domain analysis

---

## Motivation

Flight dynamics is often taught through linearized equations and isolated examples. While these methods are essential, they can obscure the interconnected nature of aircraft motion.

This project was developed as an attempt to build a simulation framework from first principles, combining aerodynamics, rigid-body mechanics, numerical integration, and feedback control into a single coherent environment.

The result is not intended to compete with professional flight simulators. Instead, it aims to be a transparent engineering tool where every force, moment, state, and assumption can be inspected, modified, and understood.

---

#### p.s.1: gpt 5.5 helped me generate this markdown using in-code docs.
#### p.s.2: I used these resources when learning about flight dynamics and control. They may help you too.
##### 1. Advaced flight dynamics course by Dr. Alireza Sharifi, sharif ocw (https://ocw.sharif.ir/course/id/567)
##### 2. Linear control course by Dr. Alireza Sharifi, sharif ocw (https://ocw.sharif.ir/course/id/555)
##### 3. Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control and simulation: Dynamics, controls design, and autonomous systems (3rd ed.). Wiley.
##### 4. Stengel, R. F. (2022). Flight dynamics (2nd ed.). Princeton University Press.
##### 5. Allerton, D. (2009). Principles of flight simulation. Wiley.