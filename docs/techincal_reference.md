# Technical Reference

## Introduction

This document describes the mathematical formulation, physical assumptions, and implementation-level modeling choices of the **3DoF Flight Dynamics Engine**.

The simulator models the longitudinal motion of a rigid fixed-wing aircraft using nonlinear equations of motion, aerodynamic force models, and simplified propulsion and ground interaction models.

The system is intended for:

* Flight dynamics education
* Control system design and testing
* Numerical simulation experiments

---

# Degrees of Freedom

The aircraft is modeled as a **3-degree-of-freedom longitudinal rigid body**, with state variables:

[
\mathbf{X} =
[x_I,\ z_I,\ \theta,\ u,\ w,\ q]^T
]

where:

* (x_I): inertial horizontal position
* (z_I): inertial vertical position (altitude)
* (\theta): pitch attitude
* (u): body-frame forward velocity
* (w): body-frame vertical velocity
* (q): pitch rate

---

# Reference Frames

## Inertial Frame ((\mathcal{F}_I))

[
x_I: \text{horizontal axis}
]
[
z_I: \text{vertical axis (positive upward)}
]

---

## Body Frame ((\mathcal{F}_B))

The body frame is rigidly attached to the aircraft:

* (x_B): forward (nose direction)
* (z_B): downward (z-down convention)
* (y_B): right wing

The body frame rotates relative to inertial frame by pitch angle (\theta).

---

# State Vector

[
\mathbf{X} =
[x_I,\ z_I,\ \theta,\ u,\ w,\ q]^T
]

Interpretation:

* First three states are inertial / orientation
* Last three are body-frame velocities

This mixed-frame representation is intentional and consistent with classical flight dynamics formulations.

---

# Control Inputs

[
\mathbf{U} =
[\delta_t,\ \delta_e]^T
]

where:

* (\delta_t \in [0,1]): throttle command
* (\delta_e): elevator deflection

---

# Aerodynamic Model

## Angle of Attack

[
\alpha = \tan^{-1}\left(\frac{w}{u}\right)
]

---

## True Airspeed

[
V = \sqrt{u^2 + w^2}
]

---

# Lift Model

## Lift Coefficient

[
C_L = C_{L0} + C_{L\alpha}\alpha
]

## Lift Force

[
L =
\frac{1}{2}\rho V^2 S C_L
]

---

# Drag Model

## Drag Coefficient

[
C_D = C_{D0} + k C_L^2
]

## Drag Force

[
D =
\frac{1}{2}\rho V^2 S C_D
]

---

# Aerodynamic Parameters: (S) and (c)

## Wing Reference Area (S)

[
S
]

represents the **planform wing area** (in m²), i.e., the projected wing surface perpendicular to the airflow direction.

It is the primary scaling factor for:

* Lift magnitude
* Drag magnitude
* Aerodynamic force generation

Physically, larger (S) implies:

* More air mass interaction
* Higher lift at same angle of attack
* Higher drag at same speed

---

## Mean Aerodynamic Chord (c)

[
c
]

is the **mean aerodynamic chord (MAC)** of the wing.

It represents an equivalent chord length that preserves aerodynamic characteristics of the entire wing planform.

It is used for:

* Moment scaling
* Non-dimensional aerodynamic coefficients
* Lever arm between aerodynamic center and center of gravity

---

## Aerodynamic Center Assumption

The aerodynamic center is fixed at:

[
x_{ac} = 0.25c
]

The center of gravity is located at:

[
x_{cg} = cg \cdot c
]

Thus the moment arm is:

[
(x_{cg} - x_{ac}) = (cg - 0.25)c
]

---

# Pitching Moment Model

## Coefficient Model

[
C_m =
C_{m0}
+
C_{m\alpha}\alpha
+
C_{m\delta_e}\delta_e
]

---

## Dynamic Contributions

[
M_q \propto C_{mq} q
]

[
M_{\dot{\alpha}} \propto C_{m\dot{\alpha}} \dot{\alpha}
]

---

## Total Aerodynamic Moment

[
M_s =
\frac{1}{2}\rho V^2 S c C_m
+
\frac{1}{4}\rho V S c^2
\left(
C_{mq} q +
C_{m\dot{\alpha}} \dot{\alpha}
\right)
]

---

# Force Model (Body Frame)

## Lift and Drag Components

[
L_s = \frac{1}{2}\rho V^2 S C_L
]

[
D_s = \frac{1}{2}\rho V^2 S C_D
]

---

## Body Axes Forces

[
F_X = L_s \sin\alpha - D_s \cos\alpha + mg \sin\theta + T_{max}\delta_t
]

[
F_Z = -L_s \cos\alpha - D_s \sin\alpha + mg \cos\theta
]

---

# Equations of Motion

## Translational Dynamics

[
\dot{u} = \frac{F_X}{m} - qw
]

[
\dot{w} = \frac{F_Z}{m} + qu
]

---

## Rotational Dynamics

[
\dot{q} = \frac{M}{I_{yy}}
]

[
\dot{\theta} = q
]

---

# Coordinate Transformation

[
R_{BI} =
\begin{bmatrix}
\cos\theta & \sin\theta \
-\sin\theta & \cos\theta
\end{bmatrix}
]

[
v_x = u\cos\theta + w\sin\theta
]

[
v_z = -u\sin\theta + w\cos\theta
]

---

# Ground Interaction Model

A spring-damper landing gear model is used:

[
F_G = -k_G x - b_G \dot{x}
]

where:

* (k_G): stiffness
* (b_G): damping

This force is applied when ground penetration occurs and is transformed into body-frame forces.

---

# Numerical Integration

The system is integrated using:

* Fourth-order Runge-Kutta (RK4)
* Fixed timestep simulation

This provides:

* Stability under nonlinear dynamics
* Good accuracy for medium timestep sizes

---

# Environmental Assumptions

* Constant air density (\rho)
* Constant gravity (g)
* No wind
* No turbulence
* No atmospheric stratification

---

# Aircraft Assumptions

* Rigid body dynamics
* Constant mass (m)
* Constant inertia (I_{yy})
* Fixed center of gravity
* No structural deformation

Neglected:

* Fuel burn
* Aeroelastic effects
* Control surface hysteresis

---

# Control System Assumptions

* Full state observability
* No sensor noise
* No actuator dynamics
* No delays

Controllers operate directly on the true state vector (\mathbf{X}).

---

# Simulation Scope

This simulator represents a **minimal nonlinear aircraft dynamics laboratory**, capturing:

* Longitudinal stability
* Trim conditions
* Control response
* Takeoff and landing behavior
* Disturbance rejection

while intentionally excluding higher-fidelity aerospace complexities.

---

# Recommended Figures (IMPORTANT)

These significantly improve interpretability of the model.

## 1. Coordinate System Definition

```
[INSERT FIGURE: inertial vs body frame, showing θ, u, w, lift, drag, thrust]
```

---

## 2. Force Diagram on Aircraft

```
[INSERT FIGURE: free-body diagram with L, D, T, mg, moment arm cg-ac]
```

---

## 3. Flight Control Architecture

```
[INSERT FIGURE: controller → actuator → dynamics → state loop]
```

---

## 4. Example Simulation Outputs

```
[INSERT FIGURE: t2 steady flight plots]
[INSERT FIGURE: t6 takeoff trajectory]
[INSERT FIGURE: t4 disturbance response]
```

---

# Closing Remark

This model is intentionally structured to preserve physical interpretability. Every force, moment, and state variable corresponds directly to a classical flight dynamics concept, making the system suitable as a bridge between theoretical aerospace coursework and numerical simulation practice.
