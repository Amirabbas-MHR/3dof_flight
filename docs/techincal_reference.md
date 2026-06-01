# Technical Reference

## Introduction

This document describes the mathematical formulation, physical assumptions, and implementation-level modeling choices of the **3DoF Flight Dynamics Engine**.

The simulator models the longitudinal motion of a rigid fixed-wing aircraft using nonlinear rigid-body dynamics, classical aerodynamic models, and simplified propulsion and ground interaction models.

It is intended for:

* Flight dynamics education
* Control system development
* Numerical simulation experiments

---

# Degrees of Freedom

The aircraft is modeled as a **3-DOF longitudinal system**.

The state vector is:

$$
\mathbf{X} =
\begin{bmatrix}
x_I \
z_I \
\theta \
u \
w \
q
\end{bmatrix}
$$

where:

* $x_I$ : inertial horizontal position
* $z_I$ : inertial vertical position (altitude)
* $\theta$ : pitch attitude
* $u$ : body-frame forward velocity
* $w$ : body-frame vertical velocity
* $q$ : pitch rate

---

# Reference Frames

## Inertial Frame $\mathcal{F}_I$

$$
x_I : \text{horizontal axis}
$$
$$
z_I : \text{vertical axis (positive upward)}
$$

---

## Body Frame $\mathcal{F}_B$

* $x_B$ : forward (nose direction)
* $z_B$ : downward (z-down convention)
* $y_B$ : right wing

The body frame is rotated relative to inertial frame by pitch angle $\theta$.

---

# Control Inputs

$$
\mathbf{U} =
\begin{bmatrix}
\delta_t \
\delta_e
\end{bmatrix}
$$

where:

* $\delta_t \in [0,1]$ : throttle command
* $\delta_e$ : elevator deflection

---

# Aerodynamic Model

## Angle of Attack

$$
\alpha = \tan^{-1}\left(\frac{w}{u}\right)
$$

---

## True Airspeed

$$
V = \sqrt{u^2 + w^2}
$$

---

# Lift Model

## Lift Coefficient

$$
C_L = C_{L0} + C_{L\alpha}\alpha
$$

## Lift Force

$$
L = \frac{1}{2}\rho V^2 S C_L
$$

---

# Drag Model

## Drag Coefficient

$$
C_D = C_{D0} + k C_L^2
$$

## Drag Force

$$
D = \frac{1}{2}\rho V^2 S C_D
$$

---

# Aerodynamic Parameters

## Wing Area $S$

$S$ is the **planform wing area (m²)**, representing the effective surface exposed to airflow and governing lift and drag magnitude.

---

## Mean Aerodynamic Chord $c$

$c$ is the **mean aerodynamic chord (MAC)**, a representative chord length of the wing used for:

* Moment scaling
* Aerodynamic coefficient normalization
* Lever arm definition for pitching moments

---

## Aerodynamic Center

The aerodynamic center is assumed fixed at:

$$
x_{ac} = 0.25c
$$

Center of gravity location:

$$
x_{cg} = cg \cdot c
$$

Moment arm:

$$
(x_{cg} - x_{ac}) = (cg - 0.25)c
$$

---

# Pitching Moment Model

## Coefficient Model

$$
C_m =
C_{m0}

* C_{m\alpha}\alpha
* C_{m\delta_e}\delta_e
  $$

---

## Dynamic Contributions

$$
M_q \propto C_{mq} q
$$

$$
M_{\dot{\alpha}} \propto C_{m\dot{\alpha}} \dot{\alpha}
$$

---

## Total Aerodynamic Moment

$$
M_s =
\frac{1}{2}\rho V^2 S c C_m
+
\frac{1}{4}\rho V S c^2
\left(
C_{mq} q + C_{m\dot{\alpha}} \dot{\alpha}
\right)
$$

---

# Force Model (Body Frame)

## Lift and Drag Forces

$$
L_s = \frac{1}{2}\rho V^2 S C_L
$$

$$
D_s = \frac{1}{2}\rho V^2 S C_D
$$

---

## Body Axes Forces

$$
F_X = L_s \sin\alpha - D_s \cos\alpha + mg \sin\theta + T_{max}\delta_t
$$

$$
F_Z = -L_s \cos\alpha - D_s \sin\alpha + mg \cos\theta
$$

---

# Equations of Motion

## Translational Dynamics

$$
\dot{u} = \frac{F_X}{m} - qw
$$

$$
\dot{w} = \frac{F_Z}{m} + qu
$$

---

## Rotational Dynamics

$$
\dot{q} = \frac{M}{I_{yy}}
$$

$$
\dot{\theta} = q
$$

---

# Coordinate Transformation

$$
R_{BI} =
\begin{bmatrix}
\cos\theta & \sin\theta \
-\sin\theta & \cos\theta
\end{bmatrix}
$$

$$
v_x = u\cos\theta + w\sin\theta
$$

$$
v_z = -u\sin\theta + w\cos\theta
$$

---

# Ground Interaction Model

$$
F_G = -k_G x - b_G \dot{x}
$$

where:

* $k_G$ : ground stiffness
* $b_G$ : ground damping

---

# Numerical Integration

The system is solved using:

* Fourth-order Runge-Kutta (RK4)
* Fixed timestep integration

---

# Environmental Assumptions

* Constant air density $\rho$
* Constant gravity $g$
* No wind
* No turbulence
* No atmospheric variation

---

# Aircraft Assumptions

* Rigid body
* Constant mass $m$
* Constant inertia $I_{yy}$
* Fixed center of gravity
* No aeroelasticity

Neglected effects:

* Fuel burn
* Structural flexibility
* Control surface hysteresis

---

# Control System Assumptions

* Full state observability
* No sensor noise
* No actuator dynamics
* No delays

Controllers act directly on the true state vector $\mathbf{X}$.

---

# Simulation Scope

This simulator represents a **minimal nonlinear longitudinal aircraft laboratory**, capturing:

* Stability behavior
* Trim conditions
* Control response
* Takeoff and landing dynamics
* Disturbance rejection

while intentionally excluding high-fidelity aerospace complexity.

---

# Recommended Figures (Highly Important)

## Coordinate System Definition

```
[INSERT FIGURE: inertial frame vs body frame with θ, u, w, lift, drag, thrust]
```

---

## Free Body Diagram

```
[INSERT FIGURE: L, D, T, mg, cg/ac moment arm]
```

---

## Control Loop Architecture

```
[INSERT FIGURE: controller → actuator → aircraft → state feedback loop]
```

---

## Simulation Outputs

```
[INSERT FIGURE: t2 steady flight]
[INSERT FIGURE: t4 disturbance response]
[INSERT FIGURE: t6 takeoff trajectory]
```

---

# Closing Remark

This model preserves direct physical interpretability: every force, moment, and state variable maps explicitly to classical flight dynamics principles, making it a transparent bridge between theoretical aerospace engineering and numerical simulation practice.
