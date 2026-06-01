# 3-DOF Fixed-Wing Aircraft Flight Dynamics Model

---

## 1. Overview

This document describes a nonlinear 3-degree-of-freedom (3-DOF) flight dynamics model implemented for a fixed-wing airborne vehicle (AV). The model is intended for simulation and control design studies under simplified but physically interpretable assumptions.

The system evolves in continuous time but is numerically integrated using a fixed-step fourth-order Runge–Kutta (RK4) scheme.

---

## 2. Coordinate Frames and State Representation

### 2.1 Inertial Frame ( \mathcal{I} )

The inertial frame is defined as:

* ( x_I ): horizontal axis (rightward)
* ( z_I ): vertical axis (upward)

### 2.2 Body Frame ( \mathcal{B} )

The body-fixed frame is defined as:

* ( x_B ): forward (tail → nose)
* ( z_B ): downward
* ( y_B ): lateral axis (right wing)

Rotational motion is restricted to pitch about ( y_B ), with angular velocity ( q ).

---

## 3. State, Input, and Output Definition

### 3.1 State Vector

[
X =
\begin{bmatrix}
x_I \
z_I \
\theta \
u \
w \
q
\end{bmatrix}
]

Where:

* ( x_I, z_I ): inertial position
* ( \theta ): pitch angle
* ( u, w ): body-frame velocities
* ( q ): pitch rate

---

### 3.2 Control Input

[
U =
\begin{bmatrix}
\delta_t \
\delta_e
\end{bmatrix}
]

Where:

* ( \delta_t \in [0,1] ): throttle
* ( \delta_e ): elevator deflection

---

### 3.3 Output

[
Y = X
]

The system is fully observable.

---

## 4. Core Modeling Assumptions

The model is constructed under the following simplifying assumptions:

### 4.1 Physical Assumptions

* 3-DOF rigid body (no roll or yaw dynamics)
* Constant mass ( m )
* Constant inertia ( I_{yy} )
* Flat, non-rotating Earth
* Constant air density ( \rho )
* No wind disturbance
* No Mach effects
* No actuator dynamics or delays
* Perfect sensors and state feedback

---

### 4.2 Aerodynamic Assumptions

* Linear lift curve:
  [
  C_L = C_{L0} + C_{L\alpha}\alpha
  ]

* Parabolic drag polar:
  [
  C_D = C_{D0} + C_{Dk} C_L^2
  ]

* Pitching moment:
  [
  C_m = C_{m0} + C_{m\alpha}\alpha + C_{m\delta_e}\delta_e
  ]

* Aerodynamic center fixed at:
  [
  x_{ac} = 0.25c
  ]

* Stability-frame aerodynamics, body-frame dynamics

---

## 5. Kinematic Definitions

### 5.1 Angle of Attack

[
\alpha = \tan^{-1}\left(\frac{w}{u}\right)
]

### 5.2 Airspeed

[
V = \sqrt{u^2 + w^2}
]

---

## 6. Aerodynamic Force Model

### 6.1 Dynamic Pressure

[
q_\infty = \frac{1}{2}\rho V^2
]

---

### 6.2 Lift and Drag (Stability Frame)

[
L_s = \frac{1}{2}\rho V^2 S C_L
]

[
D_s = \frac{1}{2}\rho V^2 S C_D
]

Where:

* ( S ): wing reference area

---

## 7. Force Transformation and Body Dynamics

### 7.1 Body-Frame Forces

[
F_X = L_s \sin\alpha - D_s \cos\alpha - mg\sin\theta + T_{\max}\delta_t
]

[
F_Z = -L_s \cos\alpha - D_s \sin\alpha + mg\cos\theta
]

---

### 7.2 Landing Gear Model (Optional)

When engaged, a linear spring-damper model is applied:

[
F_G = -k_G (l_G + z_I) - b_G \dot{z}
]

This force is projected into the body frame:
[
F_X \leftarrow F_X - F_G \sin\theta
]
[
F_Z \leftarrow F_Z + F_G \cos\theta
]

---

### 7.3 Translational Dynamics (Body Frame)

[
\dot{u} = \frac{F_X}{m} - qw
]

[
\dot{w} = \frac{F_Z}{m} + qu
]

The Coriolis coupling terms arise from the rotating body frame.

---

## 8. Rotational Dynamics

### 8.1 Pitching Moment

Aerodynamic moment about the center of gravity:

[
M = M_s + (L_s \cos\alpha + D_s \sin\alpha)(c_g - 0.25)c
]

---

### 8.2 Aerodynamic Moment Component

[
M_s =
\frac{1}{2}\rho V^2 S c C_m +
\frac{1}{4}\rho V S c^2 \left(C_{mq} q + C_{m\dot{\alpha}} \dot{\alpha}\right)
]

---

### 8.3 Angle of Attack Rate

[
\dot{\alpha} =
\frac{u\dot{w} - w\dot{u}}{V^2}
\quad \text{for } V \neq 0
]

---

### 8.4 Pitch Dynamics

[
\dot{q} = \frac{M}{I_{yy}}
]

[
\dot{\theta} = q
]

---

## 9. Inertial Kinematics

Body velocities are transformed into inertial frame velocities:

[
\begin{bmatrix}
\dot{x}_I \
\dot{z}_I
\end{bmatrix}
=============

\begin{bmatrix}
\cos\theta & \sin\theta \
-\sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix}
u \
w
\end{bmatrix}
]

Thus:

[
\dot{x}_I = u\cos\theta + w\sin\theta
]

[
\dot{z}_I = -u\sin\theta + w\cos\theta
]

---

## 10. Full State-Space Form

The system is a nonlinear time-invariant MIMO system:

[
\dot{X} = F(X, U)
]

with:

[
X =
[x_I, z_I, \theta, u, w, q]^T
\quad,\quad
U = [\delta_t, \delta_e]^T
]

---

## 11. Numerical Integration

The continuous-time dynamics are discretized using a fixed-step RK4 integrator:

[
X_{k+1} = X_k + \text{RK4}(F, X_k, U_k, \Delta t)
]

---

## 12. Model Characteristics and Limitations

### Strengths

* Physically interpretable aerodynamic model
* Includes coupling between translation and rotation
* Stability-frame aerodynamics
* Nonlinear lift/drag behavior
* Includes ground interaction (optional)

### Limitations

* No roll/yaw dynamics (3-DOF only)
* No actuator dynamics (instant control response)
* Constant air density (no altitude effects)
* No wind or turbulence
* Linearized stability derivatives (except drag polar nonlinearity)
* No stall model or post-stall behavior
* Small-angle approximations avoided, but aerodynamic validity still limited at extreme AoA

---

## 13. Interpretation as a Dynamical System

This model represents a coupled nonlinear dynamical system:

* Translational motion depends on aerodynamic forces and gravity projection
* Rotational motion is driven by aerodynamic moments and inertial coupling
* Aerodynamic coefficients introduce nonlinear dependence on ( \alpha, q, \dot{\alpha} )
* System exhibits strong coupling between:

  * ( (u, w) \leftrightarrow \theta )
  * ( \alpha \leftrightarrow M \leftrightarrow q )

---

If you want, I can next:

* derive a **linearized state-space model (A, B matrices)** around trim,
* add a **stability analysis section (eigenvalues, modes)**,
* or convert this into a **LaTeX-ready paper format (IEEE style)**.
