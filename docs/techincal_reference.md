# 3DOF Aircraft Flight Dynamics Model  
## Technical Documentation

---

## 1. Overview

This document describes a **nonlinear 3-degree-of-freedom (3DOF)** flight dynamics model for a fixed-wing airborne vehicle (AV). The model captures:

- Longitudinal translational motion in the inertial frame  
- Pitch rotational dynamics about the lateral axis  
- Coupled aerodynamics in a simplified but physically interpretable form  

The system is formulated as a **nonlinear time-invariant MIMO system**:

\[
\dot{\mathbf{X}} = \mathbf{F}(\mathbf{X}, \mathbf{U}), \quad \mathbf{Y} = \mathbf{X}
\]

where full-state observability is assumed.

---

## 2. Coordinate Frames and Conventions

### 2.1 Body Frame \( \mathcal{B} \)

- \( x_b \): longitudinal axis (tail → nose)
- \( z_b \): downward axis
- \( y_b \): lateral axis (right wing)

State variables in body frame:
- \( u \): forward velocity
- \( w \): vertical velocity (positive downward)
- \( q \): pitch rate

---

### 2.2 Inertial Frame \( \mathcal{I} \)

- \( x_I \): horizontal axis (rightward)
- \( z_I \): vertical axis (upward)

Position states:
- \( x_I, z_I \)

---

### 2.3 Frame Transformation

Rotation from body to inertial frame:

\[
\begin{bmatrix}
v_x \\
v_z
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta & \sin\theta \\
-\sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix}
u \\
w
\end{bmatrix}
\]

---

## 3. State and Input Definition

### 3.1 State Vector

\[
\mathbf{X} =
\begin{bmatrix}
x_I & z_I & \theta & u & w & q
\end{bmatrix}^T
\]

### 3.2 Control Input

\[
\mathbf{U} =
\begin{bmatrix}
\delta_t & \delta_e
\end{bmatrix}^T
\]

where:
- \( \delta_t \in [0,1] \): throttle command  
- \( \delta_e \): elevator deflection  

---

## 4. Kinematic Quantities

### 4.1 Angle of Attack

\[
\alpha = \arctan\left(\frac{w}{u}\right)
\]

### 4.2 True Airspeed

\[
V = \sqrt{u^2 + w^2}
\]

---

## 5. Aerodynamic Model

### 5.1 Lift Coefficient

\[
C_L = C_{L0} + C_{L\alpha}\alpha
\]

### 5.2 Drag Coefficient (Parabolic Polar)

\[
C_D = C_{D0} + C_{Dk} C_L^2
\]

---

### 5.3 Aerodynamic Forces

Dynamic pressure:

\[
q_\infty = \frac{1}{2} \rho V^2
\]

Lift:

\[
L_s = q_\infty S C_L
\]

Drag:

\[
D_s = q_\infty S C_D
\]

---

## 6. Force Model in Body Frame

### 6.1 Longitudinal Force \( F_X \)

\[
F_X =
L_s \sin\alpha
- D_s \cos\alpha
- mg \sin\theta
+ T_{\max}\delta_t
\]

### 6.2 Vertical Force \( F_Z \)

\[
F_Z =
- L_s \cos\alpha
- D_s \sin\alpha
+ mg \cos\theta
\]

---

## 7. Translational Dynamics

Newton’s second law in body axes:

\[
\dot{u} = \frac{F_X}{m} - qw
\]

\[
\dot{w} = \frac{F_Z}{m} + qu
\]

---

## 8. Inertial Kinematics

\[
\dot{x}_I = v_x = u\cos\theta + w\sin\theta
\]

\[
\dot{z}_I = v_z = -u\sin\theta + w\cos\theta
\]

---

## 9. Rotational Dynamics

### 9.1 Pitch Rate Dynamics

\[
\dot{q} = \frac{M}{I_{yy}}
\]

### 9.2 Pitch Angle

\[
\dot{\theta} = q
\]

---

## 10. Aerodynamic Moment Model

### 10.1 Pitching Moment Coefficient

\[
C_m =
C_{m0}
+ C_{m\alpha}\alpha
+ C_{m\delta_e}\delta_e
\]

---

### 10.2 Stability-Axis Moment

\[
M_s =
q_\infty S c C_m
+
\frac{1}{2} \rho V S c^2
\left(
C_{mq} q + C_{m\dot{\alpha}} \dot{\alpha}
\right)
\]

---

### 10.3 Angle of Attack Rate

\[
\dot{\alpha}
=
\frac{u\dot{w} - w\dot{u}}{V^2}
\quad (\text{for } V \neq 0)
\]

---

### 10.4 CG Offset Moment Contribution

Let aerodynamic center be fixed at \(0.25c\):

\[
M =
M_s
+ L_s (x_{cg} - 0.25)c \cos\alpha
+ D_s (x_{cg} - 0.25)c \sin\alpha
\]

---

## 11. Gravity Model

Gravity acts in inertial frame and is resolved into body axes:

- Coupled into \(F_X, F_Z\) via pitch angle projection
- Assumes constant magnitude:

\[
g = 9.81 \, \mathrm{m/s^2}
\]

---

## 12. Thrust Model

Thrust is aligned with body \(x\)-axis:

\[
T = T_{\max} \delta_t
\]

Assumptions:
- Instantaneous response
- No engine dynamics
- No thrust vectoring

---

## 13. Landing Gear Model (Optional)

When enabled, a simple spring-damper ground interaction is applied:

### 13.1 Contact Condition

\[
|z_I| < l_G
\]

### 13.2 Ground Force

\[
F_G = -k_G (l_G + z_I) - b_G v_z
\]

where:
\[
v_z = -u\sin\theta + w\cos\theta
\]

This force is transformed into body axes before being added to \(F_X, F_Z\).

---

## 14. Numerical Integration

The system is integrated using:

- Fixed timestep discretization
- Classical **4th-order Runge–Kutta (RK4)** method

This ensures stable integration of nonlinear coupled ODEs:

\[
\dot{\mathbf{X}} = \mathbf{F}(\mathbf{X}, \mathbf{U})
\]

---

## 15. Model Assumptions Summary

### 15.1 Physical Simplifications

- 3DOF longitudinal-only dynamics
- Constant mass and inertia
- Constant air density \( \rho \)
- No wind field
- No Mach effects
- No actuator dynamics
- No sensor noise or delay

---

### 15.2 Aerodynamic Assumptions

- Linear lift slope
- Parabolic drag polar
- Fixed aerodynamic center at \(0.25c\)
- Small-angle consistent stability-axis decomposition

---

### 15.3 Control Assumptions

- Instantaneous control response
- Direct mapping of inputs to forces/moments

---

## 16. System Interpretation

This model represents a **reduced-order nonlinear rigid-body aircraft model**, suitable for:

- Control system design (LQR, MPC, nonlinear control)
- Flight dynamics education
- Trajectory simulation
- Preliminary aircraft performance analysis

It explicitly preserves:

- Coupling between translational and rotational dynamics  
- Angle-of-attack dependent aerodynamics  
- Nonlinear inertial transformations  

while neglecting higher-order 6DOF effects.

---

## 17. Final Remarks

Despite its simplifications, the model retains key nonlinear flight physics:

- Lift–drag coupling via \(C_L^2\)
- Pitch–translation coupling through \(q u, q w\)
- Geometric CG-induced moment shifts
- Aerodynamic damping via \(C_{mq}\)

It is therefore a **physically consistent minimal aircraft dynamics core** rather than a purely kinematic simulator.
