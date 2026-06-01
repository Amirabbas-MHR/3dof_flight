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

## Inertial Frame $\mathcal{I}$

$$
x_I : \text{horizontal axis}
$$
$$
z_I : \text{vertical axis}
$$

---

## Body Frame $\mathcal{B}$

* $x_B$ : forward (nose direction)
* $z_B$ : downward (z-down convention)
* $y_B$ : right wing

The body frame is rotated relative to inertial frame $/y$ axis, by pitch angle $\theta$.

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
# Force Model (Stability Frame)

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
i want to cite theese books in my flight dynamics repo in github: brian steven's aircraft control and simulation book stengel's flight dynamics book david allerton's flight simulation book can you write the formal citing format for these books?Thought for 14sHere are the formal citations for the three books in common formats suitable for a GitHub repository (e.g., README, REFERENCES.md, or BibTeX).
1. Brian Stevens' Book
Full Title: Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems (3rd Edition)
APA Style:
Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control and simulation: Dynamics, controls design, and autonomous systems (3rd ed.). Wiley. https://doi.org/10.1002/9781119174882
BibTeX:
bibtexCopy@book{stevens2015aircraft,
  title     = {Aircraft Control and Simulation: Dynamics, Controls Design, and Autonomous Systems},
  author    = {Stevens, Brian L. and Lewis, Frank L. and Johnson, Eric N.},
  year      = {2015},
  edition   = {3},
  publisher = {Wiley},
  doi       = {10.1002/9781119174882}
}
2. Robert Stengel's Book
Full Title: Flight Dynamics (2nd Edition)
APA Style:
Stengel, R. F. (2022). Flight dynamics (2nd ed.). Princeton University Press.
BibTeX:
bibtexCopy@book{stengel2022flight,
  title     = {Flight Dynamics},
  author    = {Stengel, Robert F.},
  year      = {2022},
  edition   = {2},
  publisher = {Princeton University Press}
}
3. David Allerton's Book
Full Title: Principles of Flight Simulation
APA Style:
Allerton, D. (2009). Principles of flight simulation. Wiley.
BibTeX:
bibtexCopy@book{allerton2009principles,
  title     = {Principles of Flight Simulation},
  author    = {Allerton, David},
  year      = {2009},
  publisher = {Wiley}
}
Recommendation for GitHub
I suggest using BibTeX entries (easy to import into tools like Zotero, JabRef, or Overleaf) plus a simple markdown table in your README:
MarkdownCopy## References

| Book | Citation |
|------|----------|
| Stevens et al. | Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). *Aircraft control and simulation...* (3rd ed.). Wiley. |
| Stengel | Stengel, R. F. (2022). *Flight dynamics* (2nd ed.). Princeton University Press. |
| Allerton | Allerton, D. (2009). *Principles of flight simulation*. Wiley. |
Let me know if you want MLA, Chicago, IEEE, or any other specific style!50 sourceshow to wite the greek theta in mathematics format in markdown?Thought for 19s✅ How to write Greek Theta (θ) in Markdown:
1. Inline Math (most common)
MarkdownCopyThe angle is $\theta$.
Result: The angle is $  θ  $.

2. Display Math (centered, bigger)
MarkdownCopy$$ \theta $$
Result:
$    \theta    $

Other useful variations:






























SymbolMarkdown CodeResultLowercase theta\theta$  θ  $Uppercase theta\Theta$  Θ  $Bold theta\boldsymbol{\theta}$  \boldsymbol{\theta}  $Italic theta\mathit{\theta}$  \mathit{\theta}  $
Examples in context:
MarkdownCopy- The pitch angle is $\theta$.
- In flight dynamics: $\dot{\theta}$ (theta dot)
- Moment of inertia: $I_{yy} \ddot{\theta}$
Note: GitHub, GitLab, and many Markdown editors support MathJax / KaTeX. Just wrap the LaTeX code with $...$ or $$...$$.
Would you like me to also show you how to write common flight dynamics symbols (like phi φ, psi ψ, alpha α, etc.)?the math part of this markdown are not rendered correctly, can you edit that for me?
 
MarkdownCopy# Technical Reference

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
