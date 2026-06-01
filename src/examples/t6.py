from pathlib import Path
import sys

sys.path.append(
    str(Path(__file__).resolve().parents[1])
)

import numpy as np
from Dynamics import AV_Model
from FlightControlSystem import FCS, FlightMode, ControllerModule, ZeroController, PDController
from Utils import saturator
from Simulator import simulate
from Plotting import plot_simulation, plot_custom


# Test 6: Takeoff. 

# create the aircraft model of a rational light airplane
aircraft = AV_Model(m=1200,
                    T_max=4000,
                    I_yy=1800,
                    c=1.5,
                    s=16,
                    cg=0.35,
                    C_D0=0.03,
                    C_Dk=0.04,
                    C_L0=0.3,
                    C_La=4.5,
                    C_m0=0.05,
                    C_ma=-1.2,
                    C_mq=-20,
                    C_m_adot=-3.5,
                    C_m_de=-1.1,
                    gear_setting = {"l_G": 0.4, "k_G": 58860, "b_G": 10000}, # gear resting lengt, spring coeff and damper coeff.
                    ) 

C_Lmax = aircraft.C_L0 + (10 * np.pi /180) * aircraft.C_La # assuming maximum C_L is at alpha = 10 degrees (stall angle)
V_stall = np.sqrt((2 * aircraft.m * aircraft.g) / (aircraft.rho * aircraft.s * C_Lmax)) # based on balance of weight and lift force
V_rotate = 1.2 * V_stall # by convention
print("rotating velocity:", V_rotate)

# mode 1: accelerating on the runway with full throttle, while controlling the pitch to b zero.

# controllers
cdt_1 = ZeroController(1) # dt is 1
cde_1 = PDController(Kp=-2, Kd=-5) # controlling theta

# controller modules
cmdt_1 = ControllerModule(
                        controller=cdt_1,
                        state_map=lambda X: X, # full X as controller's current state-vector
                        ref_map=lambda X_r:X_r, # full X_ref as controller's desired state-vector
)


cmde_1 = ControllerModule(
                        controller=cde_1,
                        state_map=lambda X: np.array([X[2], X[5]]), # [theta, q]
                        ref_map=lambda X_r: np.array([X_r[2], X_r[5]]), # [theta_reference, q_reference]
)

controller_modules = [cmdt_1, cmde_1] # [delta_t, delta_e]
# accelerating flight mode
accelerating_flight_mode = FlightMode(controller_modules=controller_modules)

# FCS
t6_FCS = FCS(modes={'accelerating': accelerating_flight_mode})
t6_FCS.set_mode('accelerating')

# Saturator and U_bounds
U_bounds = np.array([[0, 1], [-0.35, 0.35]]) # dt ~ (0, 1) / de ~ (-0.35, 0.35)
sat = saturator

# Initial state
X0 = np.array([0, -0.2, 0, 0, 0, 0]) # [x0, -z0, theta0, u0, w0, q0]

# time interval and time-step
t_span = (0, 30)
dt = 0.001

# ~ 6 degrees nose-up after reaching rotating speed
X_ref_func = lambda t, X: np.array([0, 0, 0, 0, 0, 0]) if X[3] < V_rotate else np.array([0, 0, 0.1, 0, 0, 0])

log = simulate(
                model=aircraft,
                fcs=t6_FCS,
                saturator=sat,
                X0=X0,
                t_span=t_span,
                dt=dt,
                X_ref_func=X_ref_func,
                U_bounds=U_bounds,
            )

plot_simulation(log)
