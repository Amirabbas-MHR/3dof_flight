from pathlib import Path
import sys

sys.path.append(
    str(Path(__file__).resolve().parents[1])
)

import numpy as np
from Dynamics import AV_Model
from FlightControlSystem import FCS, FlightMode, ControllerModule, ZeroController
from Utils import saturator, trimmer
from Simulator import simulate
from Plotting import plot_simulation, plot_custom


# Test 3: Steady climb flight (scf) [no active control]

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
                    C_m_de=-1.1)


# trim for steady climb flight, with horizontal velocity Vx and vertical velocity Vz
Vx, Vz = 30, -5
alpha_T, delta_e_T, delta_t_T = trimmer(av_model=aircraft,
                                        mode='scf', # steady climb flight
                                        variables={'Vx':Vx, "Vz":Vz},
                                        initial_guess=np.array([0.05, 0, 0.5]), # initial guess for alpha, delta_e, delta_t
                                        ) 


# check the state vector derivative (Xdot) for resulting trim values (should be ~ 0 in all elements except Xdot[0] and Xdot[1] (Vx and Vz))

gamma_T = np.arctan2(-Vz, Vx) # flight path angle, with z-down convention consideration
theta_T = alpha_T + gamma_T

u = Vx * np.cos(theta_T) - Vz * np.sin(theta_T)
w = Vx * np.sin(theta_T) + Vz * np.cos(theta_T)

X_T = np.array([0, 0, theta_T, u, w, 0])
U_T = np.array([delta_t_T, delta_e_T])

print("X_trim:", X_T)
print("U_trim:", U_T)
print("X_dot trim: ", aircraft.X_dot(0, X_T, U_T))


# setting the controller input values to calculated trim values
c_dt = ZeroController(delta_t_T)
c_de = ZeroController(delta_e_T)

#  controller modules
cm_dt = ControllerModule(
                        controller=c_dt,
                        state_map=lambda X: X, # full X as controller's current state-vector
                        ref_map=lambda X_r:X_r, # full X_ref as controller's desired state-vector
)


cm_de = ControllerModule(
                        controller=c_de,
                        state_map=lambda X: X, # full X as controller's current state-vector
                        ref_map=lambda X_r:X_r, # full X_ref as controller's desired state-vector
)

# defining scf flight mode
controller_modules = [cm_dt, cm_de]
scf_flight_mode = FlightMode(controller_modules=controller_modules)

# defining FCS with one mode: scf, and setting FCS's mode to it
t3_FCS = FCS(modes={'scf': scf_flight_mode})
t3_FCS.set_mode('scf')

# Dummy Saturator and U_bounds
U_bounds = np.array([[-np.inf, np.inf], [-np.inf, np.inf]])
sat = saturator

# starting with trim values as initial conditions, only altitude is 500 m initially
X0 = X_T.copy() 
X0[1] = -500 

# time interval and time-step
t_span = (0, 10)
dt = 0.001

# dummy X_ref_func
X_ref_func = lambda t, X: np.array([0, 0, 0, 0, 0, 0])

log = simulate(
                model=aircraft,
                fcs=t3_FCS,
                saturator=sat,
                X0=X0,
                t_span=t_span,
                dt=dt,
                X_ref_func=X_ref_func,
                U_bounds=U_bounds,
            )

plot_simulation(log, plot_switches={"trajectory":True, "attitude":False, "body_v":False, "inertial_v":False, "aero":False, "control":True})

plot_custom(log,
            variables={"z": (480, 570), 'Vz': (3, 7), 'Vx': (28, 32), 'theta': (20, 22)},
            title='Steady Climb Flight')
