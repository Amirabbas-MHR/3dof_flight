from pathlib import Path
import sys

sys.path.append(
    str(Path(__file__).resolve().parents[1])
)

import numpy as np
from Dynamics import AV_Model
from FlightControlSystem import FCS, FlightMode, ControllerModule, ZeroController, PDController
from Utils import saturator, trimmer, Step_disturbance
from Simulator import simulate
from Plotting import plot_simulation, plot_custom


# Test 4: disturbance effect on steady level flight, with and without a PD pitch controller

# create the aircraft model of a rational light airplane
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


# trim for steady level flight, with horizontal velocity V
V = 30
alpha_T, delta_e_T, delta_t_T = trimmer(av_model=aircraft,
                                        mode='slf', # steady level flight mode
                                        variables={'Vx': V},
                                        initial_guess=np.array([0.05, 0, 0.5]), # initial guess for alpha, delta_e, delta_t
                                        ) 
theta_T = alpha_T # zero flight path angle

# # check the state vector derivative (Xdot) for resulting trim values (should be ~ 0 in all elements except Xdot[0] (v_x))
u = V * np.cos(theta_T)
w = V * np.sin(theta_T)


X_T = np.array([0, 0, theta_T, u, w, 0])
U_T = np.array([delta_t_T, delta_e_T])



# part 1: no active control:

# setting the controller input values to calculated trim values
c1_dt = ZeroController(delta_t_T)
c1_de = ZeroController(delta_e_T)

# Dummy controller modules
cm1_dt = ControllerModule(
                        controller=c1_dt,
                        state_map=lambda X: X, # full X as controller's current state-vector
                        ref_map=lambda X_r:X_r, # full X_ref as controller's desired state-vector
)

cm1_de = ControllerModule(
                        controller=c1_de,
                        state_map=lambda X: X, # full X as controller's current state-vector
                        ref_map=lambda X_r:X_r, # full X_ref as controller's desired state-vector
)

# defining slf flight mode
controller_modules1 = [cm1_dt, cm1_de]
slf_flight_mode1 = FlightMode(controller_modules=controller_modules1)

# defining FCS with one mode: slf, and setting FCS's mode to it
FCS1 = FCS(modes={'slf': slf_flight_mode1})
FCS1.set_mode('slf')

# Saturator and U_bounds
U_bounds = np.array([[0, 1], [-0.35, 0.35]]) # dt ~ (0, 1) / de ~ (-0.35, 0.35)
sat = saturator

# starting with trim values as initial conditions, only the altitude is 500 m initially
X0 = X_T.copy() 
X0[1] = -500 

# time interval and time-step
t_span = (0, 50)
dt = 0.001

# dummy X_ref_func
X_ref_func = lambda t, X: np.array([0, 0, 0, 0, 0, 0])

# creating an acceleration disturbance model to negative body z direction, from t = 2~5
disturbance_model = Step_disturbance(
                                    index=4, # Xdot[4]=wdot
                                    time_interval=(2, 5),
                                    magnitude=5,)

# simulating with no active control:
log_no_control = simulate(
                            model=aircraft,
                            fcs=FCS1,
                            saturator=sat,
                            X0=X0,
                            t_span=t_span,
                            dt=dt,
                            X_ref_func=X_ref_func,
                            U_bounds=U_bounds,
                            disturbance=disturbance_model.compute,
                        )

# part 2: simulating with a PD pitch controller

X_ref_func2 = lambda t, X: X_T

c2_de = PDController(Kp=-200, Kd=-100) #TODO something is wrong here. we try to make theta and q both zero. this may not be standard, since it is better to form a cascade, that q command depends on theta error, not a constant (zero in this trim case)
c2_dt = PDController(Kp=0, Kd=0)

# controller modules
cm2_de = ControllerModule(
                        controller=c2_de,
                        state_map=lambda X: np.array([X[2], X[5]]),     # [theta, q]
                        ref_map=lambda X_r: np.array([X_r[2], X_r[5]]), # [theta_ref, q_ref 
)

cm2_dt = ControllerModule(
                        controller=c2_dt,
                        state_map=lambda X: X, # full X as controller's current state-vector
                        ref_map=lambda X_r:X_r, # full X_ref as controller's desired state-vector
)

# defining slf flight mode
controller_modules2 = [cm2_dt, cm2_de]
slf_flight_mode2 = FlightMode(controller_modules=controller_modules2, ff=U_T)

# defining FCS with one mode: slf, and setting FCS's mode to it
FCS2 = FCS(modes={'slf': slf_flight_mode2})
FCS2.set_mode('slf')

log_controlled = simulate(
                            model=aircraft,
                            fcs=FCS2,
                            saturator=sat,
                            X0=X0,
                            t_span=t_span,
                            dt=dt,
                            X_ref_func=X_ref_func2,
                            U_bounds=U_bounds,
                            disturbance=disturbance_model.compute,
                        )

plot_custom(log_no_control,
            variables={'q': None, 'theta': None, 'delta_e': None, 'delta_t': None, 'TAS': None, 'z': None},
            title='no active control')

plot_custom(log_controlled,
            variables={'q': None, 'theta': None, 'delta_e': None, 'delta_t': None, 'TAS': None, 'z': None},
            title='PD pitch controller')
