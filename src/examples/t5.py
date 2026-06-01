from pathlib import Path
import sys

sys.path.append(
    str(Path(__file__).resolve().parents[1])
)

import numpy as np
from Dynamics import AV_Model
from FlightControlSystem import FCS, FlightMode, ControllerModule, ZeroController
from Utils import saturator
from Simulator import simulate
from Plotting import plot_simulation, plot_custom


# Test 5: gear down (as a spring/damper aligned with c/g) standing on runway 

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
                    gear_setting = {"l_G": 0.4, "k_G": 58860, "b_G": 10000}) # gear resting lengt, spring coeff and damper coeff.

# Dummy controllers, set the controller input values to 0
c1 = ZeroController(0)
c2 = ZeroController(0)

# Dummy controller modules
cm1 = ControllerModule(
                        controller=c1,
                        state_map=lambda X: X, # full X as controller's current state-vector
                        ref_map=lambda X_r:X_r, # full X_ref as controller's desired state-vector
)


cm2 = ControllerModule(
                        controller=c2,
                        state_map=lambda X: X, # full X as controller's current state-vector
                        ref_map=lambda X_r:X_r, # full X_ref as controller's desired state-vector
)

controller_modules = [cm1, cm2]
# Dummy flight mode
test1_flight_mode = FlightMode(controller_modules=controller_modules)

# Dummy FCS
test0_FCS = FCS(modes={'test1': test1_flight_mode})
test0_FCS.set_mode('test1')

# Dummy Saturator and U_bounds
U_bounds = np.array([[-np.inf, np.inf], [-np.inf, np.inf]])
sat = saturator

# Initial state
X0 = np.array([0, -0.3, 0, 0, 0, 0]) # [x0, -z0, theta0, u0, w0, q0]

# time interval and time-step
t_span = (0, 10)
dt = 0.001

# dummy X_ref_func
X_ref_func = lambda t, X: np.array([0, 0, 0, 0, 0, 0])

log = simulate(
                model=aircraft,
                fcs=test0_FCS,
                saturator=sat,
                X0=X0,
                t_span=t_span,
                dt=dt,
                X_ref_func=X_ref_func,
                U_bounds=U_bounds,
            )

plot_custom(log,
            variables={"z":None, 'Vz': None, 'Vx': None},
            title='Standing on runway, gear down    ')
#plot_simulation(log)
