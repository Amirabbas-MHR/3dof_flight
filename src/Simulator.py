import numpy as np


def simulate(
    model,              # object with method: X_dot(t, X, U)
    fcs,                # object with method: compute(t, X, X_ref)
    saturator,          # function: Saturator(U, U_bounds)
    X0,                 # initial state
    t_span,             # (t0, tf)
    dt,                 # integrating time-step (fixed)
    X_ref_func,         # function: X_ref(t)
    U_bounds,           # [[u0_min u0_max], [u1_min u1_max], ...]
    disturbance=None,   # disturbance function(t, X) [applied to X_dot]
):
    
    """
    simulates a given model of type [Dynamics.AV_Model], with a designed FCS system of type [FlightControlSystem.FCS],
    with initial condition X0, for t_span time interval, integrating (RK4) with time-step dt.

    returns log (a dict of recorded simulation values, in sync.)
    """
    t0, tf = t_span
    N = int(np.floor((tf - t0) / dt)) + 1 # number of simulation steps
    ts = t0 + np.arange(N) * dt # time stamps of simulation (calculated this way to overcome floating point accumulation error)

    Xdim = len(X0) # number of state vector elements
    Udim = len(U_bounds) # number of input vector elements

    # log dict initiation
    log = {
            "X": np.zeros((N, Xdim)),
            "U_cmd": np.zeros((N, Udim)), 
            "U_sat": np.zeros((N, Udim)),
            "AoA": np.zeros(N),
            "TAS": np.zeros(N),
            "t": ts,
            } 

    # logging initial values in log dict
    log["X"][0] = X0
    log["U_cmd"][0] = np.array(Udim * [np.nan]) # using nan as initial values of control signals, since they are not generated yet in t0
    log["U_sat"][0] = np.array(Udim * [np.nan])
    log["AoA"][0] = np.arctan2(X0[4], X0[3]) # calculating initial angle of attack based on X0 values
    log["TAS"][0] = np.sqrt(X0[4]**2 + X0[3]**2) # calculating initial true air speed based on X0 values

    
    X = X0.copy()
    for step in range(1, N):

        # getting current time
        t = ts[step-1]

        # Closed loop (Guidance/Controller/Saturator/Plant/Feedback):

        # --- reference (To be replaced with guidance systems later) ---
        X_ref = X_ref_func(t, X)

        # --- control ---
        U_cmd = fcs.compute(t, X, X_ref)
        
        # --- saturator ---
        U_sat = saturator(U_cmd, U_bounds) if U_bounds is not None else U_cmd

        # --- disturbance ---
        if disturbance is not None:
            dX = disturbance(t, X)
        else:
            dX = 0

        # RK4 on plant state vector derivative (X_dot) to achive next X
        # Note that the control value (U_sat) is calculated once per RK4, which is similar to a ZOH behaviour of a real digital controller.
        # Also disturbance is assumed to have a ZOH hold, and is added to Xdot

        k1 = model.X_dot(t, X, U_sat) + dX
        k2 = model.X_dot(t + dt/2, X + dt/2 * k1, U_sat) + dX
        k3 = model.X_dot(t + dt/2, X + dt/2 * k2, U_sat) + dX
        k4 = model.X_dot(t + dt, X + dt * k3, U_sat) + dX
        X = X + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

        # logging
        log["X"][step] = X
        log["U_cmd"][step] = U_cmd
        log["U_sat"][step] = U_sat
        log["AoA"][step] = np.arctan2(X[4], X[3]) # calculating angle of attack based on X values
        log["TAS"][step] = np.sqrt(X[4]**2 + X[3]**2) # calculating true air speed based on X values

    return log
