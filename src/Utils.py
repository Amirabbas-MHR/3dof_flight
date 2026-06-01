from scipy.optimize import fsolve
import numpy as np

def _slf_trim_f(inputs, av_model, V):

    '''
    steady level flight function (same as AV_3DoF_Model.X_dot, but for slf trim condition and inputs = [alpha, delta_e, delta_t])
    returns u_dot, w_dot and q_dot
    this function is solved to be 0 using scipy fsolve, to find the trimming values of alpha, delta_e and delta_t
    returns [u_dot, w_dot, q_dot]
    '''
    alpha, delta_e, delta_t = inputs
    
    theta = alpha # no flight path angle, straight flying in the positive x_I direction

    u = V * np.cos(theta)
    w = V * np.sin(theta)

    X = np.array([0, -100, theta, u, w, 0]) # altitude is set to 100 meters so that gear-ground forces dont have an effect
    U = np.array([delta_t, delta_e])

    Xdot = av_model.X_dot(0, X, U)

    u_dot = Xdot[3]
    w_dot = Xdot[4]
    q_dot = Xdot[5]

    return np.array([u_dot, w_dot, q_dot])


def _scf_trim_f(inputs, av_model, Vx, Vz):

    '''
    steady climb flight function (same as AV_3DoF_Model.X_dot, but for scf trim condition and inputs = [alpha, delta_e, delta_t])
    returns u_dot, w_dot and q_dot
    this function is solved to be 0 using scipy fsolve, to find the trimming values of alpha, delta_e and delta_t
    returns [u_dot, w_dot, q_dot]
    '''
    alpha, delta_e, delta_t = inputs

    # calculating theta based on inputs (alpha, Vx, Vz)
    gamma = np.arctan2(-Vz, Vx) # flight path angle, with z-down convention consideration
    
    theta = alpha + gamma

    u = Vx * np.cos(theta) - Vz * np.sin(theta)
    w = Vx * np.sin(theta) + Vz * np.cos(theta)

    X = np.array([0, -100, theta, u, w, 0]) # altitude is set to 100 meters so that gear-ground forces dont have an effect
    U = np.array([delta_t, delta_e])

    Xdot = av_model.X_dot(0, X, U)

    u_dot = Xdot[3]
    w_dot = Xdot[4]
    q_dot = Xdot[5]

    return np.array([u_dot, w_dot, q_dot])


def trimmer(av_model, mode, variables, initial_guess):

    if mode == "slf":
        # Steady level fligt condition (where Vx is non-zero and Vz = theta_dot = u_dot = w_dot = q_dot = 0)
        # In this condition three variables are unkonw and are to be find: alpha, delta_e, delta_t
        # for that we got three equations, theta_dot = 0, u_dot = 0, w_dot = 0, where the left hand sides are a function of [alpha, delta_e, delta_t]
        # so the roots of this function (trim values) can be found using fsolve
        # note that fsolve starts the root finding algorithm from initial guess [alpha_o, delta_e_o, delta_t_o]

        V = variables['Vx']

        sol, infodict, ier, msg = fsolve(_slf_trim_f, initial_guess, args=(av_model, V), full_output=True)

        if ier != 1:
            raise RuntimeError(f"{mode} trimmer failed: {msg}")

        alpha_T, delta_e_T, delta_t_T = sol

        return alpha_T, delta_e_T, delta_t_T

    if mode == "scf":
        # Steady climb fligt condition (where Vx and Vz are non-zero and  theta_dot = u_dot = w_dot = q_dot = 0)
        # In this condition three variables are unkonw and are to be find: alpha, delta_e, delta_t
        # for that we got three equations, theta_dot = 0, u_dot = 0, w_dot = 0, where the left hand sides are a function of [alpha, delta_e, delta_t]
        # so the roots of this function (trim values) can be found using fsolve
        # note that fsolve starts the root finding algorithm from initial guess [alpha_o, delta_e_o, delta_t_o]

        Vx = variables['Vx']
        Vz = variables['Vz']
        
        sol, infodict, ier, msg = fsolve(_scf_trim_f, initial_guess, args=(av_model, Vx, Vz), full_output=True)
        if ier != 1:
            raise RuntimeError(f"{mode} trimmer failed: {msg}")

        alpha_T, delta_e_T, delta_t_T = sol

        return alpha_T, delta_e_T, delta_t_T

    else:
        raise ValueError(f'mode {mode} not supported')


def saturator(U, U_bounds):

    return np.clip(U, U_bounds[:, 0], U_bounds[:, 1])


class Step_disturbance:

    def __init__(self, index, time_interval, magnitude):
        """
        use for testing controllers. initiate with an index (element of X_dot to put the disturbance on),
        desired magnitude and time-interval to apply disturbance.
        """
        self.index = index
        self.ti, self.tf = time_interval
        self.magnitude = magnitude

    def compute(self, t, X):

        dX = np.zeros(len(X))

        if t >= self.ti and t<= self.tf:
            dX[self.index] = self.magnitude

        return dX