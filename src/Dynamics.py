import numpy as np

class AV_Model:

    def __init__(self, m, T_max, I_yy, c, s, cg, C_D0, C_Dk, C_L0, C_La, C_m0, C_ma, C_mq, C_m_adot, C_m_de, gear_setting=None):

        """
        Creates an Airborne Vehicle object that undergoes assumed dynamics.

        - General assumptions:

            1. 3DOF (x_I, z_I, theta[pitch])

            2. Body frame of reference has components 
                                                    x (from tale to tip) with velocity u and force X in the same direction 
                                                    z (pointing down from body) with velocity w and force Z in the same direction,
                                                    y (pointing to the right wing) with angular velocity q and moment M
                                                    [untold forces, velocities, moments and angular velocities are not considered in this 3DoF system]

            3. Inertial frame of reference has x_I (pointing to the right) and z_I (pointing up, 90 degrees anti-clockwise from x_I)

            4. AV undergoes gravity force, basic aerodynamic forces and thrust.

            5. Gravity acceleration is always constant(g) and is always pointing to the -z_i direction

            6. Thrust is always in the direction of x with a maximum amount of T_max, controlled by the controll value 0 < delta_t < 1

            7. Iyy is constant

            8. AV mass (m) and it's distribiution (cg) are constant.

            9. No delay in any signal from control and guidance systems to the plant

            10. Navigation/Sensors are perfect and instant

            11. Equations of motion are calculated in body frame but aerodynamics forces are calculated in stability frame of reference.
            
            12. No wind in any direction

            13. Center of aerodynamic pressure (ac) is constant and considered to be in 0.25 * c (one quarter of mean aerodynamic chord length),
                    so the (ac - cg) is equal to (0.25 - cg) * c when calculating lift and drag moments
            
            14. No mach number effect

            15. Air density (rho) is constant

            16. No flap setting


        - Simulation assumptions:

            1. Fixed time-step

            2. RK4 method is used for solving ODE's


        - Dynamical system assumptions:

            1. State vector X = [x_I, z_I, theta, u, w, q]
                (Note that the first three elements are inertial but the others are in body frame)

            2. System input U = [delta_t, delta_e]

            3. System output Y = [x_I, z_I, theta, u, w, q] (full observability)

            4. System is considered to be a non-linear time invarient multiple input multiple output (NLTI-MIMO):
                |Xdot = F(X, U)
                |Y = G(X, U)


        - Dynamics:
            
            -------------------------------------------------------------------------

            definitions:

                alpha = atan(w/u) [Angle of Attack (AoA)]

                V = sqrt(u^2 + w^2) [True Air Speed (TAS)]


            -------------------------------------------------------------------------

            Equations of translation in body frame of reference:

                u_dot = F_X / m - qw
                w_dot = F_Z / m + qu

                u = integral(u_dot)
                w = integral(w_dot)

                [v_x v_z].T = R_BI * [u w].T (from body frame to inertial)

                    where R_BI = |  cos(theta)  sin(theta) |
                                 | -sin(theta)  cos(theta) |

                (** note that the calculated v_z is in reallity v_down. so to not get negative altitudes, it should be multiplied by -1)

                x_I = integral(v_x)
                z_I = integral(v_z)


            Equations of rotation in inertial frame of reference:

                q_dot = M / I_yy

                q = integral(q_dot)

                theta_dot = 1 * q (from body frame to inertial) [q is the theta_dot itself based on the 6DoF DCM]

                theta = integral(q)
                

            -------------------------------------------------------------------------

            Terms used in equations above are defined as follows:

                Forces:

                    F_X = L_s * sin(alpha) - D_s * cos(alpha) + mg * sin(theta) + T_max * delta_t

                    F_Z = -L_s * cos(alpha) - D_s * sin(alpha) + mg * cos(theta)


                Moments:

                    M = M_s + L_s * (cg - 0.25) * c * cos(alpha) + D_s * (cg - 0.25) * c * sin(alpha)

                Where:
    
                    L_s = (1/2) * rho * (V^2) * s * C_L [where C_L = C_L0 + C_La * alpha and s is wing surface]
 
                    D_s = (1/2) * rho * (V^2) * s * C_D [where C_D = C_D0 + C_Dk * C_L^2 and s is wing surface]

                    M_s = (1/2) * rho * (V^2) * s * c * C_m + (1/4) * rho * V * s * c^2 * (C_mq * q + C_m_adot * alphadot)
                            [where C_m = C_m0 + C_ma * alpha + C_m_de * delta_e and c is mean aerodynamic chord]


            -------------------------------------------------------------------------


        """

        # Environment parameters --

        self.g = 9.81
        self.rho = 1.225

        # -------------------------


        # AV body parameters -----------------------------------------------------------------------------

        self.m = m # AV mass
        self.I_yy = I_yy
        self.c = c # AV reference aerodynamic length (mean aerodynamic chord)
        self.s = s # AV reference aerodynamic surface (wing surface)
        self.T_max = T_max # Maximum generated thrust
        self.cg = cg # position of C/G with respect to the refrence point normalized by c 
        # (reference point is the place that has 0.25c distance to aerodynamic or pressure center)
        self.gear_setting = gear_setting


        # ------------------------------------------------------------------------------------------------


        # AV aerodynamic coefficients --------------------------------------------------------------------------------------------

        # Drag
        
        self.C_D0 = C_D0
        self.C_Dk = C_Dk


        # Lift 

        self.C_L0 = C_L0
        self.C_La = C_La


        # Aerodynamic moment coefficients 

        self.C_m0 = C_m0
        self.C_ma = C_ma
        self.C_m_de = C_m_de
        self.C_mq = C_mq
        self.C_m_adot = C_m_adot
        
        # -----------------------------------------------------------------------------------------------------------------------


    def X_dot(self, t, X, U):

        """
        returns the derivative of state space vector (X), for a given X and U (control input vector):
        Xdot = [v_x, v_z, theta_dot, u_dot, w_dot, q_dot]

        (t is among the arguments to preserve generality, but here, the system is designed time invariant [NLTI MIMO])

        """

        # Unpacking state and control vectors for ease of usage

        [x_I, z_I, theta, u, w, q] = X
        [delta_t, delta_e] = U


        # Caclulating AoA and TAS

        alpha = np.arctan2(w, u)
        V = np.sqrt(u**2 + w**2)

        # Lift and drag coefficients

        C_L = self.C_L0 + alpha * self.C_La
        C_D = self.C_D0 + self.C_Dk * (C_L ** 2)
        

        # Lift and drag in stability frame

        L_s = (1/2) * self.rho * (V**2) * self.s * C_L
        D_s = (1/2) * self.rho * (V**2) * self.s * C_D
        
        # Section 1: Calculating Forces and Translational dynamics:

        # Forces in body frame

        F_X = L_s * np.sin(alpha) - D_s * np.cos(alpha) - self.m * self.g * np.sin(theta) + self.T_max * delta_t
        F_Z = -L_s * np.cos(alpha) - D_s * np.sin(alpha) + self.m * self.g * np.cos(theta)

        # modeling the ground dynamics with a simple spring-damper gear, if gear setting is given

        if self.gear_setting is not None: 
            l_G = self.gear_setting['l_G'] # gear resting length

            # checking for ground-gear contact
            if abs(z_I) < l_G:

                k_G, b_G = self.gear_setting['k_G'], self.gear_setting['b_G'] # spring and damper coefficients of gear
                v_z = u * -np.sin(theta) + w * np.cos(theta) # calculating spring velocity for damper term
                F_G = -k_G * (l_G + z_I) - b_G * v_z # calculated in inertial z 

                # transforming to body frame
                F_Z += F_G * np.cos(theta)
                F_X -= F_G * np.sin(theta)



        # velocity derivatives in body frame

        u_dot = F_X / self.m - q * w
        w_dot = F_Z / self.m + q * u


        # Section 2: Calculating Moments and Rotational Dynamics:

        # estimating alphadot for alpha near zero

        alphadot = (u * w_dot - w * u_dot) / V**2 if abs(V)>1e-6 else 0 # considering near zero values of V


        # pitching moment coefficient

        C_m = self.C_m0 + self.C_ma * alpha + self.C_m_de * delta_e


        # Moments in stability frame

        M_s = (1/2) * self.rho * (V**2) * self.s * self.c * C_m + (1/4) * self.rho * V * self.s * self.c**2 * (self.C_mq * q + self.C_m_adot * alphadot)


        # Moments in body frame

        M = M_s + L_s * (self.cg - 0.25) * self.c * np.cos(alpha) + D_s * (self.cg - 0.25) * self.c * np.sin(alpha)


        # Angular velocity derivatives in body frame

        q_dot = M / self.I_yy

        # Velocities in inertial frame: [v_x v_z].T = R_BI * [u w].T (from body frame to inertial)

        v_x = u * np.cos(theta) + w * np.sin(theta)
        v_z = u * -np.sin(theta) + w * np.cos(theta)
        theta_dot = q

        Xdot = np.array([v_x, v_z, theta_dot, u_dot, w_dot, q_dot])

        return Xdot
