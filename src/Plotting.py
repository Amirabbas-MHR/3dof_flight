import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt


# ---------------------------
# STYLE SYSTEM
# ---------------------------
def _set_style():
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 11,

        "axes.titlesize": 14,
        "axes.labelsize": 11,

        "axes.facecolor": "#f8f9fb",
        "figure.facecolor": "white",

        "axes.edgecolor": "#444444",
        "axes.linewidth": 0.8,

        "grid.color": "#bbbbbb",
        "grid.alpha": 0.4,

        "lines.linewidth": 2.0,

        "legend.frameon": False,
    })


def _add_grid(ax):
    ax.grid(True, which='major', linewidth=0.8)
    ax.minorticks_on()
    ax.grid(True, which='minor', linewidth=0.3, alpha=0.2)


# ---------------------------
# MAIN PLOT FUNCTION
# ---------------------------
def plot_simulation(log, plot_switches={"trajectory":True, "attitude":True, "body_v":True, "inertial_v":True, "aero":True, "control":True}):
    _set_style()

    t = log["t"]
    X = np.array(log["X"])
    U_cmd = np.array(log["U_cmd"])
    U_sat = np.array(log["U_sat"])

    # STATES
    x, z, theta, u, w, q = X.T

    # due to z-down convention
    z = -z

    # radians → degrees
    theta_deg = np.degrees(theta)
    q_deg = np.degrees(q)
    aoa_deg = np.degrees(log["AoA"])
    delta_e_deg = np.degrees(U_sat[:, 1])

    # inertial velocities
    Vx = u * np.cos(theta) + w * np.sin(theta)
    Vz = -u * np.sin(theta) + w * np.cos(theta)

    FIGSIZE = (10, 6)

    # =========================
    # TRAJECTORY WITH ATTITUDE ARROWS
    # =========================
    if plot_switches['trajectory']:
        fig_traj, ax = plt.subplots(figsize=FIGSIZE, constrained_layout=True)
        ax.set_title("Trajectory (Inertial Frame)")

        ax.plot(x, z, color="#2c7be5", label="Trajectory")
        ax.fill_between(x, z, z.min(), alpha=0.05)

        # ---- Attitude arrows ----
        N = len(t)

        step = max(1, N // 25)  # ~25 arrows max (adaptive to sim length)

        arrow_scale = 0.05 * np.sqrt((x.max() - x.min())**2 + (z.max() - z.min())**2)  # scale based on trajectory size

        for i in range(0, N, step):
            dx = np.cos(theta[i])
            dz = np.sin(theta[i])
            ax.arrow(
                x[i], z[i],
                dx * arrow_scale,
                dz * arrow_scale,
                head_width=arrow_scale * 0.15,
                head_length=arrow_scale * 0.3,
                fc="#d62728",
                ec="#d62728",
                alpha=0.85,
                length_includes_head=True
            )

        ax.set_xlabel("x [m]")
        ax.set_ylabel("Altitude [m]")
        ax.axis('equal')

        _add_grid(ax)

    # =========================
    # ATTITUDE + RATES
    # =========================
    if plot_switches['attitude']:
        fig1, ax = plt.subplots(2, 1, sharex=True,
                                figsize=FIGSIZE, constrained_layout=True)
        fig1.suptitle("Attitude Dynamics")

        ax[0].plot(t, theta_deg, color="#d62728")
        ax[0].set_ylabel("θ [deg]")
        _add_grid(ax[0])

        ax[1].plot(t, q_deg, color="#ff7f0e")
        ax[1].set_ylabel("q [deg/s]")
        ax[1].set_xlabel("Time [s]")
        _add_grid(ax[1])

    # =========================
    # BODY VELOCITIES
    # =========================
    if plot_switches['body_v']:
        fig2, ax = plt.subplots(2, 1, sharex=True,
                                figsize=FIGSIZE, constrained_layout=True)
        fig2.suptitle("Body Velocities")

        ax[0].plot(t, u, color="#2ca02c")
        ax[0].set_ylabel("u [m/s]")
        _add_grid(ax[0])

        ax[1].plot(t, w, color="#9467bd")
        ax[1].set_ylabel("w [m/s]")
        ax[1].set_xlabel("Time [s]")
        _add_grid(ax[1])

    # =========================
    # INERTIAL VELOCITIES
    # =========================
    if plot_switches['inertial_v']:
        fig2b, ax = plt.subplots(2, 1, sharex=True,
                                 figsize=FIGSIZE, constrained_layout=True)
        fig2b.suptitle("Inertial Velocities")

        ax[0].plot(t, Vx, color="#1f77b4")
        ax[0].set_ylabel("Vx [m/s]")
        _add_grid(ax[0])

        ax[1].plot(t, -Vz, color="#17becf") #due to z-down convention
        ax[1].set_ylabel("Vz [m/s]")
        ax[1].set_xlabel("Time [s]")
        _add_grid(ax[1])

    # =========================
    # AERODYNAMICS
    # =========================
    if plot_switches['aero']:
        fig3, ax = plt.subplots(2, 1, sharex=True,
                                figsize=FIGSIZE, constrained_layout=True)
        fig3.suptitle("Aerodynamic States")

        ax[0].plot(t, aoa_deg, color="#bcbd22")
        ax[0].set_ylabel("AoA [deg]")
        _add_grid(ax[0])

        ax[1].plot(t, log["TAS"], color="#2ca02c")
        ax[1].set_ylabel("TAS [m/s]")
        ax[1].set_xlabel("Time [s]")
        _add_grid(ax[1])

    # =========================
    # CONTROL INPUTS
    # =========================
    if plot_switches['control']:
        fig4, ax = plt.subplots(2, 1, sharex=True,
                                figsize=FIGSIZE, constrained_layout=True)
        fig4.suptitle("Control System")

        ax[0].plot(t, U_cmd[:, 0], "--", label="cmd", color="#888888")
        ax[0].plot(t, U_sat[:, 0], label="sat", color="#1f77b4")
        ax[0].set_ylabel("Throttle [-]")
        ax[0].legend()
        _add_grid(ax[0])

        ax[1].plot(t, np.degrees(U_cmd[:, 1]), "--", label="cmd", color="#888888")
        ax[1].plot(t, delta_e_deg, label="sat", color="#d62728")
        ax[1].set_ylabel("δe [deg]")
        ax[1].set_xlabel("Time [s]")
        ax[1].legend()
        _add_grid(ax[1])

    plt.show()

def plot_custom(log, variables, title='custom signal stack'):
    """
    variables:
        dict format:
            {
                "theta": (-5, 5),
                "Vz": (-0.5, 0.5),
                "q": None   # auto-scale
            }

        OR list format (fallback to auto-scale):
            ["theta", "Vz"]
    """

    _set_style()

    t = log["t"]
    X = np.array(log["X"])
    U_sat = np.array(log["U_sat"])

    x, z, theta, u, w, q = X.T
    z = -z

    # Derived + unit-fixed signals
    mapping = {
        "x": x,
        "z": z,
        "theta": np.degrees(theta),
        "q": np.degrees(q),
        "u": u,
        "w": w,
        "AoA": np.degrees(log["AoA"]),
        "TAS": log["TAS"],
        "delta_t": U_sat[:, 0],
        "delta_e": np.degrees(U_sat[:, 1]),
        "Vx": u * np.cos(theta) + w * np.sin(theta),
        "Vz": -(-u * np.sin(theta) + w * np.cos(theta)), # due to zdown convention
    }

    # Normalize input
    if isinstance(variables, list):
        variables = {var: None for var in variables}

    fig, axes = plt.subplots(len(variables), 1,
                             sharex=True,
                             figsize=(10, 6),
                             constrained_layout=True)

    fig.suptitle(title, fontsize=14)

    if len(variables) == 1:
        axes = [axes]

    for i, (var, ylim) in enumerate(variables.items()):
        if var not in mapping:
            raise ValueError(f"Unknown variable: {var}")

        signal = mapping[var]

        axes[i].plot(t, signal, color="#2c7be5")
        axes[i].set_ylabel(var)

        # manual ylim
        if ylim is not None:
            axes[i].set_ylim(ylim)

        _add_grid(axes[i])

    axes[-1].set_xlabel("Time [s]")

    plt.show()