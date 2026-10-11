from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


# Paths
EXAMPLES_DIR = Path(__file__).resolve().parent
CSV_PATH = EXAMPLES_DIR / "open_loop_asymmetric_state.csv"
RESULTS_DIR = EXAMPLES_DIR / "results"


# Publication-style plotting defaults
plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 10,
    "legend.fontsize": 8,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "lines.linewidth": 1.25,
    "axes.linewidth": 0.8,
    "axes.grid": True,
    "grid.alpha": 0.25,
    "grid.linewidth": 0.6,
    "figure.facecolor": "white",
    "axes.facecolor": "white",
    "savefig.facecolor": "white",
    "savefig.dpi": 300,
})


def save_figure(fig, filename):
    """Save a high-resolution PNG figure."""
    png_path = RESULTS_DIR / f"{filename}.png"

    fig.savefig(
        png_path,
        format="png",
        dpi=300,
        bbox_inches="tight",
        facecolor="white",
    )

    print(f"Saved: {png_path}")


def load_data():
    """Load and validate the C++ open-loop simulation data."""
    if not CSV_PATH.exists():
        raise FileNotFoundError(
            f"Simulation CSV not found:\n{CSV_PATH}\n"
            "Run the C++ open-loop simulation first."
        )

    data = pd.read_csv(CSV_PATH)

    required_columns = [
        "time_s",
        "pn", "pe", "pd",
        "ub", "vb", "wb",
        "qw", "qx", "qy", "qz",
        "p", "q", "r",
        "rotor1_rad_s", "rotor2_rad_s",
        "rotor3_rad_s", "rotor4_rad_s",
    ]

    missing = sorted(set(required_columns) - set(data.columns))
    if missing:
        raise ValueError(f"Missing CSV columns: {missing}")

    if data.empty:
        raise ValueError("The simulation CSV contains no data.")

    numeric_data = data[required_columns].to_numpy(dtype=float)

    if not np.isfinite(numeric_data).all():
        raise ValueError("The CSV contains NaN or infinite values.")

    time = data["time_s"].to_numpy()

    if not np.all(np.diff(time) > 0):
        raise ValueError("Time values must be strictly increasing.")

    return data


def plot_trajectory(data):
    """Plot the trajectory in East-North-Up display coordinates."""
    east = data["pe"].to_numpy()
    north = data["pn"].to_numpy()
    up = -data["pd"].to_numpy()

    fig = plt.figure(figsize=(8.0, 6.2))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(
        east, north, up,
        color="tab:blue",
        linewidth=1.3,
        label="Trajectory",
    )

    ax.scatter(
        east[0], north[0], up[0],
        color="tab:green",
        s=35,
        label="Start",
        depthshade=False,
    )

    ax.scatter(
        east[-1], north[-1], up[-1],
        color="tab:red",
        marker="x",
        s=45,
        label="End",
        depthshade=False,
    )

    # Horizontal projection at the initial altitude
    ax.plot(
        east,
        north,
        np.full_like(up, up[0]),
        color="0.45",
        linestyle="--",
        linewidth=0.8,
        alpha=0.7,
        label="Horizontal projection",
    )

    ax.set_xlabel("East (m)", labelpad=8)
    ax.set_ylabel("North (m)", labelpad=8)
    ax.set_zlabel("Up (m)", labelpad=6)
    ax.set_title("Open-loop quadcopter trajectory", pad=12)
    ax.legend(loc="best", frameon=True)

    # Set the box proportions from the data range.
    spans = np.array([
        np.ptp(east),
        np.ptp(north),
        np.ptp(up),
    ])
    spans[spans < 1e-6] = 1.0
    ax.set_box_aspect(spans)
    ax.view_init(elev=24, azim=-55)

    fig.tight_layout()
    save_figure(fig, "open_loop_trajectory_3d")

    return fig


def plot_full_state(data):
    """Plot position, velocity, quaternion, rates, and rotor speeds."""
    time = data["time_s"]

    fig, axes = plt.subplots(
        5,
        1,
        figsize=(10.0, 12.0),
        sharex=True,
        constrained_layout=True,
    )

    # Position in the NED frame
    axes[0].plot(time, data["pn"], label="North")
    axes[0].plot(time, data["pe"], label="East")
    axes[0].plot(time, data["pd"], label="Down")
    axes[0].set_ylabel("Position (m)")
    axes[0].set_title("Position (NED)")
    axes[0].legend(ncol=3, loc="best")

    # Body-frame translational velocity
    axes[1].plot(time, data["ub"], label=r"$u_b$")
    axes[1].plot(time, data["vb"], label=r"$v_b$")
    axes[1].plot(time, data["wb"], label=r"$w_b$")
    axes[1].set_ylabel("Velocity (m/s)")
    axes[1].set_title("Body-frame velocity")
    axes[1].legend(ncol=3, loc="best")

    # Scalar-first quaternion, plotted without modifying the data
    quaternion_columns = [
        ("qw", r"$q_w$"),
        ("qx", r"$q_x$"),
        ("qy", r"$q_y$"),
        ("qz", r"$q_z$"),
    ]

    for column, label in quaternion_columns:
        axes[2].plot(time, data[column], label=label)

    axes[2].set_ylabel("Quaternion")
    axes[2].set_title("Attitude quaternion")
    axes[2].legend(ncol=4, loc="best")

    # Body angular rates
    axes[3].plot(time, data["p"], label=r"$p$")
    axes[3].plot(time, data["q"], label=r"$q$")
    axes[3].plot(time, data["r"], label=r"$r$")
    axes[3].set_ylabel("Rate (rad/s)")
    axes[3].set_title("Body angular rates")
    axes[3].legend(ncol=3, loc="best")

    # Four rotor angular speeds
    for rotor_index in range(1, 5):
        column = f"rotor{rotor_index}_rad_s"
        axes[4].plot(
            time,
            data[column],
            label=f"Rotor {rotor_index}",
        )

    axes[4].set_ylabel("Speed (rad/s)")
    axes[4].set_xlabel("Time (s)")
    axes[4].set_title("Rotor angular speeds")
    axes[4].legend(ncol=4, loc="best")

    for ax in axes:
        ax.margins(x=0.01)
        ax.grid(True, which="major")
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

    fig.suptitle(
        "Open-loop simulation: state time histories",
        fontsize=12,
    )

    save_figure(fig, "open_loop_full_state")
    return fig


def plot_body_rates(data):
    """Plot body angular rates separately for easier inspection."""
    time = data["time_s"]

    fig, ax = plt.subplots(figsize=(8.5, 4.5))

    ax.plot(time, data["p"], label=r"$p$ (roll rate)")
    ax.plot(time, data["q"], label=r"$q$ (pitch rate)")
    ax.plot(time, data["r"], label=r"$r$ (yaw rate)")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular rate (rad/s)")
    ax.set_title("Body angular-rate response")
    ax.legend(loc="best")

    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout()
    save_figure(fig, "open_loop_body_rates")
    return fig


def plot_rotor_speeds(data):
    """Plot individual rotor-speed histories."""
    time = data["time_s"]

    fig, ax = plt.subplots(figsize=(8.5, 4.5))

    for rotor_index in range(1, 5):
        column = f"rotor{rotor_index}_rad_s"
        ax.plot(time, data[column], label=f"Rotor {rotor_index}")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Rotor speed (rad/s)")
    ax.set_title("Rotor-speed response")
    ax.legend(loc="best", ncol=2)

    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    fig.tight_layout()
    save_figure(fig, "open_loop_rotor_speeds")
    return fig


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    data = load_data()
    time = data["time_s"]

    print(f"Samples: {len(data)}")
    print(
        "Duration: "
        f"{time.iloc[-1] - time.iloc[0]:.3f} s"
    )
    print(f"Results directory: {RESULTS_DIR}")

    plot_trajectory(data)
    plot_full_state(data)
    plot_body_rates(data)
    plot_rotor_speeds(data)

    plt.show()


if __name__ == "__main__":
    main()
