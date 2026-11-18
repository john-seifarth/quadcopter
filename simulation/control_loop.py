import csv
import io
import subprocess

from algorithms import pid
from simulation import dynamics


def pi_control_simulation(
    omega_target: float,

    theta_0: float,
    omega_0: float,
    inertia: float,
    damping: float,
    max_torque: float,

    duration: float,
    time_step: float,

    K_p: float,
    K_i: float
):

    # validate input parameters
    if inertia <= 0:
        raise ValueError("Inertia must be positive.")

    if time_step <= 0:
        raise ValueError("Time step must be positive.")

    if duration < 0:
        raise ValueError("Duration cannot be negative.")

    if damping < 0:
        raise ValueError("Damping cannot be negative.")

    if max_torque <= 0:
        raise ValueError("Maximum torque must be positive.")

    # initialize logging lists
    times = []
    thetas = []
    omegas = []
    alphas = []
    torques = []

    # initialize recurring vars
    t = 0
    omega = omega_0
    theta = theta_0
    accumulated_error = 0

    # loop through entire duration
    while t < duration:
        dt = min(time_step, duration - t)

        # get the commmanded torque and log
        commanded_torque = pid.pi_control(
            omega_target,
            omega,
            K_p,
            max_torque
        )
        torques.append(commanded_torque)

        # calculate alpha and append, append current omega and theta
        alphas.append((commanded_torque - damping * omega) / inertia)
        thetas.append(theta)
        omegas.append(omega)

        # update theta and omega
        theta, omega = dynamics.step_rotation_1d(
            theta,
            omega,
            commanded_torque,
            inertia,
            damping,
            dt
        )

        # log and update the time
        times.append(t)
        t += time_step

    return {
        "time": times,
        "theta": thetas,
        "omega": omegas,
        "alpha": alphas,
        "torque": torques
    }


def view_results_csv(results: dict[str, list[float]]) -> None:
    buffer = io.StringIO()
    writer = csv.writer(buffer)

    writer.writerow(results.keys())
    writer.writerows(zip(*results.values()))

    subprocess.run(
        ["code", "--reuse-window", "-"],
        input=buffer.getvalue(),
        text=True,
        check=True
    )


results = proportional_control_simulation(
    omega_target=1.0,      # rad/s
    theta_0=0.0,           # rad
    omega_0=0.0,           # rad/s
    inertia=0.01,         # kg*m²
    damping=0.002,        # N*m*s/rad
    max_torque=0.05,      # N*m
    duration=2.5,         # seconds
    time_step=0.004,      # seconds (250 Hz)
    K_p=0.02
)

view_results_csv(results)
