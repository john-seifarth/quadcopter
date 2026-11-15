
def step_rotation_1d(
    theta: float,
    omega: float,
    torque: float,
    inertia: float,
    damping: float,
    dt: float
) -> tuple[float, float]:

    alpha = (torque - damping * omega) / inertia

    omega_new = omega + alpha * dt
    theta_new = theta + omega * dt

    return theta_new, omega_new


def simulate_rotation_1d(
    commanded_torque: float,
    theta: float,
    omega: float,
    inertia: float,
    damping: float,
    duration: float,
    time_step: float
) -> dict[str, list[float]]:

    if inertia <= 0:
        raise ValueError("Inertia must be positive.")
    if time_step <= 0:
        raise ValueError("Time step must be positive.")
    if duration < 0:
        raise ValueError("Duration cannot be negative.")

    log = {
        "time": [0.0],
        "theta": [theta],
        "omega": [omega],
        "alpha": [(commanded_torque - damping * omega) / inertia],
        "torque": [commanded_torque]
    }

    t = 0.0

    while t < duration:
        dt = min(time_step, duration - t)

        theta, omega = step_rotation_1d(
            theta, omega, commanded_torque,
            inertia, damping, dt
        )

        t = min(duration, t + dt)

        data = {
            "time": t,
            "theta": theta,
            "omega": omega,
            "alpha": (commanded_torque - damping * omega) / inertia,
            "torque": commanded_torque
        }

        for key, value in data.items():
            log[key].append(value)

    return log


# Test 1: Constant positive torque, starting from rest

commanded_torque = 0.05  # N·m
theta = 0.0              # rad
omega = 0.0              # rad/s
inertia = 0.01           # kg·m²
damping = 0.002          # N·m·s/rad
duration = 2.5           # s
time_step = 0.004        # s (250 Hz)

results = simulate_rotation_1d(
    commanded_torque=commanded_torque,
    theta=theta,
    omega=omega,
    inertia=inertia,
    damping=damping,
    duration=duration,
    time_step=time_step
)

def print_simulation_log(log: dict[str, list[float]]) -> None:
    print("  ".join(f"{key:>12}" for key in log))

    for row in zip(*log.values()):
        print("  ".join(f"{value:12.4f}" for value in row))

print_simulation_log(results)
