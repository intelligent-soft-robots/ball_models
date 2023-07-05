import pathlib
from typing import Optional, Sequence

import tomlkit
from numpy import array, cross, hstack, ones, pi
from numpy.linalg import norm

RESET_HEIGHT = 0.76


def load_toml(file_path: str):
    with open(pathlib.Path(file_path), mode="r") as fp:
        config = fp.read()
        config_dict = dict(tomlkit.parse(config))

    return config_dict


class BallTrajectoryNumpy:
    def __init__(self, config_path) -> None:
        config = load_toml(config_path)

        m = config["ball_dynamics"]["ball_mass"]
        r = config["ball_dynamics"]["ball_radius"]
        rho = config["ball_dynamics"]["air_density"]
        g = config["ball_dynamics"]["graviational_constant"]
        A = pi * r**2  # cross-sectional area [m^2]

        c_drag = config["ball_dynamics"]["drag_coefficient"]
        c_lift = config["ball_dynamics"]["lift_coefficient"]
        c_decay = config["ball_dynamics"]["decay_coefficient"]

        self.k_magnus = 0.5 * rho * c_lift * A * r / m
        self.k_drag = -0.5 * rho * c_drag * A / m
        self.k_gravity = g
        self.k_decay = c_decay

    def derivative(self, q: Sequence[float], t: Optional[float] = None):
        q = array(q)

        k_magnus = self.k_magnus
        k_drag = self.k_drag
        k_decay = self.k_decay
        k_gravity = self.k_gravity

        # System states
        v = q[3:6]
        omega = q[6:9]

        F_gravity = k_gravity * array([0, 0, -1])
        F_drag = k_drag * norm(v) * v
        F_magnus = k_magnus * cross(omega, v)

        # System dynamics
        dv_dt = F_gravity + F_drag + F_magnus

        domega_dt = -k_decay * ones(3)
        dq_dt = hstack((v, dv_dt, domega_dt))

        return dq_dt

    def simulate(self, q, duration, dt):
        t = 0.0

        trajectory = []

        while t < duration:
            dq_dt = self.derivative(q)
            q_next = q + dt * dq_dt

            if q_next[2] < RESET_HEIGHT and q_next[5] < 0:
                q_next = linear_contact_model(q)

            trajectory.append(q_next)
            q = q_next
            t += dt

        return trajectory


def linear_contact_model(x_before):
    contact_matrix = array(
        [
            [1, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, -0.95, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 1],
        ]
    )

    x_after = contact_matrix @ x_before
    return x_after
