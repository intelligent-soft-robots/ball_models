import pathlib
from typing import Sequence

import numpy as np
import sympy as sp
import tomlkit

RESET_HEIGHT = 0.76


def to_sym_dict(q):
    q = np.array(q)

    q_sym = {}
    variable_names = [
        "x",
        "y",
        "z",
        "v_x",
        "v_y",
        "v_z",
        "omega_x",
        "omega_y",
        "omega_z",
    ]

    for i, value in enumerate(q):
        q_sym[variable_names[i]] = value

    return q_sym


def linear_contact_model(x_before):
    contact_matrix = np.array(
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


def load_toml(file_path: str):
    with open(pathlib.Path(file_path), mode="r") as fp:
        config = fp.read()
        config_dict = dict(tomlkit.parse(config))

    return config_dict


class BallTrajectorySymbolic:
    def __init__(self, config_path) -> None:
        config = load_toml(config_path)

        self.m_ball = config["ball_dynamics"]["ball_mass"]
        self.r_ball = config["ball_dynamics"]["ball_radius"]
        self.rho = config["ball_dynamics"]["air_density"]
        self.g = config["ball_dynamics"]["graviational_constant"]

        self.c_drag = config["ball_dynamics"]["drag_coefficient"]
        self.c_lift = config["ball_dynamics"]["lift_coefficient"]
        self.c_decay = config["ball_dynamics"]["decay_coefficient"]

        self.A = np.pi * self.r_ball**2

        x, y, z, v_x, v_y, v_z, omega_x, omega_y, omega_z = sp.symbols(
            "x y z v_x v_y v_z omega_x omega_y omega_z"
        )

        self.state_vector = [
            x,
            y,
            z,
            v_x,
            v_y,
            v_z,
            omega_x,
            omega_y,
            omega_z,
        ]

        self._q_dim = len(self.state_vector)
        self.symbolic_model()

    def symbolic_model(self) -> None:
        v_x = self.state_vector[3]
        v_y = self.state_vector[4]
        v_z = self.state_vector[5]
        omega_x = self.state_vector[6]
        omega_y = self.state_vector[7]
        omega_z = self.state_vector[8]

        # substitutions
        v_mag = sp.sqrt(v_x**2 + v_y**2 + v_z**2)

        k_d = -self.rho * self.A * self.c_drag / (2 * self.m_ball)
        k_m = self.rho * self.A * self.c_lift * self.r_ball / (2 * self.m_ball)
        k_g = -self.g

        # equations of motion
        v_x = v_x
        v_y = v_y
        v_z = v_z

        a_x = k_d * v_mag * v_x + k_m * (omega_y * v_z - omega_z * v_y)
        a_y = k_d * v_mag * v_y + k_m * (omega_z * v_x - omega_x * v_z)
        a_z = k_d * v_mag * v_z + k_m * (omega_x * v_y - omega_y * v_x) + k_g

        domega_x_dt = -self.c_decay
        domega_y_dt = -self.c_decay
        domega_z_dt = -self.c_decay

        self.dynamics = sp.Matrix(
            [v_x, v_y, v_z, a_x, a_y, a_z, domega_x_dt, domega_y_dt, domega_z_dt]
        )
        self.jacobian = self.dynamics.jacobian(self.state_vector)

    def get_q_dim(self):
        return self._q_dim

    def lin_jacobian(self, q_lin: Sequence[float]) -> np.ndarray:
        return np.array(self.jacobian.subs(to_sym_dict(q_lin))).astype(np.float64)

    def derivative(self, q: Sequence[float]) -> np.ndarray:
        return np.hstack(self.dynamics.subs(to_sym_dict(q))).astype(np.float64)

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
