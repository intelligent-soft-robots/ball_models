import pathlib
from typing import Optional, Sequence

import ball_models
import matplotlib.pyplot as plt
import tomlkit
from numpy import array, cross, hstack, ones, pi
from numpy.linalg import norm
from numpy.testing import assert_array_almost_equal

RESET_HEIGHT = 0.76


def load_toml(file_path: str):
    with open(pathlib.Path(file_path), mode="r") as fp:
        config = fp.read()
        config_dict = dict(tomlkit.parse(config))

    return config_dict


class FlightModel:
    def __init__(self, physics_config) -> None:
        m = physics_config["ball_dynamics"]["ball_mass"]
        r = physics_config["ball_dynamics"]["ball_radius"]
        rho = physics_config["ball_dynamics"]["air_density"]
        g = physics_config["ball_dynamics"]["graviational_constant"]
        A = pi * r**2  # cross-sectional area [m^2]

        c_drag = physics_config["ball_dynamics"]["drag_coefficient"]
        c_lift = physics_config["ball_dynamics"]["lift_coefficient"]
        c_decay = physics_config["ball_dynamics"]["decay_coefficient"]

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


def simulate(q, duration, dt):
    path = "/home/adittrich/test_workspace/workspace/src/ball_models/config/config.toml"
    config = load_toml(path)

    ball_dynamics_model = FlightModel(config)

    t = 0.0

    trajectory = []

    while t < duration:
        dq_dt = ball_dynamics_model.derivative(q)
        q_next = q + dt * dq_dt

        if q_next[2] < RESET_HEIGHT and q_next[5] < 0:
            q_next = linear_contact_model(q)

        trajectory.append(q_next)
        q = q_next
        t += dt

    return array(trajectory)


def test_model():
    cpp_model = ball_models.BallTrajectory(
        "/home/adittrich/test_workspace/workspace/src/ball_models/config/config.toml"
    )

    q = [0.0, 0.0, 1.0, 12.0, 2.0, 1.2, 100.0, 1.0, 20.0]
    duration = 1.0
    dt = 0.001

    cpp_trajectory = cpp_model.simulate(q, duration, dt)
    py_trajectory = simulate(q, duration, dt)

    cpp_trajectory = array(cpp_trajectory)
    py_trajectory = array(py_trajectory)

    fig, axs = plt.subplots(9, 1)

    for i, ax in enumerate(axs):
        ax.plot(cpp_trajectory[:, i])
        ax.plot(py_trajectory[:, i], linestyle="dashed")

    plt.show()
    assert_array_almost_equal(cpp_trajectory, py_trajectory)


if __name__ == "__main__":
    test_model()
