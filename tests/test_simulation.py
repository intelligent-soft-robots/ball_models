import time

import matplotlib.pyplot as plt
import numpy as np

import ball_models


def test_sim():
    model = ball_models.BallTrajectory(
        "/home/adittrich/test_workspace/workspace/src/ball_models/config/config.toml"
    )

    duration = 1.0
    dt = 0.001

    q = np.array([0.0, 0.0, 1.0, 1.0, 1.0, 1.5, 0.0, 0.0, 9.8])
    t = np.arange(0, duration, dt)

    start_time = time.time()
    trajectory = model.simulate(q, duration, dt)
    delta_time = time.time() - start_time

    print(f"Runtime: {delta_time} s")
    print(f"type trajectory: {type(trajectory)}")
    print(f"shape trajectory: {np.array(trajectory).shape}")
    print(f"sample: {trajectory[1]}")
    print(f"type sample: {type(trajectory[1])}")

    plt.plot(t, np.array(trajectory)[:, 2])
    plt.show()


if __name__ == "__main__":
    test_sim()
