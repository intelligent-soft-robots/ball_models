import pathlib
import time

import matplotlib.pyplot as plt
import numpy as np

import ball_models

script_dir = pathlib.Path(__file__).resolve().parent
config_dir = script_dir.parent / "config"
path = config_dir / "config.toml"
CONFIG_PATH = str(path)


def test_sim():
    model = ball_models.BallTrajectory(CONFIG_PATH)

    duration = 1.0
    dt = 0.001

    q = np.array([0.0, 0.0, 1.0, 1.0, 1.0, 1.5, 0.0, 0.0, 9.8])

    start_time = time.time()
    t, trajectory = model.simulate(q, duration, dt)
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
