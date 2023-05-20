import time

import ball_trajectory_numpy
import ball_trajectory_symbolic
import matplotlib.pyplot as plt
from numpy import arange, array, round
from numpy.testing import assert_array_almost_equal

import ball_models


def test_results_model():
    config_path = (
        "/home/adittrich/test_workspace/workspace/src/ball_models/config/config.toml"
    )

    cpp_model = ball_models.BallTrajectory(config_path)
    python_model = ball_trajectory_numpy.BallTrajectoryNumpy(config_path)
    symbolic_model = ball_trajectory_symbolic.BallTrajectorySymbolic(config_path)

    q = [0.0, 0.0, 1.0, 12.0, 2.0, 1.2, 100.0, 1.0, 20.0]
    duration = 1.0
    dt = 0.001

    cpp_trajectory = cpp_model.simulate(q, duration, dt)
    py_trajectory = python_model.simulate(q, duration, dt)
    symbolic_trajectory = symbolic_model.simulate(q, duration, dt)

    cpp_trajectory = array(cpp_trajectory)
    py_trajectory = array(py_trajectory)
    symbolic_trajectory = array(symbolic_trajectory)

    # Plotting of trajectories
    fig, axs = plt.subplots(9, 1)
    colors = ["#CF5369", "#0193d7", "#46b361"]

    for i, ax in enumerate(axs):
        ax.plot(cpp_trajectory[:, i], label="C++ Model", color=colors[0])
        ax.plot(
            py_trajectory[:, i],
            linestyle="dashed",
            label="Python Model",
            color=colors[1],
        )
        ax.plot(
            symbolic_trajectory[:, i],
            linestyle="dotted",
            label="Sympy Model",
            linewidth=4,
            color=colors[2],
        )

        ax.legend()

    assert_array_almost_equal(cpp_trajectory, py_trajectory)


def test_benchmark_cpp():
    config_path = (
        "/home/adittrich/test_workspace/workspace/src/ball_models/config/config.toml"
    )

    cpp_model = ball_models.BallTrajectory(config_path)

    q = [0.0, 0.0, 1.0, 12.0, 2.0, 1.2, 100.0, 1.0, 20.0]
    duration = 1.0

    frequencies = [10, 100, 200, 500, 1000, 2000, 10000, 50000, 100000]

    frequencies = [2**i for i in range(20)]

    cpp_model_time = []

    for frequency in frequencies:
        dt = 1 / frequency

        # C++ Model
        start_time = time.time()
        cpp_model.simulate(q, duration, dt)
        sim_time = time.time() - start_time

        cpp_model_time.append(sim_time)

    # Plotting of computation time
    fig, ax = plt.subplots(layout="constrained")

    ax.plot(frequencies, cpp_model_time, label="C++ Model")

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_xlabel("Sample rate [s-1]")
    ax.set_ylabel("Computation time [s]")
    ax.set_title("Computation time of C++ Model")

    ax.legend()
    ax.grid()


def test_benchmark_models():
    config_path = (
        "/home/adittrich/test_workspace/workspace/src/ball_models/config/config.toml"
    )

    cpp_model = ball_models.BallTrajectory(config_path)
    python_model = ball_trajectory_numpy.BallTrajectoryNumpy(config_path)
    symbolic_model = ball_trajectory_symbolic.BallTrajectorySymbolic(config_path)

    q = [0.0, 0.0, 1.0, 12.0, 2.0, 1.2, 100.0, 1.0, 20.0]
    duration = 1.0

    frequencies = [10, 100, 200, 500, 1000, 2000, 10000, 50000, 100000]
    frequencies = [10, 100, 200, 500, 1000, 2000]

    cpp_model_time = []
    python_model_time = []
    sympy_model_time = []

    for frequency in frequencies:
        dt = 1 / frequency

        # C++ Model
        start_time = time.time()
        cpp_model.simulate(q, duration, dt)
        sim_time = time.time() - start_time

        print(f"Frequency: {frequency}, CPP: {sim_time}")
        cpp_model_time.append(sim_time)

        # Python Model
        start_time = time.time()
        python_model.simulate(q, duration, dt)
        sim_time = time.time() - start_time

        print(f"Frequency: {frequency}, Python: {sim_time}")
        python_model_time.append(sim_time)

        # Python Model
        start_time = time.time()
        symbolic_model.simulate(q, duration, dt)
        sim_time = time.time() - start_time

        print(f"Frequency: {frequency}, Sympy: {sim_time}")
        sympy_model_time.append(sim_time)

    # Plotting of computation time
    fig, ax = plt.subplots(layout="constrained")

    x = arange(len(frequencies))
    width = 0.2  # the width of the bars
    multiplier = 0
    colors = ["#CF5369", "#0193d7", "#46b361"]

    model_data = {
        "C++": cpp_model_time,
        "Python": python_model_time,
        "Sympy": sympy_model_time,
    }

    for attribute, measurement in model_data.items():
        measurement_ns = array(measurement) * 1000
        measurement_round = round(measurement_ns, 1)
        offset = width * multiplier
        rects = ax.bar(
            x + offset, measurement_round, width, label=attribute, color=colors.pop(0)
        )
        ax.bar_label(rects, padding=3)
        multiplier += 1

    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_xlabel("Sample rate [s-1]")
    ax.set_ylabel("Computation time [ms]")
    ax.set_title("Computation time for different simulation approaches")
    ax.set_xticks(x + width, frequencies)
    ax.set_ylim(0, 100.0)
    ax.axhline(5.0, color="red")
    ax.annotate(
        "200 Hz threshold", xy=(sum(ax.get_xlim()) / 2, 6), color="red", ha="center"
    )

    ax.legend(loc="upper left", ncols=3)
    ax.grid()


if __name__ == "__main__":
    test_benchmark_cpp()
    test_results_model()
    test_benchmark_models()

    plt.show()
