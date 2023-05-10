def test_ball_models_import():
    import ball_models

    # Initialisation via config file
    ball_models.BallTrajectory("./config/config.toml")

    # Initialisation via direct paramaters
    ball_mass = 0.0027
    ball_radius = 0.02
    air_density = 1.18
    graviation = 9.81
    drag_coefficient = 0.5
    lift_coefficient = 1.5
    decay_coefficient = 0.005

    ball_models.BallTrajectory(
        ball_mass,
        ball_radius,
        air_density,
        graviation,
        drag_coefficient,
        lift_coefficient,
        decay_coefficient,
    )

def test_cpp_integrate():
    import ball_models
    import numpy as np

    state = np.array([0.0, 0.0, 1.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    dt = 0.5

    model = ball_models.BallTrajectory("./config/config.toml")
    model.integrate(state, dt)

def test_cpp_integrate_with_contacts():
    import ball_models
    import numpy as np

    ball_state = np.array([0.0, 0.0, 1.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    racket_state = [0.0]
    dt = 0.5

    model = ball_models.BallTrajectory("./config/config.toml")
    model.integrate_with_contacts(ball_state, racket_state, dt)

def test_cpp_simulate():
    import ball_models
    import numpy as np

    ball_state = np.array([0.0, 0.0, 1.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    duration = 2.0
    dt = 0.5

    model = ball_models.BallTrajectory("./config/config.toml")
    model.simulate(ball_state, duration, dt)