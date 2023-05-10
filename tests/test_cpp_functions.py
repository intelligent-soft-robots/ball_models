import ball_models

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
    ball_models
    pass

def test_cpp_integrate_with_contacts():
    pass


def test_cpp_simulate():
    pass