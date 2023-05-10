import ball_models
import numpy as np


model = ball_models.BallTrajectory("/home/adittrich/test_workspace/workspace/src/ball_models/config/config.toml")
q = np.array([0.0, 0.0, 0.0, 1.0, 1.0, 1.5, 0.0, 0.0, 0.0])

q_next = model.integrate(q, 1)
print(q_next)

model.simulate(q, 1.0, 0.001)