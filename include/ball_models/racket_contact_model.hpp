#pragma once

#include <eigen3/Eigen/Dense>

namespace ball_models
{
Eigen::VectorXd linear_racket_model(const Eigen::VectorXd ball_state, const Eigen::VectorXd racket_state);
bool detect_racket_contact(const Eigen::VectorXd ball_state, const Eigen::VectorXd racket_state);

}  // namespace ball_models