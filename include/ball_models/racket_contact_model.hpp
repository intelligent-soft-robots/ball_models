#pragma once

#include <eigen3/Eigen/Dense>

namespace ball_models
{

Eigen::VectorXd linear_racket_model(const Eigen::VectorXd& q);
bool detect_racket_contact(const Eigen::VectorXd& q);

} // namespace ball_models