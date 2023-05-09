#pragma once

#include <eigen3/Eigen/Dense>

namespace ball_models
{

Eigen::VectorXd linear_contact_model(const Eigen::VectorXd& q);
bool detect_table_contact(const Eigen::VectorXd& q, double reset_height);

} // namespace ball_models