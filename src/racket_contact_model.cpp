#include "ball_models/racket_contact_model.hpp"

namespace ball_models
{
    
Eigen::VectorXd linear_racket_model(const Eigen::VectorXd& q)
{
    Eigen::MatrixXd contact_matrix = Eigen::MatrixXd::Ones(9, 9);
    Eigen::VectorXd q_post = contact_matrix * q;

    return q_post;
}

bool detect_racket_contact(const Eigen::VectorXd& q)
{
    return false;
}

} // namespace ball_models