#include "ball_models/racket_contact_model.hpp"

namespace ball_models
{
Eigen::VectorXd racket_contact_model(const Eigen::VectorXd ball_state,
                                     const Eigen::VectorXd racket_state)
{
    Eigen::MatrixXd contact_matrix = Eigen::MatrixXd::Ones(9, 9);
    Eigen::VectorXd q_post = contact_matrix * ball_state;

    return q_post;
}

bool detect_racket_contact(const Eigen::VectorXd ball_state,
                           const Eigen::VectorXd racket_state)
{
    return false;
}

}  // namespace ball_models