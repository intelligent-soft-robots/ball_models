#include "ball_models/table_contact_model.hpp"

namespace ball_models
{
Eigen::VectorXd table_contact_model(const Eigen::VectorXd& ball_state)
{
    Eigen::MatrixXd contact_matrix = Eigen::MatrixXd::Identity(9, 9);
    contact_matrix(5, 5) = -0.95;

    Eigen::VectorXd q_post = contact_matrix * ball_state;

    return q_post;
}

bool detect_table_contact(const Eigen::VectorXd& ball_state, double reset_height = 0.77)
{
    if (ball_state(2) < reset_height && ball_state(5) < 0.0)
    {
        return true;
    }

    return false;
}

}  // namespace ball_models