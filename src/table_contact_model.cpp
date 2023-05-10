#include "ball_models/table_contact_model.hpp"

namespace ball_models
{
Eigen::VectorXd linear_contact_model(const Eigen::VectorXd q)
{
    Eigen::MatrixXd contact_matrix = Eigen::MatrixXd::Identity(9, 9);
    contact_matrix(5, 5) = -0.95;

    std::cout << contact_matrix << "\n";

    Eigen::VectorXd q_post = contact_matrix * q;

    return q_post;
}

bool detect_table_contact(const Eigen::VectorXd q, double reset_height = 0.77)
{
    if (q(2) < reset_height && q(5) < 0.0)
    {
        return true;
    }

    return false;
}

}  // namespace ball_models