#include "ball_models/ball_trajectory.hpp"

namespace ball_models
{

BallTrajectory::BallTrajectory(double drag_coefficient,
                               double lift_coefficient,
                               double decay_coefficient,
                               double ball_mass,
                               double ball_radius,
                               double air_density,
                               double graviational_constant)
    : c_drag_(drag_coefficient),
      c_lift_(lift_coefficient),
      c_decay_(decay_coefficient),
      m_ball_(ball_mass),
      r_ball_(ball_radius),
      rho_(air_density),
      g_(graviational_constant)
{
    double A = M_PI * r_ball_ * r_ball_;

    k_drag_ = -0.5 * c_drag_ * rho_ * A / m_ball_;
    k_lift_ = 0.5 * rho_ * c_lift_ * A * r_ball_ / m_ball_;
}

void BallTrajectory::update_state(Eigen::VectorXd state)
{
    position_ = state.segment<3>(0);
    velocity_ = state.segment<3>(3);
    angular_velocity_ = state.segment<3>(6);
    state_ = state;
}

Eigen::VectorXd BallTrajectory::compute_derivative()
{
    double v_norm = velocity_.norm();
    Eigen::Vector3d omega_v_cross = angular_velocity_.cross(velocity_);

    Eigen::Vector3d gravity_acceleration(0.0, 0.0, -g_);
    Eigen::Vector3d drag_acceleration = v_norm * k_drag_ * velocity_;
    Eigen::Vector3d magnus_acceleration = k_lift_ * omega_v_cross;

    Eigen::Vector3d acceleration_ =
        gravity_acceleration + drag_acceleration + magnus_acceleration;

    Eigen::Vector3d angular_acceleration_(0.0, 0.0, 0.0);

    // new state vector
    Eigen::VectorXd dq_dt(9);
    dq_dt << velocity_, acceleration_, angular_acceleration_;

    return dq_dt;
}

Eigen::VectorXd BallTrajectory::step(double dt)
{
    Eigen::VectorXd q(9);
    q << position_, velocity_, angular_velocity_;

    Eigen::VectorXd dq_dt = compute_derivative();
    Eigen::VectorXd q_post = q + dt * dq_dt;

    if (detect_table_contact(q_post, 0.77))
    {
        q_post = linear_contact_model(q);
    }

    if (detect_racket_contact(q_post))
    {
        q_post = linear_racket_model(q);
    }

    return q_post;
}

void BallTrajectory::simulate(double duration, double dt)
{
    for (double t = 0; t < duration; t += dt)
    {
        state_ = step(dt);
        trajectory_.push_back(state_);
    }
}

Eigen::MatrixXd BallTrajectory::compute_jacobian(const Eigen::VectorXd& state)
{
    // Define Jacobian matrix
    Eigen::MatrixXd FJacobian(9, 9);

    // Define the variables
    double v_x = state(3);
    double v_y = state(4);
    double v_z = state(5);

    double omega_x = state(6);
    double omega_y = state(7);
    double omega_z = state(8);

    Eigen::Vector3d velocity = state.segment<3>(3);

    double v_mag = velocity.norm();

    // Phyiscal constants
    double a = -0.12910768481608;
    double b = -0.00807145639527847;

    // Assemble matrix
    FJacobian << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, a * v_x * v_x / v_mag + a * v_mag,
        b * omega_z + a * v_x * v_y / v_mag,
        b * omega_y + a * v_x * v_z / v_mag, 0, b * v_z, -b * v_y,
        b * omega_z + a * v_x * v_y / v_mag, a * v_y * v_y / v_mag + a * v_mag,
        b * omega_x + a * v_y * v_z / v_mag, -b * v_z, 0, b * v_x,
        b * omega_y + a * v_x * v_z / v_mag,
        b * omega_x + a * v_y * v_z / v_mag, a * v_z * v_z / v_mag + a * v_mag,
        b * v_y, -b * v_x, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    return FJacobian;
}

} // namespace ball_models