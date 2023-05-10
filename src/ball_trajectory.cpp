#include "ball_models/ball_trajectory.hpp"

namespace ball_models
{
BallTrajectory::BallTrajectory(std::string config_file_path)
{   
    try {
        config_ = toml::parse_file(config_file_path);
        const auto& ballDynamics = config_["ball_dynamics"];

        // FIXME: value, as_floating_point, value_exact do not work! 
        // Can introduce an unnoticed error!

        m_ball_ = ballDynamics["ball_mass"].value_or(0.0);
        r_ball_ = ballDynamics["ball_radius"].value_or(0.0);
        rho_ = ballDynamics["air_density"].value_or(0.0);
        g_ = ballDynamics["graviational_constant"].value_or(0.0);
        c_drag_ = ballDynamics["drag_coefficient"].value_or(0.0);
        c_lift_ = ballDynamics["lift_coefficient"].value_or(0.0);
        c_decay_ = ballDynamics["decay_coefficient"].value_or(0.0);
    } catch (const toml::parse_error& err) {
        std::cerr << "Parsing failed: " << err << std::endl;
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
    }

    compute_coefficients();    
}

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
    compute_coefficients();
}

void BallTrajectory::compute_coefficients()
{
    double A = M_PI * r_ball_ * r_ball_;

    k_drag_ = -0.5 * c_drag_ * rho_ * A / m_ball_;
    k_lift_ = 0.5 * rho_ * c_lift_ * A * r_ball_ / m_ball_;
    k_decay_ = c_decay_;
}

void BallTrajectory::update_state(Eigen::VectorXd state)
{
    position_ = state.segment<3>(0);
    velocity_ = state.segment<3>(3);
    angular_velocity_ = state.segment<3>(6);
    state_ = state;
}

void BallTrajectory::compute_derivative()
{   
    position_ = state_.segment<3>(0);
    velocity_ = state_.segment<3>(3);
    angular_velocity_ = state_.segment<3>(6);

    double v_norm = velocity_.norm();
    Eigen::Vector3d omega_v_cross = angular_velocity_.cross(velocity_);

    Eigen::Vector3d gravity_acceleration(0.0, 0.0, -g_);
    Eigen::Vector3d drag_acceleration = v_norm * k_drag_ * velocity_;
    Eigen::Vector3d magnus_acceleration = k_lift_ * omega_v_cross;

    Eigen::Vector3d acceleration_ =
        gravity_acceleration + drag_acceleration + magnus_acceleration;

    Eigen::Vector3d angular_acceleration_;
        angular_acceleration_.setOnes();
        angular_acceleration_ *= k_decay_;
    
    // new state vector
    Eigen::VectorXd dq_dt(9);
    dq_dt << velocity_, acceleration_, angular_acceleration_;
    state_dot_ = dq_dt;
}

void BallTrajectory::step(double dt)
{   
    compute_derivative();
    
    Eigen::VectorXd _q = state_ + dt * state_dot_;

    if (detect_table_contact(_q, 0.77))
    {
        _q = linear_contact_model(state_);
    }

    state_ = _q;
}

Eigen::VectorXd BallTrajectory::integrate(const Eigen::VectorXd state, double dt)
{
    state_ = state;
    step(dt);

    return state_;
}


Eigen::VectorXd BallTrajectory::integrate_with_contacts(const Eigen::VectorXd ball_state, const Eigen::VectorXd racket_state, double dt) 
{
    Eigen::VectorXd ball_state_after_step = integrate(ball_state, dt);

    if (detect_racket_contact(ball_state_after_step, racket_state))
    {
        state_ = linear_racket_model(ball_state, racket_state);
    }
    else
    {
       state_ = ball_state_after_step;
    }

    return state_;
}

std::vector<Eigen::VectorXd> BallTrajectory::simulate(const Eigen::VectorXd state, double duration,
                                                      double dt)
{
    Eigen::VectorXd current_state = state;

    for (double t = 0; t < duration; t += dt)
    {
        current_state = integrate(current_state, dt);
        trajectory_.push_back(current_state);
    }

    return trajectory_;
}

Eigen::MatrixXd BallTrajectory::compute_jacobian(const Eigen::VectorXd state)
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

}  // namespace ball_models