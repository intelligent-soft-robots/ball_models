#pragma once

#ifndef BALL_TRAJECTORY_HPP
#define BALL_TRAJECTORY_HPP

#define _USE_MATH_DEFINES

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>

#include "ball_models/toml/toml.hpp"
#include "ball_models/racket_contact_model.hpp"
#include "ball_models/table_contact_model.hpp"

namespace ball_models
{
class BallTrajectory
{
private:
    // physics parameter
    double c_drag_;   // Drag coefficient
    double c_lift_;   // Lift coefficient
    double c_decay_;  // Spin decay coefficient
    double m_ball_;   // Mass of the ball
    double r_ball_;   // Radius of the ball
    double rho_;      // Air density
    double g_;        // Local gravitational constant

    // precalculated constants
    double k_drag_;
    double k_lift_;
    double k_decay_;

    // system variables
    Eigen::Vector3d position_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d angular_velocity_;
    Eigen::VectorXd state_;
    Eigen::VectorXd state_dot_;

    // position trajectory
    std::vector<Eigen::VectorXd> trajectory_;

    // configuration file path
    toml::table config_;

    void compute_coefficients();

    // internal functions
    void update_state(Eigen::VectorXd state);
    void compute_derivative();
    void step(double dt);
    
public:
    /**
     * Constructor
     */
    BallTrajectory(double drag_coefficient,
                   double lift_coefficient,
                   double decay_coefficient,
                   double ball_mass,
                   double ball_radius,
                   double air_density,
                   double graviational_constant);
    
    BallTrajectory(std::string config_path);
    
    Eigen::VectorXd integrate(const Eigen::VectorXd state, double dt);
    Eigen::VectorXd integrate_with_contacts(const Eigen::VectorXd ball_state, const Eigen::VectorXd racket_state, double dt);

    std::vector<Eigen::VectorXd> simulate(const Eigen::VectorXd state, double duration, double dt);

    Eigen::MatrixXd compute_jacobian(const Eigen::VectorXd state);
};

#endif  // BALL_TRAJECTORY_HPP

}  // namespace ball_models