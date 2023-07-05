#pragma once

#ifndef BALL_TRAJECTORY_HPP
#define BALL_TRAJECTORY_HPP

#define _USE_MATH_DEFINES

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "ball_models/racket_contact_model.hpp"
#include "ball_models/table_contact_model.hpp"
#include "ball_models/toml/toml.hpp"

namespace ball_models
{
class BallTrajectory
{
private:
    // physics parameter
    double m_ball_;   // Mass of the ball
    double r_ball_;   // Radius of the ball
    double rho_;      // Air density
    double g_;        // Local gravitational constant
    double c_drag_;   // Drag coefficient
    double c_lift_;   // Lift coefficient
    double c_decay_;  // Spin decay coefficient

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

    // configuration file path
    toml::table config_;

    /**
     * @brief Computes physics coefficients
     *
     * After physics coefficients are loaded manually or via
     * a TOML file, compute_coefficients calculates for each
     * equation a specific coefficient once.
     *
     */
    void compute_coefficients();

    /**
     * @brief updates internal system state of the ball
     *
     * Updates the internal system state of the ball and
     * segments it to the substates.
     *
     * @param state current ball state
     */
    void update_state(Eigen::VectorXd& state);

    /**
     * @brief Computes derivate of the system state
     *
     * Computes the derivative of the system state
     * according to equations of motions.     *
     */
    void compute_derivative();

    /**
     * @brief P
     *
     * Performs one integration step with given time step
     * with Euler Forward using the derivate generated by
     * compute derivative().
     *
     * @param dt integration time step.
     */
    void step(double dt);

public:
    /**
     * @brief Construct a new Ball Trajectory object
     *
     * Constructor providing an absolute file path to a TOML file.
     *
     * @param config_path file path to TOML file.
     */
    BallTrajectory(std::string config_path);

    /**
     * @brief Constructor with directly specifying the
     * simulation parameters via given attributes.
     *
     * @param ball_mass mass of standard table tennis ball.
     * @param ball_radius radius of standard table tennis ball.
     * @param air_density air density in normal room conditions.
     * @param graviational_constant gravitational constant in
     * Tuebingen.
     * @param drag_coefficient drag coefficient of ball motion
     * for table tennis balls.
     * @param lift_coefficient lift coefficient of ball motion
     * for table tennis balls.
     * @param decay_coefficient spin decay coefficient of ball
     * motion for table tennis balls.
     */
    BallTrajectory(double ball_mass,
                   double ball_radius,
                   double air_density,
                   double graviational_constant,
                   double drag_coefficient,
                   double lift_coefficient,
                   double decay_coefficient);

    /**
     * @brief One integration step
     *
     * Performs one integration step from given initial ball state
     * with given time step. Integration only specifies passive motion
     * of the ball including table contacts.
     *
     * @param state current ball state.
     * @param dt time step.
     *
     * @return Eigen::VectorXd new ball state after integration
     */
    Eigen::VectorXd integrate(const Eigen::VectorXd& state, double dt);

    /**
     * @brief One integration step
     *
     * Performs one integration from given initial ball state with
     * given time step. Integration also considers racket motion
     * and depending on the racket state adjusts ball motion.
     *
     * @param ball_state current ball state.
     * @param racket_state current racket state.
     * @param dt time step.
     * @return Eigen::VectorXd
     */
    Eigen::VectorXd integrate_with_contacts(const Eigen::VectorXd& ball_state,
                                            const Eigen::VectorXd& racket_state,
                                            double dt);

    /**
     * @brief Simulates passive ball motion.
     *
     * Simulates ball motion for specified duration with given time steps
     * depending on the physical ball model with given initial ball state.
     * For integration Euler Forward is used.
     *
     * @param state initial ball state.
     * @param duration simulation duration.
     * @param dt simulation time step length
     * @return std::vector<Eigen::VectorXd> Returns a array of ball states
     * for each specified time step.
     */
    std::tuple<std::vector<float>, std::vector<Eigen::VectorXd>> simulate(const Eigen::VectorXd& state,
                                          double duration,
                                          double dt);

    /**
     * @brief Computes jacobian of the system
     *
     * Efficiently calculates the Jacobian of the ball motion and returns
     * an linearized matrix around given ball state.
     *
     * @param state supporting ball state for linearization.
     * @return Eigen::MatrixXd linear motion jacobian.
     */
    Eigen::MatrixXd compute_jacobian(const Eigen::VectorXd& state);
};

#endif  // BALL_TRAJECTORY_HPP

}  // namespace ball_models