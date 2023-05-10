#pragma once

#include <eigen3/Eigen/Dense>

namespace ball_models
{
/**
 * @brief Racket contact model
 *
 * Calculates a new ball state after collision with a racket with
 * given prior ball state and racket state.
 *
 * @param ball_state prior ball state.
 * @param racket_state racket state.
 * @return Eigen::VectorXd ball state after contact.
 */
Eigen::VectorXd racket_contact_model(const Eigen::VectorXd ball_state,
                                     const Eigen::VectorXd racket_state);

/**
 * @brief Racket contact detection.
 *
 * Detects contact of racket and ball according to given racket state.
 * Returns true if given ball state and given racket state results in
 * contact.
 *
 * @param ball_state new ball state after integration.
 * @param racket_state current racket state.
 * @return true if contact is detected
 * @return false if no contact is detected.
 */
bool detect_racket_contact(const Eigen::VectorXd ball_state,
                           const Eigen::VectorXd racket_state);

}  // namespace ball_models