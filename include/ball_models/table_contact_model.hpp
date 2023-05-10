#pragma once

#include <eigen3/Eigen/Dense>
#include <iostream>

namespace ball_models
{
/**
 * @brief table contact model
 *
 * Calculates new ball state after contact with table.
 *
 * @param ball_state ball state prior contact
 * @return Eigen::VectorXd ball state after contact
 */
Eigen::VectorXd table_contact_model(const Eigen::VectorXd ball_state);

/**
 * @brief Table contact detector
 *
 * Detects contact between table and ball depending on preset
 * table height and current ball state
 *
 * @param ball_state current ball state
 * @param reset_height table height
 * @return true if ball collided with table
 * @return false if ball did not collide with the table
 */
bool detect_table_contact(const Eigen::VectorXd ball_state,
                          double reset_height);

}  // namespace ball_models