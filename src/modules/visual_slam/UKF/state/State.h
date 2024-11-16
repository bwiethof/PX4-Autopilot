//
// Created by bene on 26.09.24.
//

#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace visukf  {

/**
 * @struct State
 * @brief Represents the state of an object in three-dimensional space.
 *
 * This structure encapsulates the position, velocity, and attitude
 * (orientation) of an object using Eigen library types.
 *
 */
struct State {
  Eigen::Vector3d pos{0, 0, 0};       ///< 3D position vector.
  Eigen::Vector3d vel{0, 0, 0};       ///< 3D velocity vector.
  Eigen::Quaterniond att{0, 0, 0, 0}; ///< Quaternion representing orientation.
};

} // namespace vislam
