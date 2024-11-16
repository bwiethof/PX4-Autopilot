//
// Created by bene on 05.05.24.
//

#ifndef PX4_STRAP_DOWN_H
#define PX4_STRAP_DOWN_H

#include "entities.h"
#include <chrono>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace visslam {
namespace strap_down {

/**
 * StrapDown implements the strap down algorithm for x_k = x(t+dt) based on the
 * IMU Sample
 */
class StrapDown {
  struct State {
    Eigen::Quaterniond att; // Body -> world
    Eigen::Vector3d pos;    // NED
    Eigen::Vector3d vel;    // NED
  } state;

public:
  StrapDown() = default;
  ~StrapDown() = default;

  // make position accessible
  Position position() const;
  Velocity velocity() const;
  Attitude attitude() const;

  void step(const Sample &, const std::chrono::milliseconds &dt);
};

} // namespace strap_down
} // namespace visslam

#endif // PX4_STRAP_DOWN_H
