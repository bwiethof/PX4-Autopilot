//
// Created by bene on 05.05.24.
//

#ifndef PX4_STRAP_DOWN_H
#define PX4_STRAP_DOWN_H

#include "earth_model.h"
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
  // State shall represent the current state in ECI frame including the
  // transformation from body to ECI
  struct State {
    Eigen::Quaterniond att; // Body -> ECI
    Eigen::Vector3d pos;    // ECI
    Eigen::Vector3d vel;    // ECI
  } _state;

public:
  StrapDown() = default;
  ~StrapDown() = default;

  Position position() const;
  Velocity velocity() const;
  Attitude attitude() const;

  void step(const Sample &, const std::chrono::milliseconds &dt);

  void initialize(const State&);

  bool isInitialized() const { return _initialized; }

private:
  bool _initialized{false};
  Earth _earth{};
};

} // namespace strap_down
} // namespace visslam

#endif // PX4_STRAP_DOWN_H
