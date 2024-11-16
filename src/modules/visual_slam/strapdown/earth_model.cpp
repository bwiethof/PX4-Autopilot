//
// Created by bene on 16.11.24.
//

#include "earth_model.h"

constexpr double EarthMass = 5.9722e24;
//.2921159 × 10−5 radians/second.
constexpr double EarthAngularVelocity = 0.2921159e-5;

namespace visslam {

Earth::Earth() : _gravitation(EarthMass), _angularVelocity() {}

Eigen::Vector3d Earth::gravitation(const Eigen::Vector3d &pos) const {
  return _gravitation.calculate(pos);
}

Eigen::Vector3d Earth::angularVelocity() const { return _angularVelocity; }

} // namespace visslam