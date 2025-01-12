//
// Created by bene on 16.11.24.
//

#include "earth_model.h"

constexpr double EarthMass = 5.9722e24;
// around the tilted axe -> resulting vector depends on the  coordinate system
constexpr double EarthAngularVelocity = 7.2921159e-5;
// => Use ECI which is with respect to the stars and therefore static. This
// means there exists now an angular velocity to incorporate

namespace visslam {

Earth::Earth()
    : _gravitation(EarthMass), _angularVelocity(EarthAngularVelocity) {}

Eigen::Vector3d Earth::gravitation(const Eigen::Vector3d &pos) const {
  return _gravitation.calculate(pos);
}

Eigen::Vector3d Earth::angularVelocity() const {
  // in ECI frame (y-axis in orbit plane == rotation axis)
  return Eigen::Vector3d{0.0, _angularVelocity, 0.0};
}

} // namespace visslam