//
// Created by bene on 16.11.24.
//

#pragma once

#include <eigen3/Eigen/Core>

namespace visslam {

struct Gravitation {
  explicit Gravitation(double mass)
      : _mass(mass), muh(gravitationalConstant * _mass) {}
  Eigen::Vector<double, 3>
  calculate(const Eigen::Vector<double, 3> &pos) const {
    return pos * muh / pos.norm();
  }

private:
  static constexpr double gravitationalConstant = 6.67430e-11;
  double _mass{};
  double muh{};
};

class Earth {
public:
  Earth();

  Eigen::Vector3d gravitation(const Eigen::Vector3d &pos) const;

  // Angular velocity of the earth in ECI frame
  Eigen::Vector3d angularVelocity() const;

private:
  Gravitation _gravitation;

  double _angularVelocity;
};

} // namespace visslam
