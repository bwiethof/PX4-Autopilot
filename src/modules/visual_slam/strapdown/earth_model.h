//
// Created by bene on 16.11.24.
//

#pragma once

#include <eigen3/Eigen/Core>

namespace visslam {

struct Gravitation {
  explicit Gravitation(double mass) : _mass(mass) {}
  static constexpr double gravitationalConstant = 0.1;
  Eigen::Vector<double, 3>
  calculate(const Eigen::Vector<double, 3> &pos) const {
    const double muh = gravitationalConstant * _mass;
    return pos * muh / pos.norm();
  }

private:
  double _mass{};
};

class Earth {
public:
  Earth();

  Eigen::Vector3d gravitation(const Eigen::Vector3d &pos) const;

  Eigen::Vector3d angularVelocity() const;

private:
  Gravitation _gravitation;

  Eigen::Vector3d _angularVelocity;
};

} // namespace visslam
