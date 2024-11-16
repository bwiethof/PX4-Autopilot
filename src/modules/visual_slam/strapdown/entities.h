//
// Created by bene on 16.11.24.
//

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <utility>

namespace visslam {
namespace strap_down {

struct Attitude {

  Attitude() = default;

  explicit Attitude(const Eigen::Quaterniond &att) : _att(att) {}
  explicit Attitude(Eigen::Quaterniond &&att) : _att(std::move(att)) {}

  Eigen::Quaterniond value() const { return _att; }

private:
  Eigen::Quaterniond _att{Eigen::Quaterniond::Identity()};
};

struct Position {

  Position() = default;
  explicit Position(const Eigen::Vector<double, 3> &vec) : _pos(vec) {}
  explicit Position(Eigen::Vector<double, 3> &&vec) : _pos(std::move(vec)) {}

  Eigen::Vector3d value() const { return _pos; }

private:
  Eigen::Vector3d _pos{Eigen::Vector3d::Constant(0)};
};

struct Velocity {

  Velocity() = default;
  explicit Velocity(const Eigen::Vector<double, 3> &vel) : _vel(vel) {}
  explicit Velocity(Eigen::Vector<double, 3> &&vel) : _vel(std::move(vel)) {}

  Eigen::Vector3d value() const { return _vel; }

private:
  Eigen::Vector3d _vel{Eigen::Vector3d::Constant(0)};
};

struct Sample {
  Eigen::Vector<double, 3> a_body{};     // acceleration in m/s²
  Eigen::Vector<double, 3> omega_body{}; // angular rate in rad/s
};

} // namespace strap_down
} // namespace visslam