//
// Created by bene on 05.05.24.
//

#include "strap_down.h"
#include <chrono>
#include <eigen3/Eigen/Geometry>

namespace visslam {
namespace strap_down {

namespace {
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

// TODO: Move in separate file/member
const Gravitation earthGravitation{1234.};

template <typename Derived>
using SkewMatrixType = Eigen::Matrix<typename Derived::Scalar, 3, 3>;
template <typename VectorType>
SkewMatrixType<VectorType>
asSkewSymmetric(const Eigen::MatrixBase<VectorType> &vec) {

  SkewMatrixType<VectorType> skewMatrix;
  skewMatrix.diagonal().setZero();
  skewMatrix(0, 1) = -vec(2);
  skewMatrix(1, 0) = vec(2);
  skewMatrix(0, 2) = vec(1);
  skewMatrix(2, 0) = -vec(1);
  skewMatrix(1, 2) = -vec(0);
  skewMatrix(2, 1) = vec(0);
  return skewMatrix;
}

} // namespace
using std::chrono::milliseconds;
using std::chrono::seconds;

/*
 *
 */
void StrapDown::step(const Sample &sample, const milliseconds &dt_ms) {
  State localState = _state; // in ECI

  const auto dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(dt_ms).count();

  // create copy to work on
  localState.pos += _state.vel * dt;

  const Eigen::Vector<double, 3> gravitation =
      _earth.gravitation(localState.pos);

  // TODO: where do I get this from, for now: full trust in model === no bias
  const Eigen::Vector<double, 3> gravitation_bias{0, 0, 0};

  const Eigen::Matrix<double, 3, 3> R_w = localState.att.toRotationMatrix();

  const Eigen::Vector<double, 3> omega_m = _earth.angularVelocity();

  // vel in eci = body acc to ECI frame + corrected gravitation -  coriolis
  // force in ECI
  localState.vel += (R_w * sample.a_body + gravitation + gravitation_bias -
                     omega_m.cross(localState.vel)) *
                    dt;

  // Potential optimization:
  // Evaluation of skew symmetric is done after everything is done
  const Eigen::Matrix<double, 3, 3> attitudeMatrix =
      (R_w * asSkewSymmetric(sample.omega_body) -
       asSkewSymmetric(omega_m) * R_w) *
      dt;

  localState.att = Eigen::Quaterniond(attitudeMatrix).normalized();

  // finally update the actual state
  _state = localState;
}

void StrapDown::initialize(const State &initialState) {
  _state = initialState;
  _initialized = true;
}

Position StrapDown::position() const { return Position{_state.pos}; }

Velocity StrapDown::velocity() const { return Velocity{_state.vel}; }

Attitude StrapDown::attitude() const { return Attitude{_state.att}; }

} // namespace strap_down
} // namespace visslam
