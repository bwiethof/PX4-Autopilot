//
// Created by bene on 20.03.24.
//

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include "sensor/GPS.h"
#include "state/Position.h"
#include "state/State.h"
#include "state/Velocity.h"

#include <chrono>

#include <core/ukf.hpp>

namespace visukf {

class UKF {
public:
  UKF();

  void update(const State &current, std::chrono::milliseconds dt);

  ukf::core::state::StateBase state() const;

private:
  ukf::core::Ukf<ukf::core::StateFields<Position, Velocity>> _impl{};
  ukf::core::SensorData<ukf::core::StaticFields<sensor::GPS>> _sensorData{};

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace visukf
#pragma GCC diagnostic pop
