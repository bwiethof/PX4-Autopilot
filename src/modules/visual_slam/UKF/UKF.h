//
// Created by bene on 20.03.24.
//

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
#include "sensor/GPS.h"
#include "state/Position.h"
#include "state/Velocity.h"

#include <core/ukf.hpp>

namespace visukf {

class UKF {
public:
  UKF();


  void add(sensor_gps_s);

  void update();

  ukf::core::state::StateBase state() const;

private:
  ukf::core::Ukf<ukf::core::StateFields<Position, Velocity>> _impl{};
  ukf::core::SensorData<ukf::core::StaticFields<sensor::GPS>> _sensorData{};

  sensor_gps_s _tempStorage{};
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace visukf
#pragma GCC diagnostic pop
