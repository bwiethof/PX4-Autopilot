//
// Created by bene on 23.03.24.
//

#pragma once
#include "visual_slam/UKF/state/Position.h"

#include <core/sensor.hpp>
#include <uORB/topics/sensor_gps.h>

namespace visukf {
namespace sensor {

class GPS : public ukf::core::Sensor<3, sensor_gps_s, Position> {
public:
  using Sensor::Sensor;
  Noising noising() const override;
  ukf::core::Vector<3> toVector(sensor_gps_s &&gps_data) const override;

private:
  Data predict(const Position &) const override;
};

} // namespace sensor

} // namespace visukf
