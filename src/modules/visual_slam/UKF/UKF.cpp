//
// Created by bene on 20.03.24.
//

#include "UKF.h"
#include "core/sensor_data.h"
namespace visukf {

UKF::UKF() {

  constexpr auto Size = ukf::core::StateFields<Position, Velocity>::StateSize;
  ukf::core::Vector<Size> initialState{4293.286, 627.853, 4659.166,
                                       0,        0,       0}; // in km

  ukf::core::SquaredMatrix<Size> initialP;
  initialP.setZero();
  initialP.diagonal() << 100, 100, 200, 2, 2, 2;

  _impl = ukf::core::Ukf<ukf::core::StateFields<Position, Velocity>>(
      initialState, initialP);
}

void UKF::add(sensor_gps_s gps_data) {
  _tempStorage = gps_data;
  _sensorData.setMeasurement<sensor::GPS>(std::move(gps_data));
}

void UKF::update() { _impl.step(_sensorData, 0.1, _tempStorage); }

ukf::core::state::StateBase UKF::state() const {
  return _impl.getState().matrix();
}
} // namespace visukf
