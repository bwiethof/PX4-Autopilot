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

void UKF::update(const State &current, std::chrono::milliseconds ms) {
  const auto dt =
      std::chrono::duration_cast<std::chrono::duration<double>>(ms).count();
  _impl.step(_sensorData, dt, current);
}

ukf::core::state::StateBase UKF::state() const {
  return _impl.getState().matrix();
}
} // namespace visukf
