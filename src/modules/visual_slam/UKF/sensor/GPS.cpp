//
// Created by bene on 23.03.24.
//

#include "GPS.h"

#include "visual_slam/UKF/math/coordinates.h"
#include "visual_slam/UKF/state/Position.h"

namespace visukf {

namespace sensor {

namespace {
const ukf::core::SquaredMatrix<3> StaticNoising =
    ukf::core::SquaredMatrix<3>::Identity() * std::sqrt(0.1);
}

GPS::Noising GPS::noising() const {
  // For now hardcoded
  return StaticNoising;
}

ukf::core::Vector<3> GPS::toVector(sensor_gps_s &&gps_data) const {

  ukf::core::Vector<3> result{gps_data.latitude_deg, gps_data.longitude_deg,
                              gps_data.altitude_ellipsoid_m};
  return result;
}

GPS::Data GPS::predict(const Position &position) const {
  // inital data ist the problem
  // (0,0,0) -> earth center -> how to correctly initialize?
  // first hardcoded a value
  // GPS is in LLA-> conversion to LLA necessary
  return math::ECEFToLLa(position.data * 1000); // data in km
}

} // namespace sensor

} // namespace visukf
