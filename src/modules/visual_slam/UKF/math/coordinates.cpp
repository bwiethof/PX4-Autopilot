//
// Created by bene on 27.03.24.
//

#include "coordinates.h"
#include <Eigen/Core>
#include <core/typedefs.h>

namespace visukf {
namespace math {

const Ellipsoid<double> WGS84{6'378'137.0L, 298.257223563L};

ukf::core::Vector<3> ECEFToLLa(const ukf::core::Vector<3> &xyz) {
  const auto x = xyz[0];
  const auto y = xyz[1];
  const auto z = xyz[2];

  // Initialize output values
  ukf::core::Vector<3> lla{0, 0, 0};
  auto &lon_rad = lla[0];
  auto &lat_rad = lla[1];
  auto &ht = lla[2];

  const auto w2 = x * x + y * y;
  const auto w = xyz.segment<2>(0).norm();
  const auto z2 = z * z;
  lon_rad = std::atan2(y, x);

  const auto a1 = WGS84.a * WGS84.e2;
  const auto a2 = a1 * a1;
  const auto a3 = a1 * WGS84.e2 / 2;
  const auto a4 = 2.5 * a2;
  const auto a5 = a1 + a3;
  // constexpr auto a6 = (1 - WGS84.e2);

  const auto r2 = w2 + z2;
  const auto r = xyz.norm();
  //  const auto r2_prim = xyz.squaredNorm();

  const auto s2 = z2 / r2; // z²/||xyz||²
  const auto c2 = w2 / r2; // w²/||xyz||²
  auto u = a2 / r;
  auto v = a3 - a4 / r;

  ukf::core::Float_t s{};
  ukf::core::Float_t c{};
  ukf::core::Float_t ss{};

  // cos(45 deg)^2 == 0.5
  if (c2 > 0.5) // Equatorial
  {
    s = (z / r) * (1 + c2 * (a1 + u + s2 * v) / r);
    lat_rad = std::asin(s);
    ss = s * s;
    c = std::sqrt(1 - ss);
  } else // Polar
  {
    c = (w / r) * (1 - s2 * (a5 - u - c2 * v) / r);
    lat_rad = std::acos(c);
    ss = 1.0 - c * c;
    s = std::sqrt(ss);

    if (z < 0) {
      lat_rad = -lat_rad;
      s = -s;
    }
  }

  const auto d2 = 1.0 - WGS84.e2 * ss;
  const auto Rn = WGS84.a / std::sqrt(d2);
  const auto Rm = (1 - WGS84.e2) * Rn / d2;
  const auto rf = (1 - WGS84.e2) * Rn;
  u = w - Rn * c;
  v = z - rf * s;
  const auto f = c * u + s * v;
  const auto m = c * v - s * u;
  const auto p = m / (Rm + f);

  lat_rad += p;

  ht = f + m * p / 2.;
  const auto ht2 = WGS84.get_ht(w, z, lat_rad);
  (void)ht2;

  return lla;
}

} // namespace math
} // namespace visukf
