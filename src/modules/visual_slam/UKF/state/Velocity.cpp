//
// Created by bene on 24.03.24.
//

#include "Velocity.h"

namespace visukf {
Velocity::Data Velocity::timeUpdate(float, const Velocity &vel) const {
  return vel.data;
}

Velocity::Noising Velocity::noising() const { return Noising::Identity() * 10; }

} // namespace visukf
