//
// Created by bene on 22.03.24.
//

#include "Position.h"

#include "Velocity.h"

namespace visukf {

Position::Noising Position::noising() const { return Noising::Identity() * 1; }

Position::Data Position::timeUpdate(float dt, const Position &pos,
                                    const Velocity &vel) const {
  // km + s * m/s
  return pos.data + vel.data * static_cast<double>(dt)/1000;
}

} // namespace visukf
