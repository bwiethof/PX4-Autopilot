//
// Created by bene on 15.04.24.
//

#include "Position_NED.h"
#include "core/typedefs.h"

namespace  visukf {

ukf::core::state::FieldNoising<3> Position_NED::noising() const {
  return ukf::core::state::FieldNoising<3>();
}

ukf::core::state::FieldData<3>
Position_NED::timeUpdate(float d, const Position_NED &) const {
  return ukf::core::state::FieldData<3>();
}

} // namespace vislam