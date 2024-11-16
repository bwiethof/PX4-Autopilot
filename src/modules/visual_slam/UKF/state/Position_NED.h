//
// Created by bene on 15.04.24.
//

#ifndef PX4_POSITION_NED_H
#define PX4_POSITION_NED_H
#include <core/field.hpp>

namespace visukf{

class Position_NED : public ukf::core::SimpleField<3, Position_NED> {
public:
  ukf::core::state::FieldNoising<3> noising() const override;
  ukf::core::state::FieldData<3>
  timeUpdate(float d, const Position_NED &inputs) const override;
  using ukf::core::SimpleField<3, Position_NED>::Field;

  // Init shall set the initial value for positioning // needed?
  void init() {};
};

} // namespace vislam

#endif // PX4_POSITION_NED_H
