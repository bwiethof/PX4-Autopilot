//
// Created by bene on 24.03.24.
//

#pragma once
#include <core/field.hpp>

namespace visukf {

class Velocity : public ukf::core::SimpleField<3, Velocity> {


public:
  Noising noising() const override;
  Data timeUpdate(float, const Velocity &) const override;
  using Field::Field;
};

} // namespace visukf
